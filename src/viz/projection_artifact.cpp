#include "sfr/viz/projection_artifact.hpp"

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>
#include <opencv2/imgcodecs.hpp>

#include "sfr/core/build_info.hpp"

namespace sfr::viz {

namespace {

using Json = nlohmann::json;

class TemporaryRunDirectory final {
public:
  explicit TemporaryRunDirectory(std::filesystem::path path) : path_(std::move(path)) {}

  TemporaryRunDirectory(const TemporaryRunDirectory&) = delete;
  TemporaryRunDirectory& operator=(const TemporaryRunDirectory&) = delete;
  TemporaryRunDirectory(TemporaryRunDirectory&&) = delete;
  TemporaryRunDirectory& operator=(TemporaryRunDirectory&&) = delete;

  ~TemporaryRunDirectory() {
    if (!committed_) {
      std::error_code error;
      std::filesystem::remove_all(path_, error);
    }
  }

  void commit() noexcept { committed_ = true; }

private:
  std::filesystem::path path_;
  bool committed_{false};
};

[[nodiscard]] bool isSafeRunId(const std::string_view run_id) {
  if (run_id.empty() || run_id == "." || run_id == "..") {
    return false;
  }
  return std::ranges::all_of(run_id, [](const unsigned char character) {
    return std::isalnum(character) != 0 || character == '-' || character == '_';
  });
}

void validateFiniteDuration(double value, const std::string_view name) {
  if (!std::isfinite(value) || value < 0.0) {
    throw ArtifactError(ArtifactErrorCode::kInvalidRequest,
                        std::string(name) + " duration must be finite and nonnegative");
  }
}

[[nodiscard]] std::uint64_t checkedProjectionTotal(const geometry::ProjectionCounts& counts) {
  std::uint64_t total = 0U;
  const auto add = [&total](const std::uint64_t value) {
    if (value > std::numeric_limits<std::uint64_t>::max() - total) {
      throw ArtifactError(ArtifactErrorCode::kInvalidRequest, "projection counter overflow");
    }
    total += value;
  };
  add(counts.visible_points);
  add(counts.non_finite_input);
  add(counts.behind_or_too_near);
  add(counts.non_positive_homogeneous_depth);
  add(counts.outside_image);
  if (counts.visible_points > counts.input_points || total > counts.input_points) {
    throw ArtifactError(ArtifactErrorCode::kInvalidRequest,
                        "projection counters overflow or violate input accounting");
  }
  return total;
}

void validateRequest(const ProjectionArtifactRequest& request) {
  if (request.output_root.empty() || !isSafeRunId(request.run_id) || request.sequence_id.empty() ||
      request.image_relative_name.empty() || request.lidar_relative_name.empty() ||
      request.image_timestamp.empty() || request.lidar_timestamp.empty()) {
    throw ArtifactError(ArtifactErrorCode::kInvalidRequest,
                        "artifact request has an empty path/identifier or unsafe run ID");
  }
  if (!std::isfinite(request.config.z_min_m) || request.config.z_min_m <= 0.0 ||
      !std::isfinite(request.config.depth_min_m) || !std::isfinite(request.config.depth_max_m) ||
      request.config.depth_min_m < 0.0 ||
      request.config.depth_max_m <= request.config.depth_min_m ||
      request.config.point_radius_px < 0 || request.config.point_radius_px > 20) {
    throw ArtifactError(ArtifactErrorCode::kInvalidRequest,
                        "artifact request contains an invalid projection configuration");
  }
  validateFiniteDuration(request.durations.source_decode_ms, "source_decode");
  validateFiniteDuration(request.durations.geometry_ms, "geometry");
  validateFiniteDuration(request.durations.visualization_ms, "visualization");
  if (checkedProjectionTotal(request.projection_counts) != request.projection_counts.input_points ||
      request.overlay.projected_points != request.projection_counts.visible_points ||
      request.overlay.rendered_pixels > request.overlay.projected_points ||
      request.overlay.occluded_points !=
          request.overlay.projected_points - request.overlay.rendered_pixels) {
    throw ArtifactError(ArtifactErrorCode::kInvalidRequest,
                        "artifact request violates projection or z-buffer accounting");
  }
  if (request.overlay.image_bgr8.empty() || request.overlay.image_bgr8.type() != CV_8UC3) {
    throw ArtifactError(ArtifactErrorCode::kInvalidRequest,
                        "artifact overlay must be a nonempty BGR8 image");
  }
}

[[nodiscard]] Json singletonLatency(double milliseconds) {
  return Json{{"count", 1U},          {"min", milliseconds}, {"max", milliseconds},
              {"mean", milliseconds}, {"p50", milliseconds}, {"p95", milliseconds},
              {"p99", milliseconds}};
}

void writeBytes(const std::filesystem::path& path, const std::vector<std::uint8_t>& bytes) {
  std::ofstream output(path, std::ios::binary);
  output.exceptions(std::ios::failbit | std::ios::badbit);
  try {
    output.write(reinterpret_cast<const char*>(bytes.data()),
                 static_cast<std::streamsize>(bytes.size()));
    output.close();
  } catch (const std::ios_base::failure& error) {
    throw ArtifactError(ArtifactErrorCode::kWriteFailure,
                        "failed to write " + path.string() + ": " + error.what());
  }
}

void writeText(const std::filesystem::path& path, const std::string& content) {
  std::ofstream output(path);
  output.exceptions(std::ios::failbit | std::ios::badbit);
  try {
    output << content;
    output.close();
  } catch (const std::ios_base::failure& error) {
    throw ArtifactError(ArtifactErrorCode::kWriteFailure,
                        "failed to write " + path.string() + ": " + error.what());
  }
}

[[nodiscard]] std::string frameFilename(const std::uint64_t frame_id) {
  std::ostringstream name;
  name << std::setfill('0') << std::setw(10) << frame_id << ".png";
  return name.str();
}

} // namespace

ArtifactError::ArtifactError(ArtifactErrorCode code, const std::string& message)
    : std::runtime_error(message), code_(code) {}

ArtifactErrorCode ArtifactError::code() const noexcept { return code_; }

std::string makeRunId() {
  static std::atomic<std::uint64_t> sequence{0U};
  const auto timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
                             std::chrono::system_clock::now().time_since_epoch())
                             .count();
  return "sfr-v0_1-" + std::to_string(timestamp) + "-" + std::to_string(sequence.fetch_add(1U));
}

ProjectionArtifactResult writeProjectionArtifacts(const ProjectionArtifactRequest& request) {
  validateRequest(request);

  const std::filesystem::path final_directory = request.output_root / request.run_id;
  const std::filesystem::path temporary_directory = request.output_root / (request.run_id + ".tmp");
  const std::filesystem::path temporary_overlay_directory = temporary_directory / "overlays";
  TemporaryRunDirectory cleanup(temporary_directory);

  try {
    std::filesystem::create_directories(request.output_root);
    if (!std::filesystem::is_directory(request.output_root)) {
      throw ArtifactError(ArtifactErrorCode::kFilesystem,
                          "output root is not a directory: " + request.output_root.string());
    }
    if (std::filesystem::exists(temporary_directory)) {
      throw ArtifactError(ArtifactErrorCode::kExistingRun,
                          "temporary run directory already exists: " +
                              temporary_directory.string());
    }
    if (std::filesystem::exists(final_directory)) {
      if (!std::filesystem::is_directory(final_directory)) {
        throw ArtifactError(ArtifactErrorCode::kExistingRun,
                            "run output exists and is not a directory: " +
                                final_directory.string());
      }
      const bool nonempty = std::filesystem::directory_iterator(final_directory) !=
                            std::filesystem::directory_iterator();
      if (nonempty && !request.overwrite) {
        throw ArtifactError(ArtifactErrorCode::kExistingRun,
                            "run directory is nonempty; pass --overwrite to replace it: " +
                                final_directory.string());
      }
    }
    std::filesystem::create_directories(temporary_overlay_directory);
  } catch (const std::filesystem::filesystem_error& error) {
    throw ArtifactError(ArtifactErrorCode::kFilesystem,
                        std::string("unable to prepare run directory: ") + error.what());
  }

  const std::string overlay_name = frameFilename(request.image_frame_id);
  const std::filesystem::path temporary_overlay_file = temporary_overlay_directory / overlay_name;
  const auto serialization_start = std::chrono::steady_clock::now();
  std::vector<std::uint8_t> encoded_overlay;
  try {
    if (!cv::imencode(".png", request.overlay.image_bgr8, encoded_overlay)) {
      throw ArtifactError(ArtifactErrorCode::kImageEncoding, "OpenCV failed to encode overlay PNG");
    }
  } catch (const cv::Exception& error) {
    throw ArtifactError(ArtifactErrorCode::kImageEncoding,
                        std::string("OpenCV failed to encode overlay PNG: ") + error.what());
  }
  writeBytes(temporary_overlay_file, encoded_overlay);
  const double serialization_ms = std::chrono::duration<double, std::milli>(
                                      std::chrono::steady_clock::now() - serialization_start)
                                      .count();

  const double processing_total_ms = request.durations.source_decode_ms +
                                     request.durations.geometry_ms +
                                     request.durations.visualization_ms + serialization_ms;
  const double signed_sync_delta_ms =
      std::chrono::duration<double, std::milli>(request.signed_sync_delta).count();
  const Json durations{{"source_decode", request.durations.source_decode_ms},
                       {"geometry", request.durations.geometry_ms},
                       {"visualization", request.durations.visualization_ms},
                       {"serialization", serialization_ms},
                       {"processing_total", processing_total_ms}};
  const Json counts{{"lidar_input_finite", request.projection_counts.input_points},
                    {"lidar_non_finite", request.lidar_non_finite_points},
                    {"projected_visible", request.projection_counts.visible_points},
                    {"rejected_non_finite", request.projection_counts.non_finite_input},
                    {"rejected_behind_or_too_near", request.projection_counts.behind_or_too_near},
                    {"rejected_non_positive_homogeneous_depth",
                     request.projection_counts.non_positive_homogeneous_depth},
                    {"rejected_outside_image", request.projection_counts.outside_image},
                    {"rendered_pixels", request.overlay.rendered_pixels},
                    {"occluded_points", request.overlay.occluded_points}};
  const Json config{{"z_min_m", request.config.z_min_m},
                    {"depth_min_m", request.config.depth_min_m},
                    {"depth_max_m", request.config.depth_max_m},
                    {"point_radius_px", request.config.point_radius_px},
                    {"overwrite", request.overwrite},
                    {"pixel_conversion", "nearest_integer_then_clamp_after_continuous_bounds"},
                    {"z_buffer", "nearest_positive_camera_depth"},
                    {"depth_colormap", depthColormapName()}};

  const Json frame_record{{"schema_version", "1.0.0"},
                          {"pair_id", 0U},
                          {"image_frame_id", request.image_frame_id},
                          {"lidar_frame_id", request.lidar_frame_id},
                          {"image_relative_name", request.image_relative_name},
                          {"lidar_relative_name", request.lidar_relative_name},
                          {"image_original_timestamp", request.image_timestamp},
                          {"image_effective_timestamp", request.image_timestamp},
                          {"lidar_original_timestamp", request.lidar_timestamp},
                          {"lidar_effective_timestamp", request.lidar_timestamp},
                          {"signed_sync_delta_ms", signed_sync_delta_ms},
                          {"terminal_status", "complete"},
                          {"terminal_reason", "completed"},
                          {"stage_duration_ms", durations},
                          {"point_counts", counts},
                          {"overlay_relative_path", "overlays/" + overlay_name}};
  writeText(temporary_directory / "frames.jsonl", frame_record.dump() + '\n');

  const Json summary{
      {"schema_version", "1.0.0"},
      {"status", "complete"},
      {"run_id", request.run_id},
      {"capability_version", "v0.1"},
      {"dataset", Json{{"kind", "kitti_raw_sync"},
                       {"sequence_id", request.sequence_id},
                       {"camera", "image_02"}}},
      {"build", Json{{"git_commit", std::string(core::kBuildInfo.git_commit)},
                     {"git_dirty", core::kBuildInfo.git_dirty},
                     {"build_type", std::string(core::kBuildInfo.build_type)},
                     {"compiler", std::string(core::kBuildInfo.compiler)}}},
      {"host", Json{{"os", std::string(core::kBuildInfo.host_os)},
                    {"arch", std::string(core::kBuildInfo.host_arch)},
                    {"cpu", std::string(core::kBuildInfo.host_cpu)}}},
      {"config", config},
      {"counts", counts},
      {"latency_ms", Json{{"source_decode", singletonLatency(request.durations.source_decode_ms)},
                          {"geometry", singletonLatency(request.durations.geometry_ms)},
                          {"visualization", singletonLatency(request.durations.visualization_ms)},
                          {"serialization", singletonLatency(serialization_ms)},
                          {"processing_total", singletonLatency(processing_total_ms)}}},
      {"percentile_method", "nearest_rank"},
      {"warmup_frames", 0U},
      {"measured_frames", 1U}};
  writeText(temporary_directory / "run_summary.json", summary.dump(2) + '\n');

  try {
    if (std::filesystem::exists(final_directory)) {
      std::filesystem::remove_all(final_directory);
    }
    std::filesystem::rename(temporary_directory, final_directory);
  } catch (const std::filesystem::filesystem_error& error) {
    throw ArtifactError(ArtifactErrorCode::kFilesystem,
                        std::string("unable to publish run directory atomically: ") + error.what());
  }
  cleanup.commit();

  return ProjectionArtifactResult{final_directory, final_directory / "run_summary.json",
                                  final_directory / "frames.jsonl",
                                  final_directory / "overlays" / overlay_name};
}

} // namespace sfr::viz
