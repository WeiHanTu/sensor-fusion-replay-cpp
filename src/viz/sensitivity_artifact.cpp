#include "sfr/viz/sensitivity_artifact.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <numbers>
#include <set>
#include <sstream>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>
#include <opencv2/imgcodecs.hpp>

#include "sfr/core/build_info.hpp"
#include "sfr/viz/panel.hpp"

namespace sfr::viz {

namespace {

using Json = nlohmann::json;

class TemporarySensitivityDirectory final {
public:
  explicit TemporarySensitivityDirectory(std::filesystem::path path) : path_(std::move(path)) {
    try {
      if (!std::filesystem::create_directory(path_)) {
        throw ArtifactError(ArtifactErrorCode::kExistingRun,
                            "temporary sensitivity directory already exists: " + path_.string());
      }
    } catch (const std::filesystem::filesystem_error& error) {
      throw ArtifactError(ArtifactErrorCode::kFilesystem,
                          std::string("unable to claim temporary sensitivity directory: ") +
                              error.what());
    }
  }

  TemporarySensitivityDirectory(const TemporarySensitivityDirectory&) = delete;
  TemporarySensitivityDirectory& operator=(const TemporarySensitivityDirectory&) = delete;
  TemporarySensitivityDirectory(TemporarySensitivityDirectory&&) = delete;
  TemporarySensitivityDirectory& operator=(TemporarySensitivityDirectory&&) = delete;

  ~TemporarySensitivityDirectory() {
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

struct PerturbationMetadata final {
  std::string kind;
  std::string axis;
  std::string unit;
  int precision;
};

[[nodiscard]] PerturbationMetadata metadata(const SensitivityPerturbationKind kind) {
  switch (kind) {
  case SensitivityPerturbationKind::kYawCameraYDegrees:
    return {"rotation", "camera +y yaw", "deg", 1};
  case SensitivityPerturbationKind::kTranslationCameraXMeters:
    return {"translation", "camera +x", "m", 2};
  }
  throw ArtifactError(ArtifactErrorCode::kInvalidRequest, "unknown sensitivity perturbation kind");
}

[[nodiscard]] std::string formattedValue(const SensitivityPanelResult& panel) {
  const PerturbationMetadata details = metadata(panel.kind);
  std::ostringstream value;
  value << std::showpos << std::fixed << std::setprecision(details.precision) << panel.signed_value;
  return value.str();
}

[[nodiscard]] std::string panelStem(const SensitivityPanelResult& panel) {
  const PerturbationMetadata details = metadata(panel.kind);
  std::string stem = details.kind == "rotation" ? "yaw_camera_y_" : "translation_camera_x_";
  stem += panel.signed_value < 0.0 ? "neg_" : "pos_";
  std::ostringstream magnitude;
  magnitude << std::fixed << std::setprecision(details.precision) << std::abs(panel.signed_value);
  std::string magnitude_text = magnitude.str();
  std::ranges::replace(magnitude_text, '.', '_');
  return stem + magnitude_text + '_' + details.unit;
}

[[nodiscard]] std::string panelTitle(const SensitivityPanelResult& panel) {
  const PerturbationMetadata details = metadata(panel.kind);
  return details.axis + ": " + formattedValue(panel) + ' ' + details.unit;
}

[[nodiscard]] std::string panelSubtitle(const SensitivityPanelResult& panel) {
  std::ostringstream subtitle;
  subtitle << "left-applied | common=" << panel.displacement.common_visible_points
           << " | p95=" << std::fixed << std::setprecision(2)
           << panel.displacement.p95_displacement_px << " px";
  return subtitle.str();
}

[[nodiscard]] bool isSafeRunId(const std::string_view run_id) {
  if (run_id.empty() || run_id == "." || run_id == "..") {
    return false;
  }
  return std::ranges::all_of(run_id, [](const unsigned char character) {
    return std::isalnum(character) != 0 || character == '-' || character == '_';
  });
}

[[nodiscard]] std::uint64_t projectionTotal(const geometry::ProjectionCounts& counts) {
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
  return total;
}

void validateOverlayAccounting(const geometry::ProjectionCounts& counts,
                               const OverlayResult& overlay) {
  if (projectionTotal(counts) != counts.input_points ||
      overlay.projected_points != counts.visible_points ||
      overlay.rendered_pixels > overlay.projected_points ||
      overlay.occluded_points != overlay.projected_points - overlay.rendered_pixels ||
      overlay.image_bgr8.empty() || overlay.image_bgr8.type() != CV_8UC3) {
    throw ArtifactError(ArtifactErrorCode::kInvalidRequest,
                        "sensitivity panel violates projection or overlay accounting");
  }
}

void validateRequest(const SensitivityArtifactRequest& request) {
  if (request.output_root.empty() || !isSafeRunId(request.run_id) || request.sequence_id.empty() ||
      request.image_relative_name.empty() || request.lidar_relative_name.empty() ||
      request.image_timestamp.empty() || request.lidar_timestamp.empty() ||
      request.perturbations.size() != 8U) {
    throw ArtifactError(ArtifactErrorCode::kInvalidRequest,
                        "sensitivity request requires identifiers and exactly eight perturbations");
  }
  if (!std::isfinite(request.config.z_min_m) || request.config.z_min_m <= 0.0 ||
      !std::isfinite(request.config.depth_min_m) || !std::isfinite(request.config.depth_max_m) ||
      request.config.depth_min_m < 0.0 ||
      request.config.depth_max_m <= request.config.depth_min_m ||
      request.config.point_radius_px < 0 || request.config.point_radius_px > 20 ||
      !std::isfinite(request.durations.source_decode_ms) ||
      !std::isfinite(request.durations.geometry_ms) ||
      !std::isfinite(request.durations.visualization_ms) ||
      request.durations.source_decode_ms < 0.0 || request.durations.geometry_ms < 0.0 ||
      request.durations.visualization_ms < 0.0) {
    throw ArtifactError(ArtifactErrorCode::kInvalidRequest,
                        "sensitivity request contains an invalid configuration or duration");
  }
  validateOverlayAccounting(request.baseline_projection_counts, request.baseline_overlay);
  std::set<std::string> stems;
  for (const SensitivityPanelResult& panel : request.perturbations) {
    if (!std::isfinite(panel.signed_value) || panel.signed_value == 0.0 ||
        panel.displacement.common_visible_points == 0U ||
        !std::isfinite(panel.displacement.median_displacement_px) ||
        !std::isfinite(panel.displacement.p95_displacement_px) ||
        panel.displacement.median_displacement_px < 0.0 ||
        panel.displacement.p95_displacement_px < panel.displacement.median_displacement_px) {
      throw ArtifactError(
          ArtifactErrorCode::kInvalidRequest,
          "sensitivity perturbation contains invalid value or displacement metrics");
    }
    validateOverlayAccounting(panel.projection_counts, panel.overlay);
    if (panel.overlay.image_bgr8.size() != request.baseline_overlay.image_bgr8.size() ||
        !stems.insert(panelStem(panel)).second) {
      throw ArtifactError(ArtifactErrorCode::kInvalidRequest,
                          "sensitivity panels must have matching images and unique perturbations");
    }
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

void writePng(const std::filesystem::path& path, const cv::Mat& image) {
  try {
    if (!cv::imwrite(path.string(), image)) {
      throw ArtifactError(ArtifactErrorCode::kImageEncoding,
                          "OpenCV failed to write sensitivity PNG: " + path.string());
    }
  } catch (const cv::Exception& error) {
    throw ArtifactError(ArtifactErrorCode::kImageEncoding,
                        "OpenCV failed to write sensitivity PNG: " + path.string() + ": " +
                            error.what());
  }
}

[[nodiscard]] Json projectionCountsJson(const geometry::ProjectionCounts& counts) {
  return Json{{"input", counts.input_points},
              {"visible", counts.visible_points},
              {"non_finite", counts.non_finite_input},
              {"behind_or_too_near", counts.behind_or_too_near},
              {"non_positive_homogeneous_depth", counts.non_positive_homogeneous_depth},
              {"outside_image", counts.outside_image}};
}

[[nodiscard]] Json singletonLatency(const double milliseconds) {
  return Json{{"count", 1U},          {"min", milliseconds}, {"max", milliseconds},
              {"mean", milliseconds}, {"p50", milliseconds}, {"p95", milliseconds},
              {"p99", milliseconds}};
}

} // namespace

SensitivityArtifactResult writeSensitivityArtifacts(const SensitivityArtifactRequest& request) {
  validateRequest(request);
  const std::filesystem::path final_directory = request.output_root / request.run_id;
  const std::filesystem::path temporary_directory = request.output_root / (request.run_id + ".tmp");
  const std::filesystem::path temporary_sensitivity_directory =
      temporary_directory / "calibration_sensitivity";

  try {
    std::filesystem::create_directories(request.output_root);
    if (!std::filesystem::is_directory(request.output_root)) {
      throw ArtifactError(ArtifactErrorCode::kFilesystem,
                          "sensitivity output root is not a directory");
    }
    if (std::filesystem::exists(final_directory)) {
      if (!std::filesystem::is_directory(final_directory)) {
        throw ArtifactError(ArtifactErrorCode::kExistingRun,
                            "sensitivity run output exists and is not a directory");
      }
      const bool nonempty = std::filesystem::directory_iterator(final_directory) !=
                            std::filesystem::directory_iterator();
      if (nonempty && !request.overwrite) {
        throw ArtifactError(
            ArtifactErrorCode::kExistingRun,
            "sensitivity run directory is nonempty; pass --overwrite to replace it");
      }
    }
  } catch (const std::filesystem::filesystem_error& error) {
    throw ArtifactError(ArtifactErrorCode::kFilesystem,
                        std::string("unable to prepare sensitivity run directory: ") +
                            error.what());
  }

  TemporarySensitivityDirectory cleanup(temporary_directory);
  try {
    std::filesystem::create_directory(temporary_sensitivity_directory);
  } catch (const std::filesystem::filesystem_error& error) {
    throw ArtifactError(ArtifactErrorCode::kFilesystem,
                        std::string("unable to prepare sensitivity directory: ") + error.what());
  }

  const auto serialization_start = std::chrono::steady_clock::now();
  std::vector<cv::Mat> labeled_panels;
  labeled_panels.reserve(9U);
  const cv::Mat baseline_labeled = addPanelHeader(
      request.baseline_overlay.image_bgr8, "baseline",
      "supplied calibration | " +
          std::to_string(request.baseline_projection_counts.visible_points) + " visible");
  labeled_panels.push_back(baseline_labeled);
  writePng(temporary_sensitivity_directory / "baseline.png", baseline_labeled);

  Json experiments = Json::array();
  for (const SensitivityPanelResult& panel : request.perturbations) {
    const std::string stem = panelStem(panel);
    const cv::Mat labeled =
        addPanelHeader(panel.overlay.image_bgr8, panelTitle(panel), panelSubtitle(panel));
    labeled_panels.push_back(labeled);
    writePng(temporary_sensitivity_directory / (stem + ".png"), labeled);
    const PerturbationMetadata details = metadata(panel.kind);
    experiments.push_back(
        Json{{"id", stem},
             {"kind", details.kind},
             {"axis", details.axis},
             {"signed_value", panel.signed_value},
             {"unit", details.unit},
             {"application", "left"},
             {"coordinate_frame", "camera_rect_00"},
             {"common_visible_points", panel.displacement.common_visible_points},
             {"disappeared_points", panel.displacement.disappeared_points},
             {"appeared_points", panel.displacement.appeared_points},
             {"median_displacement_px", panel.displacement.median_displacement_px},
             {"p95_displacement_px", panel.displacement.p95_displacement_px},
             {"projection_counts", projectionCountsJson(panel.projection_counts)},
             {"image", stem + ".png"}});
  }
  const cv::Mat comparison = composePanelGrid(labeled_panels, 3);
  writePng(temporary_sensitivity_directory / "comparison.png", comparison);

  const Json report{
      {"schema_version", "1.0.0"},
      {"baseline",
       Json{{"application", "supplied_calibration"},
            {"projection_counts", projectionCountsJson(request.baseline_projection_counts)},
            {"image", "baseline.png"}}},
      {"perturbation_application", "T_prime_camera_rect_00_lidar = Delta_camera * "
                                   "T_camera_rect_00_lidar"},
      {"percentile_method", "nearest_rank"},
      {"experiments", experiments},
      {"analytic_illustration",
       Json{{"range_m", 50.0},
            {"angle_deg", 1.0},
            {"lateral_offset_m", 50.0 * std::tan(std::numbers::pi / 180.0)},
            {"claim_boundary", "analytic angular-offset illustration; not measured KITTI error "
                               "or pixel reprojection accuracy"}}},
      {"comparison_image", "comparison.png"}};
  writeText(temporary_sensitivity_directory / "report.json", report.dump(2) + '\n');
  const double serialization_ms = std::chrono::duration<double, std::milli>(
                                      std::chrono::steady_clock::now() - serialization_start)
                                      .count();
  const double processing_total_ms = request.durations.source_decode_ms +
                                     request.durations.geometry_ms +
                                     request.durations.visualization_ms + serialization_ms;
  const double signed_sync_delta_ms =
      std::chrono::duration<double, std::milli>(request.signed_sync_delta).count();
  const Json config{{"z_min_m", request.config.z_min_m},
                    {"depth_min_m", request.config.depth_min_m},
                    {"depth_max_m", request.config.depth_max_m},
                    {"point_radius_px", request.config.point_radius_px},
                    {"depth_colormap", depthColormapName()},
                    {"overwrite", request.overwrite}};
  const Json frame_record{
      {"schema_version", "1.0.0"},
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
      {"stage_duration_ms", Json{{"source_decode", request.durations.source_decode_ms},
                                 {"geometry", request.durations.geometry_ms},
                                 {"visualization", request.durations.visualization_ms},
                                 {"serialization", serialization_ms},
                                 {"processing_total", processing_total_ms}}},
      {"point_counts", projectionCountsJson(request.baseline_projection_counts)},
      {"sensitivity_report", "calibration_sensitivity/report.json"}};
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
      {"counts", Json{{"lidar_input_finite", request.baseline_projection_counts.input_points},
                      {"lidar_non_finite", request.lidar_non_finite_points},
                      {"baseline_visible", request.baseline_projection_counts.visible_points},
                      {"perturbation_count", request.perturbations.size()}}},
      {"latency_ms", Json{{"source_decode", singletonLatency(request.durations.source_decode_ms)},
                          {"geometry", singletonLatency(request.durations.geometry_ms)},
                          {"visualization", singletonLatency(request.durations.visualization_ms)},
                          {"serialization", singletonLatency(serialization_ms)},
                          {"processing_total", singletonLatency(processing_total_ms)}}},
      {"percentile_method", "nearest_rank"},
      {"warmup_frames", 0U},
      {"measured_frames", 1U},
      {"sensitivity_report", "calibration_sensitivity/report.json"}};
  writeText(temporary_directory / "run_summary.json", summary.dump(2) + '\n');

  try {
    if (std::filesystem::exists(final_directory)) {
      std::filesystem::remove_all(final_directory);
    }
    std::filesystem::rename(temporary_directory, final_directory);
  } catch (const std::filesystem::filesystem_error& error) {
    throw ArtifactError(ArtifactErrorCode::kFilesystem,
                        std::string("unable to publish sensitivity run atomically: ") +
                            error.what());
  }
  cleanup.commit();

  return SensitivityArtifactResult{
      final_directory,
      final_directory / "run_summary.json",
      final_directory / "frames.jsonl",
      final_directory / "calibration_sensitivity/report.json",
      final_directory / "calibration_sensitivity/comparison.png",
  };
}

} // namespace sfr::viz
