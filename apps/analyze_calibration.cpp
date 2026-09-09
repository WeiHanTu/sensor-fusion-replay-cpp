#include <algorithm>
#include <array>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <numbers>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "sfr/geometry/calibration_sensitivity.hpp"
#include "sfr/geometry/point_cloud_projection.hpp"
#include "sfr/io/io_error.hpp"
#include "sfr/io/kitti_calibration.hpp"
#include "sfr/io/kitti_io.hpp"
#include "sfr/viz/overlay.hpp"
#include "sfr/viz/projection_artifact.hpp"
#include "sfr/viz/sensitivity_artifact.hpp"

namespace {

constexpr int kExitSuccess = 0;
constexpr int kExitInvalidArguments = 2;
constexpr int kExitInputFailure = 3;
constexpr int kExitProcessingFailure = 4;
constexpr int kExitOutputFailure = 5;

class ArgumentError final : public std::runtime_error {
public:
  explicit ArgumentError(const std::string& message) : std::runtime_error(message) {}
};

struct AnalyzeOptions final {
  std::filesystem::path dataset_root;
  std::string drive;
  std::uint64_t frame_id{0U};
  std::filesystem::path output_root;
  double z_min_m{0.1};
  double depth_min_m{1.0};
  double depth_max_m{80.0};
  int point_radius_px{1};
  bool overwrite{false};
};

struct NumericArgument final {
  std::string text;
  std::string_view option;
};

struct ComputedPerturbation final {
  sfr::viz::SensitivityPerturbationKind kind;
  double signed_value;
  sfr::geometry::PointCloudProjectionResult projection;
  sfr::geometry::ProjectionDisplacementMetrics displacement;
};

void printHelp(std::ostream& output) {
  output << "Usage: analyze_calibration --dataset-root <daily-root> --drive <*_sync> "
            "--frame <id> --output-dir <dir> [options]\n\n"
            "Runs the fixed v0.1 left-applied camera-frame sensitivity suite:\n"
            "  yaw about camera +y: -1.0, -0.5, +0.5, +1.0 deg\n"
            "  translation along camera +x: -0.05, -0.01, +0.01, +0.05 m\n\n"
            "Required:\n"
            "  --dataset-root <path>  KITTI Raw daily directory containing calibration\n"
            "  --drive <name>         Synced/rectified drive basename ending in _sync\n"
            "  --frame <id>           Numeric frame ID present in image_02 and Velodyne data\n"
            "  --output-dir <path>    Parent directory for a unique atomic run directory\n\n"
            "Options (defaults):\n"
            "  --z-min-m <meters>     Minimum rectified camera depth (0.1)\n"
            "  --depth-min-m <meters> Colormap lower clamp (1.0)\n"
            "  --depth-max-m <meters> Colormap upper clamp (80.0)\n"
            "  --point-radius-px <n>  Overlay point radius in [0,20] (1)\n"
            "  --overwrite            Allow replacement of the exact generated run directory\n"
            "  --help                 Show this help\n";
}

[[nodiscard]] std::uint64_t parseUnsigned(const NumericArgument& argument) {
  std::uint64_t value = 0U;
  const auto [end, error] =
      std::from_chars(argument.text.data(), argument.text.data() + argument.text.size(), value);
  if (argument.text.empty() || error != std::errc{} ||
      end != argument.text.data() + argument.text.size()) {
    throw ArgumentError(std::string(argument.option) + " requires an unsigned integer");
  }
  return value;
}

[[nodiscard]] int parseInteger(const NumericArgument& argument) {
  int value = 0;
  const auto [end, error] =
      std::from_chars(argument.text.data(), argument.text.data() + argument.text.size(), value);
  if (argument.text.empty() || error != std::errc{} ||
      end != argument.text.data() + argument.text.size()) {
    throw ArgumentError(std::string(argument.option) + " requires an integer");
  }
  return value;
}

[[nodiscard]] double parseDouble(const NumericArgument& argument) {
  std::size_t consumed = 0U;
  double value = 0.0;
  try {
    value = std::stod(argument.text, &consumed);
  } catch (const std::exception&) {
    throw ArgumentError(std::string(argument.option) + " requires a finite number");
  }
  if (consumed != argument.text.size() || !std::isfinite(value)) {
    throw ArgumentError(std::string(argument.option) + " requires a finite number");
  }
  return value;
}

[[nodiscard]] AnalyzeOptions parseArguments(int argument_count, char** argument_values) {
  AnalyzeOptions options;
  bool dataset_set = false;
  bool drive_set = false;
  bool frame_set = false;
  bool output_set = false;
  const auto requireValue = [&](int& index, const std::string_view option) -> std::string {
    if (index + 1 >= argument_count) {
      throw ArgumentError(std::string(option) + " requires a value");
    }
    ++index;
    return argument_values[index];
  };

  for (int index = 1; index < argument_count; ++index) {
    const std::string_view argument(argument_values[index]);
    if (argument == "--help") {
      printHelp(std::cout);
      throw ArgumentError("__help_shown__");
    }
    if (argument == "--overwrite") {
      options.overwrite = true;
      continue;
    }
    if (argument == "--dataset-root") {
      if (dataset_set) {
        throw ArgumentError("--dataset-root may be specified only once");
      }
      options.dataset_root = requireValue(index, argument);
      dataset_set = true;
    } else if (argument == "--drive") {
      if (drive_set) {
        throw ArgumentError("--drive may be specified only once");
      }
      options.drive = requireValue(index, argument);
      drive_set = true;
    } else if (argument == "--frame") {
      if (frame_set) {
        throw ArgumentError("--frame may be specified only once");
      }
      options.frame_id = parseUnsigned({requireValue(index, argument), argument});
      frame_set = true;
    } else if (argument == "--output-dir") {
      if (output_set) {
        throw ArgumentError("--output-dir may be specified only once");
      }
      options.output_root = requireValue(index, argument);
      output_set = true;
    } else if (argument == "--z-min-m") {
      options.z_min_m = parseDouble({requireValue(index, argument), argument});
    } else if (argument == "--depth-min-m") {
      options.depth_min_m = parseDouble({requireValue(index, argument), argument});
    } else if (argument == "--depth-max-m") {
      options.depth_max_m = parseDouble({requireValue(index, argument), argument});
    } else if (argument == "--point-radius-px") {
      options.point_radius_px = parseInteger({requireValue(index, argument), argument});
    } else {
      throw ArgumentError("unknown argument: " + std::string(argument));
    }
  }
  if (!dataset_set || !drive_set || !frame_set || !output_set || options.dataset_root.empty() ||
      options.drive.empty() || options.output_root.empty()) {
    throw ArgumentError("--dataset-root, --drive, --frame, and --output-dir are all required");
  }
  if (options.z_min_m <= 0.0 || options.depth_min_m < 0.0 ||
      options.depth_max_m <= options.depth_min_m || options.point_radius_px < 0 ||
      options.point_radius_px > 20) {
    throw ArgumentError(
        "depth limits must increase, z-min must be positive, and point radius must be in [0,20]");
  }
  return options;
}

[[nodiscard]] const sfr::io::TimedFrameFile&
findFrame(const std::vector<sfr::io::TimedFrameFile>& frames, const std::uint64_t frame_id,
          const std::string_view stream_name) {
  const auto match =
      std::ranges::lower_bound(frames, frame_id, {}, [](const sfr::io::TimedFrameFile& frame) {
        return frame.frame.frame_id;
      });
  if (match == frames.end() || match->frame.frame_id != frame_id) {
    throw sfr::io::IoError(sfr::io::IoErrorCode::kInvalidLayout, std::string(stream_name) +
                                                                     " does not contain frame " +
                                                                     std::to_string(frame_id));
  }
  return *match;
}

[[nodiscard]] double millisecondsSince(const std::chrono::steady_clock::time_point start) {
  return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start)
      .count();
}

[[nodiscard]] int run(const AnalyzeOptions& options) {
  const auto decode_start = std::chrono::steady_clock::now();
  const sfr::io::KittiCalibration calibration = sfr::io::loadKittiCalibration(options.dataset_root);
  const sfr::io::KittiSequenceLayout layout =
      sfr::io::loadKittiSequenceLayout(options.dataset_root, options.drive);
  const sfr::io::TimedFrameFile& image_frame =
      findFrame(layout.image_frames, options.frame_id, "image_02");
  const sfr::io::TimedFrameFile& lidar_frame =
      findFrame(layout.lidar_frames, options.frame_id, "velodyne_points");
  const cv::Mat image = sfr::io::loadBgrImage(image_frame.frame.path, calibration.image_width_px,
                                              calibration.image_height_px);
  const sfr::io::LidarLoadResult lidar = sfr::io::loadVelodyneFrame(lidar_frame.frame.path);
  const double source_decode_ms = millisecondsSince(decode_start);

  const auto geometry_start = std::chrono::steady_clock::now();
  const sfr::geometry::RigidTransform baseline_transform = calibration.TCameraRect00Lidar();
  const sfr::geometry::RectifiedProjection camera =
      calibration.rectifiedProjection(options.z_min_m);
  const sfr::geometry::PointCloudProjectionResult baseline =
      sfr::geometry::projectPointCloud(lidar.points, baseline_transform, camera);
  std::vector<ComputedPerturbation> computed;
  computed.reserve(8U);
  for (const double yaw_deg : std::array{-1.0, -0.5, 0.5, 1.0}) {
    sfr::geometry::PointCloudProjectionResult projection = sfr::geometry::projectPointCloud(
        lidar.points,
        sfr::geometry::leftApplyCameraYaw(baseline_transform, yaw_deg * std::numbers::pi / 180.0),
        camera);
    const sfr::geometry::ProjectionDisplacementMetrics displacement =
        sfr::geometry::compareProjectedPoints(baseline.visible_points, projection.visible_points);
    computed.push_back({sfr::viz::SensitivityPerturbationKind::kYawCameraYDegrees, yaw_deg,
                        std::move(projection), displacement});
  }
  for (const double translation_m : std::array{-0.05, -0.01, 0.01, 0.05}) {
    sfr::geometry::PointCloudProjectionResult projection = sfr::geometry::projectPointCloud(
        lidar.points, sfr::geometry::leftApplyCameraXTranslation(baseline_transform, translation_m),
        camera);
    const sfr::geometry::ProjectionDisplacementMetrics displacement =
        sfr::geometry::compareProjectedPoints(baseline.visible_points, projection.visible_points);
    computed.push_back({sfr::viz::SensitivityPerturbationKind::kTranslationCameraXMeters,
                        translation_m, std::move(projection), displacement});
  }
  const double geometry_ms = millisecondsSince(geometry_start);

  const auto visualization_start = std::chrono::steady_clock::now();
  const sfr::viz::OverlayConfig overlay_config{.depth_min_m = options.depth_min_m,
                                               .depth_max_m = options.depth_max_m,
                                               .point_radius_px = options.point_radius_px};
  sfr::viz::OverlayResult baseline_overlay =
      sfr::viz::renderDepthOverlay(image, baseline.visible_points, overlay_config);
  std::vector<sfr::viz::SensitivityPanelResult> panels;
  panels.reserve(computed.size());
  for (ComputedPerturbation& perturbation : computed) {
    const sfr::geometry::ProjectionCounts counts = perturbation.projection.counts;
    sfr::viz::OverlayResult overlay =
        sfr::viz::renderDepthOverlay(image, perturbation.projection.visible_points, overlay_config);
    panels.push_back({perturbation.kind, perturbation.signed_value, counts,
                      perturbation.displacement, std::move(overlay)});
  }
  const double visualization_ms = millisecondsSince(visualization_start);

  const sfr::viz::SensitivityArtifactResult artifacts = sfr::viz::writeSensitivityArtifacts({
      .output_root = options.output_root,
      .run_id = sfr::viz::makeRunId(),
      .sequence_id = layout.sequence_id,
      .image_frame_id = image_frame.frame.frame_id,
      .lidar_frame_id = lidar_frame.frame.frame_id,
      .image_relative_name = image_frame.frame.path.filename().string(),
      .lidar_relative_name = lidar_frame.frame.path.filename().string(),
      .image_timestamp = image_frame.timestamp.original_text,
      .lidar_timestamp = lidar_frame.timestamp.original_text,
      .signed_sync_delta =
          lidar_frame.timestamp.timestamp.civilTime() - image_frame.timestamp.timestamp.civilTime(),
      .config = {.z_min_m = options.z_min_m,
                 .depth_min_m = options.depth_min_m,
                 .depth_max_m = options.depth_max_m,
                 .point_radius_px = options.point_radius_px},
      .baseline_projection_counts = baseline.counts,
      .lidar_non_finite_points = lidar.non_finite_points,
      .baseline_overlay = std::move(baseline_overlay),
      .perturbations = std::move(panels),
      .durations = {.source_decode_ms = source_decode_ms,
                    .geometry_ms = geometry_ms,
                    .visualization_ms = visualization_ms},
      .overwrite = options.overwrite,
  });
  std::cout << "completed calibration sensitivity run: " << artifacts.run_directory.string()
            << '\n';
  return kExitSuccess;
}

} // namespace

int main(int argument_count, char** argument_values) {
  try {
    return run(parseArguments(argument_count, argument_values));
  } catch (const ArgumentError& error) {
    if (std::string_view(error.what()) == "__help_shown__") {
      return kExitSuccess;
    }
    std::cerr << "argument error: " << error.what() << '\n';
    return kExitInvalidArguments;
  } catch (const sfr::io::IoError& error) {
    std::cerr << "input error: " << error.what() << '\n';
    return kExitInputFailure;
  } catch (const sfr::viz::ArtifactError& error) {
    std::cerr << "output error: " << error.what() << '\n';
    return kExitOutputFailure;
  } catch (const std::filesystem::filesystem_error& error) {
    std::cerr << "input error: " << error.what() << '\n';
    return kExitInputFailure;
  } catch (const std::exception& error) {
    std::cerr << "processing error: " << error.what() << '\n';
    return kExitProcessingFailure;
  }
}
