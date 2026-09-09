#ifndef SFR_VIZ_SENSITIVITY_ARTIFACT_HPP_
#define SFR_VIZ_SENSITIVITY_ARTIFACT_HPP_

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

#include "sfr/geometry/calibration_sensitivity.hpp"
#include "sfr/geometry/point_cloud_projection.hpp"
#include "sfr/viz/overlay.hpp"
#include "sfr/viz/projection_artifact.hpp"

namespace sfr::viz {

enum class SensitivityPerturbationKind : std::uint8_t {
  kYawCameraYDegrees,
  kTranslationCameraXMeters,
};

struct SensitivityPanelResult final {
  SensitivityPerturbationKind kind;
  double signed_value;
  geometry::ProjectionCounts projection_counts;
  geometry::ProjectionDisplacementMetrics displacement;
  OverlayResult overlay;
};

struct SensitivityArtifactRequest final {
  std::filesystem::path output_root;
  std::string run_id;
  std::string sequence_id;
  std::uint64_t image_frame_id;
  std::uint64_t lidar_frame_id;
  std::string image_relative_name;
  std::string lidar_relative_name;
  std::string image_timestamp;
  std::string lidar_timestamp;
  std::chrono::nanoseconds signed_sync_delta;
  ProjectionRunConfig config;
  geometry::ProjectionCounts baseline_projection_counts;
  std::uint64_t lidar_non_finite_points;
  OverlayResult baseline_overlay;
  std::vector<SensitivityPanelResult> perturbations;
  ProjectionStageDurations durations;
  bool overwrite{false};
};

struct SensitivityArtifactResult final {
  std::filesystem::path run_directory;
  std::filesystem::path summary_file;
  std::filesystem::path frames_file;
  std::filesystem::path report_file;
  std::filesystem::path comparison_file;
};

[[nodiscard]] SensitivityArtifactResult
writeSensitivityArtifacts(const SensitivityArtifactRequest& request);

} // namespace sfr::viz

#endif // SFR_VIZ_SENSITIVITY_ARTIFACT_HPP_
