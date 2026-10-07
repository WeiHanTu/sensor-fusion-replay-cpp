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

/// Supported left-applied perturbation family and serialized unit convention.
enum class SensitivityPerturbationKind : std::uint8_t {
  kYawCameraYDegrees,
  kTranslationCameraXMeters,
};

/// Owned metrics and ref-counted overlay for one nonzero perturbation.
struct SensitivityPanelResult final {
  SensitivityPerturbationKind kind;
  double signed_value;
  geometry::ProjectionCounts projection_counts;
  geometry::ProjectionDisplacementMetrics displacement;
  OverlayResult overlay;
};

/// Baseline/perturbation input bundle required for sensitivity output.
///
/// Exactly eight perturbations are required. Their image dimensions must match
/// the baseline, identifiers must be unique, and common-visible sets nonempty.
/// Relative source names must not disclose absolute private dataset paths.
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

/// Owned paths to the published sensitivity run and its primary outputs.
struct SensitivityArtifactResult final {
  std::filesystem::path run_directory;
  std::filesystem::path summary_file;
  std::filesystem::path frames_file;
  std::filesystem::path report_file;
  std::filesystem::path comparison_file;
};

/// Validates and publishes baseline/eight-perturbation sensitivity artifacts.
///
/// Input OpenCV storage is borrowed only for this call. The writer labels nine
/// panels, creates a comparison grid and JSON reports, then publishes through an
/// exclusively claimed temporary sibling. It has the same non-durable,
/// non-transactional overwrite boundary as `writeProjectionArtifacts`.
///
/// @throws ArtifactError For invalid accounting/configuration/perturbations,
///         collisions, filesystem failure, image encoding, or write failure.
[[nodiscard]] SensitivityArtifactResult
writeSensitivityArtifacts(const SensitivityArtifactRequest& request);

} // namespace sfr::viz

#endif // SFR_VIZ_SENSITIVITY_ARTIFACT_HPP_
