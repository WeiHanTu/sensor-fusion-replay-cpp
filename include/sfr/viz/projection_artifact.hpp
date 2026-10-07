#ifndef SFR_VIZ_PROJECTION_ARTIFACT_HPP_
#define SFR_VIZ_PROJECTION_ARTIFACT_HPP_

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <stdexcept>
#include <string>

#include <opencv2/core/mat.hpp>

#include "sfr/geometry/point_cloud_projection.hpp"
#include "sfr/viz/overlay.hpp"

namespace sfr::viz {

/// Stable categories for artifact validation and publication failures.
enum class ArtifactErrorCode : std::uint8_t {
  kInvalidRequest,
  kExistingRun,
  kFilesystem,
  kImageEncoding,
  kWriteFailure,
};

/// Typed artifact exception with a machine-checkable category.
class ArtifactError final : public std::runtime_error {
public:
  /// Copies `message` into `std::runtime_error` and stores `code`.
  ArtifactError(ArtifactErrorCode code, const std::string& message);

  /// Returns the stored error category.
  [[nodiscard]] ArtifactErrorCode code() const noexcept;

private:
  ArtifactErrorCode code_;
};

/// Geometry and overlay configuration serialized into a run report.
struct ProjectionRunConfig final {
  double z_min_m;
  double depth_min_m;
  double depth_max_m;
  int point_radius_px;
};

/// Caller-measured one-frame stage durations in milliseconds.
///
/// These partial stages are not runtime sojourn or end-to-end latency.
struct ProjectionStageDurations final {
  double source_decode_ms;
  double geometry_ms;
  double visualization_ms;
};

/// One-frame input bundle required for publication.
///
/// Relative source names must not disclose absolute private dataset paths.
/// `overlay.image_bgr8` uses OpenCV reference-counted storage. The writer borrows
/// the entire request for its call. All stage durations are finite non-negative
/// milliseconds.
struct ProjectionArtifactRequest final {
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
  geometry::ProjectionCounts projection_counts;
  std::uint64_t lidar_non_finite_points;
  OverlayResult overlay;
  ProjectionStageDurations durations;
  bool overwrite{false};
};

/// Owned paths to the published run and its required outputs.
struct ProjectionArtifactResult final {
  std::filesystem::path run_directory;
  std::filesystem::path summary_file;
  std::filesystem::path frames_file;
  std::filesystem::path overlay_file;
};

/// Creates a filesystem-safe run-ID candidate using wall time and process sequence.
///
/// Callers must still handle collisions at publication boundaries.
[[nodiscard]] std::string makeRunId();

/// Validates and publishes one projection run through a temporary sibling.
///
/// The request, including OpenCV image storage, is borrowed only for this call.
/// All output files close before a new run directory is renamed into view. RAII
/// removes only staging claimed by this call. There is no fsync/crash-durability
/// guarantee. Explicit overwrite removes the old run before rename; replacement
/// is not gap-free or transactional and a later failure can lose the old run.
///
/// @throws ArtifactError For invalid accounting/configuration, collisions,
///         filesystem failure, image encoding, or write failure.
[[nodiscard]] ProjectionArtifactResult
writeProjectionArtifacts(const ProjectionArtifactRequest& request);

} // namespace sfr::viz

#endif // SFR_VIZ_PROJECTION_ARTIFACT_HPP_
