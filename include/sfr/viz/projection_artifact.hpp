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

enum class ArtifactErrorCode : std::uint8_t {
  kInvalidRequest,
  kExistingRun,
  kFilesystem,
  kImageEncoding,
  kWriteFailure,
};

class ArtifactError final : public std::runtime_error {
public:
  ArtifactError(ArtifactErrorCode code, const std::string& message);

  [[nodiscard]] ArtifactErrorCode code() const noexcept;

private:
  ArtifactErrorCode code_;
};

struct ProjectionRunConfig final {
  double z_min_m;
  double depth_min_m;
  double depth_max_m;
  int point_radius_px;
};

struct ProjectionStageDurations final {
  double source_decode_ms;
  double geometry_ms;
  double visualization_ms;
};

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

struct ProjectionArtifactResult final {
  std::filesystem::path run_directory;
  std::filesystem::path summary_file;
  std::filesystem::path frames_file;
  std::filesystem::path overlay_file;
};

[[nodiscard]] std::string makeRunId();
[[nodiscard]] ProjectionArtifactResult
writeProjectionArtifacts(const ProjectionArtifactRequest& request);

} // namespace sfr::viz

#endif // SFR_VIZ_PROJECTION_ARTIFACT_HPP_
