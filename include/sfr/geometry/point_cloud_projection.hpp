#ifndef SFR_GEOMETRY_POINT_CLOUD_PROJECTION_HPP_
#define SFR_GEOMETRY_POINT_CLOUD_PROJECTION_HPP_

#include <cstdint>
#include <span>
#include <vector>

#include "sfr/core/point_types.hpp"
#include "sfr/geometry/projection.hpp"
#include "sfr/geometry/rigid_transform.hpp"

namespace sfr::geometry {

struct ProjectionCounts final {
  std::uint64_t input_points;
  std::uint64_t visible_points;
  std::uint64_t non_finite_input;
  std::uint64_t behind_or_too_near;
  std::uint64_t non_positive_homogeneous_depth;
  std::uint64_t outside_image;
};

struct PointCloudProjectionResult final {
  std::vector<core::ProjectedPoint> visible_points;
  ProjectionCounts counts;
};

// Borrows LiDAR-frame meter coordinates for this call; retains no references.
// Requires T_camera_rect_00_lidar endpoints exactly as named (else GeometryError).
// Normal per-point rejection is counted, not thrown. Visible output owns storage
// and preserves input-span order/index, continuous pixels, camera-z meters, and
// reflectance. Loader compaction means source_index need not be a file-record ID.
// Each input reaches one terminal bucket. Artifact/benchmark boundaries check
// the total. O(N) time and O(N) reserved output capacity, even if few are visible.
[[nodiscard]] PointCloudProjectionResult
projectPointCloud(std::span<const core::PointXYZI> points_lidar,
                  const RigidTransform& T_camera_rect_00_lidar,
                  const RectifiedProjection& projection);

} // namespace sfr::geometry

#endif // SFR_GEOMETRY_POINT_CLOUD_PROJECTION_HPP_
