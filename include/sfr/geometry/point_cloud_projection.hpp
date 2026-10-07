#ifndef SFR_GEOMETRY_POINT_CLOUD_PROJECTION_HPP_
#define SFR_GEOMETRY_POINT_CLOUD_PROJECTION_HPP_

#include <cstdint>
#include <span>
#include <vector>

#include "sfr/core/point_types.hpp"
#include "sfr/geometry/projection.hpp"
#include "sfr/geometry/rigid_transform.hpp"

namespace sfr::geometry {

/// Mutually exclusive terminal counts for one projection input span.
struct ProjectionCounts final {
  std::uint64_t input_points;
  std::uint64_t visible_points;
  std::uint64_t non_finite_input;
  std::uint64_t behind_or_too_near;
  std::uint64_t non_positive_homogeneous_depth;
  std::uint64_t outside_image;
};

/// Owned visible points plus complete terminal accounting for the input span.
struct PointCloudProjectionResult final {
  std::vector<core::ProjectedPoint> visible_points;
  ProjectionCounts counts;
};

/// Projects LiDAR-frame points through a rectified camera model.
///
/// The input span is borrowed only for the call; no references are retained.
/// `T_camera_rect_00_lidar` must map `lidar` to `camera_rect_00`. Normal
/// per-point rejection is counted rather than thrown. Visible output owns its
/// storage and preserves input-span order/index, reflectance, continuous pixels,
/// and camera-forward depth in meters. Loader compaction means `source_index`
/// need not identify the original file record.
///
/// @throws GeometryError For wrong transform endpoints or accounting overflow.
/// Complexity: O(N) time and O(N) reserved output capacity.
[[nodiscard]] PointCloudProjectionResult
projectPointCloud(std::span<const core::PointXYZI> points_lidar,
                  const RigidTransform& T_camera_rect_00_lidar,
                  const RectifiedProjection& projection);

} // namespace sfr::geometry

#endif // SFR_GEOMETRY_POINT_CLOUD_PROJECTION_HPP_
