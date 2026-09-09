#include "sfr/geometry/point_cloud_projection.hpp"

#include <cmath>
#include <limits>

namespace sfr::geometry {

namespace {

void checkedIncrement(std::uint64_t& counter) {
  if (counter == std::numeric_limits<std::uint64_t>::max()) {
    throw GeometryError(GeometryErrorCode::kInvalidProjection, "projection counter overflow");
  }
  ++counter;
}

} // namespace

PointCloudProjectionResult projectPointCloud(const std::span<const core::PointXYZI> points_lidar,
                                             const RigidTransform& T_camera_rect_00_lidar,
                                             const RectifiedProjection& projection) {
  if (T_camera_rect_00_lidar.sourceFrame() != FrameId("lidar") ||
      T_camera_rect_00_lidar.targetFrame() != FrameId("camera_rect_00")) {
    throw GeometryError(GeometryErrorCode::kFrameMismatch,
                        "point-cloud projection requires T_camera_rect_00_lidar");
  }
  if (points_lidar.size() > std::numeric_limits<std::uint64_t>::max()) {
    throw GeometryError(GeometryErrorCode::kInvalidProjection,
                        "point-cloud size exceeds uint64 accounting range");
  }

  PointCloudProjectionResult result{
      {},
      ProjectionCounts{static_cast<std::uint64_t>(points_lidar.size()), 0U, 0U, 0U, 0U, 0U},
  };
  result.visible_points.reserve(points_lidar.size());

  for (std::size_t index = 0; index < points_lidar.size(); ++index) {
    const core::PointXYZI& point = points_lidar[index];
    if (!std::isfinite(point.x_m) || !std::isfinite(point.y_m) || !std::isfinite(point.z_m) ||
        !std::isfinite(point.reflectance)) {
      checkedIncrement(result.counts.non_finite_input);
      continue;
    }

    const Vector3d point_camera_rect_00_m =
        T_camera_rect_00_lidar.transformPoint(Vector3d(point.x_m, point.y_m, point.z_m));
    const ProjectionResult projected = projection.project(point_camera_rect_00_m);
    switch (projected.status) {
    case ProjectionStatus::kVisible: {
      const ImageProjection visible = projected.point.value_or(ImageProjection{});
      result.visible_points.push_back(
          core::ProjectedPoint{static_cast<std::uint64_t>(index), visible.u_px, visible.v_px,
                               visible.depth_camera_m, point.reflectance});
      checkedIncrement(result.counts.visible_points);
      break;
    }
    case ProjectionStatus::kNonFiniteInput:
      checkedIncrement(result.counts.non_finite_input);
      break;
    case ProjectionStatus::kBehindOrTooNear:
      checkedIncrement(result.counts.behind_or_too_near);
      break;
    case ProjectionStatus::kNonPositiveHomogeneousDepth:
      checkedIncrement(result.counts.non_positive_homogeneous_depth);
      break;
    case ProjectionStatus::kOutsideImage:
      checkedIncrement(result.counts.outside_image);
      break;
    }
  }
  return result;
}

} // namespace sfr::geometry
