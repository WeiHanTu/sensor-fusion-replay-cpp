#ifndef SFR_GEOMETRY_CALIBRATION_SENSITIVITY_HPP_
#define SFR_GEOMETRY_CALIBRATION_SENSITIVITY_HPP_

#include <cstdint>
#include <span>

#include "sfr/core/point_types.hpp"
#include "sfr/geometry/rigid_transform.hpp"

namespace sfr::geometry {

struct ProjectionDisplacementMetrics final {
  std::uint64_t common_visible_points;
  std::uint64_t disappeared_points;
  std::uint64_t appeared_points;
  double median_displacement_px;
  double p95_displacement_px;
};

[[nodiscard]] RigidTransform leftApplyCameraYaw(const RigidTransform& T_camera_rect_00_lidar,
                                                double yaw_camera_y_rad);

[[nodiscard]] RigidTransform
leftApplyCameraXTranslation(const RigidTransform& T_camera_rect_00_lidar,
                            double translation_camera_x_m);

[[nodiscard]] ProjectionDisplacementMetrics
compareProjectedPoints(std::span<const core::ProjectedPoint> baseline,
                       std::span<const core::ProjectedPoint> perturbed);

} // namespace sfr::geometry

#endif // SFR_GEOMETRY_CALIBRATION_SENSITIVITY_HPP_
