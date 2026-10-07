#ifndef SFR_GEOMETRY_CALIBRATION_SENSITIVITY_HPP_
#define SFR_GEOMETRY_CALIBRATION_SENSITIVITY_HPP_

#include <cstdint>
#include <span>

#include "sfr/core/point_types.hpp"
#include "sfr/geometry/rigid_transform.hpp"

namespace sfr::geometry {

/// Pixel-displacement statistics over source indices visible in both results.
///
/// Appeared/disappeared counts cover indices present in only one result.
/// Values produced by `compareProjectedPoints` use the exact nearest-rank
/// definition from `spec.md`.
struct ProjectionDisplacementMetrics final {
  std::uint64_t common_visible_points;
  std::uint64_t disappeared_points;
  std::uint64_t appeared_points;
  double median_displacement_px;
  double p95_displacement_px;
};

/// Left-applies a camera +y yaw to `T_camera_rect_00_lidar`.
///
/// @param T_camera_rect_00_lidar Validated transform from `lidar` to
///        `camera_rect_00`; borrowed for this call.
/// @param yaw_camera_y_rad Finite signed angle in radians.
/// @return Owned transform `Delta_camera * T_camera_rect_00_lidar`.
/// @throws GeometryError For wrong frame endpoints or a non-finite angle.
[[nodiscard]] RigidTransform leftApplyCameraYaw(const RigidTransform& T_camera_rect_00_lidar,
                                                double yaw_camera_y_rad);

/// Left-applies a camera +x translation to `T_camera_rect_00_lidar`.
///
/// @param T_camera_rect_00_lidar Validated transform from `lidar` to
///        `camera_rect_00`; borrowed for this call.
/// @param translation_camera_x_m Finite signed displacement in meters.
/// @return Owned transform `Delta_camera * T_camera_rect_00_lidar`.
/// @throws GeometryError For wrong frame endpoints or non-finite displacement.
[[nodiscard]] RigidTransform
leftApplyCameraXTranslation(const RigidTransform& T_camera_rect_00_lidar,
                            double translation_camera_x_m);

/// Compares two visible-projection sets by unique `source_index`.
///
/// Both spans are borrowed for the call. Points must have unique source indices,
/// finite pixels, and finite positive depth. The return value owns only scalar
/// metrics; no input references are retained.
///
/// @throws GeometryError For invalid/duplicate points or an empty common set.
/// Complexity: O((B + P) log(B + P)) time and O(B + P) auxiliary storage.
[[nodiscard]] ProjectionDisplacementMetrics
compareProjectedPoints(std::span<const core::ProjectedPoint> baseline,
                       std::span<const core::ProjectedPoint> perturbed);

} // namespace sfr::geometry

#endif // SFR_GEOMETRY_CALIBRATION_SENSITIVITY_HPP_
