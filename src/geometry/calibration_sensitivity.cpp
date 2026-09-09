#include "sfr/geometry/calibration_sensitivity.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <string_view>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

namespace sfr::geometry {

namespace {

void validateBaselineFrames(const RigidTransform& T_camera_rect_00_lidar) {
  if (T_camera_rect_00_lidar.targetFrame() != FrameId("camera_rect_00") ||
      T_camera_rect_00_lidar.sourceFrame() != FrameId("lidar")) {
    throw GeometryError(GeometryErrorCode::kFrameMismatch,
                        "calibration perturbation requires T_camera_rect_00_lidar");
  }
}

[[nodiscard]] RigidTransform cameraDelta(const Matrix3d& rotation, const Vector3d& translation) {
  return RigidTransform(
      {FrameId("camera_rect_00"), FrameId("camera_rect_00"), rotation, translation});
}

using ProjectionBySource = std::map<std::uint64_t, const core::ProjectedPoint*>;

[[nodiscard]] ProjectionBySource indexBySource(const std::span<const core::ProjectedPoint> points,
                                               const std::string_view set_name) {
  ProjectionBySource indexed;
  for (const core::ProjectedPoint& point : points) {
    if (!std::isfinite(point.u_px) || !std::isfinite(point.v_px) ||
        !std::isfinite(point.depth_camera_m) || point.depth_camera_m <= 0.0) {
      throw GeometryError(GeometryErrorCode::kInvalidProjection,
                          std::string(set_name) + " contains an invalid projected point");
    }
    if (!indexed.emplace(point.source_index, &point).second) {
      throw GeometryError(GeometryErrorCode::kInvalidProjection,
                          std::string(set_name) + " contains a duplicate source index");
    }
  }
  return indexed;
}

[[nodiscard]] double nearestRank(std::vector<double> sorted_values, const double percentile) {
  std::ranges::sort(sorted_values);
  const double rank = std::ceil(percentile * static_cast<double>(sorted_values.size()));
  const auto one_based_rank = static_cast<std::size_t>(std::max(1.0, rank));
  return sorted_values.at(one_based_rank - 1U);
}

} // namespace

RigidTransform leftApplyCameraYaw(const RigidTransform& T_camera_rect_00_lidar,
                                  const double yaw_camera_y_rad) {
  validateBaselineFrames(T_camera_rect_00_lidar);
  if (!std::isfinite(yaw_camera_y_rad)) {
    throw GeometryError(GeometryErrorCode::kNonFiniteValue,
                        "camera-frame yaw perturbation must be finite");
  }
  const Matrix3d rotation =
      Eigen::AngleAxisd(yaw_camera_y_rad, Vector3d::UnitY()).toRotationMatrix();
  return compose(cameraDelta(rotation, Vector3d::Zero()), T_camera_rect_00_lidar);
}

RigidTransform leftApplyCameraXTranslation(const RigidTransform& T_camera_rect_00_lidar,
                                           const double translation_camera_x_m) {
  validateBaselineFrames(T_camera_rect_00_lidar);
  if (!std::isfinite(translation_camera_x_m)) {
    throw GeometryError(GeometryErrorCode::kNonFiniteValue,
                        "camera-frame x translation perturbation must be finite");
  }
  return compose(cameraDelta(Matrix3d::Identity(), Vector3d(translation_camera_x_m, 0.0, 0.0)),
                 T_camera_rect_00_lidar);
}

ProjectionDisplacementMetrics
compareProjectedPoints(const std::span<const core::ProjectedPoint> baseline,
                       const std::span<const core::ProjectedPoint> perturbed) {
  const ProjectionBySource baseline_by_source = indexBySource(baseline, "baseline");
  const ProjectionBySource perturbed_by_source = indexBySource(perturbed, "perturbed result");
  std::vector<double> displacements;
  displacements.reserve(std::min(baseline.size(), perturbed.size()));

  std::uint64_t disappeared_points = 0U;
  for (const auto& [source_index, baseline_point] : baseline_by_source) {
    const auto match = perturbed_by_source.find(source_index);
    if (match == perturbed_by_source.end()) {
      ++disappeared_points;
      continue;
    }
    displacements.push_back(std::hypot(match->second->u_px - baseline_point->u_px,
                                       match->second->v_px - baseline_point->v_px));
  }

  std::uint64_t appeared_points = 0U;
  for (const auto& [source_index, point] : perturbed_by_source) {
    static_cast<void>(point);
    if (!baseline_by_source.contains(source_index)) {
      ++appeared_points;
    }
  }
  if (displacements.empty()) {
    throw GeometryError(GeometryErrorCode::kInvalidProjection,
                        "calibration sensitivity has an empty common-visible point set");
  }
  if (displacements.size() > std::numeric_limits<std::uint64_t>::max()) {
    throw GeometryError(GeometryErrorCode::kInvalidProjection,
                        "common-visible point count exceeds uint64 range");
  }

  return ProjectionDisplacementMetrics{
      static_cast<std::uint64_t>(displacements.size()), disappeared_points, appeared_points,
      nearestRank(displacements, 0.50), nearestRank(std::move(displacements), 0.95)};
}

} // namespace sfr::geometry
