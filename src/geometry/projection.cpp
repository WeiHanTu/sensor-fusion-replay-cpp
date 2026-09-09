#include "sfr/geometry/projection.hpp"

#include <cmath>
#include <utility>

namespace sfr::geometry {

RectifiedProjection::RectifiedProjection(RectifiedProjectionConfig config)
    : P_image_camera_rect_00_(std::move(config.P_image_camera_rect_00)),
      image_width_px_(config.image_width_px), image_height_px_(config.image_height_px),
      z_min_m_(config.z_min_m) {
  if (!P_image_camera_rect_00_.allFinite() || !std::isfinite(z_min_m_)) {
    throw GeometryError(GeometryErrorCode::kNonFiniteValue,
                        "projection configuration contains a non-finite value");
  }
  if (image_width_px_ <= 0 || image_height_px_ <= 0 || z_min_m_ <= 0.0) {
    throw GeometryError(GeometryErrorCode::kInvalidProjection,
                        "projection dimensions and minimum depth must be positive");
  }
}

ProjectionResult RectifiedProjection::project(const Vector3d& point_camera_rect_00_m) const {
  if (!point_camera_rect_00_m.allFinite()) {
    return ProjectionResult{ProjectionStatus::kNonFiniteInput, std::nullopt};
  }
  if (point_camera_rect_00_m.z() <= z_min_m_) {
    return ProjectionResult{ProjectionStatus::kBehindOrTooNear, std::nullopt};
  }

  Eigen::Vector4d homogeneous_point;
  homogeneous_point << point_camera_rect_00_m, 1.0;
  const Eigen::Vector3d homogeneous_pixel = P_image_camera_rect_00_ * homogeneous_point;
  if (!homogeneous_pixel.allFinite() || homogeneous_pixel.z() <= 0.0) {
    return ProjectionResult{ProjectionStatus::kNonPositiveHomogeneousDepth, std::nullopt};
  }

  const double u_px = homogeneous_pixel.x() / homogeneous_pixel.z();
  const double v_px = homogeneous_pixel.y() / homogeneous_pixel.z();
  if (!std::isfinite(u_px) || !std::isfinite(v_px) || u_px < 0.0 || v_px < 0.0 ||
      u_px >= static_cast<double>(image_width_px_) ||
      v_px >= static_cast<double>(image_height_px_)) {
    return ProjectionResult{ProjectionStatus::kOutsideImage, std::nullopt};
  }

  return ProjectionResult{
      ProjectionStatus::kVisible,
      ImageProjection{u_px, v_px, point_camera_rect_00_m.z()},
  };
}

const Matrix34d& RectifiedProjection::matrix() const noexcept { return P_image_camera_rect_00_; }

int RectifiedProjection::imageWidthPixels() const noexcept { return image_width_px_; }

int RectifiedProjection::imageHeightPixels() const noexcept { return image_height_px_; }

double RectifiedProjection::minimumDepthMeters() const noexcept { return z_min_m_; }

} // namespace sfr::geometry
