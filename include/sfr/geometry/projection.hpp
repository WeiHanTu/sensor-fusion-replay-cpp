#ifndef SFR_GEOMETRY_PROJECTION_HPP_
#define SFR_GEOMETRY_PROJECTION_HPP_

#include <cstdint>
#include <optional>

#include <Eigen/Core>

#include "sfr/geometry/rigid_transform.hpp"

namespace sfr::geometry {

using Matrix34d = Eigen::Matrix<double, 3, 4>;

enum class ProjectionStatus : std::uint8_t {
  kVisible,
  kNonFiniteInput,
  kBehindOrTooNear,
  kNonPositiveHomogeneousDepth,
  kOutsideImage,
};

struct ImageProjection final {
  double u_px;
  double v_px;
  double depth_camera_m;
};

struct ProjectionResult final {
  ProjectionStatus status;
  std::optional<ImageProjection> point;
};

struct RectifiedProjectionConfig final {
  Matrix34d P_image_camera_rect_00;
  int image_width_px;
  int image_height_px;
  double z_min_m{0.1};
};

class RectifiedProjection final {
public:
  explicit RectifiedProjection(RectifiedProjectionConfig config);

  [[nodiscard]] ProjectionResult project(const Vector3d& point_camera_rect_00_m) const;
  [[nodiscard]] const Matrix34d& matrix() const noexcept;
  [[nodiscard]] int imageWidthPixels() const noexcept;
  [[nodiscard]] int imageHeightPixels() const noexcept;
  [[nodiscard]] double minimumDepthMeters() const noexcept;

private:
  Matrix34d P_image_camera_rect_00_;
  int image_width_px_;
  int image_height_px_;
  double z_min_m_;
};

} // namespace sfr::geometry

#endif // SFR_GEOMETRY_PROJECTION_HPP_
