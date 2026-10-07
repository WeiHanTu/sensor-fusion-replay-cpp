#ifndef SFR_GEOMETRY_PROJECTION_HPP_
#define SFR_GEOMETRY_PROJECTION_HPP_

#include <cstdint>
#include <optional>

#include <Eigen/Core>

#include "sfr/geometry/rigid_transform.hpp"

namespace sfr::geometry {

using Matrix34d = Eigen::Matrix<double, 3, 4>;

/// Terminal classification for one camera-frame point.
enum class ProjectionStatus : std::uint8_t {
  kVisible,
  kNonFiniteInput,
  kBehindOrTooNear,
  kNonPositiveHomogeneousDepth,
  kOutsideImage,
};

/// Continuous image coordinates and positive camera-forward depth.
struct ImageProjection final {
  double u_px;
  double v_px;
  double depth_camera_m;
};

/// Immutable status/payload pair for one projected point.
struct ProjectionResult final {
  /// Constructs a validated result.
  ///
  /// `kVisible` requires a payload; every rejection status requires absence.
  /// @throws GeometryError For an unknown status or status/payload disagreement.
  ProjectionResult(ProjectionStatus status_value, std::optional<ImageProjection> point_value);

  const ProjectionStatus status;
  const std::optional<ImageProjection> point;
};

/// Full rectified projection matrix and image/depth validity bounds.
struct RectifiedProjectionConfig final {
  Matrix34d P_image_camera_rect_00;
  int image_width_px;
  int image_height_px;
  double z_min_m{0.1};
};

/// Owns a validated full `3x4` rectified-camera projection configuration.
class RectifiedProjection final {
public:
  /// Copies the full matrix and validates finite values and positive bounds.
  /// @throws GeometryError For non-finite or non-positive configuration.
  explicit RectifiedProjection(RectifiedProjectionConfig config);

  /// Projects one borrowed `camera_rect_00` point in meters.
  ///
  /// Returns an owned status/payload value. Expected rejection uses status and
  /// absence rather than exceptions. Camera z and homogeneous q.z are distinct
  /// checks; image bounds are continuous `[0,width) x [0,height)` before raster
  /// rounding.
  [[nodiscard]] ProjectionResult project(const Vector3d& point_camera_rect_00_m) const;

  /// Returns a borrowed matrix reference valid for this object's lifetime.
  [[nodiscard]] const Matrix34d& matrix() const noexcept;

  /// Returns the configured image width in pixels.
  [[nodiscard]] int imageWidthPixels() const noexcept;

  /// Returns the configured image height in pixels.
  [[nodiscard]] int imageHeightPixels() const noexcept;

  /// Returns the exclusive minimum camera-forward depth in meters.
  [[nodiscard]] double minimumDepthMeters() const noexcept;

private:
  Matrix34d P_image_camera_rect_00_;
  int image_width_px_;
  int image_height_px_;
  double z_min_m_;
};

} // namespace sfr::geometry

#endif // SFR_GEOMETRY_PROJECTION_HPP_
