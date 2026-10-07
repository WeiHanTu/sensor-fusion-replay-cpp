#ifndef SFR_GEOMETRY_RIGID_TRANSFORM_HPP_
#define SFR_GEOMETRY_RIGID_TRANSFORM_HPP_

#include <compare>
#include <cstdint>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/LU>

namespace sfr::geometry {

using Matrix3d = Eigen::Matrix3d;
using Matrix4d = Eigen::Matrix4d;
using Vector3d = Eigen::Vector3d;

/// Owned, non-empty runtime coordinate-frame label.
///
/// Runtime labels support dataset-driven frames; they neither validate physical
/// calibration nor provide compile-time frame compatibility.
class FrameId final {
public:
  /// Takes ownership of `value`.
  /// @throws GeometryError When the frame name is empty.
  explicit FrameId(std::string value);

  /// Returns a borrowed name reference valid for this object's lifetime.
  [[nodiscard]] const std::string& value() const noexcept;

  auto operator<=>(const FrameId&) const = default;

private:
  std::string value_;
};

/// Stable categories for geometry contract failures.
enum class GeometryErrorCode : std::uint8_t {
  kInvalidFrame,
  kNonFiniteValue,
  kInvalidRotation,
  kFrameMismatch,
  kDuplicateEdge,
  kRedundantEdge,
  kInconsistentEdge,
  kInvalidProjection,
};

/// Typed geometry exception with a machine-checkable category.
class GeometryError final : public std::runtime_error {
public:
  /// Copies `message` into `std::runtime_error` and stores `code`.
  GeometryError(GeometryErrorCode code, const std::string& message);

  /// Returns the stored error category.
  [[nodiscard]] GeometryErrorCode code() const noexcept;

private:
  GeometryErrorCode code_;
};

/// Constructor parameters for `T_target_source` with translation in meters.
struct RigidTransformParameters final {
  FrameId target_frame;
  FrameId source_frame;
  Matrix3d rotation_target_source;
  Vector3d translation_target_source_m;
};

/// Owns a validated SE(3) value.
///
/// Applies `p_target = R_target_source * p_source + t_target_source_m`.
class RigidTransform final {
public:
  static constexpr double kRotationTolerance = 1e-6;

  /// Takes parameter values and validates finiteness and rotation tolerance.
  /// @throws GeometryError For non-finite values or an invalid rotation.
  explicit RigidTransform(RigidTransformParameters parameters);

  /// Returns an owned identity transform from `frame` to itself.
  [[nodiscard]] static RigidTransform Identity(const FrameId& frame);

  /// Returns a borrowed target-frame reference valid for this object's lifetime.
  [[nodiscard]] const FrameId& targetFrame() const noexcept;

  /// Returns a borrowed source-frame reference valid for this object's lifetime.
  [[nodiscard]] const FrameId& sourceFrame() const noexcept;

  /// Returns a borrowed `R_target_source` reference valid for this object's lifetime.
  [[nodiscard]] const Matrix3d& rotation() const noexcept;

  /// Returns borrowed `t_target_source` in meters, valid for this object's lifetime.
  [[nodiscard]] const Vector3d& translationMeters() const noexcept;

  /// Returns the owned homogeneous `4x4 T_target_source` matrix value.
  [[nodiscard]] Matrix4d matrix() const;

  /// Returns the owned inverse transform with source/target labels swapped.
  [[nodiscard]] RigidTransform inverse() const;

  /// Transforms one source-frame point in meters and returns an owned value.
  /// @throws GeometryError For any non-finite coordinate.
  [[nodiscard]] Vector3d transformPoint(const Vector3d& point_source_m) const;

  /// Transforms a borrowed span and returns owned points in input order.
  /// @throws GeometryError When any input point has a non-finite coordinate.
  /// Complexity: O(N) time and O(N) output storage.
  [[nodiscard]] std::vector<Vector3d>
  transformPoints(std::span<const Vector3d> points_source_m) const;

private:
  FrameId target_frame_;
  FrameId source_frame_;
  Matrix3d rotation_target_source_;
  Vector3d translation_target_source_m_;
};

/// Composes transforms as `T_target_source = T_target_middle * T_middle_source`.
///
/// @throws GeometryError With `kFrameMismatch` when middle labels differ.
[[nodiscard]] RigidTransform compose(const RigidTransform& T_target_middle,
                                     const RigidTransform& T_middle_source);

} // namespace sfr::geometry

#endif // SFR_GEOMETRY_RIGID_TRANSFORM_HPP_
