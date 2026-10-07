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

// Runtime labels support dataset-driven frames; they do not validate physical
// calibration or provide compile-time frame compatibility.
class FrameId final {
public:
  explicit FrameId(std::string value);

  [[nodiscard]] const std::string& value() const noexcept;

  auto operator<=>(const FrameId&) const = default;

private:
  std::string value_;
};

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

class GeometryError final : public std::runtime_error {
public:
  GeometryError(GeometryErrorCode code, const std::string& message);

  [[nodiscard]] GeometryErrorCode code() const noexcept;

private:
  GeometryErrorCode code_;
};

struct RigidTransformParameters final {
  FrameId target_frame;
  FrameId source_frame;
  Matrix3d rotation_target_source;
  Vector3d translation_target_source_m;
};

// Owns a validated SE(3) value: p_target = R_target_source * p_source + t_m.
// Construction rejects non-finite values and rotations outside tolerance.
class RigidTransform final {
public:
  static constexpr double kRotationTolerance = 1e-6;

  explicit RigidTransform(RigidTransformParameters parameters);

  [[nodiscard]] static RigidTransform Identity(const FrameId& frame);

  [[nodiscard]] const FrameId& targetFrame() const noexcept;
  [[nodiscard]] const FrameId& sourceFrame() const noexcept;
  [[nodiscard]] const Matrix3d& rotation() const noexcept;
  [[nodiscard]] const Vector3d& translationMeters() const noexcept;
  [[nodiscard]] Matrix4d matrix() const;
  [[nodiscard]] RigidTransform inverse() const;
  // Coordinates are meters in sourceFrame(); no input reference is retained.
  // Non-finite inputs throw GeometryError. The returned point is a value.
  [[nodiscard]] Vector3d transformPoint(const Vector3d& point_source_m) const;
  // Borrows only for this call; owns the returned O(N) vector in input order.
  [[nodiscard]] std::vector<Vector3d>
  transformPoints(std::span<const Vector3d> points_source_m) const;

private:
  FrameId target_frame_;
  FrameId source_frame_;
  Matrix3d rotation_target_source_;
  Vector3d translation_target_source_m_;
};

// Applies T_middle_source first. Mismatched middle labels throw kFrameMismatch.
[[nodiscard]] RigidTransform compose(const RigidTransform& T_target_middle,
                                     const RigidTransform& T_middle_source);

} // namespace sfr::geometry

#endif // SFR_GEOMETRY_RIGID_TRANSFORM_HPP_
