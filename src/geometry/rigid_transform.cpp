#include "sfr/geometry/rigid_transform.hpp"

#include <cmath>
#include <sstream>

namespace sfr::geometry {

FrameId::FrameId(std::string value) : value_(std::move(value)) {
  if (value_.empty()) {
    throw GeometryError(GeometryErrorCode::kInvalidFrame, "frame name must not be empty");
  }
}

const std::string& FrameId::value() const noexcept { return value_; }

GeometryError::GeometryError(GeometryErrorCode code, const std::string& message)
    : std::runtime_error(message), code_(code) {}

GeometryErrorCode GeometryError::code() const noexcept { return code_; }

RigidTransform::RigidTransform(RigidTransformParameters parameters)
    : target_frame_(std::move(parameters.target_frame)),
      source_frame_(std::move(parameters.source_frame)),
      rotation_target_source_(std::move(parameters.rotation_target_source)),
      translation_target_source_m_(std::move(parameters.translation_target_source_m)) {
  if (!rotation_target_source_.allFinite() || !translation_target_source_m_.allFinite()) {
    throw GeometryError(GeometryErrorCode::kNonFiniteValue,
                        "rigid transform contains a non-finite value");
  }

  const double determinant_error = std::abs(rotation_target_source_.determinant() - 1.0);
  const double orthogonality_error =
      (rotation_target_source_ * rotation_target_source_.transpose() - Matrix3d::Identity()).norm();
  if (determinant_error > kRotationTolerance || orthogonality_error > kRotationTolerance) {
    std::ostringstream message;
    message << "invalid rotation: determinant error=" << determinant_error
            << ", orthogonality error=" << orthogonality_error;
    throw GeometryError(GeometryErrorCode::kInvalidRotation, message.str());
  }
}

RigidTransform RigidTransform::Identity(const FrameId& frame) {
  return RigidTransform({frame, frame, Matrix3d::Identity(), Vector3d::Zero()});
}

const FrameId& RigidTransform::targetFrame() const noexcept { return target_frame_; }

const FrameId& RigidTransform::sourceFrame() const noexcept { return source_frame_; }

const Matrix3d& RigidTransform::rotation() const noexcept { return rotation_target_source_; }

const Vector3d& RigidTransform::translationMeters() const noexcept {
  return translation_target_source_m_;
}

Matrix4d RigidTransform::matrix() const {
  Matrix4d homogeneous = Matrix4d::Identity();
  homogeneous.block<3, 3>(0, 0) = rotation_target_source_;
  homogeneous.block<3, 1>(0, 3) = translation_target_source_m_;
  return homogeneous;
}

RigidTransform RigidTransform::inverse() const {
  const Matrix3d inverse_rotation = rotation_target_source_.transpose();
  return RigidTransform({source_frame_, target_frame_, inverse_rotation,
                         -(inverse_rotation * translation_target_source_m_)});
}

Vector3d RigidTransform::transformPoint(const Vector3d& point_source_m) const {
  if (!point_source_m.allFinite()) {
    throw GeometryError(GeometryErrorCode::kNonFiniteValue,
                        "cannot transform a point with a non-finite coordinate");
  }
  return rotation_target_source_ * point_source_m + translation_target_source_m_;
}

std::vector<Vector3d>
RigidTransform::transformPoints(std::span<const Vector3d> points_source_m) const {
  std::vector<Vector3d> transformed;
  transformed.reserve(points_source_m.size());
  for (const Vector3d& point_source_m : points_source_m) {
    transformed.push_back(transformPoint(point_source_m));
  }
  return transformed;
}

RigidTransform compose(const RigidTransform& T_target_middle,
                       const RigidTransform& T_middle_source) {
  if (T_target_middle.sourceFrame() != T_middle_source.targetFrame()) {
    std::ostringstream message;
    message << "cannot compose T_" << T_target_middle.targetFrame().value() << '_'
            << T_target_middle.sourceFrame().value() << " with T_"
            << T_middle_source.targetFrame().value() << '_'
            << T_middle_source.sourceFrame().value();
    throw GeometryError(GeometryErrorCode::kFrameMismatch, message.str());
  }

  const Matrix3d rotation_target_source = T_target_middle.rotation() * T_middle_source.rotation();
  const Vector3d translation_target_source_m =
      T_target_middle.rotation() * T_middle_source.translationMeters() +
      T_target_middle.translationMeters();
  return RigidTransform({T_target_middle.targetFrame(), T_middle_source.sourceFrame(),
                         rotation_target_source, translation_target_source_m});
}

} // namespace sfr::geometry
