#include "sfr/geometry/rigid_transform.hpp"

#include <limits>
#include <numbers>
#include <vector>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

namespace sfr::geometry {
namespace {

TEST(RigidTransformTest, IdentityPreservesPointAndFrame) {
  const FrameId lidar("lidar");
  const RigidTransform identity = RigidTransform::Identity(lidar);
  const Vector3d point(1.0, -2.0, 3.0);

  EXPECT_EQ(identity.targetFrame(), lidar);
  EXPECT_EQ(identity.sourceFrame(), lidar);
  EXPECT_TRUE(identity.transformPoint(point).isApprox(point, 0.0));
  EXPECT_TRUE(identity.matrix().isApprox(Matrix4d::Identity(), 0.0));
}

TEST(RigidTransformTest, ComposeUsesTargetMiddleAndMiddleSourceOrder) {
  const RigidTransform T_vehicle_lidar(
      {FrameId("vehicle"), FrameId("lidar"), Matrix3d::Identity(), Vector3d(1.0, 0.0, 0.0)});
  const Matrix3d R_camera_vehicle =
      Eigen::AngleAxisd(std::numbers::pi / 2.0, Vector3d::UnitZ()).toRotationMatrix();
  const RigidTransform T_camera_vehicle(
      {FrameId("camera"), FrameId("vehicle"), R_camera_vehicle, Vector3d(0.0, 2.0, 0.0)});

  const RigidTransform T_camera_lidar = compose(T_camera_vehicle, T_vehicle_lidar);

  EXPECT_EQ(T_camera_lidar.targetFrame(), FrameId("camera"));
  EXPECT_EQ(T_camera_lidar.sourceFrame(), FrameId("lidar"));
  EXPECT_TRUE(T_camera_lidar.transformPoint(Vector3d(1.0, 0.0, 0.0))
                  .isApprox(Vector3d(0.0, 4.0, 0.0), 1e-12));
}

TEST(RigidTransformTest, InverseRoundTripMeetsNumericContract) {
  const Matrix3d rotation =
      Eigen::AngleAxisd(0.3, Vector3d(1.0, 2.0, 3.0).normalized()).toRotationMatrix();
  const RigidTransform T_camera_lidar(
      {FrameId("camera"), FrameId("lidar"), rotation, Vector3d(0.4, -1.2, 2.5)});
  const Vector3d point_lidar(8.0, -3.0, 0.7);

  const Vector3d recovered =
      T_camera_lidar.inverse().transformPoint(T_camera_lidar.transformPoint(point_lidar));

  EXPECT_LE((recovered - point_lidar).norm(), 1e-9);
}

TEST(RigidTransformTest, BatchTransformMatchesIndependentExpectedValues) {
  const RigidTransform T_b_a(
      {FrameId("b"), FrameId("a"), Matrix3d::Identity(), Vector3d(1.0, 2.0, 3.0)});
  const std::vector<Vector3d> input{Vector3d::Zero(), Vector3d(-1.0, -2.0, -3.0)};

  const std::vector<Vector3d> output = T_b_a.transformPoints(input);

  ASSERT_EQ(output.size(), 2U);
  EXPECT_TRUE(output[0].isApprox(Vector3d(1.0, 2.0, 3.0), 0.0));
  EXPECT_TRUE(output[1].isApprox(Vector3d::Zero(), 0.0));
}

TEST(RigidTransformTest, RejectsEmptyFrame) {
  try {
    static_cast<void>(FrameId(""));
    FAIL() << "expected GeometryError";
  } catch (const GeometryError& error) {
    EXPECT_EQ(error.code(), GeometryErrorCode::kInvalidFrame);
  }
}

TEST(RigidTransformTest, RejectsNonFiniteTranslation) {
  const Vector3d translation(0.0, std::numeric_limits<double>::infinity(), 0.0);
  try {
    static_cast<void>(
        RigidTransform({FrameId("b"), FrameId("a"), Matrix3d::Identity(), translation}));
    FAIL() << "expected GeometryError";
  } catch (const GeometryError& error) {
    EXPECT_EQ(error.code(), GeometryErrorCode::kNonFiniteValue);
  }
}

TEST(RigidTransformTest, RejectsInvalidRotation) {
  Matrix3d scaled = Matrix3d::Identity();
  scaled(0, 0) = 2.0;
  try {
    static_cast<void>(RigidTransform({FrameId("b"), FrameId("a"), scaled, Vector3d::Zero()}));
    FAIL() << "expected GeometryError";
  } catch (const GeometryError& error) {
    EXPECT_EQ(error.code(), GeometryErrorCode::kInvalidRotation);
  }
}

TEST(RigidTransformTest, RejectsFrameMismatchDuringComposition) {
  const RigidTransform T_c_b({FrameId("c"), FrameId("b"), Matrix3d::Identity(), Vector3d::Zero()});
  const RigidTransform T_x_a({FrameId("x"), FrameId("a"), Matrix3d::Identity(), Vector3d::Zero()});
  try {
    static_cast<void>(compose(T_c_b, T_x_a));
    FAIL() << "expected GeometryError";
  } catch (const GeometryError& error) {
    EXPECT_EQ(error.code(), GeometryErrorCode::kFrameMismatch);
  }
}

} // namespace
} // namespace sfr::geometry
