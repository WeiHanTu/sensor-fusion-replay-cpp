#include "sfr/geometry/frame_graph.hpp"

#include <gtest/gtest.h>

namespace sfr::geometry {
namespace {

TEST(FrameGraphTest, LooksUpForwardAndInverseComposedPaths) {
  FrameGraph graph;
  graph.addTransform(RigidTransform(
      {FrameId("vehicle"), FrameId("lidar"), Matrix3d::Identity(), Vector3d(1.0, 0.0, 0.0)}));
  graph.addTransform(RigidTransform(
      {FrameId("camera"), FrameId("vehicle"), Matrix3d::Identity(), Vector3d(0.0, 2.0, 0.0)}));

  const std::optional<RigidTransform> T_camera_lidar =
      graph.lookup(FrameId("camera"), FrameId("lidar"));
  ASSERT_TRUE(T_camera_lidar.has_value());
  const RigidTransform camera_lidar_value =
      T_camera_lidar.value_or(RigidTransform::Identity(FrameId("missing_camera_lidar")));
  EXPECT_TRUE(camera_lidar_value.translationMeters().isApprox(Vector3d(1.0, 2.0, 0.0), 0.0));

  const std::optional<RigidTransform> T_lidar_camera =
      graph.lookup(FrameId("lidar"), FrameId("camera"));
  ASSERT_TRUE(T_lidar_camera.has_value());
  const RigidTransform lidar_camera_value =
      T_lidar_camera.value_or(RigidTransform::Identity(FrameId("missing_lidar_camera")));
  EXPECT_TRUE(lidar_camera_value.translationMeters().isApprox(Vector3d(-1.0, -2.0, 0.0), 0.0));
}

TEST(FrameGraphTest, MissingPathIsExpectedAbsence) {
  FrameGraph graph;
  graph.addTransform(RigidTransform(
      {FrameId("camera"), FrameId("lidar"), Matrix3d::Identity(), Vector3d::Zero()}));

  EXPECT_FALSE(graph.lookup(FrameId("world"), FrameId("lidar")).has_value());
}

TEST(FrameGraphTest, RejectsDuplicateDirectedEdge) {
  FrameGraph graph;
  const RigidTransform T_camera_lidar(
      {FrameId("camera"), FrameId("lidar"), Matrix3d::Identity(), Vector3d::Zero()});
  graph.addTransform(T_camera_lidar);

  try {
    graph.addTransform(T_camera_lidar);
    FAIL() << "expected GeometryError";
  } catch (const GeometryError& error) {
    EXPECT_EQ(error.code(), GeometryErrorCode::kDuplicateEdge);
  }
}

TEST(FrameGraphTest, RejectsRedundantConsistentPath) {
  FrameGraph graph;
  graph.addTransform(
      RigidTransform({FrameId("b"), FrameId("a"), Matrix3d::Identity(), Vector3d(1.0, 0.0, 0.0)}));
  graph.addTransform(
      RigidTransform({FrameId("c"), FrameId("b"), Matrix3d::Identity(), Vector3d(0.0, 2.0, 0.0)}));

  try {
    graph.addTransform(RigidTransform(
        {FrameId("c"), FrameId("a"), Matrix3d::Identity(), Vector3d(1.0, 2.0, 0.0)}));
    FAIL() << "expected GeometryError";
  } catch (const GeometryError& error) {
    EXPECT_EQ(error.code(), GeometryErrorCode::kRedundantEdge);
  }
}

TEST(FrameGraphTest, RejectsInconsistentCycle) {
  FrameGraph graph;
  graph.addTransform(
      RigidTransform({FrameId("b"), FrameId("a"), Matrix3d::Identity(), Vector3d(1.0, 0.0, 0.0)}));
  graph.addTransform(
      RigidTransform({FrameId("c"), FrameId("b"), Matrix3d::Identity(), Vector3d(0.0, 2.0, 0.0)}));

  try {
    graph.addTransform(RigidTransform(
        {FrameId("c"), FrameId("a"), Matrix3d::Identity(), Vector3d(9.0, 9.0, 9.0)}));
    FAIL() << "expected GeometryError";
  } catch (const GeometryError& error) {
    EXPECT_EQ(error.code(), GeometryErrorCode::kInconsistentEdge);
  }
}

} // namespace
} // namespace sfr::geometry
