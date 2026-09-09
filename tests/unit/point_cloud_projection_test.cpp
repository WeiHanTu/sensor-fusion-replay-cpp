#include "sfr/geometry/point_cloud_projection.hpp"

#include <limits>
#include <vector>

#include <gtest/gtest.h>

namespace sfr::geometry {
namespace {

[[nodiscard]] RectifiedProjection makeProjection() {
  Matrix34d matrix;
  matrix << 2.0, 0.0, 4.0, 0.0, 0.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0, 0.0;
  return RectifiedProjection({matrix, 8, 6, 0.1});
}

TEST(PointCloudProjectionTest, AccountsForEveryInputAndPreservesSourceIndex) {
  const RigidTransform transform(
      {FrameId("camera_rect_00"), FrameId("lidar"), Matrix3d::Identity(), Vector3d::Zero()});
  const std::vector<core::PointXYZI> points{
      {1.0, 1.0, 2.0, 0.25F},
      {0.0, 0.0, 0.1, 0.5F},
      {10.0, 0.0, 1.0, 0.75F},
      {std::numeric_limits<double>::quiet_NaN(), 0.0, 1.0, 1.0F},
  };

  const PointCloudProjectionResult result = projectPointCloud(points, transform, makeProjection());

  ASSERT_EQ(result.visible_points.size(), 1U);
  EXPECT_EQ(result.visible_points[0].source_index, 0U);
  EXPECT_DOUBLE_EQ(result.visible_points[0].u_px, 5.0);
  EXPECT_DOUBLE_EQ(result.visible_points[0].v_px, 4.0);
  EXPECT_DOUBLE_EQ(result.visible_points[0].depth_camera_m, 2.0);
  EXPECT_FLOAT_EQ(result.visible_points[0].reflectance, 0.25F);
  EXPECT_EQ(result.counts.input_points, 4U);
  EXPECT_EQ(result.counts.visible_points, 1U);
  EXPECT_EQ(result.counts.non_finite_input, 1U);
  EXPECT_EQ(result.counts.behind_or_too_near, 1U);
  EXPECT_EQ(result.counts.non_positive_homogeneous_depth, 0U);
  EXPECT_EQ(result.counts.outside_image, 1U);
}

TEST(PointCloudProjectionTest, RejectsWrongTransformEndpoints) {
  const RigidTransform wrong_transform(
      {FrameId("camera_raw_00"), FrameId("lidar"), Matrix3d::Identity(), Vector3d::Zero()});
  const std::vector<core::PointXYZI> points{{1.0, 1.0, 2.0, 0.25F}};

  try {
    static_cast<void>(projectPointCloud(points, wrong_transform, makeProjection()));
    FAIL() << "expected GeometryError";
  } catch (const GeometryError& error) {
    EXPECT_EQ(error.code(), GeometryErrorCode::kFrameMismatch);
  }
}

TEST(PointCloudProjectionTest, CountsNonPositiveHomogeneousDepthSeparately) {
  Matrix34d matrix;
  matrix << 2.0, 0.0, 4.0, 0.0, 0.0, 2.0, 3.0, 0.0, 0.0, 0.0, -1.0, 0.0;
  const RectifiedProjection projection({matrix, 8, 6, 0.1});
  const RigidTransform transform(
      {FrameId("camera_rect_00"), FrameId("lidar"), Matrix3d::Identity(), Vector3d::Zero()});
  const std::vector<core::PointXYZI> points{{0.0, 0.0, 2.0, 0.25F}};

  const PointCloudProjectionResult result = projectPointCloud(points, transform, projection);

  EXPECT_TRUE(result.visible_points.empty());
  EXPECT_EQ(result.counts.non_positive_homogeneous_depth, 1U);
}

} // namespace
} // namespace sfr::geometry
