#include "sfr/geometry/calibration_sensitivity.hpp"

#include <cmath>
#include <numbers>
#include <vector>

#include <gtest/gtest.h>

#include "sfr/geometry/point_cloud_projection.hpp"

namespace sfr::geometry {
namespace {

[[nodiscard]] RectifiedProjection makeCamera() {
  Matrix34d matrix;
  matrix << 100.0, 0.0, 50.0, 0.0, 0.0, 100.0, 50.0, 0.0, 0.0, 0.0, 1.0, 0.0;
  return RectifiedProjection({matrix, 100, 100, 0.1});
}

[[nodiscard]] RigidTransform makeBaseline() {
  return RigidTransform(
      {FrameId("camera_rect_00"), FrameId("lidar"), Matrix3d::Identity(), Vector3d::Zero()});
}

TEST(CalibrationSensitivityTest, PositiveCameraYawAndTranslationMovePixelRight) {
  const std::vector<core::PointXYZI> point{{0.0, 0.0, 10.0, 0.5F}};
  const PointCloudProjectionResult baseline =
      projectPointCloud(point, makeBaseline(), makeCamera());
  const double yaw_rad = std::numbers::pi / 180.0;
  const PointCloudProjectionResult yaw =
      projectPointCloud(point, leftApplyCameraYaw(makeBaseline(), yaw_rad), makeCamera());
  const PointCloudProjectionResult translation =
      projectPointCloud(point, leftApplyCameraXTranslation(makeBaseline(), 0.05), makeCamera());

  ASSERT_EQ(baseline.visible_points.size(), 1U);
  ASSERT_EQ(yaw.visible_points.size(), 1U);
  ASSERT_EQ(translation.visible_points.size(), 1U);
  EXPECT_NEAR(yaw.visible_points[0].u_px - baseline.visible_points[0].u_px,
              100.0 * std::tan(yaw_rad), 1e-12);
  EXPECT_NEAR(translation.visible_points[0].u_px - baseline.visible_points[0].u_px, 0.5, 1e-12);
  EXPECT_DOUBLE_EQ(yaw.visible_points[0].v_px, baseline.visible_points[0].v_px);
}

TEST(CalibrationSensitivityTest, ComputesCommonSetNearestRankAndVisibilityChanges) {
  const std::vector<core::ProjectedPoint> baseline{
      {0U, 0.0, 0.0, 10.0, 0.0F},
      {1U, 5.0, 5.0, 10.0, 0.0F},
      {2U, 2.0, 2.0, 10.0, 0.0F},
  };
  const std::vector<core::ProjectedPoint> perturbed{
      {0U, 3.0, 4.0, 10.0, 0.0F},
      {1U, 11.0, 13.0, 10.0, 0.0F},
      {3U, 1.0, 1.0, 10.0, 0.0F},
  };

  const ProjectionDisplacementMetrics metrics = compareProjectedPoints(baseline, perturbed);

  EXPECT_EQ(metrics.common_visible_points, 2U);
  EXPECT_EQ(metrics.disappeared_points, 1U);
  EXPECT_EQ(metrics.appeared_points, 1U);
  EXPECT_DOUBLE_EQ(metrics.median_displacement_px, 5.0);
  EXPECT_DOUBLE_EQ(metrics.p95_displacement_px, 10.0);
}

TEST(CalibrationSensitivityTest, RejectsEmptyCommonSetDuplicateSourceAndWrongFrames) {
  const std::vector<core::ProjectedPoint> first{{0U, 0.0, 0.0, 1.0, 0.0F}};
  const std::vector<core::ProjectedPoint> second{{1U, 0.0, 0.0, 1.0, 0.0F}};
  const std::vector<core::ProjectedPoint> duplicate{{0U, 0.0, 0.0, 1.0, 0.0F},
                                                    {0U, 1.0, 1.0, 1.0, 0.0F}};

  EXPECT_THROW(static_cast<void>(compareProjectedPoints(first, second)), GeometryError);
  EXPECT_THROW(static_cast<void>(compareProjectedPoints(duplicate, first)), GeometryError);

  const RigidTransform wrong(
      {FrameId("camera_raw_00"), FrameId("lidar"), Matrix3d::Identity(), Vector3d::Zero()});
  EXPECT_THROW(static_cast<void>(leftApplyCameraYaw(wrong, 0.1)), GeometryError);
}

} // namespace
} // namespace sfr::geometry
