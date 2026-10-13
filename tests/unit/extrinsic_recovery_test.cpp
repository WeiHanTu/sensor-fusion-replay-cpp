#include "sfr/geometry/extrinsic_recovery.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <vector>

#include <gtest/gtest.h>

namespace sfr::geometry {
namespace {

[[nodiscard]] RectifiedProjection camera() {
  Matrix34d matrix;
  matrix << 100.0, 0.0, 50.0, 0.0, 0.0, 100.0, 50.0, 0.0, 0.0, 0.0, 1.0, 0.0;
  return RectifiedProjection({matrix, 100, 100, 0.1});
}

[[nodiscard]] RigidTransform nominal() {
  return RigidTransform(
      {FrameId("camera_rect_00"), FrameId("lidar"), Matrix3d::Identity(), Vector3d::Zero()});
}

[[nodiscard]] std::vector<ExtrinsicObservationFrame> observations() {
  return {{9U, {{42U, {0.0, 0.0, 10.0, 0.5F}, {53.0, 54.0}, 1.0}}}};
}

TEST(ExtrinsicRecoveryTest, ComputesHandDerivedWeightedRadialL2AndHuber) {
  auto frames = observations();
  frames[0].observations[0].weight = 2.0;
  const auto l2 =
      evaluateExtrinsicCandidate(nominal(), camera(), frames, {}, {RecoveryLoss::kSquaredL2, 2.0});
  const auto huber =
      evaluateExtrinsicCandidate(nominal(), camera(), frames, {}, {RecoveryLoss::kHuber, 2.0});
  // Radial residual sqrt(3^2+4^2)=5; L2=25, Huber=2*(5-1)=8.
  ASSERT_TRUE(l2.loss_sum.has_value());
  ASSERT_TRUE(huber.loss_sum.has_value());
  EXPECT_DOUBLE_EQ(l2.loss_sum.value(), 50.0);
  EXPECT_DOUBLE_EQ(l2.loss_mean.value(), 25.0);
  EXPECT_DOUBLE_EQ(huber.loss_sum.value(), 16.0);
  EXPECT_DOUBLE_EQ(huber.loss_mean.value(), 8.0);
}

TEST(ExtrinsicRecoveryTest, AppliesYawThenCameraTranslationWithNonIdentityNominal) {
  const RigidTransform shifted(
      {FrameId("camera_rect_00"), FrameId("lidar"), Matrix3d::Identity(), {1.0, 2.0, 3.0}});
  const double yaw = std::numbers::pi / 6.0;
  const auto candidate = applyExtrinsicCorrection(shifted, {yaw, 0.04});
  const Vector3d point = candidate.transformPoint({0.0, 0.0, 10.0});
  EXPECT_NEAR(point.x(), std::cos(yaw) + 13.0 * std::sin(yaw) + 0.04, 1e-12);
  EXPECT_DOUBLE_EQ(point.y(), 2.0);
  EXPECT_NEAR(point.z(), -std::sin(yaw) + 13.0 * std::cos(yaw), 1e-12);
}

TEST(ExtrinsicRecoveryTest, CountsEveryInvisibleCandidatePointWithoutDroppingLossTerms) {
  auto frames = observations();
  frames[0].observations.push_back({7U, {100.0, 0.0, 1.0, 0.5F}, {50.0, 50.0}, 1.0});
  frames[0].observations.push_back({3U, {0.0, 0.0, -1.0, 0.5F}, {50.0, 50.0}, 0.0});
  const auto result = evaluateExtrinsicCandidate(nominal(), camera(), frames, {});
  EXPECT_FALSE(result.loss_sum.has_value());
  EXPECT_EQ(result.projection_counts.input_points, 3U);
  EXPECT_EQ(result.projection_counts.visible_points, 1U);
  EXPECT_EQ(result.projection_counts.outside_image, 1U);
  EXPECT_EQ(result.projection_counts.behind_or_too_near, 1U);
  EXPECT_EQ(result.positive_weight, 2U);
  EXPECT_EQ(result.zero_weight, 1U);
  EXPECT_EQ(result.positive_weight_visible, 1U);
  EXPECT_EQ(result.positive_weight_invalid, 1U);
}

TEST(ExtrinsicRecoveryTest, ZeroWeightMaskIsExplicitAndDoesNotRejectVisibleActiveData) {
  auto frames = observations();
  frames[0].observations.push_back({3U, {0.0, 0.0, -1.0, 0.5F}, {50.0, 50.0}, 0.0});
  const auto result = evaluateExtrinsicCandidate(nominal(), camera(), frames, {});
  ASSERT_TRUE(result.loss_sum.has_value());
  EXPECT_DOUBLE_EQ(result.loss_sum.value(), 8.0);
  EXPECT_EQ(result.zero_weight, 1U);
  EXPECT_EQ(result.projection_counts.behind_or_too_near, 1U);
}

TEST(ExtrinsicRecoveryTest, PreservesKeyBindingsWhenObservationOrderChanges) {
  auto frames = observations();
  frames[0].observations.push_back({7U, {1.0, 0.0, 10.0, 0.5F}, {60.0, 50.0}, 1.0});
  const auto first = evaluateExtrinsicCandidate(nominal(), camera(), frames, {});
  std::ranges::reverse(frames[0].observations);
  const auto second = evaluateExtrinsicCandidate(nominal(), camera(), frames, {});
  EXPECT_EQ(first.loss_sum, second.loss_sum);
  EXPECT_EQ(first.projection_counts.visible_points, second.projection_counts.visible_points);
}

TEST(ExtrinsicRecoveryTest, RejectsDuplicateSourceAndFrameIds) {
  auto frames = observations();
  frames[0].observations.push_back(frames[0].observations.front());
  EXPECT_THROW(validateExtrinsicObservations(frames, camera()), GeometryError);
  frames = observations();
  frames.push_back(frames.front());
  EXPECT_THROW(validateExtrinsicObservations(frames, camera()), GeometryError);
}

TEST(ExtrinsicRecoveryTest, RejectsNonFinitePointsPixelsWeightsAndEmptyActiveSets) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  auto frames = observations();
  frames[0].observations[0].point_lidar.x_m = nan;
  EXPECT_THROW(validateExtrinsicObservations(frames, camera()), GeometryError);
  frames = observations();
  frames[0].observations[0].observed_pixel.u_px = nan;
  EXPECT_THROW(validateExtrinsicObservations(frames, camera()), GeometryError);
  frames = observations();
  frames[0].observations[0].weight = nan;
  EXPECT_THROW(validateExtrinsicObservations(frames, camera()), GeometryError);
  frames[0].observations[0].weight = -1.0;
  EXPECT_THROW(validateExtrinsicObservations(frames, camera()), GeometryError);
  frames[0].observations[0].weight = 0.0;
  EXPECT_THROW(validateExtrinsicObservations(frames, camera()), GeometryError);
  frames[0].observations.clear();
  EXPECT_THROW(validateExtrinsicObservations(frames, camera()), GeometryError);
}

TEST(ExtrinsicRecoveryTest, ReportsLossOverflowWithoutFabricatingFiniteObjective) {
  auto frames = observations();
  frames[0].observations[0].weight = std::numeric_limits<double>::max() / 2.0;
  const auto result = evaluateExtrinsicCandidate(nominal(), camera(), frames, {});
  EXPECT_TRUE(result.numeric_failure);
  EXPECT_FALSE(result.loss_sum.has_value());
  EXPECT_EQ(result.projection_counts.visible_points, 1U);
}

TEST(ExtrinsicRecoveryTest, RejectsInvalidLossAndCorrection) {
  EXPECT_THROW(static_cast<void>(evaluateExtrinsicCandidate(nominal(), camera(), observations(), {},
                                                            {RecoveryLoss::kHuber, 0.0})),
               GeometryError);
  EXPECT_THROW(static_cast<void>(applyExtrinsicCorrection(
                   nominal(), {std::numeric_limits<double>::infinity(), 0.0})),
               GeometryError);
}

} // namespace
} // namespace sfr::geometry
