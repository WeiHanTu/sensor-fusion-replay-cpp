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

// Independent analytic camera projection, not the production transform path.
[[nodiscard]] std::vector<ExtrinsicObservationFrame>
recoveryFrames(const double yaw, const double tx, const bool repeated_axis = false) {
  std::vector<ExtrinsicObservationFrame> frames;
  for (std::uint64_t frame = 0U; frame < 2U; ++frame) {
    ExtrinsicObservationFrame output{frame, {}};
    for (std::uint64_t index = 0U; index < 6U; ++index) {
      const double z = repeated_axis ? 10.0 : 5.0 + 3.0 * static_cast<double>(index + frame);
      const double x = repeated_axis ? 0.0 : (static_cast<double>(index % 3U) - 1.0) * z * 0.2;
      const double y = repeated_axis ? 0.0 : (index % 2U == 0U ? -0.1 : 0.1) * z;
      const double cx = std::cos(yaw) * x + std::sin(yaw) * z + tx;
      const double cz = -std::sin(yaw) * x + std::cos(yaw) * z;
      output.observations.push_back({11U + index * 3U,
                                     {x, y, z, 0.5F},
                                     {50.0 + 100.0 * cx / cz, 50.0 + 100.0 * y / cz},
                                     1.0});
    }
    frames.push_back(std::move(output));
  }
  return frames;
}

TEST(ExtrinsicRecoveryTest, RecoversZeroSignedAxisOnlyAndJointAnalyticTargets) {
  const std::vector<ExtrinsicCorrection> cases{{0.0, 0.0},
                                               {0.7 * std::numbers::pi / 180.0, 0.0},
                                               {-0.7 * std::numbers::pi / 180.0, 0.0},
                                               {0.0, 0.04},
                                               {0.0, -0.04},
                                               {0.7 * std::numbers::pi / 180.0, -0.04},
                                               {-0.6 * std::numbers::pi / 180.0, 0.035}};
  for (const auto& truth : cases) {
    const auto result = searchExtrinsicCorrection(
        nominal(), camera(), recoveryFrames(truth.yaw_camera_y_rad, truth.translation_camera_x_m));
    ASSERT_TRUE(result.best.has_value());
    EXPECT_NEAR(result.best.value().yaw_camera_y_rad, truth.yaw_camera_y_rad,
                0.005 * std::numbers::pi / 180.0 + 1e-12);
    EXPECT_NEAR(result.best.value().translation_camera_x_m, truth.translation_camera_x_m,
                0.00025 + 1e-12);
    EXPECT_EQ(result.status, RecoverySearchStatus::kGridComplete);
    EXPECT_EQ(result.consumed_frame_ids, (std::vector<std::uint64_t>{0U, 1U}));
    EXPECT_EQ(result.points_evaluated, result.candidates.size() * 12U);
    EXPECT_TRUE(result.identifiability.available);
    EXPECT_EQ(result.identifiability.rank, 2U);
    EXPECT_FALSE(result.identifiability.poorly_identifiable);
  }
}

TEST(ExtrinsicRecoveryTest, TieOrderingIsExactStableAndSigned) {
  ExtrinsicCandidateEvaluation a;
  a.loss_sum = 1.0;
  a.correction = {-0.01, -0.02};
  auto b = a;
  b.correction.yaw_camera_y_rad = 0.01;
  EXPECT_TRUE(betterExtrinsicCandidate(a, b));
  b.correction.yaw_camera_y_rad = 0.0;
  EXPECT_TRUE(betterExtrinsicCandidate(b, a));
  b.loss_sum = std::nextafter(1.0, 2.0);
  EXPECT_TRUE(betterExtrinsicCandidate(a, b));
  EXPECT_FALSE(betterExtrinsicCandidate(a, a));
  b.loss_sum.reset();
  EXPECT_TRUE(betterExtrinsicCandidate(a, b));
  EXPECT_FALSE(betterExtrinsicCandidate(b, a));
}

TEST(ExtrinsicRecoveryTest, RejectsInvalidGridDiagnosticThresholdsAndBudgets) {
  const auto frames = recoveryFrames(0.0, 0.0);
  const auto reject = [&](const ExtrinsicSearchConfig& config) {
    EXPECT_THROW(static_cast<void>(searchExtrinsicCorrection(nominal(), camera(), frames, config)),
                 GeometryError);
  };
  ExtrinsicSearchConfig config;
  config.yaw.fine_step = 0.0;
  reject(config);
  config = {};
  config.translation.coarse_step = 0.0031;
  reject(config);
  config = {};
  config.yaw.minimum = config.yaw.maximum;
  reject(config);
  config = {};
  config.translation.fine_step = std::numeric_limits<double>::quiet_NaN();
  reject(config);
  config = {};
  config.max_candidates = 1U;
  reject(config);
  config = {};
  config.max_point_projections = 1U;
  reject(config);
  config = {};
  config.condition_limit = 1.0;
  reject(config);
}

TEST(ExtrinsicRecoveryTest, RetainsEveryRejectedCandidateWhenAllAreInvisible) {
  auto frames = recoveryFrames(0.0, 0.0);
  for (auto& frame : frames) {
    for (auto& observation : frame.observations) {
      observation.point_lidar.z_m = -10.0;
    }
  }
  const auto result = searchExtrinsicCorrection(nominal(), camera(), frames);
  EXPECT_EQ(result.status, RecoverySearchStatus::kNoValidCandidate);
  EXPECT_FALSE(result.best.has_value());
  EXPECT_FALSE(result.identifiability.available);
  EXPECT_EQ(result.candidates.size(), 1681U);
  for (const auto& row : result.candidates) {
    EXPECT_FALSE(row.evaluation.loss_sum.has_value());
    EXPECT_EQ(row.evaluation.positive_weight_invalid, 12U);
  }
}

TEST(ExtrinsicRecoveryTest, ReportsGlobalBoundInsteadOfSuccessfulRecovery) {
  ExtrinsicSearchConfig config;
  config.translation = {-0.02, 0.02, 0.005, 0.0005};
  const auto result =
      searchExtrinsicCorrection(nominal(), camera(), recoveryFrames(0.0, 0.05), config);
  ASSERT_TRUE(result.best.has_value());
  EXPECT_TRUE(result.translation_bound_hit);
  EXPECT_EQ(result.status, RecoverySearchStatus::kBoundLimited);
  EXPECT_TRUE(result.identifiability.used_one_sided_difference);
}

TEST(ExtrinsicRecoveryTest, MatchesReducedExhaustiveFineGridReferenceAndDeduplicates) {
  ExtrinsicSearchConfig config;
  config.yaw = {-std::numbers::pi / 180.0, std::numbers::pi / 180.0, 0.5 * std::numbers::pi / 180.0,
                0.1 * std::numbers::pi / 180.0};
  config.translation = {-0.05, 0.05, 0.025, 0.005};
  config.loss.kind = RecoveryLoss::kSquaredL2;
  const auto frames = recoveryFrames(0.4 * std::numbers::pi / 180.0, 0.015);
  const auto result = searchExtrinsicCorrection(nominal(), camera(), frames, config);
  ExtrinsicCandidateEvaluation exhaustive;
  for (int yaw = 0; yaw <= 20; ++yaw) {
    for (int tx = 0; tx <= 20; ++tx) {
      const auto candidate = evaluateExtrinsicCandidate(
          nominal(), camera(), frames,
          {config.yaw.minimum + yaw * config.yaw.fine_step,
           config.translation.minimum + tx * config.translation.fine_step},
          config.loss);
      if (betterExtrinsicCandidate(candidate, exhaustive)) {
        exhaustive = candidate;
      }
    }
  }
  ASSERT_TRUE(result.best.has_value());
  EXPECT_EQ(result.loss_sum, exhaustive.loss_sum);
  EXPECT_DOUBLE_EQ(result.best.value().yaw_camera_y_rad, exhaustive.correction.yaw_camera_y_rad);
  EXPECT_DOUBLE_EQ(result.best.value().translation_camera_x_m,
                   exhaustive.correction.translation_camera_x_m);
  std::uint64_t shared = 0U;
  for (const auto& row : result.candidates) {
    shared += row.coarse && row.fine ? 1U : 0U;
  }
  EXPECT_GT(shared, 0U);
  EXPECT_LT(result.candidates.size(), 25U + 121U);
}

TEST(ExtrinsicRecoveryTest, RepeatedAxisHasRankOneAndCannotClaimIdentifiability) {
  const auto result =
      searchExtrinsicCorrection(nominal(), camera(), recoveryFrames(0.0, 0.0, true));
  ASSERT_TRUE(result.identifiability.available);
  EXPECT_EQ(result.identifiability.rank, 1U);
  EXPECT_TRUE(result.identifiability.poorly_identifiable);
}

TEST(ExtrinsicRecoveryTest, SearchIsDeterministicAcrossFrameAndObservationOrder) {
  auto frames = recoveryFrames(0.7 * std::numbers::pi / 180.0, -0.04);
  const auto first = searchExtrinsicCorrection(nominal(), camera(), frames);
  std::ranges::reverse(frames);
  std::ranges::reverse(frames.front().observations);
  const auto second = searchExtrinsicCorrection(nominal(), camera(), frames);
  ASSERT_EQ(first.candidates.size(), second.candidates.size());
  EXPECT_EQ(first.loss_sum, second.loss_sum);
  for (std::size_t index = 0; index < first.candidates.size(); ++index) {
    EXPECT_EQ(first.candidates[index].evaluation.loss_sum,
              second.candidates[index].evaluation.loss_sum);
  }
}

TEST(ExtrinsicRecoveryTest, EvaluationUsesUnweightedRetainedRecordsAndNearestRank) {
  auto frames = observations();
  frames[0].observations[0].weight = 0.0;
  frames[0].observations.push_back({7U, {0.0, 0.0, 10.0, 0.5F}, {50.0, 50.0}, 1.0});
  const auto result =
      compareExtrinsicCorrections(nominal(), camera(), frames, ExtrinsicCorrection{}, {});
  EXPECT_EQ(result.nominal.pooled.counts.visible_points, 2U);
  EXPECT_EQ(result.nominal.pooled.zero_weight, 1U);
  EXPECT_EQ(result.nominal.pooled.common_visible, 2U);
  EXPECT_DOUBLE_EQ(result.nominal.pooled.median_px.value(), 0.0);
  EXPECT_DOUBLE_EQ(result.nominal.pooled.p95_px.value(), 5.0);
  EXPECT_NEAR(result.nominal.pooled.rmse_px.value(), std::sqrt(12.5), 1e-12);
  EXPECT_EQ(result.points_evaluated, 6U);
}

TEST(ExtrinsicRecoveryTest, EmptyVisibleMetricsAreNullAndReasonsRemainExplicit) {
  auto frames = observations();
  frames[0].observations[0].point_lidar.z_m = -10.0;
  const auto result = compareExtrinsicCorrections(nominal(), camera(), frames, std::nullopt, {});
  EXPECT_FALSE(result.nominal.pooled.median_px.has_value());
  EXPECT_FALSE(result.nominal.pooled.rmse_px.has_value());
  EXPECT_FALSE(result.recovered.has_value());
  EXPECT_EQ(result.nominal.pooled.counts.behind_or_too_near, 1U);
  EXPECT_EQ(result.nominal.residuals[0].status, ProjectionStatus::kBehindOrTooNear);
  EXPECT_FALSE(result.nominal.residuals[0].predicted.has_value());
  EXPECT_EQ(result.points_evaluated, 4U);
}

TEST(ExtrinsicRecoveryTest, EvaluationCannotAlterFrozenSearchAndHeldOutIsNotConsumed) {
  const auto training = recoveryFrames(0.7 * std::numbers::pi / 180.0, -0.04);
  const auto result = searchExtrinsicCorrection(nominal(), camera(), training);
  auto held_out = recoveryFrames(0.0, 0.0);
  held_out.resize(1U);
  held_out.front().frame_id = 2U;
  const auto saved_loss = result.loss_sum;
  const auto evaluation =
      compareExtrinsicCorrections(nominal(), camera(), held_out, result.best, {});
  EXPECT_EQ(result.loss_sum, saved_loss);
  EXPECT_EQ(result.consumed_frame_ids, (std::vector<std::uint64_t>{0U, 1U}));
  EXPECT_EQ(evaluation.nominal.frames.front().frame_id, 2U);
}

TEST(ExtrinsicRecoveryTest, HuberImprovesCleanResidualsOverL2OnAuthoredOutliers) {
  const double yaw = 0.7 * std::numbers::pi / 180.0;
  const auto clean = recoveryFrames(yaw, -0.04);
  auto corrupted = clean;
  for (auto& frame : corrupted) {
    frame.observations.front().observed_pixel.u_px += 20.0;
    frame.observations.front().observed_pixel.v_px += 10.0;
  }
  ExtrinsicSearchConfig l2_config;
  l2_config.loss.kind = RecoveryLoss::kSquaredL2;
  const auto l2 = searchExtrinsicCorrection(nominal(), camera(), corrupted, l2_config);
  const auto huber = searchExtrinsicCorrection(nominal(), camera(), corrupted);
  const auto l2_eval =
      compareExtrinsicCorrections(nominal(), camera(), clean, l2.best, {yaw, -0.04});
  const auto huber_eval =
      compareExtrinsicCorrections(nominal(), camera(), clean, huber.best, {yaw, -0.04});
  ASSERT_TRUE(l2_eval.recovered.has_value());
  ASSERT_TRUE(huber_eval.recovered.has_value());
  EXPECT_LT(huber_eval.recovered.value().pooled.rmse_px.value(),
            l2_eval.recovered.value().pooled.rmse_px.value());
}

} // namespace
} // namespace sfr::geometry
