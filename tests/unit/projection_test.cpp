#include "sfr/geometry/projection.hpp"

#include <limits>

#include <gtest/gtest.h>

namespace sfr::geometry {
namespace {

[[nodiscard]] ImageProjection missingProjection() {
  const double not_a_number = std::numeric_limits<double>::quiet_NaN();
  return ImageProjection{not_a_number, not_a_number, not_a_number};
}

[[nodiscard]] RectifiedProjection makeProjection() {
  Matrix34d projection;
  projection << 2.0, 0.0, 0.0, 1.0, 0.0, 4.0, 0.0, 2.0, 0.0, 0.0, 1.0, 0.0;
  return RectifiedProjection({projection, 10, 10, 0.1});
}

TEST(ProjectionTest, PreservesFullThreeByFourProjectionMatrix) {
  const RectifiedProjection projector = makeProjection();

  const ProjectionResult result = projector.project(Vector3d(1.0, 2.0, 4.0));

  ASSERT_EQ(result.status, ProjectionStatus::kVisible);
  ASSERT_TRUE(result.point.has_value());
  const ImageProjection point = result.point.value_or(missingProjection());
  EXPECT_DOUBLE_EQ(point.u_px, 0.75);
  EXPECT_DOUBLE_EQ(point.v_px, 2.5);
  EXPECT_DOUBLE_EQ(point.depth_camera_m, 4.0);
}

TEST(ProjectionTest, IncludesContinuousLowerBoundary) {
  Matrix34d projection = Matrix34d::Zero();
  projection(0, 0) = 1.0;
  projection(1, 1) = 1.0;
  projection(2, 2) = 1.0;
  const RectifiedProjection projector({projection, 10, 10});

  const ProjectionResult result = projector.project(Vector3d(0.0, 0.0, 1.0));

  ASSERT_EQ(result.status, ProjectionStatus::kVisible);
  ASSERT_TRUE(result.point.has_value());
  const ImageProjection point = result.point.value_or(missingProjection());
  EXPECT_DOUBLE_EQ(point.u_px, 0.0);
  EXPECT_DOUBLE_EQ(point.v_px, 0.0);
}

TEST(ProjectionTest, RejectsNonFiniteInput) {
  const ProjectionResult result =
      makeProjection().project(Vector3d(std::numeric_limits<double>::quiet_NaN(), 0.0, 1.0));
  EXPECT_EQ(result.status, ProjectionStatus::kNonFiniteInput);
  EXPECT_FALSE(result.point.has_value());
}

TEST(ProjectionTest, RejectsPointAtMinimumDepth) {
  const ProjectionResult result = makeProjection().project(Vector3d(0.0, 0.0, 0.1));
  EXPECT_EQ(result.status, ProjectionStatus::kBehindOrTooNear);
  EXPECT_FALSE(result.point.has_value());
}

TEST(ProjectionTest, RejectsNonPositiveHomogeneousDepth) {
  Matrix34d projection = Matrix34d::Zero();
  projection(0, 0) = 1.0;
  projection(1, 1) = 1.0;
  projection(2, 2) = -1.0;
  const RectifiedProjection projector({projection, 10, 10});

  const ProjectionResult result = projector.project(Vector3d(1.0, 1.0, 2.0));

  EXPECT_EQ(result.status, ProjectionStatus::kNonPositiveHomogeneousDepth);
  EXPECT_FALSE(result.point.has_value());
}

TEST(ProjectionTest, RejectsContinuousUpperBoundary) {
  Matrix34d projection = Matrix34d::Zero();
  projection(0, 0) = 1.0;
  projection(1, 1) = 1.0;
  projection(2, 2) = 1.0;
  const RectifiedProjection projector({projection, 10, 10});

  const ProjectionResult result = projector.project(Vector3d(10.0, 0.0, 1.0));

  EXPECT_EQ(result.status, ProjectionStatus::kOutsideImage);
  EXPECT_FALSE(result.point.has_value());
}

TEST(ProjectionTest, RejectsInvalidConfiguration) {
  try {
    static_cast<void>(RectifiedProjection({Matrix34d::Zero(), 0, 10}));
    FAIL() << "expected GeometryError";
  } catch (const GeometryError& error) {
    EXPECT_EQ(error.code(), GeometryErrorCode::kInvalidProjection);
  }
}

} // namespace
} // namespace sfr::geometry
