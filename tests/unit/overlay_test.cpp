#include "sfr/viz/overlay.hpp"

#include <limits>
#include <vector>

#include <gtest/gtest.h>

namespace sfr::viz {
namespace {

TEST(OverlayTest, UsesNearestDepthAndClampsRoundedUpperBoundary) {
  const cv::Mat image(6, 8, CV_8UC3, cv::Scalar(0, 0, 0));
  const std::vector<core::ProjectedPoint> points{
      {0U, 2.2, 1.8, 8.0, 0.0F},
      {1U, 2.4, 2.1, 4.0, 0.0F},
      {2U, 7.8, 5.8, 6.0, 0.0F},
  };
  const std::vector<core::ProjectedPoint> near_only{{1U, 2.4, 2.1, 4.0, 0.0F}};
  const OverlayConfig config{.depth_min_m = 1.0, .depth_max_m = 10.0, .point_radius_px = 0};

  const OverlayResult result = renderDepthOverlay(image, points, config);
  const OverlayResult expected_near = renderDepthOverlay(image, near_only, config);

  EXPECT_EQ(result.projected_points, 3U);
  EXPECT_EQ(result.rendered_pixels, 2U);
  EXPECT_EQ(result.occluded_points, 1U);
  EXPECT_EQ(result.image_bgr8.at<cv::Vec3b>(2, 2), expected_near.image_bgr8.at<cv::Vec3b>(2, 2));
  EXPECT_NE(result.image_bgr8.at<cv::Vec3b>(5, 7), cv::Vec3b(0, 0, 0));
}

TEST(OverlayTest, RejectsInvalidImageConfigurationAndPoints) {
  const cv::Mat image(6, 8, CV_8UC3, cv::Scalar(0, 0, 0));
  const std::vector<core::ProjectedPoint> invalid_point{
      {0U, std::numeric_limits<double>::quiet_NaN(), 1.0, 2.0, 0.0F}};

  EXPECT_THROW(static_cast<void>(renderDepthOverlay(cv::Mat{}, {}, {})), VizError);
  EXPECT_THROW(static_cast<void>(renderDepthOverlay(
                   image, {}, {.depth_min_m = 10.0, .depth_max_m = 1.0, .point_radius_px = 1})),
               VizError);
  EXPECT_THROW(static_cast<void>(renderDepthOverlay(image, invalid_point, {})), VizError);
}

} // namespace
} // namespace sfr::viz
