#include "sfr/viz/panel.hpp"

#include <vector>

#include <gtest/gtest.h>

#include "sfr/viz/overlay.hpp"

namespace sfr::viz {
namespace {

TEST(PanelTest, AddsHeaderWithoutMutatingImageAndComposesRowMajorGrid) {
  const cv::Mat source(40, 80, CV_8UC3, cv::Scalar(1, 2, 3));
  const cv::Mat labeled = addPanelHeader(source, "baseline", "supplied calibration");
  const std::vector<cv::Mat> panels(9U, labeled);

  const cv::Mat grid = composePanelGrid(panels, 3);

  EXPECT_EQ(source.rows, 40);
  EXPECT_EQ(source.cols, 80);
  EXPECT_EQ(labeled.rows, 96);
  EXPECT_EQ(labeled.cols, 80);
  EXPECT_EQ(grid.rows, 288);
  EXPECT_EQ(grid.cols, 240);
  EXPECT_EQ(labeled.at<cv::Vec3b>(95, 79), cv::Vec3b(1, 2, 3));
}

TEST(PanelTest, RejectsMissingLabelsAndMismatchedPanels) {
  const cv::Mat source(40, 80, CV_8UC3, cv::Scalar(1, 2, 3));
  EXPECT_THROW(static_cast<void>(addPanelHeader(source, "", "subtitle")), VizError);
  EXPECT_THROW(static_cast<void>(composePanelGrid({}, 3)), VizError);

  const std::vector<cv::Mat> mismatched{
      cv::Mat(2, 2, CV_8UC3),
      cv::Mat(3, 2, CV_8UC3),
  };
  EXPECT_THROW(static_cast<void>(composePanelGrid(mismatched, 2)), VizError);
}

} // namespace
} // namespace sfr::viz
