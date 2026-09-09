#include "sfr/viz/panel.hpp"

#include <algorithm>
#include <string>

#include <opencv2/imgproc.hpp>

#include "sfr/viz/overlay.hpp"

namespace sfr::viz {

cv::Mat addPanelHeader(const cv::Mat& image_bgr8, const std::string_view title,
                       const std::string_view subtitle) {
  if (image_bgr8.empty() || image_bgr8.type() != CV_8UC3 || title.empty() || subtitle.empty()) {
    throw VizError(VizErrorCode::kInvalidImage,
                   "panel requires a nonempty BGR8 image, title, and subtitle");
  }
  constexpr int kHeaderHeightPixels = 56;
  cv::Mat panel(image_bgr8.rows + kHeaderHeightPixels, image_bgr8.cols, CV_8UC3,
                cv::Scalar(24, 24, 24));
  image_bgr8.copyTo(panel(cv::Rect(0, kHeaderHeightPixels, image_bgr8.cols, image_bgr8.rows)));
  const double font_scale = std::clamp(static_cast<double>(image_bgr8.cols) / 640.0, 0.35, 0.7);
  cv::putText(panel, std::string(title), cv::Point(8, 21), cv::FONT_HERSHEY_SIMPLEX, font_scale,
              cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
  cv::putText(panel, std::string(subtitle), cv::Point(8, 44), cv::FONT_HERSHEY_SIMPLEX,
              font_scale * 0.85, cv::Scalar(190, 190, 190), 1, cv::LINE_AA);
  return panel;
}

cv::Mat composePanelGrid(const std::span<const cv::Mat> panels, const int columns) {
  if (panels.empty() || columns <= 0) {
    throw VizError(VizErrorCode::kInvalidConfiguration,
                   "panel grid requires at least one panel and a positive column count");
  }
  const cv::Mat& first = panels.front();
  if (first.empty() || first.type() != CV_8UC3) {
    throw VizError(VizErrorCode::kInvalidImage, "panel grid input must be nonempty BGR8");
  }
  for (const cv::Mat& panel : panels) {
    if (panel.empty() || panel.type() != CV_8UC3 || panel.size() != first.size()) {
      throw VizError(VizErrorCode::kInvalidImage,
                     "all panel grid inputs must have identical BGR8 dimensions");
    }
  }

  const int rows = static_cast<int>((panels.size() + static_cast<std::size_t>(columns) - 1U) /
                                    static_cast<std::size_t>(columns));
  cv::Mat grid(rows * first.rows, columns * first.cols, CV_8UC3, cv::Scalar(12, 12, 12));
  for (std::size_t index = 0; index < panels.size(); ++index) {
    const int grid_row = static_cast<int>(index / static_cast<std::size_t>(columns));
    const int grid_column = static_cast<int>(index % static_cast<std::size_t>(columns));
    panels[index].copyTo(
        grid(cv::Rect(grid_column * first.cols, grid_row * first.rows, first.cols, first.rows)));
  }
  return grid;
}

} // namespace sfr::viz
