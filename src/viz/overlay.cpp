#include "sfr/viz/overlay.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace sfr::viz {

VizError::VizError(VizErrorCode code, const std::string& message)
    : std::runtime_error(message), code_(code) {}

VizErrorCode VizError::code() const noexcept { return code_; }

std::string depthColormapName() { return "OpenCV COLORMAP_TURBO"; }

OverlayResult renderDepthOverlay(const cv::Mat& image_bgr8,
                                 const std::span<const core::ProjectedPoint> projected_points,
                                 const OverlayConfig& config) {
  if (!std::isfinite(config.depth_min_m) || !std::isfinite(config.depth_max_m) ||
      config.depth_min_m < 0.0 || config.depth_max_m <= config.depth_min_m ||
      config.point_radius_px < 0 || config.point_radius_px > 20) {
    throw VizError(VizErrorCode::kInvalidConfiguration,
                   "overlay requires a finite increasing depth range and radius in [0, 20]");
  }
  if (image_bgr8.empty() || image_bgr8.type() != CV_8UC3) {
    throw VizError(VizErrorCode::kInvalidImage, "overlay input must be a nonempty BGR8 image");
  }

  cv::Mat depth_indices(1, 256, CV_8UC1);
  for (int index = 0; index < depth_indices.cols; ++index) {
    depth_indices.at<std::uint8_t>(0, index) = static_cast<std::uint8_t>(index);
  }
  cv::Mat color_table;
  cv::applyColorMap(depth_indices, color_table, cv::COLORMAP_TURBO);

  const auto raster_size =
      static_cast<std::size_t>(image_bgr8.rows) * static_cast<std::size_t>(image_bgr8.cols);
  std::vector<double> z_buffer(raster_size, std::numeric_limits<double>::infinity());
  constexpr std::size_t kNoWinner = std::numeric_limits<std::size_t>::max();
  std::vector<std::size_t> winner_indices(raster_size, kNoWinner);

  for (std::size_t point_index = 0; point_index < projected_points.size(); ++point_index) {
    const core::ProjectedPoint& point = projected_points[point_index];
    if (!std::isfinite(point.u_px) || !std::isfinite(point.v_px) ||
        !std::isfinite(point.depth_camera_m) || point.depth_camera_m <= 0.0 || point.u_px < 0.0 ||
        point.v_px < 0.0 || point.u_px >= static_cast<double>(image_bgr8.cols) ||
        point.v_px >= static_cast<double>(image_bgr8.rows)) {
      throw VizError(VizErrorCode::kInvalidProjectedPoint,
                     "projected point violates the finite, depth, or image-bounds contract");
    }
    const int column =
        std::clamp(static_cast<int>(std::lround(point.u_px)), 0, image_bgr8.cols - 1);
    const int row = std::clamp(static_cast<int>(std::lround(point.v_px)), 0, image_bgr8.rows - 1);
    const auto raster_index =
        static_cast<std::size_t>(row) * static_cast<std::size_t>(image_bgr8.cols) +
        static_cast<std::size_t>(column);
    if (point.depth_camera_m < z_buffer[raster_index]) {
      z_buffer[raster_index] = point.depth_camera_m;
      winner_indices[raster_index] = point_index;
    }
  }

  cv::Mat overlay = image_bgr8.clone();
  std::uint64_t rendered_pixels = 0U;
  for (std::size_t raster_index = 0; raster_index < winner_indices.size(); ++raster_index) {
    const std::size_t point_index = winner_indices[raster_index];
    if (point_index == kNoWinner) {
      continue;
    }
    const core::ProjectedPoint& point = projected_points[point_index];
    const double normalized = std::clamp((point.depth_camera_m - config.depth_min_m) /
                                             (config.depth_max_m - config.depth_min_m),
                                         0.0, 1.0);
    const int color_index = static_cast<int>(std::lround(normalized * 255.0));
    const cv::Vec3b color = color_table.at<cv::Vec3b>(0, color_index);
    const int row = static_cast<int>(raster_index / static_cast<std::size_t>(image_bgr8.cols));
    const int column = static_cast<int>(raster_index % static_cast<std::size_t>(image_bgr8.cols));
    cv::circle(overlay, cv::Point(column, row), config.point_radius_px,
               cv::Scalar(color[0], color[1], color[2]), cv::FILLED, cv::LINE_8);
    ++rendered_pixels;
  }

  const auto projected_count = static_cast<std::uint64_t>(projected_points.size());
  return OverlayResult{std::move(overlay), projected_count, rendered_pixels,
                       projected_count - rendered_pixels};
}

} // namespace sfr::viz
