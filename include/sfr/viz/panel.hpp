#ifndef SFR_VIZ_PANEL_HPP_
#define SFR_VIZ_PANEL_HPP_

#include <span>
#include <string_view>

#include <opencv2/core/mat.hpp>

namespace sfr::viz {

/// Returns an owned BGR8 panel with a fixed-height title/subtitle header.
///
/// The source image is borrowed and not mutated.
/// @throws VizError For an empty/non-BGR8 image or empty label.
[[nodiscard]] cv::Mat addPanelHeader(const cv::Mat& image_bgr8, std::string_view title,
                                     std::string_view subtitle);

/// Returns an owned row-major grid of borrowed, equal-sized BGR8 panels.
///
/// Unused cells in the final row retain the dark background.
/// @throws VizError For empty input, non-positive columns, or mismatched panels.
[[nodiscard]] cv::Mat composePanelGrid(std::span<const cv::Mat> panels, int columns);

} // namespace sfr::viz

#endif // SFR_VIZ_PANEL_HPP_
