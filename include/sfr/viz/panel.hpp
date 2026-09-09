#ifndef SFR_VIZ_PANEL_HPP_
#define SFR_VIZ_PANEL_HPP_

#include <span>
#include <string_view>

#include <opencv2/core/mat.hpp>

namespace sfr::viz {

[[nodiscard]] cv::Mat addPanelHeader(const cv::Mat& image_bgr8, std::string_view title,
                                     std::string_view subtitle);

[[nodiscard]] cv::Mat composePanelGrid(std::span<const cv::Mat> panels, int columns);

} // namespace sfr::viz

#endif // SFR_VIZ_PANEL_HPP_
