#ifndef SFR_VIZ_OVERLAY_HPP_
#define SFR_VIZ_OVERLAY_HPP_

#include <cstdint>
#include <span>
#include <stdexcept>
#include <string>

#include <opencv2/core/mat.hpp>

#include "sfr/core/point_types.hpp"

namespace sfr::viz {

enum class VizErrorCode : std::uint8_t {
  kInvalidConfiguration,
  kInvalidImage,
  kInvalidProjectedPoint,
};

class VizError final : public std::runtime_error {
public:
  VizError(VizErrorCode code, const std::string& message);

  [[nodiscard]] VizErrorCode code() const noexcept;

private:
  VizErrorCode code_;
};

struct OverlayConfig final {
  double depth_min_m{1.0};
  double depth_max_m{80.0};
  int point_radius_px{1};
};

struct OverlayResult final {
  cv::Mat image_bgr8;
  std::uint64_t projected_points;
  std::uint64_t rendered_pixels;
  std::uint64_t occluded_points;
};

[[nodiscard]] OverlayResult
renderDepthOverlay(const cv::Mat& image_bgr8,
                   std::span<const core::ProjectedPoint> projected_points,
                   const OverlayConfig& config = {});

[[nodiscard]] std::string depthColormapName();

} // namespace sfr::viz

#endif // SFR_VIZ_OVERLAY_HPP_
