#ifndef SFR_VIZ_OVERLAY_HPP_
#define SFR_VIZ_OVERLAY_HPP_

#include <cstdint>
#include <span>
#include <stdexcept>
#include <string>

#include <opencv2/core/mat.hpp>

#include "sfr/core/point_types.hpp"

namespace sfr::viz {

/// Stable categories for visualization contract failures.
enum class VizErrorCode : std::uint8_t {
  kInvalidConfiguration,
  kInvalidImage,
  kInvalidProjectedPoint,
};

/// Typed visualization exception with a machine-checkable category.
class VizError final : public std::runtime_error {
public:
  /// Copies `message` into `std::runtime_error` and stores `code`.
  VizError(VizErrorCode code, const std::string& message);

  /// Returns the stored error category.
  [[nodiscard]] VizErrorCode code() const noexcept;

private:
  VizErrorCode code_;
};

/// Depth-color range in meters and rendered circle radius in pixels.
struct OverlayConfig final {
  double depth_min_m{1.0};
  double depth_max_m{80.0};
  int point_radius_px{1};
};

/// Owned overlay image and center-pixel z-buffer accounting.
///
/// Results produced by `renderDepthOverlay` count winning projected centers,
/// not every pixel covered by their circles, and satisfy
/// `projected_points = rendered_pixels + occluded_points`.
struct OverlayResult final {
  cv::Mat image_bgr8;
  std::uint64_t projected_points;
  std::uint64_t rendered_pixels;
  std::uint64_t occluded_points;
};

/// Renders projected points onto a clone of a borrowed BGR8 image.
///
/// Projected coordinates must be finite and continuously in bounds with finite
/// positive camera-forward depth in meters. Coordinates are rounded to nearest
/// integer and clamped after the continuous check. At equal center pixels,
/// strictly nearer depth wins; equal depth preserves the first input point.
/// Depth is clamped to the configured range and colored with OpenCV Turbo.
///
/// @throws VizError For invalid configuration, image, or projected point.
/// Complexity: O(W*H + P) time and O(W*H) z-buffer/index storage.
[[nodiscard]] OverlayResult
renderDepthOverlay(const cv::Mat& image_bgr8,
                   std::span<const core::ProjectedPoint> projected_points,
                   const OverlayConfig& config = {});

/// Returns the stable report name for the renderer's OpenCV colormap.
[[nodiscard]] std::string depthColormapName();

} // namespace sfr::viz

#endif // SFR_VIZ_OVERLAY_HPP_
