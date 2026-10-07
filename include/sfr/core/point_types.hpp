#ifndef SFR_CORE_POINT_TYPES_HPP_
#define SFR_CORE_POINT_TYPES_HPP_

#include <cstdint>

namespace sfr::core {

/// Cartesian point in a caller-defined sensor frame.
///
/// Coordinates are meters by project convention. Loader-produced values have
/// finite, unnormalized reflectance; this aggregate itself does not enforce
/// finiteness or encode a coordinate frame.
struct PointXYZI final {
  double x_m;
  double y_m;
  double z_m;
  float reflectance;
};

/// Visible image projection associated with one point-cloud input position.
///
/// `source_index` indexes the span supplied to the projection operation, not
/// necessarily the original file record after loader filtering. Image
/// coordinates are continuous pixels and depth is camera-forward z in meters.
/// The aggregate itself does not enforce the visible-point invariants.
struct ProjectedPoint final {
  std::uint64_t source_index;
  double u_px;
  double v_px;
  double depth_camera_m;
  float reflectance;
};

} // namespace sfr::core

#endif // SFR_CORE_POINT_TYPES_HPP_
