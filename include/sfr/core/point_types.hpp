#ifndef SFR_CORE_POINT_TYPES_HPP_
#define SFR_CORE_POINT_TYPES_HPP_

#include <cstdint>

namespace sfr::core {

struct PointXYZI final {
  double x_m;
  double y_m;
  double z_m;
  float reflectance;
};

struct ProjectedPoint final {
  std::uint64_t source_index;
  double u_px;
  double v_px;
  double depth_camera_m;
  float reflectance;
};

} // namespace sfr::core

#endif // SFR_CORE_POINT_TYPES_HPP_
