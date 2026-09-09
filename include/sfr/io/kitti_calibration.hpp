#ifndef SFR_IO_KITTI_CALIBRATION_HPP_
#define SFR_IO_KITTI_CALIBRATION_HPP_

#include <filesystem>

#include "sfr/geometry/projection.hpp"
#include "sfr/geometry/rigid_transform.hpp"

namespace sfr::io {

struct KittiCalibration final {
  geometry::RigidTransform T_camera_raw_00_lidar;
  geometry::RigidTransform T_camera_rect_00_camera_raw_00;
  geometry::Matrix34d P_rect_02;
  int image_width_px;
  int image_height_px;

  [[nodiscard]] geometry::RigidTransform TCameraRect00Lidar() const;
  [[nodiscard]] geometry::RectifiedProjection rectifiedProjection(double z_min_m = 0.1) const;
};

[[nodiscard]] KittiCalibration loadKittiCalibration(const std::filesystem::path& daily_root);

} // namespace sfr::io

#endif // SFR_IO_KITTI_CALIBRATION_HPP_
