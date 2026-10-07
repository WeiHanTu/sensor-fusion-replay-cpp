#ifndef SFR_IO_KITTI_CALIBRATION_HPP_
#define SFR_IO_KITTI_CALIBRATION_HPP_

#include <filesystem>

#include "sfr/geometry/projection.hpp"
#include "sfr/geometry/rigid_transform.hpp"

namespace sfr::io {

/// Owned KITTI camera/Velodyne calibration required by the v0.1 pipeline.
///
/// Transform names follow `T_target_source`; dimensions describe rectified
/// `image_02`, and `P_rect_02` retains the complete `3x4` matrix.
struct KittiCalibration final {
  geometry::RigidTransform T_camera_raw_00_lidar;
  geometry::RigidTransform T_camera_rect_00_camera_raw_00;
  geometry::Matrix34d P_rect_02;
  int image_width_px;
  int image_height_px;

  /// Composes and returns owned `T_camera_rect_00_lidar`.
  /// @throws GeometryError If stored frame endpoints are inconsistent.
  [[nodiscard]] geometry::RigidTransform TCameraRect00Lidar() const;

  /// Returns an owned rectified projection using the stored matrix/dimensions.
  /// @param z_min_m Exclusive positive camera-depth threshold in meters.
  /// @throws GeometryError For invalid stored projection data or `z_min_m`.
  [[nodiscard]] geometry::RectifiedProjection rectifiedProjection(double z_min_m = 0.1) const;
};

/// Strictly loads daily camera-to-camera and Velodyne-to-camera calibration.
///
/// Reads `calib_cam_to_cam.txt` and `calib_velo_to_cam.txt` below the borrowed
/// path. Required keys must appear exactly once with finite values and exact
/// element counts. The returned value owns all parsed calibration data.
///
/// @throws IoError For file, syntax, key, dimension, or rotation failures.
[[nodiscard]] KittiCalibration loadKittiCalibration(const std::filesystem::path& daily_root);

} // namespace sfr::io

#endif // SFR_IO_KITTI_CALIBRATION_HPP_
