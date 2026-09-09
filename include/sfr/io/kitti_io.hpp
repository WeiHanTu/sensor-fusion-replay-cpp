#ifndef SFR_IO_KITTI_IO_HPP_
#define SFR_IO_KITTI_IO_HPP_

#include <cstdint>
#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

#include <opencv2/core/mat.hpp>

#include "sfr/core/point_types.hpp"
#include "sfr/io/timestamp.hpp"

namespace sfr::io {

struct LidarLoadResult final {
  std::vector<core::PointXYZI> points;
  std::uint64_t non_finite_points;
};

struct FrameFile final {
  std::uint64_t frame_id;
  std::filesystem::path path;
};

struct TimedFrameFile final {
  FrameFile frame;
  TimestampRecord timestamp;
};

struct KittiSequenceLayout final {
  std::string sequence_id;
  std::filesystem::path daily_root;
  std::filesystem::path drive_root;
  std::vector<TimedFrameFile> image_frames;
  std::vector<TimedFrameFile> lidar_frames;
};

[[nodiscard]] std::vector<FrameFile>
enumerateNumericFrameFiles(const std::filesystem::path& data_directory,
                           std::string_view required_extension);
[[nodiscard]] KittiSequenceLayout loadKittiSequenceLayout(const std::filesystem::path& daily_root,
                                                          std::string_view drive_name);
[[nodiscard]] LidarLoadResult loadVelodyneFrame(const std::filesystem::path& frame_file);
[[nodiscard]] cv::Mat loadBgrImage(const std::filesystem::path& image_file, int expected_width_px,
                                   int expected_height_px);

} // namespace sfr::io

#endif // SFR_IO_KITTI_IO_HPP_
