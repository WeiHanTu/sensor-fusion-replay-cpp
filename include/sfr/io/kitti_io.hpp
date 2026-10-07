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

/// Owned finite LiDAR points plus the count filtered as non-finite.
///
/// `points.size() + non_finite_points` equals the source record count.
struct LidarLoadResult final {
  std::vector<core::PointXYZI> points;
  std::uint64_t non_finite_points;
};

/// Numeric frame identity and owned local source path.
struct FrameFile final {
  std::uint64_t frame_id;
  std::filesystem::path path;
};

/// Frame file paired by sorted index with its original timestamp record.
struct TimedFrameFile final {
  FrameFile frame;
  TimestampRecord timestamp;
};

/// Owned, validated paths and timestamped files for one KITTI Raw sync drive.
///
/// Image and LiDAR vectors are independently ordered by numeric frame ID. This
/// layout does not perform nearest-timestamp synchronization.
struct KittiSequenceLayout final {
  std::string sequence_id;
  std::filesystem::path daily_root;
  std::filesystem::path drive_root;
  std::vector<TimedFrameFile> image_frames;
  std::vector<TimedFrameFile> lidar_frames;
};

/// Enumerates regular files with the requested extension by numeric stem.
///
/// The returned vector owns its paths and is sorted by numeric frame ID.
/// Files with other extensions are ignored.
///
/// @throws IoError For missing/empty directories, invalid/duplicate stems, or
///         the configured frame-count limit.
[[nodiscard]] std::vector<FrameFile>
enumerateNumericFrameFiles(const std::filesystem::path& data_directory,
                           std::string_view required_extension);

/// Loads the supported `image_02` and `velodyne_points` synced-drive layout.
///
/// `drive_name` must be one basename ending in `_sync`. Each stream is paired
/// with its timestamp file by numeric-file order and timestamp-line index; frame
/// IDs are not required to match across streams here.
///
/// @throws IoError For an unsafe/unsupported name, invalid layout, malformed
///         timestamps, or frame/timestamp count mismatch.
[[nodiscard]] KittiSequenceLayout loadKittiSequenceLayout(const std::filesystem::path& daily_root,
                                                          std::string_view drive_name);

/// Loads little-endian KITTI Velodyne `(x,y,z,reflectance)` float32 records.
///
/// Finite records become owned meter-coordinate points in file order;
/// non-finite records are counted and omitted. Files are capped at 128 MiB.
///
/// @throws IoError For unsupported endianness, file/size/read failure, limit
///         violations, or a frame containing no finite points.
[[nodiscard]] LidarLoadResult loadVelodyneFrame(const std::filesystem::path& frame_file);

/// Decodes an image as owned/ref-counted OpenCV BGR8 storage.
///
/// Expected dimensions are positive pixels and must match the decoded image;
/// the configured pixel and file-size limits are enforced.
///
/// @throws IoError For invalid dimensions, limits, file access, decode, type,
///         or dimension mismatch.
[[nodiscard]] cv::Mat loadBgrImage(const std::filesystem::path& image_file, int expected_width_px,
                                   int expected_height_px);

} // namespace sfr::io

#endif // SFR_IO_KITTI_IO_HPP_
