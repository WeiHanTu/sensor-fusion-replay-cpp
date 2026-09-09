#include "sfr/io/kitti_io.hpp"

#include <array>
#include <bit>
#include <charconv>
#include <cmath>
#include <fstream>
#include <limits>
#include <map>
#include <sstream>
#include <system_error>

#include <opencv2/imgcodecs.hpp>

#include "sfr/io/io_error.hpp"

namespace sfr::io {

namespace {

constexpr std::uintmax_t kMaximumFrameFileBytes =
    std::uintmax_t{128} * std::uintmax_t{1024} * std::uintmax_t{1024};
constexpr std::size_t kMaximumFrameCount = 1'000'000U;
constexpr std::uint64_t kMaximumImagePixels = 100'000'000U;
constexpr std::uintmax_t kVelodyneRecordBytes = 4U * sizeof(float);

[[nodiscard]] std::uintmax_t checkedFileSize(const std::filesystem::path& path) {
  std::error_code error;
  const std::uintmax_t size = std::filesystem::file_size(path, error);
  if (error) {
    throw IoError(IoErrorCode::kInvalidFrameFile,
                  "unable to read frame file size: " + path.string());
  }
  if (size > kMaximumFrameFileBytes) {
    throw IoError(IoErrorCode::kLimitExceeded,
                  "frame file exceeds 128 MiB limit: " + path.string());
  }
  return size;
}

[[nodiscard]] std::uint64_t parseFrameId(const std::filesystem::path& path) {
  const std::string stem = path.stem().string();
  if (stem.empty()) {
    throw IoError(IoErrorCode::kInvalidFrameFile, "frame filename has an empty numeric stem");
  }
  std::uint64_t frame_id = 0;
  const auto [end, error] = std::from_chars(stem.data(), stem.data() + stem.size(), frame_id);
  if (error != std::errc{} || end != stem.data() + stem.size()) {
    throw IoError(IoErrorCode::kInvalidFrameFile,
                  "frame filename stem must be numeric: " + path.filename().string());
  }
  return frame_id;
}

[[nodiscard]] std::vector<TimedFrameFile> attachTimestamps(std::vector<FrameFile> frames,
                                                           std::vector<TimestampRecord> timestamps,
                                                           std::string_view stream_name) {
  if (frames.size() != timestamps.size()) {
    std::ostringstream message;
    message << stream_name << " frame/timestamp count mismatch: " << frames.size() << " frames, "
            << timestamps.size() << " timestamps";
    throw IoError(IoErrorCode::kInvalidLayout, message.str());
  }
  std::vector<TimedFrameFile> timed_frames;
  timed_frames.reserve(frames.size());
  for (std::size_t index = 0; index < frames.size(); ++index) {
    timed_frames.push_back(TimedFrameFile{std::move(frames[index]), std::move(timestamps[index])});
  }
  return timed_frames;
}

} // namespace

std::vector<FrameFile> enumerateNumericFrameFiles(const std::filesystem::path& data_directory,
                                                  std::string_view required_extension) {
  std::error_code error;
  if (!std::filesystem::is_directory(data_directory, error) || error) {
    throw IoError(IoErrorCode::kInvalidLayout,
                  "frame data directory is missing: " + data_directory.string());
  }

  std::map<std::uint64_t, std::filesystem::path> ordered;
  for (const std::filesystem::directory_entry& entry :
       std::filesystem::directory_iterator(data_directory)) {
    if (!entry.is_regular_file() || entry.path().extension() != required_extension) {
      continue;
    }
    if (ordered.size() >= kMaximumFrameCount) {
      throw IoError(IoErrorCode::kLimitExceeded, "frame count exceeds configured limit");
    }
    const std::uint64_t frame_id = parseFrameId(entry.path());
    if (!ordered.emplace(frame_id, entry.path()).second) {
      throw IoError(IoErrorCode::kInvalidFrameFile,
                    "duplicate numeric frame ID: " + std::to_string(frame_id));
    }
  }
  if (ordered.empty()) {
    throw IoError(IoErrorCode::kInvalidLayout, "frame data directory contains no " +
                                                   std::string(required_extension) +
                                                   " files: " + data_directory.string());
  }

  std::vector<FrameFile> frames;
  frames.reserve(ordered.size());
  for (auto& [frame_id, path] : ordered) {
    frames.push_back(FrameFile{frame_id, std::move(path)});
  }
  return frames;
}

KittiSequenceLayout loadKittiSequenceLayout(const std::filesystem::path& daily_root,
                                            std::string_view drive_name) {
  const std::string drive_name_string(drive_name);
  const std::filesystem::path drive_path(drive_name_string);
  if (drive_name_string.empty() || drive_path.has_parent_path() ||
      !drive_name_string.ends_with("_sync")) {
    throw IoError(IoErrorCode::kInvalidLayout, "drive name must be one basename ending in '_sync'");
  }

  const std::filesystem::path drive_root = daily_root / drive_path;
  const std::filesystem::path image_root = drive_root / "image_02";
  const std::filesystem::path lidar_root = drive_root / "velodyne_points";
  std::vector<FrameFile> image_frames = enumerateNumericFrameFiles(image_root / "data", ".png");
  std::vector<FrameFile> lidar_frames = enumerateNumericFrameFiles(lidar_root / "data", ".bin");
  std::vector<TimestampRecord> image_timestamps =
      loadKittiTimestamps(image_root / "timestamps.txt");
  std::vector<TimestampRecord> lidar_timestamps =
      loadKittiTimestamps(lidar_root / "timestamps.txt");

  return KittiSequenceLayout{
      drive_name_string,
      daily_root,
      drive_root,
      attachTimestamps(std::move(image_frames), std::move(image_timestamps), "image_02"),
      attachTimestamps(std::move(lidar_frames), std::move(lidar_timestamps), "velodyne_points"),
  };
}

LidarLoadResult loadVelodyneFrame(const std::filesystem::path& frame_file) {
  if constexpr (std::endian::native != std::endian::little) {
    throw IoError(IoErrorCode::kUnsupportedPlatform,
                  "Velodyne loader currently requires a little-endian host");
  }

  const std::uintmax_t size_bytes = checkedFileSize(frame_file);
  if (size_bytes == 0U || size_bytes % kVelodyneRecordBytes != 0U) {
    throw IoError(IoErrorCode::kInvalidFrameFile,
                  "Velodyne file must be nonempty and divisible by 16 bytes: " +
                      frame_file.string());
  }

  std::ifstream input(frame_file, std::ios::binary);
  if (!input.is_open()) {
    throw IoError(IoErrorCode::kFileOpen, "unable to open Velodyne frame: " + frame_file.string());
  }

  const auto record_count = static_cast<std::size_t>(size_bytes / kVelodyneRecordBytes);
  LidarLoadResult result;
  result.points.reserve(record_count);
  result.non_finite_points = 0U;
  std::array<float, 4> record{};
  for (std::size_t index = 0; index < record_count; ++index) {
    input.read(reinterpret_cast<char*>(record.data()),
               static_cast<std::streamsize>(kVelodyneRecordBytes));
    if (!input) {
      throw IoError(IoErrorCode::kInvalidFrameFile,
                    "short read in Velodyne frame at record " + std::to_string(index));
    }
    if (!std::isfinite(record[0]) || !std::isfinite(record[1]) || !std::isfinite(record[2]) ||
        !std::isfinite(record[3])) {
      ++result.non_finite_points;
      continue;
    }
    result.points.push_back(core::PointXYZI{static_cast<double>(record[0]),
                                            static_cast<double>(record[1]),
                                            static_cast<double>(record[2]), record[3]});
  }
  if (result.points.empty()) {
    throw IoError(IoErrorCode::kInvalidFrameFile,
                  "Velodyne frame contains no finite points: " + frame_file.string());
  }
  return result;
}

cv::Mat loadBgrImage(const std::filesystem::path& image_file, int expected_width_px,
                     int expected_height_px) {
  if (expected_width_px <= 0 || expected_height_px <= 0) {
    throw IoError(IoErrorCode::kInvalidCalibration, "expected image dimensions must be positive");
  }
  const auto pixel_count = static_cast<std::uint64_t>(expected_width_px) *
                           static_cast<std::uint64_t>(expected_height_px);
  if (pixel_count > kMaximumImagePixels) {
    throw IoError(IoErrorCode::kLimitExceeded, "expected image size exceeds pixel limit");
  }
  static_cast<void>(checkedFileSize(image_file));

  cv::Mat image;
  try {
    image = cv::imread(image_file.string(), cv::IMREAD_COLOR);
  } catch (const cv::Exception& error) {
    throw IoError(IoErrorCode::kInvalidFrameFile,
                  "unable to decode image as BGR8: " + image_file.string() + ": " + error.what());
  }
  if (image.empty() || image.type() != CV_8UC3) {
    throw IoError(IoErrorCode::kInvalidFrameFile,
                  "unable to decode image as BGR8: " + image_file.string());
  }
  if (image.cols != expected_width_px || image.rows != expected_height_px) {
    std::ostringstream message;
    message << "image dimensions " << image.cols << 'x' << image.rows
            << " do not match calibration " << expected_width_px << 'x' << expected_height_px;
    throw IoError(IoErrorCode::kInvalidFrameFile, message.str());
  }
  return image;
}

} // namespace sfr::io
