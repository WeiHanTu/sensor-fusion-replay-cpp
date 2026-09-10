#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace {

constexpr int kExitSuccess = 0;
constexpr int kExitInvalidArguments = 2;
constexpr int kExitOutputFailure = 5;
constexpr int kImageWidthPx = 960;
constexpr int kImageHeightPx = 540;
constexpr double kFocalLengthPx = 520.0;
constexpr double kPrincipalXpx = 480.0;
constexpr double kPrincipalYpx = 270.0;
constexpr double kGroundCameraYm = 1.7;
constexpr std::string_view kDriveName = "synthetic_drive_sync";

class ArgumentError final : public std::runtime_error {
public:
  explicit ArgumentError(const std::string& message) : std::runtime_error(message) {}
};

class OutputError final : public std::runtime_error {
public:
  explicit OutputError(const std::string& message) : std::runtime_error(message) {}
};

class TemporaryDirectory final {
public:
  explicit TemporaryDirectory(std::filesystem::path path) : path_(std::move(path)) {
    try {
      if (!std::filesystem::create_directory(path_)) {
        throw OutputError("temporary synthetic directory already exists: " + path_.string());
      }
    } catch (const std::filesystem::filesystem_error& error) {
      throw OutputError(std::string("unable to claim temporary synthetic directory: ") +
                        error.what());
    }
  }

  TemporaryDirectory(const TemporaryDirectory&) = delete;
  TemporaryDirectory& operator=(const TemporaryDirectory&) = delete;
  TemporaryDirectory(TemporaryDirectory&&) = delete;
  TemporaryDirectory& operator=(TemporaryDirectory&&) = delete;

  ~TemporaryDirectory() {
    if (!published_) {
      std::error_code error;
      std::filesystem::remove_all(path_, error);
    }
  }

  void publish() noexcept { published_ = true; }

private:
  std::filesystem::path path_;
  bool published_{false};
};

struct Options final {
  std::filesystem::path output_directory;
  bool overwrite{false};
};

struct CameraPoint final {
  double x_m;
  double y_m;
  double depth_m;
  float reflectance;
};

struct PanelSpec final {
  double center_x_m;
  double depth_m;
  double width_m;
  double height_m;
  float reflectance;
  std::array<double, 3> color_bgr;
};

constexpr std::array<PanelSpec, 3> kPanels{{
    {-2.4, 13.0, 1.8, 1.9, 0.95F, {60.0, 85.0, 175.0}},
    {2.7, 23.0, 2.4, 2.5, 0.75F, {85.0, 150.0, 70.0}},
    {-5.2, 38.0, 3.0, 3.2, 0.55F, {155.0, 95.0, 55.0}},
}};

void printHelp(std::ostream& output) {
  output << "Usage: generate_synthetic_kitti --output-dir <daily-root> [--overwrite]\n\n"
            "Creates an authored one-frame KITTI-shaped fixture for public demos.\n"
            "The fixture contains no KITTI data and must not be used as real-data evidence.\n\n"
            "Required:\n"
            "  --output-dir <path>  Synthetic daily root to publish from staging\n\n"
            "Options:\n"
            "  --overwrite          Replace only the exact output directory\n"
            "  --help               Show this help\n";
}

[[nodiscard]] Options parseArguments(const int argument_count, char** argument_values) {
  Options options;
  bool output_set = false;
  bool overwrite_set = false;
  for (int index = 1; index < argument_count; ++index) {
    const std::string_view argument(argument_values[index]);
    if (argument == "--help") {
      printHelp(std::cout);
      throw ArgumentError("__help_shown__");
    }
    if (argument == "--overwrite") {
      if (overwrite_set) {
        throw ArgumentError("--overwrite may be specified only once");
      }
      options.overwrite = true;
      overwrite_set = true;
      continue;
    }
    if (argument == "--output-dir") {
      if (output_set) {
        throw ArgumentError("--output-dir may be specified only once");
      }
      if (index + 1 >= argument_count) {
        throw ArgumentError("--output-dir requires a value");
      }
      ++index;
      options.output_directory = argument_values[index];
      output_set = true;
      continue;
    }
    throw ArgumentError("unknown argument: " + std::string(argument));
  }

  const std::filesystem::path normalized = options.output_directory.lexically_normal();
  if (!output_set || options.output_directory.empty() || normalized.filename().empty() ||
      normalized.filename() == "." || normalized.filename() == "..") {
    throw ArgumentError("--output-dir must name a non-root directory");
  }
  options.output_directory = normalized;
  return options;
}

[[nodiscard]] cv::Point projectPixel(const CameraPoint& point) {
  const double u_px = kFocalLengthPx * point.x_m / point.depth_m + kPrincipalXpx;
  const double v_px = kFocalLengthPx * point.y_m / point.depth_m + kPrincipalYpx;
  return {static_cast<int>(std::lround(u_px)), static_cast<int>(std::lround(v_px))};
}

void appendLidarRecord(std::vector<float>& records, const CameraPoint& point) {
  if (!std::isfinite(point.x_m) || !std::isfinite(point.y_m) || !std::isfinite(point.depth_m) ||
      point.depth_m == 0.0F || !std::isfinite(point.reflectance)) {
    throw std::logic_error("synthetic point must be finite and have nonzero depth");
  }
  records.push_back(static_cast<float>(point.depth_m));
  records.push_back(static_cast<float>(-point.x_m));
  records.push_back(static_cast<float>(-point.y_m));
  records.push_back(point.reflectance);
}

void addPanel(std::vector<float>& records, const PanelSpec& panel) {
  constexpr int kHorizontalSamples = 28;
  constexpr int kVerticalSamples = 24;
  for (int vertical = 0; vertical < kVerticalSamples; ++vertical) {
    const double vertical_fraction =
        static_cast<double>(vertical) / static_cast<double>(kVerticalSamples - 1);
    const double camera_y_m = kGroundCameraYm - panel.height_m * vertical_fraction;
    for (int horizontal = 0; horizontal < kHorizontalSamples; ++horizontal) {
      const double horizontal_fraction =
          static_cast<double>(horizontal) / static_cast<double>(kHorizontalSamples - 1);
      const double camera_x_m =
          panel.center_x_m - panel.width_m / 2.0 + panel.width_m * horizontal_fraction;
      appendLidarRecord(records, {camera_x_m, camera_y_m, panel.depth_m, panel.reflectance});
    }
  }
}

[[nodiscard]] std::vector<float> makePointRecords() {
  std::vector<float> records;
  records.reserve(18'000U);

  for (int depth_index = 0; depth_index < 37; ++depth_index) {
    const double depth_m = 6.0 + 1.5 * static_cast<double>(depth_index);
    const double half_width_m = std::min(9.0, 0.72 * depth_m);
    const int lateral_samples = static_cast<int>(std::floor(half_width_m / 0.30));
    for (int lateral_index = -lateral_samples; lateral_index <= lateral_samples; ++lateral_index) {
      const double camera_x_m = 0.30 * static_cast<double>(lateral_index);
      const float reflectance =
          static_cast<float>(0.25 + 0.65 * static_cast<double>(depth_index % 7) / 6.0);
      appendLidarRecord(records, {camera_x_m, kGroundCameraYm, depth_m, reflectance});
    }
  }

  for (const PanelSpec& panel : kPanels) {
    addPanel(records, panel);
  }

  for (int index = 0; index < 60; ++index) {
    const double depth_m = 8.0 + static_cast<double>(index % 20);
    appendLidarRecord(records, {2.0 * depth_m, 0.0, depth_m, 0.1F});
  }
  for (int index = 0; index < 60; ++index) {
    appendLidarRecord(records, {0.2 * static_cast<double>(index % 5), 0.0, -1.0 - index, 0.1F});
  }
  return records;
}

[[nodiscard]] cv::Mat makeSyntheticImage() {
  cv::Mat image(kImageHeightPx, kImageWidthPx, CV_8UC3);
  constexpr int kHorizonRow = 278;
  for (int row = 0; row < image.rows; ++row) {
    const double fraction = static_cast<double>(row) / static_cast<double>(image.rows - 1);
    const cv::Vec3b sky{static_cast<std::uint8_t>(150.0 - 45.0 * fraction),
                        static_cast<std::uint8_t>(105.0 - 25.0 * fraction),
                        static_cast<std::uint8_t>(70.0 - 10.0 * fraction)};
    image.row(row).setTo(sky);
  }

  const std::array<cv::Point, 4> road{{{365, kHorizonRow},
                                       {595, kHorizonRow},
                                       {900, kImageHeightPx - 1},
                                       {60, kImageHeightPx - 1}}};
  cv::fillConvexPoly(image, road.data(), static_cast<int>(road.size()), cv::Scalar(52, 55, 58),
                     cv::LINE_AA);
  cv::line(image, {365, kHorizonRow}, {60, kImageHeightPx - 1}, cv::Scalar(130, 135, 140), 4,
           cv::LINE_AA);
  cv::line(image, {595, kHorizonRow}, {900, kImageHeightPx - 1}, cv::Scalar(130, 135, 140), 4,
           cv::LINE_AA);
  for (int index = 0; index < 7; ++index) {
    const int top = kHorizonRow + 10 + index * index * 5;
    const int bottom = std::min(kImageHeightPx - 1, top + 8 + index * 3);
    const int left_top = 477 - index * 3;
    const int right_top = 483 + index * 3;
    cv::fillConvexPoly(
        image,
        std::array<cv::Point, 4>{
            {{left_top, top}, {right_top, top}, {right_top + 5, bottom}, {left_top - 5, bottom}}}
            .data(),
        4, cv::Scalar(210, 210, 210), cv::LINE_AA);
  }

  const auto drawPanel = [&image](const PanelSpec& panel) {
    const cv::Point top_left =
        projectPixel({panel.center_x_m - panel.width_m / 2.0, kGroundCameraYm - panel.height_m,
                      panel.depth_m, 0.0F});
    const cv::Point bottom_right = projectPixel(
        {panel.center_x_m + panel.width_m / 2.0, kGroundCameraYm, panel.depth_m, 0.0F});
    cv::rectangle(image, top_left, bottom_right,
                  cv::Scalar(panel.color_bgr[0], panel.color_bgr[1], panel.color_bgr[2]),
                  cv::FILLED, cv::LINE_AA);
    cv::rectangle(image, top_left, bottom_right, cv::Scalar(230, 230, 230), 2, cv::LINE_AA);
  };
  for (const PanelSpec& panel : kPanels) {
    drawPanel(panel);
  }

  cv::putText(image, "AUTHORED SYNTHETIC SCENE", {28, 45}, cv::FONT_HERSHEY_SIMPLEX, 0.8,
              cv::Scalar(245, 245, 245), 2, cv::LINE_AA);
  cv::putText(image, "not KITTI data", {31, 75}, cv::FONT_HERSHEY_SIMPLEX, 0.55,
              cv::Scalar(230, 230, 230), 1, cv::LINE_AA);
  return image;
}

void writeTextFile(const std::filesystem::path& path, const std::string_view contents) {
  std::ofstream output;
  output.exceptions(std::ios::failbit | std::ios::badbit);
  output.open(path);
  output << contents;
  output.close();
}

void writeFixture(const std::filesystem::path& root) {
  const std::filesystem::path drive = root / std::string(kDriveName);
  std::filesystem::create_directories(drive / "image_02/data");
  std::filesystem::create_directories(drive / "velodyne_points/data");

  writeTextFile(root / "calib_cam_to_cam.txt", "S_rect_02: 960 540\n"
                                               "R_rect_00: 1 0 0 0 1 0 0 0 1\n"
                                               "P_rect_02: 520 0 480 0 0 520 270 0 0 0 1 0\n");
  writeTextFile(root / "calib_velo_to_cam.txt", "R: 0 -1 0 0 0 -1 1 0 0\nT: 0 0 0\n");
  writeTextFile(drive / "image_02/timestamps.txt", "2011-09-26 13:02:45.000000001\n");
  writeTextFile(drive / "velodyne_points/timestamps.txt", "2011-09-26 13:02:45.000000002\n");

  if (!cv::imwrite((drive / "image_02/data/0000000000.png").string(), makeSyntheticImage())) {
    throw OutputError("OpenCV failed to write the synthetic image");
  }

  const std::vector<float> records = makePointRecords();
  if (records.size() >
      static_cast<std::size_t>(std::numeric_limits<std::streamsize>::max()) / sizeof(float)) {
    throw OutputError("synthetic point cloud exceeds stream size range");
  }
  std::ofstream lidar;
  lidar.exceptions(std::ios::failbit | std::ios::badbit);
  lidar.open(drive / "velodyne_points/data/0000000000.bin", std::ios::binary);
  lidar.write(reinterpret_cast<const char*>(records.data()),
              static_cast<std::streamsize>(records.size() * sizeof(float)));
  lidar.close();

  writeTextFile(root / "README.txt",
                "Authored synthetic KITTI-shaped fixture for sensor-fusion-replay-cpp.\n"
                "Contains no KITTI data. Frame 0 contains a drawn road-like scene and finite\n"
                "LiDAR points generated from explicit camera-frame geometry.\n");
}

[[nodiscard]] int run(const Options& options) {
  const std::filesystem::path final_directory = options.output_directory;
  std::filesystem::path temporary_directory = final_directory;
  temporary_directory += ".tmp";
  try {
    if (final_directory.has_parent_path()) {
      std::filesystem::create_directories(final_directory.parent_path());
    }
    if (std::filesystem::exists(final_directory) &&
        (!options.overwrite || !std::filesystem::is_directory(final_directory) ||
         std::filesystem::is_symlink(final_directory))) {
      throw OutputError("synthetic output exists; pass --overwrite to replace the exact "
                        "directory: " +
                        final_directory.string());
    }
  } catch (const std::filesystem::filesystem_error& error) {
    throw OutputError(std::string("unable to prepare synthetic output parent: ") + error.what());
  }

  TemporaryDirectory cleanup(temporary_directory);
  try {
    writeFixture(temporary_directory);
    if (std::filesystem::exists(final_directory)) {
      std::filesystem::remove_all(final_directory);
    }
    std::filesystem::rename(temporary_directory, final_directory);
  } catch (const OutputError&) {
    throw;
  } catch (const cv::Exception& error) {
    throw OutputError(std::string("OpenCV failed while creating synthetic output: ") +
                      error.what());
  } catch (const std::filesystem::filesystem_error& error) {
    throw OutputError(std::string("unable to publish synthetic output: ") + error.what());
  } catch (const std::ios_base::failure& error) {
    throw OutputError(std::string("unable to write synthetic output: ") + error.what());
  }
  cleanup.publish();
  std::cout << "created authored synthetic fixture: " << final_directory.string() << '\n';
  return kExitSuccess;
}

} // namespace

int main(int argument_count, char** argument_values) {
  try {
    return run(parseArguments(argument_count, argument_values));
  } catch (const ArgumentError& error) {
    if (std::string_view(error.what()) == "__help_shown__") {
      return kExitSuccess;
    }
    std::cerr << "argument error: " << error.what() << '\n';
    return kExitInvalidArguments;
  } catch (const OutputError& error) {
    std::cerr << "output error: " << error.what() << '\n';
    return kExitOutputFailure;
  } catch (const std::exception& error) {
    std::cerr << "output error: " << error.what() << '\n';
    return kExitOutputFailure;
  }
}
