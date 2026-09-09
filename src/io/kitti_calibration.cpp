#include "sfr/io/kitti_calibration.hpp"

#include <cmath>
#include <fstream>
#include <limits>
#include <map>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "sfr/io/io_error.hpp"

namespace sfr::io {

namespace {

using RequiredFields = std::map<std::string, std::size_t>;
using ParsedFields = std::map<std::string, std::vector<double>>;

[[nodiscard]] std::vector<double> parseValues(const std::string& key, std::size_t expected_count,
                                              const std::string& text) {
  std::istringstream stream(text);
  std::vector<double> values;
  std::string token;
  while (stream >> token) {
    std::size_t consumed = 0;
    double value = 0.0;
    try {
      value = std::stod(token, &consumed);
    } catch (const std::exception&) {
      throw IoError(IoErrorCode::kMalformedLine,
                    "calibration key '" + key + "' contains a non-numeric token");
    }
    if (consumed != token.size()) {
      throw IoError(IoErrorCode::kMalformedLine,
                    "calibration key '" + key + "' contains trailing token content");
    }
    if (!std::isfinite(value)) {
      throw IoError(IoErrorCode::kNonFiniteValue,
                    "calibration key '" + key + "' contains a non-finite value");
    }
    values.push_back(value);
  }

  if (values.size() != expected_count) {
    std::ostringstream message;
    message << "calibration key '" << key << "' expected " << expected_count << " values but got "
            << values.size();
    throw IoError(IoErrorCode::kWrongValueCount, message.str());
  }
  return values;
}

[[nodiscard]] ParsedFields parseRequiredFields(const std::filesystem::path& path,
                                               const RequiredFields& required_fields) {
  std::ifstream input(path);
  if (!input.is_open()) {
    throw IoError(IoErrorCode::kFileOpen, "unable to open calibration file: " + path.string());
  }

  ParsedFields parsed;
  std::string line;
  std::size_t line_number = 0;
  while (std::getline(input, line)) {
    ++line_number;
    const std::size_t colon = line.find(':');
    if (colon == std::string::npos) {
      continue;
    }
    const std::string key = line.substr(0, colon);
    const auto required = required_fields.find(key);
    if (required == required_fields.end()) {
      continue;
    }
    if (parsed.contains(key)) {
      throw IoError(IoErrorCode::kDuplicateKey, "duplicate calibration key '" + key + "' at line " +
                                                    std::to_string(line_number));
    }
    parsed.emplace(key, parseValues(key, required->second, line.substr(colon + 1)));
  }

  for (const auto& [key, expected_count] : required_fields) {
    static_cast<void>(expected_count);
    if (!parsed.contains(key)) {
      throw IoError(IoErrorCode::kMissingKey,
                    "missing calibration key '" + key + "' in " + path.string());
    }
  }
  return parsed;
}

[[nodiscard]] geometry::Matrix3d matrix3FromRowMajor(const std::vector<double>& values) {
  geometry::Matrix3d matrix;
  for (Eigen::Index row = 0; row < matrix.rows(); ++row) {
    for (Eigen::Index column = 0; column < matrix.cols(); ++column) {
      const auto index = static_cast<std::size_t>(row * matrix.cols() + column);
      matrix(row, column) = values.at(index);
    }
  }
  return matrix;
}

[[nodiscard]] geometry::Matrix34d matrix34FromRowMajor(const std::vector<double>& values) {
  geometry::Matrix34d matrix;
  for (Eigen::Index row = 0; row < matrix.rows(); ++row) {
    for (Eigen::Index column = 0; column < matrix.cols(); ++column) {
      const auto index = static_cast<std::size_t>(row * matrix.cols() + column);
      matrix(row, column) = values.at(index);
    }
  }
  return matrix;
}

[[nodiscard]] int positiveImageDimension(double value, const std::string& key) {
  if (value <= 0.0 || value > static_cast<double>(std::numeric_limits<int>::max()) ||
      std::floor(value) != value) {
    throw IoError(IoErrorCode::kInvalidCalibration,
                  "calibration key '" + key + "' must contain positive integer dimensions");
  }
  return static_cast<int>(value);
}

} // namespace

geometry::RigidTransform KittiCalibration::TCameraRect00Lidar() const {
  return geometry::compose(T_camera_rect_00_camera_raw_00, T_camera_raw_00_lidar);
}

geometry::RectifiedProjection KittiCalibration::rectifiedProjection(double z_min_m) const {
  return geometry::RectifiedProjection({P_rect_02, image_width_px, image_height_px, z_min_m});
}

KittiCalibration loadKittiCalibration(const std::filesystem::path& daily_root) {
  const ParsedFields velo =
      parseRequiredFields(daily_root / "calib_velo_to_cam.txt", {{"R", 9U}, {"T", 3U}});
  const ParsedFields camera =
      parseRequiredFields(daily_root / "calib_cam_to_cam.txt",
                          {{"P_rect_02", 12U}, {"R_rect_00", 9U}, {"S_rect_02", 2U}});

  const geometry::Matrix3d R_camera_raw_00_lidar = matrix3FromRowMajor(velo.at("R"));
  const std::vector<double>& translation = velo.at("T");
  const geometry::Vector3d t_camera_raw_00_lidar_m(translation.at(0), translation.at(1),
                                                   translation.at(2));
  const geometry::Matrix3d R_camera_rect_00_camera_raw_00 =
      matrix3FromRowMajor(camera.at("R_rect_00"));
  const std::vector<double>& dimensions = camera.at("S_rect_02");

  try {
    return KittiCalibration{
        geometry::RigidTransform({.target_frame = geometry::FrameId("camera_raw_00"),
                                  .source_frame = geometry::FrameId("lidar"),
                                  .rotation_target_source = R_camera_raw_00_lidar,
                                  .translation_target_source_m = t_camera_raw_00_lidar_m}),
        geometry::RigidTransform({.target_frame = geometry::FrameId("camera_rect_00"),
                                  .source_frame = geometry::FrameId("camera_raw_00"),
                                  .rotation_target_source = R_camera_rect_00_camera_raw_00,
                                  .translation_target_source_m = geometry::Vector3d::Zero()}),
        matrix34FromRowMajor(camera.at("P_rect_02")),
        positiveImageDimension(dimensions.at(0), "S_rect_02"),
        positiveImageDimension(dimensions.at(1), "S_rect_02"),
    };
  } catch (const geometry::GeometryError& error) {
    throw IoError(IoErrorCode::kInvalidCalibration,
                  std::string("invalid KITTI rigid calibration: ") + error.what());
  }
}

} // namespace sfr::io
