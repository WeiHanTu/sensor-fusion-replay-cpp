#include "sfr/io/kitti_calibration.hpp"

#include <filesystem>
#include <fstream>
#include <limits>
#include <string>

#include <gtest/gtest.h>

#include "sfr/io/io_error.hpp"

namespace sfr::io {
namespace {

class CalibrationTest : public ::testing::Test {
protected:
  void SetUp() override {
    root_ = std::filesystem::path(::testing::TempDir()) / "sfr_calibration_test";
    std::error_code error;
    std::filesystem::remove_all(root_, error);
    ASSERT_TRUE(std::filesystem::create_directories(root_));
  }

  void TearDown() override {
    std::error_code error;
    std::filesystem::remove_all(root_, error);
  }

  void writePair(const std::string& camera, const std::string& velo) const {
    std::ofstream(root_ / "calib_cam_to_cam.txt") << camera;
    std::ofstream(root_ / "calib_velo_to_cam.txt") << velo;
  }

  std::filesystem::path root_;
};

constexpr const char* kValidCamera = "S_rect_02: 8 6\n"
                                     "R_rect_00: 1 0 0 0 1 0 0 0 1\n"
                                     "P_rect_02: 2 0 4 1 0 2 3 0 0 0 1 0\n";
constexpr const char* kValidVelo = "R: 0 -1 0 0 0 -1 1 0 0\nT: 0 0 0\n";

TEST_F(CalibrationTest, LoadsFullSyntheticProjectionAndAxisTransform) {
  writePair(kValidCamera, kValidVelo);

  const KittiCalibration calibration = loadKittiCalibration(root_);
  const geometry::Vector3d point_camera =
      calibration.TCameraRect00Lidar().transformPoint(geometry::Vector3d(4.0, -2.0, -1.0));
  const geometry::ProjectionResult projected =
      calibration.rectifiedProjection().project(point_camera);

  EXPECT_TRUE(point_camera.isApprox(geometry::Vector3d(2.0, 1.0, 4.0), 0.0));
  ASSERT_EQ(projected.status, geometry::ProjectionStatus::kVisible);
  const geometry::ImageProjection pixel = projected.point.value_or(
      geometry::ImageProjection{std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0});
  EXPECT_DOUBLE_EQ(pixel.u_px, 5.25);
  EXPECT_DOUBLE_EQ(pixel.v_px, 3.5);
  EXPECT_EQ(calibration.image_width_px, 8);
  EXPECT_EQ(calibration.image_height_px, 6);
}

TEST_F(CalibrationTest, RejectsMissingRequiredKey) {
  writePair("S_rect_02: 8 6\nR_rect_00: 1 0 0 0 1 0 0 0 1\n", kValidVelo);
  try {
    static_cast<void>(loadKittiCalibration(root_));
    FAIL() << "expected IoError";
  } catch (const IoError& error) {
    EXPECT_EQ(error.code(), IoErrorCode::kMissingKey);
  }
}

TEST_F(CalibrationTest, RejectsDuplicateRequiredKey) {
  writePair(kValidCamera, std::string(kValidVelo) + "R: 1 0 0 0 1 0 0 0 1\n");
  try {
    static_cast<void>(loadKittiCalibration(root_));
    FAIL() << "expected IoError";
  } catch (const IoError& error) {
    EXPECT_EQ(error.code(), IoErrorCode::kDuplicateKey);
  }
}

TEST_F(CalibrationTest, RejectsWrongValueCount) {
  writePair(kValidCamera, "R: 1 0 0\nT: 0 0 0\n");
  try {
    static_cast<void>(loadKittiCalibration(root_));
    FAIL() << "expected IoError";
  } catch (const IoError& error) {
    EXPECT_EQ(error.code(), IoErrorCode::kWrongValueCount);
  }
}

TEST_F(CalibrationTest, RejectsNonFiniteValue) {
  writePair(kValidCamera, "R: 1 0 0 0 1 0 0 0 nan\nT: 0 0 0\n");
  try {
    static_cast<void>(loadKittiCalibration(root_));
    FAIL() << "expected IoError";
  } catch (const IoError& error) {
    EXPECT_EQ(error.code(), IoErrorCode::kNonFiniteValue);
  }
}

TEST_F(CalibrationTest, RejectsInvalidRotation) {
  writePair(kValidCamera, "R: 2 0 0 0 1 0 0 0 1\nT: 0 0 0\n");
  try {
    static_cast<void>(loadKittiCalibration(root_));
    FAIL() << "expected IoError";
  } catch (const IoError& error) {
    EXPECT_EQ(error.code(), IoErrorCode::kInvalidCalibration);
  }
}

TEST_F(CalibrationTest, RejectsNonNumericValuesAndInvalidDimensions) {
  writePair(kValidCamera, "R: 1 0 0 0 1 0 0 0 value\nT: 0 0 0\n");
  try {
    static_cast<void>(loadKittiCalibration(root_));
    FAIL() << "expected IoError";
  } catch (const IoError& error) {
    EXPECT_EQ(error.code(), IoErrorCode::kMalformedLine);
  }

  writePair("S_rect_02: 8 0\n"
            "R_rect_00: 1 0 0 0 1 0 0 0 1\n"
            "P_rect_02: 2 0 4 1 0 2 3 0 0 0 1 0\n",
            kValidVelo);
  try {
    static_cast<void>(loadKittiCalibration(root_));
    FAIL() << "expected IoError";
  } catch (const IoError& error) {
    EXPECT_EQ(error.code(), IoErrorCode::kInvalidCalibration);
  }
}

} // namespace
} // namespace sfr::io
