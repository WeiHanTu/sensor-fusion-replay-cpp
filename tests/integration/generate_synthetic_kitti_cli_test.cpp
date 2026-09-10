#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>
#include <string_view>
#include <system_error>

#include <gtest/gtest.h>

#include <sys/wait.h>

#include "sfr/io/kitti_calibration.hpp"
#include "sfr/io/kitti_io.hpp"

#ifndef SFR_GENERATE_SYNTHETIC_KITTI_EXE
#error "SFR_GENERATE_SYNTHETIC_KITTI_EXE must name the generator executable"
#endif

namespace {

class GenerateSyntheticKittiCliTest : public ::testing::Test {
protected:
  void SetUp() override {
    root_ = std::filesystem::path(::testing::TempDir()) / "sfr_generate_synthetic_kitti_test";
    std::error_code error;
    std::filesystem::remove_all(root_, error);
    ASSERT_TRUE(std::filesystem::create_directories(root_));
  }

  void TearDown() override {
    std::error_code error;
    std::filesystem::remove_all(root_, error);
  }

  [[nodiscard]] static std::string shellQuote(const std::string_view text) {
    std::string quoted("'");
    for (const char character : text) {
      if (character == '\'') {
        quoted += "'\\''";
      } else {
        quoted += character;
      }
    }
    quoted += '\'';
    return quoted;
  }

  [[nodiscard]] static int exitCode(const int system_result) {
    if (system_result == -1 || !WIFEXITED(system_result)) {
      return -1;
    }
    return WEXITSTATUS(system_result);
  }

  [[nodiscard]] std::string command(const std::filesystem::path& output) const {
    return shellQuote(SFR_GENERATE_SYNTHETIC_KITTI_EXE) + " --output-dir " +
           shellQuote(output.string());
  }

  std::filesystem::path root_;
};

TEST_F(GenerateSyntheticKittiCliTest, WritesLoadableAuthoredFixture) {
  const std::filesystem::path output = root_ / "fixture";
  ASSERT_EQ(exitCode(std::system(command(output).c_str())), 0);
  EXPECT_FALSE(std::filesystem::exists(root_ / "fixture.tmp"));

  const sfr::io::KittiCalibration calibration = sfr::io::loadKittiCalibration(output);
  EXPECT_EQ(calibration.image_width_px, 960);
  EXPECT_EQ(calibration.image_height_px, 540);
  const sfr::io::KittiSequenceLayout layout =
      sfr::io::loadKittiSequenceLayout(output, "synthetic_drive_sync");
  ASSERT_EQ(layout.image_frames.size(), 1U);
  ASSERT_EQ(layout.lidar_frames.size(), 1U);
  const cv::Mat image = sfr::io::loadBgrImage(layout.image_frames.front().frame.path, 960, 540);
  EXPECT_EQ(image.cols, 960);
  EXPECT_EQ(image.rows, 540);
  const sfr::io::LidarLoadResult lidar =
      sfr::io::loadVelodyneFrame(layout.lidar_frames.front().frame.path);
  EXPECT_GT(lidar.points.size(), 4'000U);
  EXPECT_EQ(lidar.non_finite_points, 0U);
  EXPECT_TRUE(std::filesystem::is_regular_file(output / "README.txt"));
}

TEST_F(GenerateSyntheticKittiCliTest, RequiresExplicitOverwrite) {
  const std::filesystem::path output = root_ / "fixture";
  ASSERT_EQ(exitCode(std::system(command(output).c_str())), 0);
  EXPECT_EQ(exitCode(std::system((command(output) + " 2>/dev/null").c_str())), 5);
  EXPECT_EQ(exitCode(std::system((command(output) + " --overwrite").c_str())), 0);
}

TEST_F(GenerateSyntheticKittiCliTest, RefusesAnExistingEmptyDirectoryWithoutStagingSideEffects) {
  const std::filesystem::path output = root_ / "fixture";
  ASSERT_TRUE(std::filesystem::create_directory(output));

  EXPECT_EQ(exitCode(std::system((command(output) + " 2>/dev/null").c_str())), 5);
  EXPECT_TRUE(std::filesystem::is_empty(output));
  EXPECT_FALSE(std::filesystem::exists(root_ / "fixture.tmp"));
}

TEST_F(GenerateSyntheticKittiCliTest, PreservesPreexistingTemporaryDirectory) {
  const std::filesystem::path output = root_ / "fixture";
  const std::filesystem::path temporary = root_ / "fixture.tmp";
  ASSERT_TRUE(std::filesystem::create_directory(temporary));
  std::ofstream(temporary / "owner-marker.txt") << "not owned by this generator";

  EXPECT_EQ(exitCode(std::system((command(output) + " 2>/dev/null").c_str())), 5);
  EXPECT_TRUE(std::filesystem::is_regular_file(temporary / "owner-marker.txt"));
  EXPECT_FALSE(std::filesystem::exists(output));
}

TEST_F(GenerateSyntheticKittiCliTest, UsesDocumentedArgumentExitCategory) {
  const std::string invalid =
      shellQuote(SFR_GENERATE_SYNTHETIC_KITTI_EXE) + " --unknown 2>/dev/null";
  EXPECT_EQ(exitCode(std::system(invalid.c_str())), 2);
}

} // namespace
