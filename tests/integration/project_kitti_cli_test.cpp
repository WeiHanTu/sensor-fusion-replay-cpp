#include <array>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>
#include <string_view>
#include <system_error>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
#include <opencv2/imgcodecs.hpp>

#include <sys/wait.h>

#ifndef SFR_PROJECT_KITTI_EXE
#error "SFR_PROJECT_KITTI_EXE must name the project_kitti executable"
#endif

namespace {

class ProjectKittiCliTest : public ::testing::Test {
protected:
  void SetUp() override {
    root_ = std::filesystem::path(::testing::TempDir()) / "sfr_project_kitti_cli_test";
    std::error_code error;
    std::filesystem::remove_all(root_, error);
    const std::filesystem::path drive = root_ / "synthetic_drive_sync";
    ASSERT_TRUE(std::filesystem::create_directories(drive / "image_02/data"));
    ASSERT_TRUE(std::filesystem::create_directories(drive / "velodyne_points/data"));

    std::ofstream(root_ / "calib_cam_to_cam.txt") << "S_rect_02: 8 6\n"
                                                     "R_rect_00: 1 0 0 0 1 0 0 0 1\n"
                                                     "P_rect_02: 2 0 4 1 0 2 3 0 0 0 1 0\n";
    std::ofstream(root_ / "calib_velo_to_cam.txt") << "R: 0 -1 0 0 0 -1 1 0 0\nT: 0 0 0\n";
    std::ofstream(drive / "image_02/timestamps.txt") << "2011-09-26 13:02:45.000000001\n";
    std::ofstream(drive / "velodyne_points/timestamps.txt") << "2011-09-26 13:02:45.000000002\n";

    const cv::Mat image(6, 8, CV_8UC3, cv::Scalar(8, 16, 32));
    ASSERT_TRUE(cv::imwrite((drive / "image_02/data/0000000000.png").string(), image));
    const std::array<float, 4> point{4.0F, -2.0F, -1.0F, 0.5F};
    std::ofstream lidar(drive / "velodyne_points/data/0000000000.bin", std::ios::binary);
    lidar.write(reinterpret_cast<const char*>(point.data()),
                static_cast<std::streamsize>(point.size() * sizeof(float)));
    ASSERT_TRUE(lidar.good());
  }

  void TearDown() override {
    if (std::getenv("SFR_KEEP_CLI_FIXTURE") != nullptr) {
      return;
    }
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

  std::filesystem::path root_;
};

TEST_F(ProjectKittiCliTest, ProducesValidAtomicSyntheticRun) {
  const std::filesystem::path output_root = root_ / "artifacts";
  const std::string command =
      shellQuote(SFR_PROJECT_KITTI_EXE) + " --dataset-root " + shellQuote(root_.string()) +
      " --drive synthetic_drive_sync --frame 0 --output-dir " + shellQuote(output_root.string());

  ASSERT_EQ(exitCode(std::system(command.c_str())), 0);
  ASSERT_TRUE(std::filesystem::is_directory(output_root));
  const std::filesystem::directory_iterator first(output_root);
  ASSERT_NE(first, std::filesystem::directory_iterator());
  const std::filesystem::path run_directory = first->path();
  auto next = first;
  ++next;
  EXPECT_EQ(next, std::filesystem::directory_iterator());
  EXPECT_FALSE(run_directory.filename().string().ends_with(".tmp"));

  nlohmann::json summary;
  std::ifstream(run_directory / "run_summary.json") >> summary;
  EXPECT_EQ(summary.at("status"), "complete");
  EXPECT_EQ(summary.at("counts").at("projected_visible"), 1U);
  EXPECT_EQ(summary.at("measured_frames"), 1U);
  EXPECT_TRUE(std::filesystem::is_regular_file(run_directory / "frames.jsonl"));
  EXPECT_TRUE(std::filesystem::is_regular_file(run_directory / "overlays/0000000000.png"));
}

TEST_F(ProjectKittiCliTest, UsesDocumentedArgumentExitCategory) {
  const std::string command = shellQuote(SFR_PROJECT_KITTI_EXE) + " --unknown 2>/dev/null";
  EXPECT_EQ(exitCode(std::system(command.c_str())), 2);
}

TEST_F(ProjectKittiCliTest, UsesDocumentedInputAndOutputExitCategories) {
  const std::string missing_input = shellQuote(SFR_PROJECT_KITTI_EXE) + " --dataset-root " +
                                    shellQuote((root_ / "missing").string()) +
                                    " --drive synthetic_drive_sync --frame 0 --output-dir " +
                                    shellQuote((root_ / "unused-output").string()) + " 2>/dev/null";
  EXPECT_EQ(exitCode(std::system(missing_input.c_str())), 3);

  const std::string unwritable_output =
      shellQuote(SFR_PROJECT_KITTI_EXE) + " --dataset-root " + shellQuote(root_.string()) +
      " --drive synthetic_drive_sync --frame 0 --output-dir " +
      shellQuote((root_ / "calib_cam_to_cam.txt").string()) + " 2>/dev/null";
  EXPECT_EQ(exitCode(std::system(unwritable_output.c_str())), 5);
}

} // namespace
