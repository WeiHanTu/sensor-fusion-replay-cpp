#include <array>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
#include <opencv2/imgcodecs.hpp>

#include <sys/wait.h>

#ifndef SFR_ANALYZE_CALIBRATION_EXE
#error "SFR_ANALYZE_CALIBRATION_EXE must name the analyze_calibration executable"
#endif

namespace {

class AnalyzeCalibrationCliTest : public ::testing::Test {
protected:
  void SetUp() override {
    root_ = std::filesystem::path(::testing::TempDir()) / "sfr_analyze_calibration_cli_test";
    std::error_code error;
    std::filesystem::remove_all(root_, error);
    const std::filesystem::path drive = root_ / "synthetic_drive_sync";
    ASSERT_TRUE(std::filesystem::create_directories(drive / "image_02/data"));
    ASSERT_TRUE(std::filesystem::create_directories(drive / "velodyne_points/data"));

    std::ofstream(root_ / "calib_cam_to_cam.txt") << "S_rect_02: 320 180\n"
                                                     "R_rect_00: 1 0 0 0 1 0 0 0 1\n"
                                                     "P_rect_02: 160 0 160 0 0 160 90 0 0 0 1 0\n";
    std::ofstream(root_ / "calib_velo_to_cam.txt") << "R: 0 -1 0 0 0 -1 1 0 0\nT: 0 0 0\n";
    std::ofstream(drive / "image_02/timestamps.txt") << "2011-09-26 13:02:45.000000001\n";
    std::ofstream(drive / "velodyne_points/timestamps.txt") << "2011-09-26 13:02:45.000000002\n";

    cv::Mat image(180, 320, CV_8UC3);
    for (int row = 0; row < image.rows; ++row) {
      for (int column = 0; column < image.cols; ++column) {
        image.at<cv::Vec3b>(row, column) = {
            static_cast<std::uint8_t>(32 + (column * 96 / image.cols)),
            static_cast<std::uint8_t>(32 + (row * 96 / image.rows)), 48U};
      }
    }
    ASSERT_TRUE(cv::imwrite((drive / "image_02/data/0000000000.png").string(), image));

    std::vector<float> point_records;
    for (int vertical_pixel = 35; vertical_pixel <= 145; vertical_pixel += 11) {
      for (int horizontal_pixel = 50; horizontal_pixel <= 270; horizontal_pixel += 20) {
        const float depth_m = 8.0F + static_cast<float>((horizontal_pixel + vertical_pixel) % 33);
        const float camera_x_m = (static_cast<float>(horizontal_pixel) - 160.0F) * depth_m / 160.0F;
        const float camera_y_m = (static_cast<float>(vertical_pixel) - 90.0F) * depth_m / 160.0F;
        point_records.insert(point_records.end(), {depth_m, -camera_x_m, -camera_y_m, 0.5F});
      }
    }
    std::ofstream lidar(drive / "velodyne_points/data/0000000000.bin", std::ios::binary);
    lidar.write(reinterpret_cast<const char*>(point_records.data()),
                static_cast<std::streamsize>(point_records.size() * sizeof(float)));
    ASSERT_TRUE(lidar.good());
  }

  void TearDown() override {
    if (std::getenv("SFR_KEEP_SENSITIVITY_FIXTURE") != nullptr) {
      std::cout << "preserved sensitivity fixture: " << root_ << '\n';
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

TEST_F(AnalyzeCalibrationCliTest, ProducesCompleteSyntheticSensitivityRun) {
  const std::filesystem::path output_root = root_ / "artifacts";
  const std::string command =
      shellQuote(SFR_ANALYZE_CALIBRATION_EXE) + " --dataset-root " + shellQuote(root_.string()) +
      " --drive synthetic_drive_sync --frame 0 --output-dir " + shellQuote(output_root.string());

  ASSERT_EQ(exitCode(std::system(command.c_str())), 0);
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
  EXPECT_EQ(summary.at("counts").at("baseline_visible"), 132U);
  EXPECT_EQ(summary.at("counts").at("perturbation_count"), 8U);

  nlohmann::json report;
  const std::filesystem::path sensitivity_directory = run_directory / "calibration_sensitivity";
  std::ifstream(sensitivity_directory / "report.json") >> report;
  ASSERT_EQ(report.at("experiments").size(), 8U);
  EXPECT_EQ(report.at("perturbation_application"),
            "T_prime_camera_rect_00_lidar = Delta_camera * T_camera_rect_00_lidar");
  for (const nlohmann::json& experiment : report.at("experiments")) {
    EXPECT_GT(experiment.at("common_visible_points").get<std::uint64_t>(), 0U);
    EXPECT_GT(experiment.at("p95_displacement_px").get<double>(), 0.0);
    EXPECT_TRUE(std::filesystem::is_regular_file(sensitivity_directory /
                                                 experiment.at("image").get<std::string>()));
  }

  const cv::Mat comparison =
      cv::imread((sensitivity_directory / "comparison.png").string(), cv::IMREAD_COLOR);
  EXPECT_EQ(comparison.cols, 960);
  EXPECT_EQ(comparison.rows, 708);
  std::size_t png_count = 0U;
  for (const std::filesystem::directory_entry& entry :
       std::filesystem::directory_iterator(sensitivity_directory)) {
    if (entry.path().extension() == ".png") {
      ++png_count;
    }
  }
  EXPECT_EQ(png_count, 10U);
}

TEST_F(AnalyzeCalibrationCliTest, UsesDocumentedArgumentExitCategory) {
  const std::string command = shellQuote(SFR_ANALYZE_CALIBRATION_EXE) + " --unknown 2>/dev/null";
  EXPECT_EQ(exitCode(std::system(command.c_str())), 2);
}

TEST_F(AnalyzeCalibrationCliTest, UsesDocumentedInputAndOutputExitCategories) {
  const std::string missing_input = shellQuote(SFR_ANALYZE_CALIBRATION_EXE) + " --dataset-root " +
                                    shellQuote((root_ / "missing").string()) +
                                    " --drive synthetic_drive_sync --frame 0 --output-dir " +
                                    shellQuote((root_ / "unused-output").string()) + " 2>/dev/null";
  EXPECT_EQ(exitCode(std::system(missing_input.c_str())), 3);

  const std::string unwritable_output =
      shellQuote(SFR_ANALYZE_CALIBRATION_EXE) + " --dataset-root " + shellQuote(root_.string()) +
      " --drive synthetic_drive_sync --frame 0 --output-dir " +
      shellQuote((root_ / "calib_cam_to_cam.txt").string()) + " 2>/dev/null";
  EXPECT_EQ(exitCode(std::system(unwritable_output.c_str())), 5);
}

} // namespace
