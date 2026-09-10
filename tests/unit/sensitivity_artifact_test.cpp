#include "sfr/viz/sensitivity_artifact.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
#include <opencv2/imgcodecs.hpp>

namespace sfr::viz {
namespace {

class SensitivityArtifactTest : public ::testing::Test {
protected:
  void SetUp() override {
    root_ = std::filesystem::path(::testing::TempDir()) / "sfr_sensitivity_artifact_test";
    std::error_code error;
    std::filesystem::remove_all(root_, error);
  }

  void TearDown() override {
    std::error_code error;
    std::filesystem::remove_all(root_, error);
  }

  [[nodiscard]] static OverlayResult overlay() {
    return {.image_bgr8 = cv::Mat(60, 120, CV_8UC3, cv::Scalar(8, 16, 32)),
            .projected_points = 1U,
            .rendered_pixels = 1U,
            .occluded_points = 0U};
  }

  [[nodiscard]] static geometry::ProjectionCounts counts() {
    return {.input_points = 2U,
            .visible_points = 1U,
            .non_finite_input = 0U,
            .behind_or_too_near = 0U,
            .non_positive_homogeneous_depth = 0U,
            .outside_image = 1U};
  }

  [[nodiscard]] static SensitivityPanelResult panel(SensitivityPerturbationKind kind,
                                                    double value) {
    return {.kind = kind,
            .signed_value = value,
            .projection_counts = counts(),
            .displacement = {.common_visible_points = 1U,
                             .disappeared_points = 0U,
                             .appeared_points = 0U,
                             .median_displacement_px = std::abs(value),
                             .p95_displacement_px = std::abs(value)},
            .overlay = overlay()};
  }

  [[nodiscard]] SensitivityArtifactRequest request() const {
    std::vector<SensitivityPanelResult> panels;
    for (const double value : std::array{-1.0, -0.5, 0.5, 1.0}) {
      panels.push_back(panel(SensitivityPerturbationKind::kYawCameraYDegrees, value));
    }
    for (const double value : std::array{-0.05, -0.01, 0.01, 0.05}) {
      panels.push_back(panel(SensitivityPerturbationKind::kTranslationCameraXMeters, value));
    }
    return {
        .output_root = root_,
        .run_id = "sensitivity-run",
        .sequence_id = "synthetic_drive_sync",
        .image_frame_id = 0U,
        .lidar_frame_id = 0U,
        .image_relative_name = "0000000000.png",
        .lidar_relative_name = "0000000000.bin",
        .image_timestamp = "2011-09-26 13:02:45.000000001",
        .lidar_timestamp = "2011-09-26 13:02:45.000000002",
        .signed_sync_delta = std::chrono::nanoseconds(1),
        .config = {.z_min_m = 0.1, .depth_min_m = 1.0, .depth_max_m = 80.0, .point_radius_px = 1},
        .baseline_projection_counts = counts(),
        .lidar_non_finite_points = 0U,
        .baseline_overlay = overlay(),
        .perturbations = std::move(panels),
        .durations = {.source_decode_ms = 1.0, .geometry_ms = 2.0, .visualization_ms = 3.0},
        .overwrite = false,
    };
  }

  std::filesystem::path root_;
};

TEST_F(SensitivityArtifactTest, WritesNineLabeledPanelsComparisonAndMetricsAtomically) {
  const SensitivityArtifactResult result = writeSensitivityArtifacts(request());

  EXPECT_TRUE(std::filesystem::is_regular_file(result.summary_file));
  EXPECT_TRUE(std::filesystem::is_regular_file(result.frames_file));
  EXPECT_TRUE(std::filesystem::is_regular_file(result.report_file));
  EXPECT_TRUE(std::filesystem::is_regular_file(result.comparison_file));
  EXPECT_FALSE(std::filesystem::exists(root_ / "sensitivity-run.tmp"));

  nlohmann::json report;
  std::ifstream(result.report_file) >> report;
  EXPECT_EQ(report.at("experiments").size(), 8U);
  EXPECT_EQ(report.at("experiments").at(0).at("axis"), "camera +y yaw");
  EXPECT_EQ(report.at("experiments").at(4).at("axis"), "camera +x");
  EXPECT_NEAR(report.at("analytic_illustration").at("lateral_offset_m").get<double>(), 0.872753,
              1e-6);

  const cv::Mat comparison = cv::imread(result.comparison_file.string(), cv::IMREAD_COLOR);
  EXPECT_EQ(comparison.cols, 360);
  EXPECT_EQ(comparison.rows, 348);

  std::size_t png_count = 0U;
  for (const std::filesystem::directory_entry& entry :
       std::filesystem::directory_iterator(result.report_file.parent_path())) {
    if (entry.path().extension() == ".png") {
      ++png_count;
    }
  }
  EXPECT_EQ(png_count, 10U);
}

TEST_F(SensitivityArtifactTest, RejectsIncompleteOrDuplicatePerturbationSets) {
  SensitivityArtifactRequest incomplete = request();
  incomplete.perturbations.pop_back();
  EXPECT_THROW(static_cast<void>(writeSensitivityArtifacts(incomplete)), ArtifactError);

  SensitivityArtifactRequest duplicate = request();
  duplicate.perturbations.back() = duplicate.perturbations.front();
  EXPECT_THROW(static_cast<void>(writeSensitivityArtifacts(duplicate)), ArtifactError);
}

TEST_F(SensitivityArtifactTest, PreservesPreexistingTemporaryDirectory) {
  const std::filesystem::path temporary_directory = root_ / "sensitivity-run.tmp";
  ASSERT_TRUE(std::filesystem::create_directories(temporary_directory));
  std::ofstream(temporary_directory / "owner-marker.txt") << "not owned by this writer";

  EXPECT_THROW(static_cast<void>(writeSensitivityArtifacts(request())), ArtifactError);
  EXPECT_TRUE(std::filesystem::is_regular_file(temporary_directory / "owner-marker.txt"));
}

} // namespace
} // namespace sfr::viz
