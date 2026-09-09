#include "sfr/viz/projection_artifact.hpp"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

namespace sfr::viz {
namespace {

class ProjectionArtifactTest : public ::testing::Test {
protected:
  void SetUp() override {
    root_ = std::filesystem::path(::testing::TempDir()) / "sfr_projection_artifact_test";
    std::error_code error;
    std::filesystem::remove_all(root_, error);
  }

  void TearDown() override {
    std::error_code error;
    std::filesystem::remove_all(root_, error);
  }

  [[nodiscard]] ProjectionArtifactRequest request() const {
    return ProjectionArtifactRequest{
        .output_root = root_,
        .run_id = "deterministic-run",
        .sequence_id = "synthetic_drive_sync",
        .image_frame_id = 2U,
        .lidar_frame_id = 2U,
        .image_relative_name = "0000000002.png",
        .lidar_relative_name = "0000000002.bin",
        .image_timestamp = "2011-09-26 13:02:45.000000001",
        .lidar_timestamp = "2011-09-26 13:02:45.000000002",
        .signed_sync_delta = std::chrono::nanoseconds(1),
        .config = {.z_min_m = 0.1, .depth_min_m = 1.0, .depth_max_m = 80.0, .point_radius_px = 1},
        .projection_counts = {.input_points = 2U,
                              .visible_points = 1U,
                              .non_finite_input = 0U,
                              .behind_or_too_near = 0U,
                              .non_positive_homogeneous_depth = 0U,
                              .outside_image = 1U},
        .lidar_non_finite_points = 0U,
        .overlay = {.image_bgr8 = cv::Mat(3, 4, CV_8UC3, cv::Scalar(1, 2, 3)),
                    .projected_points = 1U,
                    .rendered_pixels = 1U,
                    .occluded_points = 0U},
        .durations = {.source_decode_ms = 1.0, .geometry_ms = 2.0, .visualization_ms = 3.0},
        .overwrite = false,
    };
  }

  std::filesystem::path root_;
};

TEST_F(ProjectionArtifactTest, AtomicallyWritesRequiredSchemaAndRelativeSources) {
  const ProjectionArtifactResult result = writeProjectionArtifacts(request());

  EXPECT_TRUE(std::filesystem::is_regular_file(result.summary_file));
  EXPECT_TRUE(std::filesystem::is_regular_file(result.frames_file));
  EXPECT_TRUE(std::filesystem::is_regular_file(result.overlay_file));
  EXPECT_FALSE(std::filesystem::exists(root_ / "deterministic-run.tmp"));

  nlohmann::json summary;
  std::ifstream(result.summary_file) >> summary;
  EXPECT_EQ(summary.at("schema_version"), "1.0.0");
  EXPECT_EQ(summary.at("status"), "complete");
  EXPECT_EQ(summary.at("dataset").at("sequence_id"), "synthetic_drive_sync");
  EXPECT_EQ(summary.at("counts").at("projected_visible"), 1U);
  EXPECT_EQ(summary.at("latency_ms").at("geometry").at("p99"), 2.0);
  EXPECT_EQ(summary.at("percentile_method"), "nearest_rank");

  nlohmann::json frame;
  std::ifstream(result.frames_file) >> frame;
  EXPECT_EQ(frame.at("image_relative_name"), "0000000002.png");
  EXPECT_EQ(frame.at("lidar_relative_name"), "0000000002.bin");
  EXPECT_EQ(frame.dump().find(root_.string()), std::string::npos);
}

TEST_F(ProjectionArtifactTest, RefusesNonemptyRunUnlessOverwriteIsExplicit) {
  ProjectionArtifactRequest first = request();
  static_cast<void>(writeProjectionArtifacts(first));

  try {
    static_cast<void>(writeProjectionArtifacts(first));
    FAIL() << "expected ArtifactError";
  } catch (const ArtifactError& error) {
    EXPECT_EQ(error.code(), ArtifactErrorCode::kExistingRun);
  }

  first.overwrite = true;
  EXPECT_NO_THROW(static_cast<void>(writeProjectionArtifacts(first)));
}

TEST_F(ProjectionArtifactTest, RejectsUnsafeRunIdAndBrokenAccounting) {
  ProjectionArtifactRequest invalid_id = request();
  invalid_id.run_id = "../outside";
  EXPECT_THROW(static_cast<void>(writeProjectionArtifacts(invalid_id)), ArtifactError);

  ProjectionArtifactRequest invalid_counts = request();
  invalid_counts.projection_counts.outside_image = 2U;
  EXPECT_THROW(static_cast<void>(writeProjectionArtifacts(invalid_counts)), ArtifactError);
}

} // namespace
} // namespace sfr::viz
