#include "sfr/io/extrinsic_fixture.hpp"

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <numbers>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include "sfr/io/io_error.hpp"

namespace sfr::io {
namespace {

using Json = nlohmann::json;

[[nodiscard]] Json read(const std::filesystem::path& file) {
  Json value;
  std::ifstream(file) >> value;
  return value;
}

void write(const std::filesystem::path& file, const Json& value) {
  std::ofstream(file) << value.dump(2);
}

class ExtrinsicFixtureTest : public ::testing::Test {
protected:
  void SetUp() override {
    const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
    root_ = std::filesystem::path(::testing::TempDir()) /
            ("sfr_extrinsic_fixture_" + std::to_string(stamp));
    claimed_ = std::filesystem::create_directory(root_);
    ASSERT_TRUE(claimed_);
    dataset_ = root_ / "dataset";
  }
  void TearDown() override {
    if (claimed_) {
      std::error_code error;
      std::filesystem::remove_all(root_, error);
    }
  }
  std::filesystem::path root_;
  std::filesystem::path dataset_;
  bool claimed_{false};
};

TEST_F(ExtrinsicFixtureTest, GeneratesDistinctFramesAndIndependentFullMatrixPixels) {
  generateExtrinsicFixture(dataset_, {});
  const auto fixture = loadExtrinsicFixture(dataset_);
  const std::vector<std::uint64_t> ids{0U, 1U, 2U};
  const auto frames = loadExtrinsicObservationFrames(fixture, ids);
  ASSERT_EQ(frames.size(), 3U);
  EXPECT_EQ(frames[0].observations.size(), 192U);
  EXPECT_EQ(frames[1].observations.size(), 180U);
  EXPECT_EQ(frames[2].observations.size(), 160U);
  EXPECT_NE(fixture.frames[0].scene_identity, fixture.frames[1].scene_identity);
  EXPECT_NE(fixture.frames[1].scene_identity, fixture.frames[2].scene_identity);
  const auto& first = frames[0].observations.front();
  EXPECT_EQ(first.source_index, 11U);
  EXPECT_DOUBLE_EQ(first.point_lidar.x_m, 6.0);
  EXPECT_NEAR(first.point_lidar.y_m, 2.4, 1e-12);
  EXPECT_NEAR(first.observed_pixel.u_px, 480.0 - 208.0 - 52.0 / 6.0, 1e-12);
  EXPECT_NEAR(first.observed_pixel.v_px, 270.0 - 93.6, 1e-12);
  const auto image = loadExtrinsicFixtureImage(fixture, 2U);
  EXPECT_EQ(image.cols, 960);
  EXPECT_EQ(image.rows, 540);
  const auto oracle = loadExtrinsicOracle(fixture);
  EXPECT_EQ(oracle.pixels.size(), 532U);
  EXPECT_DOUBLE_EQ(oracle.hidden.yaw_camera_y_rad, 0.0);
}

TEST_F(ExtrinsicFixtureTest, LoadsTrainingWithoutOpeningOracleOrHeldOutObservations) {
  generateExtrinsicFixture(dataset_, {});
  std::filesystem::remove(dataset_ / "oracle.json");
  std::filesystem::remove(dataset_ / "frames/2/observations.json");
  const auto fixture = loadExtrinsicFixture(dataset_);
  const std::vector<std::uint64_t> ids{0U, 1U};
  EXPECT_NO_THROW(static_cast<void>(loadExtrinsicObservationFrames(fixture, ids)));
  EXPECT_THROW(static_cast<void>(loadExtrinsicOracle(fixture)), IoError);
}

TEST_F(ExtrinsicFixtureTest, NoiseIsFixedSeedReproducibleWithRecordedZeroMeans) {
  ExtrinsicFixtureConfig config;
  config.hidden = {0.7 * std::numbers::pi / 180.0, -0.04};
  config.noise_stddev_px = 1.0;
  generateExtrinsicFixture(dataset_, config);
  generateExtrinsicFixture(root_ / "same", config);
  const auto first = loadExtrinsicFixture(dataset_);
  const auto same = loadExtrinsicFixture(root_ / "same");
  EXPECT_EQ(first.content_identity, same.content_identity);
  EXPECT_EQ(read(dataset_ / "frames/0/observations.json"),
            read(root_ / "same/frames/0/observations.json"));
  for (const auto& frame : first.frames) {
    EXPECT_NEAR(frame.noise_sample_mean.u_px, 0.0, 1e-15);
    EXPECT_NEAR(frame.noise_sample_mean.v_px, 0.0, 1e-15);
  }
  ++config.seed;
  generateExtrinsicFixture(root_ / "different", config);
  EXPECT_NE(first.content_identity, loadExtrinsicFixture(root_ / "different").content_identity);
}

TEST_F(ExtrinsicFixtureTest, OutliersHaveExplicitKeysCountsAndCleanEvaluationTargets) {
  ExtrinsicFixtureConfig config;
  config.training_outlier_fraction = 0.10;
  generateExtrinsicFixture(dataset_, config);
  const auto fixture = loadExtrinsicFixture(dataset_);
  EXPECT_EQ(fixture.frames[0].outlier_count, 19U);
  EXPECT_EQ(fixture.frames[1].outlier_count, 18U);
  EXPECT_EQ(fixture.frames[2].outlier_count, 0U);
  const auto oracle = loadExtrinsicOracle(fixture);
  const std::vector<std::uint64_t> ids{0U, 1U, 2U};
  const auto frames = loadExtrinsicObservationFrames(fixture, ids);
  const auto clean = cleanExtrinsicObservations(frames, oracle);
  std::size_t changed = 0U;
  for (std::size_t frame = 0U; frame < frames.size(); ++frame) {
    for (std::size_t index = 0U; index < frames[frame].observations.size(); ++index) {
      const auto& observed = frames[frame].observations[index].observed_pixel;
      const auto& target = clean[frame].observations[index].observed_pixel;
      if (observed.u_px != target.u_px) {
        ++changed;
        EXPECT_NEAR(observed.u_px - target.u_px, 40.0, 1e-12);
        EXPECT_NEAR(observed.v_px - target.v_px, -30.0, 1e-12);
      }
    }
  }
  EXPECT_EQ(changed, 37U);
}

TEST_F(ExtrinsicFixtureTest, ShuffledRecordsRetainExplicitPointBindings) {
  generateExtrinsicFixture(dataset_, {});
  auto points = read(dataset_ / "frames/0/points.json");
  auto observations = read(dataset_ / "frames/0/observations.json");
  std::reverse(points.begin(), points.end());
  std::reverse(observations.begin(), observations.end());
  write(dataset_ / "frames/0/points.json", points);
  write(dataset_ / "frames/0/observations.json", observations);
  const std::vector<std::uint64_t> ids{0U};
  const auto frames = loadExtrinsicObservationFrames(loadExtrinsicFixture(dataset_), ids);
  EXPECT_EQ(frames.front().observations.front().source_index, 11U);
  EXPECT_DOUBLE_EQ(frames.front().observations.front().point_lidar.x_m, 6.0);
}

TEST_F(ExtrinsicFixtureTest, RejectsMutatedCorrespondencesAndOracleIdentity) {
  generateExtrinsicFixture(dataset_, {});
  const auto fixture = loadExtrinsicFixture(dataset_);
  auto points = read(dataset_ / "frames/0/points.json");
  points[0]["x_m"] = 99.0;
  write(dataset_ / "frames/0/points.json", points);
  const std::vector<std::uint64_t> ids{0U};
  EXPECT_THROW(static_cast<void>(loadExtrinsicObservationFrames(fixture, ids)), IoError);
  auto oracle = read(dataset_ / "oracle.json");
  oracle["fixture_identity"] = "another-fixture";
  write(dataset_ / "oracle.json", oracle);
  EXPECT_THROW(static_cast<void>(loadExtrinsicOracle(fixture)), IoError);
}

TEST_F(ExtrinsicFixtureTest, RejectsSplitOverlapMissingIdsAndDuplicatedSceneContent) {
  generateExtrinsicFixture(dataset_, {});
  auto fixture = loadExtrinsicFixture(dataset_);
  const std::vector<std::uint64_t> train{0U, 1U};
  const std::vector<std::uint64_t> validation{2U};
  EXPECT_NO_THROW(validateExtrinsicSplit(fixture, train, validation));
  EXPECT_THROW(validateExtrinsicSplit(fixture, train, train), IoError);
  const std::vector<std::uint64_t> missing{99U};
  EXPECT_THROW(validateExtrinsicSplit(fixture, train, missing), IoError);
  fixture.frames[2].scene_identity = fixture.frames[0].scene_identity;
  EXPECT_THROW(validateExtrinsicSplit(fixture, train, validation), IoError);
}

TEST_F(ExtrinsicFixtureTest, RejectsDuplicateJsonKeysAndMalformedSyntax) {
  generateExtrinsicFixture(dataset_, {});
  std::ofstream(dataset_ / "fixture.json") << "{\"schema_name\":1,\"schema_name\":2}";
  try {
    static_cast<void>(loadExtrinsicFixture(dataset_));
    FAIL() << "duplicate keys should fail";
  } catch (const IoError& error) {
    EXPECT_NE(std::string(error.what()).find("duplicate JSON"), std::string::npos);
  }
  std::ofstream(dataset_ / "fixture.json") << "{broken";
  EXPECT_THROW(static_cast<void>(loadExtrinsicFixture(dataset_)), IoError);
}

TEST_F(ExtrinsicFixtureTest, RequiresExplicitOverwriteAndPreservesUnclaimedStaging) {
  generateExtrinsicFixture(dataset_, {});
  EXPECT_THROW(generateExtrinsicFixture(dataset_, {}), IoError);
  EXPECT_NO_THROW(generateExtrinsicFixture(dataset_, {}, true));
  const auto staging = std::filesystem::path(dataset_.string() + ".tmp");
  std::filesystem::create_directory(staging);
  std::ofstream(staging / "owner-marker") << "owned elsewhere";
  EXPECT_THROW(generateExtrinsicFixture(dataset_, {}, true), IoError);
  EXPECT_TRUE(std::filesystem::is_regular_file(staging / "owner-marker"));
  EXPECT_TRUE(std::filesystem::is_regular_file(dataset_ / "fixture.json"));
}

TEST_F(ExtrinsicFixtureTest, DoesNotOverwriteAnUnrelatedNonemptyDirectory) {
  std::filesystem::create_directory(dataset_);
  std::ofstream(dataset_ / "owner-marker") << "not an authored fixture";
  EXPECT_THROW(generateExtrinsicFixture(dataset_, {}, true), IoError);
  EXPECT_TRUE(std::filesystem::is_regular_file(dataset_ / "owner-marker"));
}

TEST_F(ExtrinsicFixtureTest, SceneIdentityMustAgreeWithLoadedPointGeometry) {
  generateExtrinsicFixture(dataset_, {});
  auto fixture = loadExtrinsicFixture(dataset_);
  fixture.frames[0].scene_identity = "not-the-point-geometry";
  const std::vector<std::uint64_t> ids{0U};
  EXPECT_THROW(static_cast<void>(loadExtrinsicObservationFrames(fixture, ids)), IoError);
}

TEST_F(ExtrinsicFixtureTest, RejectsUnsupportedVersionUnsafePathsAndOversizedCounts) {
  generateExtrinsicFixture(dataset_, {});
  const auto original = read(dataset_ / "fixture.json");
  auto changed = original;
  changed["schema_version"] = "99.0.0";
  write(dataset_ / "fixture.json", changed);
  EXPECT_THROW(static_cast<void>(loadExtrinsicFixture(dataset_)), IoError);
  write(dataset_ / "fixture.json", original);
  auto fixture = loadExtrinsicFixture(dataset_);
  const std::vector<std::uint64_t> ids{0U};
  fixture.frames[0].points_file = "../outside.json";
  EXPECT_THROW(static_cast<void>(loadExtrinsicObservationFrames(fixture, ids)), IoError);
  changed = original;
  changed["frames"][0]["generation_counts"]["input_points"] = 100001U;
  write(dataset_ / "fixture.json", changed);
  EXPECT_THROW(static_cast<void>(loadExtrinsicFixture(dataset_)), IoError);
}

TEST_F(ExtrinsicFixtureTest, RejectsMutatedCleanOracleTargets) {
  generateExtrinsicFixture(dataset_, {});
  const auto fixture = loadExtrinsicFixture(dataset_);
  auto oracle = read(dataset_ / "oracle.json");
  oracle["pixels"][0]["u_px"] = 1.0;
  write(dataset_ / "oracle.json", oracle);
  EXPECT_THROW(static_cast<void>(loadExtrinsicOracle(fixture)), IoError);
}

} // namespace
} // namespace sfr::io
