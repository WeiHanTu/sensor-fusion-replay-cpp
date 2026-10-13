#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <numbers>
#include <string>
#include <string_view>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
#include <opencv2/imgcodecs.hpp>
#include <sys/wait.h>

#include "sfr/io/extrinsic_fixture.hpp"

namespace {
using Json = nlohmann::json;

[[nodiscard]] std::string quote(const std::string_view value) {
  std::string result = "'";
  for (const char c : value) {
    result += c == '\'' ? "'\\''" : std::string(1, c);
  }
  return result + "'";
}
[[nodiscard]] Json read(const std::filesystem::path& path) {
  Json value;
  std::ifstream(path) >> value;
  return value;
}
class EstimateExtrinsicsCliTest : public ::testing::Test {
protected:
  void SetUp() override {
    root_ = std::filesystem::path(::testing::TempDir()) /
            ("sfr_recovery_cli_" +
             std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    claimed_ = std::filesystem::create_directory(root_);
    ASSERT_TRUE(claimed_);
    dataset_ = root_ / "dataset";
    output_ = root_ / "output";
    sfr::io::ExtrinsicFixtureConfig config;
    config.hidden = {0.7 * std::numbers::pi / 180.0, -0.04};
    sfr::io::generateExtrinsicFixture(dataset_, config);
  }
  void TearDown() override {
    if (claimed_) {
      std::error_code error;
      std::filesystem::remove_all(root_, error);
    }
  }
  [[nodiscard]] int run(const std::string& extra = "") const {
    const auto command = quote(SFR_ESTIMATE_EXTRINSICS_EXE) + " --dataset-root " +
                         quote(dataset_.string()) + " --output-dir " + quote(output_.string()) +
                         " --run-id run " + extra + " > " + quote((root_ / "stdout.txt").string()) +
                         " 2> " + quote((root_ / "stderr.txt").string());
    const int result = std::system(command.c_str());
    return result != -1 && WIFEXITED(result) ? WEXITSTATUS(result) : -1;
  }
  std::filesystem::path root_, dataset_, output_;
  bool claimed_{false};
};

TEST_F(EstimateExtrinsicsCliTest, PublishesCompleteKeyedReportWithHeldOutEvidenceAndImages) {
  ASSERT_EQ(run(), 0);
  const auto report = read(output_ / "run/report.json");
  EXPECT_EQ(report.at("schema_name"), "sfr.extrinsic_recovery");
  EXPECT_EQ(report.at("schema_version"), "1.0.0");
  EXPECT_EQ(report.at("status"), "complete");
  EXPECT_TRUE(report.at("acceptance").at("passed").get<bool>());
  EXPECT_EQ(report.at("split").at("consumed_train_ids"), Json::array({0, 1}));
  EXPECT_EQ(report.at("split").at("validation_frame_ids"), Json::array({2}));
  EXPECT_TRUE(report.at("reproducibility").at("equal").get<bool>());
  EXPECT_EQ(report.at("build").at("git_commit").get<std::string>().size(), 40U);
  EXPECT_EQ(report.at("model").at("projection_matrix")[0][3], -52.0);
  EXPECT_EQ(report.at("search").at("result_status"), "grid_complete");
  EXPECT_EQ(report.at("search").at("points_evaluated"),
            report.at("search").at("evaluations").get<std::uint64_t>() * 372U);
  EXPECT_EQ(
      report.at("metrics").at("validation").at("clean").at("recovered").at("pooled").at("samples"),
      160U);
  const auto surface = read(output_ / "run/loss_surface.json");
  EXPECT_EQ(surface.at("rows").size(), report.at("search").at("evaluations").get<std::size_t>());
  const auto residuals = read(output_ / "run/residuals.json");
  EXPECT_EQ(residuals.at("train").at("clean").at("nominal")[0]["source_index"], 11U);
  for (const auto id : {0, 1, 2}) {
    for (const auto* model : {"nominal", "recovered", "oracle"}) {
      EXPECT_FALSE(
          cv::imread((output_ / "run/overlays" / std::to_string(id) / (std::string(model) + ".png"))
                         .string())
              .empty());
    }
  }
  EXPECT_FALSE(cv::imread((output_ / "run/loss_surface.png").string()).empty());
  EXPECT_FALSE(cv::imread((output_ / "run/residual_comparison.png").string()).empty());
  const auto background = cv::imread((dataset_ / "frames/2/image.png").string());
  const auto overlay = cv::imread((output_ / "run/overlays/2/nominal.png").string());
  // Keep model/role annotation below the fixture's own header, not on top of it.
  const cv::Rect model_header(0, 35, 960, 30);
  EXPECT_GT(cv::norm(overlay(model_header), background(model_header), cv::NORM_L1), 0.0);
  EXPECT_FALSE(std::filesystem::exists(output_ / "run.tmp"));
}

TEST_F(EstimateExtrinsicsCliTest, RefusesOverlapInvalidGridUnknownParametersAndDuplicateOptions) {
  EXPECT_EQ(run("--validation-frames 0"), 3);
  EXPECT_EQ(run("--yaw-fine-deg 0"), 2);
  EXPECT_EQ(run("--parameters full_se3"), 2);
  EXPECT_EQ(run("--loss bogus"), 2);
  EXPECT_EQ(run("--loss huber --loss l2"), 2);
  EXPECT_EQ(run("--train-frames 0,,1"), 2);
  EXPECT_EQ(run("--max-point-projections 1"), 2);
}

TEST_F(EstimateExtrinsicsCliTest, MissingPrivateInputHasActionableNonzeroExit) {
  dataset_ = root_ / "missing";
  EXPECT_EQ(run(), 3);
}

TEST_F(EstimateExtrinsicsCliTest, BoundLimitedPublishesUnacceptedResultAndExitFour) {
  ASSERT_EQ(run("--yaw-min-deg -0.5 --yaw-max-deg 0.5"), 4);
  const auto report = read(output_ / "run/report.json");
  EXPECT_EQ(report.at("search").at("result_status"), "bound_limited");
  EXPECT_FALSE(report.at("acceptance").at("passed").get<bool>());
  EXPECT_TRUE(report.at("search").at("global_bound_hits").at("yaw").get<bool>());
}

TEST_F(EstimateExtrinsicsCliTest, AllInvalidPreservesSurfaceAndNullRecoveredMetrics) {
  ASSERT_EQ(run("--yaw-min-deg 170 --yaw-max-deg 174"), 4);
  const auto report = read(output_ / "run/report.json");
  EXPECT_EQ(report.at("search").at("result_status"), "no_valid_candidate");
  EXPECT_TRUE(report.at("search").at("loss_sum").is_null());
  EXPECT_TRUE(report.at("model").at("recovered_parameters").is_null());
  EXPECT_FALSE(report.at("artifacts").at("recovered_available").get<bool>());
  const auto surface = read(output_ / "run/loss_surface.json");
  EXPECT_EQ(surface.at("rows").size(), 1681U);
  EXPECT_TRUE(surface.at("rows")[0].at("loss_sum").is_null());
  EXPECT_FALSE(std::filesystem::exists(output_ / "run/overlays/2/recovered.png"));
}

TEST_F(EstimateExtrinsicsCliTest, RequiresOverwriteAndPreservesPreexistingStage) {
  ASSERT_EQ(run(), 0);
  EXPECT_EQ(run(), 5);
  EXPECT_EQ(run("--overwrite"), 0);
  std::filesystem::create_directory(output_ / "run.tmp");
  std::ofstream(output_ / "run.tmp/owner") << "unowned stage";
  EXPECT_EQ(run("--overwrite"), 5);
  EXPECT_TRUE(std::filesystem::exists(output_ / "run.tmp/owner"));
  EXPECT_EQ(read(output_ / "run/report.json").at("status"), "complete");
}

TEST_F(EstimateExtrinsicsCliTest, RejectsOutputInsideFixtureAndUnrelatedOverwrite) {
  output_ = dataset_ / "output";
  EXPECT_EQ(run(), 5);
  output_ = root_ / "output";
  std::filesystem::create_directories(output_ / "run");
  std::ofstream(output_ / "run/owner") << "unrelated";
  EXPECT_EQ(run("--overwrite"), 5);
  EXPECT_TRUE(std::filesystem::exists(output_ / "run/owner"));
}

TEST_F(EstimateExtrinsicsCliTest, FrozenOffGridCaseReportsRefinementEdgeInsteadOfConvergence) {
  sfr::io::ExtrinsicFixtureConfig config;
  config.hidden = {0.733 * std::numbers::pi / 180.0, 0.0268};
  sfr::io::generateExtrinsicFixture(dataset_, config, true);
  ASSERT_EQ(run(), 4);
  const auto report = read(output_ / "run/report.json");
  EXPECT_EQ(report.at("search").at("result_status"), "refinement_limited");
  EXPECT_TRUE(report.at("search").at("refinement_edge_hit").get<bool>());
  EXPECT_FALSE(report.at("search").at("global_bound_hits").at("yaw").get<bool>());
  EXPECT_FALSE(report.at("search").at("global_bound_hits").at("translation").get<bool>());
  EXPECT_FALSE(report.at("acceptance").at("passed").get<bool>());
}

} // namespace
