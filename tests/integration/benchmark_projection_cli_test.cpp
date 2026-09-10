#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>
#include <string_view>
#include <system_error>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include <sys/wait.h>

#ifndef SFR_BENCHMARK_PROJECTION_EXE
#error "SFR_BENCHMARK_PROJECTION_EXE must name the benchmark_projection executable"
#endif

namespace {

class BenchmarkProjectionCliTest : public ::testing::Test {
protected:
  void SetUp() override {
    root_ = std::filesystem::path(::testing::TempDir()) / "sfr_benchmark_projection_cli_test";
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

  std::filesystem::path root_;
};

TEST_F(BenchmarkProjectionCliTest, WritesVerifiedSyntheticSmokeReport) {
  const std::filesystem::path report_path = root_ / "report.json";
  const std::string command = shellQuote(SFR_BENCHMARK_PROJECTION_EXE) + " --smoke --output-json " +
                              shellQuote(report_path.string());

  ASSERT_EQ(exitCode(std::system(command.c_str())), 0);
  ASSERT_TRUE(std::filesystem::is_regular_file(report_path));
  EXPECT_FALSE(std::filesystem::exists(report_path.string() + ".tmp"));

  nlohmann::json report;
  std::ifstream(report_path) >> report;
  EXPECT_EQ(report.at("schema_version"), "1.1.0");
  EXPECT_EQ(report.at("benchmark"), "projection_pipeline");
  EXPECT_EQ(report.at("percentile_method"), "nearest_rank");
  EXPECT_EQ(report.at("measurement").at("clock"), "std::chrono::steady_clock");
  EXPECT_EQ(report.at("synthetic_fixture").at("id"), "authored_projection_benchmark_v1");
  EXPECT_EQ(report.at("synthetic_fixture").at("seed_hex"), "0x123456789abcdef0");
  EXPECT_EQ(report.at("config").at("points_per_sample"), 1'000U);
  EXPECT_EQ(report.at("config").at("warmup_iterations"), 1U);
  EXPECT_EQ(report.at("config").at("measured_iterations"), 2U);
  EXPECT_EQ(report.at("latency_ms").at("count"), 2U);
  EXPECT_EQ(report.at("counts").at("visible_points"), 700U);
  EXPECT_EQ(report.at("counts").at("outside_image"), 150U);
  EXPECT_EQ(report.at("counts").at("behind_or_too_near"), 150U);
  EXPECT_EQ(report.at("counts").at("input_points"), 1'000U);
  EXPECT_GT(report.at("throughput_points_per_sec").at("aggregate").get<double>(), 0.0);
  EXPECT_GT(report.at("throughput_points_per_sec").at("at_p50_latency").get<double>(), 0.0);
  EXPECT_FALSE(report.at("throughput_points_per_sec").contains("mean"));
  EXPECT_FALSE(report.at("throughput_points_per_sec").contains("p50"));
  EXPECT_TRUE(report.at("verification").at("all_warmup_iterations_accounted"));
  EXPECT_TRUE(report.at("verification").at("all_measured_iterations_accounted"));
  EXPECT_LE(report.at("latency_ms").at("min").get<double>(),
            report.at("latency_ms").at("p50").get<double>());
  EXPECT_LE(report.at("latency_ms").at("p50").get<double>(),
            report.at("latency_ms").at("p95").get<double>());
  EXPECT_LE(report.at("latency_ms").at("p95").get<double>(),
            report.at("latency_ms").at("p99").get<double>());
  EXPECT_LE(report.at("latency_ms").at("p99").get<double>(),
            report.at("latency_ms").at("max").get<double>());
}

TEST_F(BenchmarkProjectionCliTest, RefusesExistingReportWithoutOverwrite) {
  const std::filesystem::path report_path = root_ / "report.json";
  const std::string command = shellQuote(SFR_BENCHMARK_PROJECTION_EXE) + " --smoke --output-json " +
                              shellQuote(report_path.string());
  ASSERT_EQ(exitCode(std::system(command.c_str())), 0);
  EXPECT_EQ(exitCode(std::system((command + " 2>/dev/null").c_str())), 5);
  EXPECT_EQ(exitCode(std::system((command + " --overwrite").c_str())), 0);
}

TEST_F(BenchmarkProjectionCliTest, PreservesPreexistingTemporaryDirectory) {
  const std::filesystem::path report_path = root_ / "report.json";
  const std::filesystem::path temporary_directory = root_ / "report.json.tmp";
  ASSERT_TRUE(std::filesystem::create_directory(temporary_directory));
  std::ofstream(temporary_directory / "owner-marker.txt") << "not owned by this writer";
  const std::string command = shellQuote(SFR_BENCHMARK_PROJECTION_EXE) + " --smoke --output-json " +
                              shellQuote(report_path.string()) + " 2>/dev/null";

  EXPECT_EQ(exitCode(std::system(command.c_str())), 5);
  EXPECT_TRUE(std::filesystem::is_regular_file(temporary_directory / "owner-marker.txt"));
  EXPECT_FALSE(std::filesystem::exists(report_path));
}

TEST_F(BenchmarkProjectionCliTest, UsesDocumentedArgumentAndOutputExitCategories) {
  const std::string invalid =
      shellQuote(SFR_BENCHMARK_PROJECTION_EXE) + " --smoke --points 1000 2>/dev/null";
  EXPECT_EQ(exitCode(std::system(invalid.c_str())), 2);

  std::ofstream(root_ / "not-a-directory") << "regular file";
  const std::string output_failure =
      shellQuote(SFR_BENCHMARK_PROJECTION_EXE) + " --smoke --output-json " +
      shellQuote((root_ / "not-a-directory/report.json").string()) + " 2>/dev/null";
  EXPECT_EQ(exitCode(std::system(output_failure.c_str())), 5);
}

} // namespace
