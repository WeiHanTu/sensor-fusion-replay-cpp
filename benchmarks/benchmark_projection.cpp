#include <algorithm>
#include <array>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include "sfr/core/build_info.hpp"
#include "sfr/core/point_types.hpp"
#include "sfr/geometry/point_cloud_projection.hpp"
#include "sfr/geometry/projection.hpp"
#include "sfr/geometry/rigid_transform.hpp"

namespace {

using Json = nlohmann::json;

constexpr int kExitSuccess = 0;
constexpr int kExitInvalidArguments = 2;
constexpr int kExitProcessingFailure = 4;
constexpr int kExitOutputFailure = 5;

constexpr std::uint64_t kDefaultPointCount = 100'000U;
constexpr std::size_t kDefaultWarmupIterations = 10U;
constexpr std::size_t kDefaultMeasuredIterations = 100U;
constexpr std::uint64_t kMaximumPointCount = 10'000'000U;
constexpr std::size_t kMaximumIterations = 10'000U;
constexpr std::uint64_t kGeneratorSeed = 0x123456789ABCDEF0ULL;
constexpr std::uint64_t kVisiblePercent = 70U;
constexpr std::uint64_t kOutsidePercent = 15U;
constexpr std::string_view kFixtureId = "authored_projection_benchmark_v1";

struct BenchmarkOptions final {
  std::uint64_t point_count{kDefaultPointCount};
  std::size_t warmup_iterations{kDefaultWarmupIterations};
  std::size_t measured_iterations{kDefaultMeasuredIterations};
  std::filesystem::path output_json{};
  bool smoke_mode{false};
  bool overwrite{false};
};

struct LatencyStats final {
  std::size_t count{0U};
  double min_ms{0.0};
  double max_ms{0.0};
  double mean_ms{0.0};
  double p50_ms{0.0};
  double p95_ms{0.0};
  double p99_ms{0.0};
};

struct NumericArgument final {
  std::string_view text;
  std::string_view option;
};

struct SyntheticFixture final {
  sfr::geometry::RigidTransform T_camera_rect_00_lidar;
  sfr::geometry::RectifiedProjection projection;
};

class ArgumentError final : public std::runtime_error {
public:
  explicit ArgumentError(const std::string& message) : std::runtime_error(message) {}
};

class OutputError final : public std::runtime_error {
public:
  explicit OutputError(const std::string& message) : std::runtime_error(message) {}
};

class TemporaryReportDirectory final {
public:
  explicit TemporaryReportDirectory(std::filesystem::path path) : path_(std::move(path)) {
    try {
      if (!std::filesystem::create_directory(path_)) {
        throw OutputError("temporary benchmark directory already exists: " + path_.string());
      }
    } catch (const std::filesystem::filesystem_error& error) {
      throw OutputError(std::string("unable to claim temporary benchmark directory: ") +
                        error.what());
    }
  }

  TemporaryReportDirectory(const TemporaryReportDirectory&) = delete;
  TemporaryReportDirectory& operator=(const TemporaryReportDirectory&) = delete;
  TemporaryReportDirectory(TemporaryReportDirectory&&) = delete;
  TemporaryReportDirectory& operator=(TemporaryReportDirectory&&) = delete;

  ~TemporaryReportDirectory() {
    std::error_code error;
    std::filesystem::remove_all(path_, error);
  }

private:
  std::filesystem::path path_;
};

void printHelp(std::ostream& output) {
  output << "Usage: benchmark_projection [options]\n\n"
            "Options (defaults):\n"
            "  --points <n>       Synthetic finite LiDAR points per iteration ("
         << kDefaultPointCount
         << ")\n"
            "  --warmup <n>       Unmeasured warm-up iterations ("
         << kDefaultWarmupIterations
         << ")\n"
            "  --iterations <n>   Measured iterations ("
         << kDefaultMeasuredIterations
         << ")\n"
            "  --output-json <p>  Write a machine-readable benchmark report atomically\n"
            "  --overwrite        Replace only the exact output JSON path\n"
            "  --smoke            Use 1,000 points, 1 warm-up, and 2 measured iterations\n"
            "  --help             Show this help message\n\n"
            "Non-smoke runs require an optimized build. A release report additionally\n"
            "requires a clean Git tree. Smoke mode may run from a dirty debug build.\n";
}

[[nodiscard]] std::uint64_t parseUnsigned(const NumericArgument& argument) {
  std::uint64_t value = 0U;
  const auto [end, error] =
      std::from_chars(argument.text.data(), argument.text.data() + argument.text.size(), value);
  if (argument.text.empty() || error != std::errc{} ||
      end != argument.text.data() + argument.text.size()) {
    throw ArgumentError(std::string(argument.option) + " requires an unsigned integer");
  }
  return value;
}

[[nodiscard]] std::size_t parseSize(const NumericArgument& argument) {
  const std::uint64_t value = parseUnsigned(argument);
  if (value > std::numeric_limits<std::size_t>::max()) {
    throw ArgumentError(std::string(argument.option) + " exceeds this platform's size range");
  }
  return static_cast<std::size_t>(value);
}

[[nodiscard]] BenchmarkOptions parseArguments(int argc, char** argv) {
  BenchmarkOptions options;
  bool points_set = false;
  bool warmup_set = false;
  bool iterations_set = false;
  bool output_set = false;
  bool smoke_set = false;
  bool overwrite_set = false;
  const auto requireValue = [&](int& index, const std::string_view option) -> std::string_view {
    if (index + 1 >= argc) {
      throw ArgumentError(std::string(option) + " requires a value");
    }
    ++index;
    return argv[index];
  };

  for (int index = 1; index < argc; ++index) {
    const std::string_view argument(argv[index]);
    if (argument == "--help") {
      printHelp(std::cout);
      throw ArgumentError("__help_shown__");
    }
    if (argument == "--smoke") {
      if (smoke_set) {
        throw ArgumentError("--smoke may be specified only once");
      }
      options.smoke_mode = true;
      smoke_set = true;
      continue;
    }
    if (argument == "--overwrite") {
      if (overwrite_set) {
        throw ArgumentError("--overwrite may be specified only once");
      }
      options.overwrite = true;
      overwrite_set = true;
      continue;
    }
    if (argument == "--points") {
      if (points_set) {
        throw ArgumentError("--points may be specified only once");
      }
      options.point_count = parseUnsigned({requireValue(index, argument), argument});
      points_set = true;
    } else if (argument == "--warmup") {
      if (warmup_set) {
        throw ArgumentError("--warmup may be specified only once");
      }
      options.warmup_iterations = parseSize({requireValue(index, argument), argument});
      warmup_set = true;
    } else if (argument == "--iterations") {
      if (iterations_set) {
        throw ArgumentError("--iterations may be specified only once");
      }
      options.measured_iterations = parseSize({requireValue(index, argument), argument});
      iterations_set = true;
    } else if (argument == "--output-json") {
      if (output_set) {
        throw ArgumentError("--output-json may be specified only once");
      }
      options.output_json = requireValue(index, argument);
      output_set = true;
    } else {
      throw ArgumentError("unknown argument: " + std::string(argument));
    }
  }

  if (options.smoke_mode && (points_set || warmup_set || iterations_set)) {
    throw ArgumentError("--smoke cannot be combined with size or iteration overrides");
  }
  if (options.smoke_mode) {
    options.point_count = 1'000U;
    options.warmup_iterations = 1U;
    options.measured_iterations = 2U;
  }
  if (output_set && options.output_json.empty()) {
    throw ArgumentError("--output-json requires a nonempty path");
  }
  if (options.point_count == 0U || options.point_count > kMaximumPointCount ||
      options.measured_iterations == 0U || options.measured_iterations > kMaximumIterations ||
      options.warmup_iterations > kMaximumIterations) {
    throw ArgumentError("points must be in [1,10000000], measured iterations in [1,10000], "
                        "and warm-up iterations in [0,10000]");
  }
  return options;
}

[[nodiscard]] sfr::geometry::ProjectionCounts expectedCounts(const std::uint64_t count) {
  const std::uint64_t visible = count * kVisiblePercent / 100U;
  const std::uint64_t outside = count * kOutsidePercent / 100U;
  return {.input_points = count,
          .visible_points = visible,
          .non_finite_input = 0U,
          .behind_or_too_near = count - visible - outside,
          .non_positive_homogeneous_depth = 0U,
          .outside_image = outside};
}

[[nodiscard]] SyntheticFixture makeSyntheticFixture() {
  sfr::geometry::Matrix3d R_camera_rect_00_lidar;
  R_camera_rect_00_lidar << 0.0, -1.0, 0.0, 0.0, 0.0, -1.0, 1.0, 0.0, 0.0;
  const sfr::geometry::RigidTransform T_camera_rect_00_lidar({
      .target_frame = sfr::geometry::FrameId("camera_rect_00"),
      .source_frame = sfr::geometry::FrameId("lidar"),
      .rotation_target_source = R_camera_rect_00_lidar,
      .translation_target_source_m = sfr::geometry::Vector3d::Zero(),
  });

  sfr::geometry::Matrix34d P_image_camera_rect_00;
  P_image_camera_rect_00 << 700.0, 0.0, 620.0, 35.0, 0.0, 700.0, 188.0, 0.25, 0.0, 0.0, 1.0, 0.0;
  return {
      T_camera_rect_00_lidar,
      sfr::geometry::RectifiedProjection({.P_image_camera_rect_00 = P_image_camera_rect_00,
                                          .image_width_px = 1240,
                                          .image_height_px = 376,
                                          .z_min_m = 0.1}),
  };
}

[[nodiscard]] std::vector<sfr::core::PointXYZI>
generateDeterministicCloud(const std::uint64_t count) {
  const sfr::geometry::ProjectionCounts expected = expectedCounts(count);
  std::vector<sfr::core::PointXYZI> points;
  points.reserve(static_cast<std::size_t>(count));
  std::uint64_t state = kGeneratorSeed;
  const auto nextRandom = [&state]() -> double {
    state = state * 6364136223846793005ULL + 1442695040888963407ULL;
    return static_cast<double>(state >> 11U) * (1.0 / 9007199254740992.0);
  };
  const auto lidarPointForPixel = [](const double u_px, const double v_px, const double depth_m,
                                     const float reflectance) {
    const double camera_x_m = (u_px * depth_m - 620.0 * depth_m - 35.0) / 700.0;
    const double camera_y_m = (v_px * depth_m - 188.0 * depth_m - 0.25) / 700.0;
    return sfr::core::PointXYZI{depth_m, -camera_x_m, -camera_y_m, reflectance};
  };

  for (std::uint64_t index = 0U; index < count; ++index) {
    const float reflectance = static_cast<float>(nextRandom());
    if (index < expected.visible_points) {
      const double depth_m = 3.0 + nextRandom() * 55.0;
      const double u_px = 100.0 + nextRandom() * 1040.0;
      const double v_px = 40.0 + nextRandom() * 296.0;
      points.push_back(lidarPointForPixel(u_px, v_px, depth_m, reflectance));
    } else if (index < expected.visible_points + expected.outside_image) {
      const double depth_m = 3.0 + nextRandom() * 55.0;
      const double u_px = index % 2U == 0U ? -50.0 : 1290.0;
      const double v_px = 40.0 + nextRandom() * 296.0;
      points.push_back(lidarPointForPixel(u_px, v_px, depth_m, reflectance));
    } else {
      const double camera_depth_m = index % 2U == 0U ? -1.0 - nextRandom() * 29.0 : 0.05;
      points.push_back({camera_depth_m, nextRandom() - 0.5, nextRandom() - 0.5, reflectance});
    }
  }
  return points;
}

[[nodiscard]] bool equalCounts(const sfr::geometry::ProjectionCounts& first,
                               const sfr::geometry::ProjectionCounts& second) {
  return first.input_points == second.input_points &&
         first.visible_points == second.visible_points &&
         first.non_finite_input == second.non_finite_input &&
         first.behind_or_too_near == second.behind_or_too_near &&
         first.non_positive_homogeneous_depth == second.non_positive_homogeneous_depth &&
         first.outside_image == second.outside_image;
}

void validateCounts(const sfr::geometry::ProjectionCounts& actual,
                    const sfr::geometry::ProjectionCounts& expected) {
  const std::array categories{actual.visible_points, actual.non_finite_input,
                              actual.behind_or_too_near, actual.non_positive_homogeneous_depth,
                              actual.outside_image};
  std::uint64_t accounted = 0U;
  for (const std::uint64_t count : categories) {
    if (count > std::numeric_limits<std::uint64_t>::max() - accounted) {
      throw std::runtime_error("projection accounting overflow");
    }
    accounted += count;
  }
  if (accounted != actual.input_points || !equalCounts(actual, expected)) {
    throw std::runtime_error("projection result violates the authored benchmark accounting");
  }
}

[[nodiscard]] double nearestRank(const std::vector<double>& sorted_values,
                                 const double percentile) {
  const double rank = std::ceil(percentile * static_cast<double>(sorted_values.size()));
  const auto one_based_rank = std::clamp(static_cast<std::size_t>(std::max(1.0, rank)),
                                         std::size_t{1}, sorted_values.size());
  return sorted_values[one_based_rank - 1U];
}

[[nodiscard]] LatencyStats computeStats(std::vector<double> samples_ms) {
  if (samples_ms.empty()) {
    throw std::runtime_error("benchmark requires at least one measured sample");
  }
  std::ranges::sort(samples_ms);
  double sum = 0.0;
  for (const double value : samples_ms) {
    if (!std::isfinite(value) || value <= 0.0) {
      throw std::runtime_error("benchmark clock produced a non-positive or non-finite duration");
    }
    sum += value;
  }
  return {.count = samples_ms.size(),
          .min_ms = samples_ms.front(),
          .max_ms = samples_ms.back(),
          .mean_ms = sum / static_cast<double>(samples_ms.size()),
          .p50_ms = nearestRank(samples_ms, 0.50),
          .p95_ms = nearestRank(samples_ms, 0.95),
          .p99_ms = nearestRank(samples_ms, 0.99)};
}

[[nodiscard]] Json matrixJson(const sfr::geometry::Matrix34d& matrix) {
  Json rows = Json::array();
  for (Eigen::Index row = 0; row < matrix.rows(); ++row) {
    Json values = Json::array();
    for (Eigen::Index column = 0; column < matrix.cols(); ++column) {
      values.push_back(matrix(row, column));
    }
    rows.push_back(std::move(values));
  }
  return rows;
}

[[nodiscard]] std::string seedHex() {
  std::ostringstream stream;
  stream << "0x" << std::hex << kGeneratorSeed;
  return stream.str();
}

void writeReportAtomically(const std::filesystem::path& output_path, const Json& report,
                           const bool overwrite) {
  try {
    if (output_path.has_parent_path()) {
      std::filesystem::create_directories(output_path.parent_path());
    }
  } catch (const std::filesystem::filesystem_error& error) {
    throw OutputError(std::string("unable to prepare benchmark output: ") + error.what());
  }

  std::filesystem::path temporary_directory = output_path;
  temporary_directory += ".tmp";
  TemporaryReportDirectory cleanup(temporary_directory);
  try {
    if (std::filesystem::exists(output_path) &&
        (std::filesystem::is_directory(output_path) || !overwrite)) {
      throw OutputError("output JSON exists; pass --overwrite to replace the exact file: " +
                        output_path.string());
    }
    const std::filesystem::path temporary_path = temporary_directory / "report.json";
    std::ofstream output;
    output.exceptions(std::ios::failbit | std::ios::badbit);
    output.open(temporary_path);
    output << report.dump(2) << '\n';
    output.close();
    std::filesystem::rename(temporary_path, output_path);
  } catch (const std::filesystem::filesystem_error& error) {
    throw OutputError(std::string("unable to publish benchmark report: ") + error.what());
  } catch (const std::ios_base::failure& error) {
    throw OutputError(std::string("unable to write benchmark report: ") + error.what());
  }
}

[[nodiscard]] Json makeReport(const BenchmarkOptions& options, const SyntheticFixture& fixture,
                              const LatencyStats& stats,
                              const sfr::geometry::ProjectionCounts& counts,
                              const double mean_throughput_points_per_sec,
                              const double p50_throughput_points_per_sec) {
  return {
      {"schema_version", "1.1.0"},
      {"benchmark", "projection_pipeline"},
      {"percentile_method", "nearest_rank"},
      {"measurement",
       {{"clock", "std::chrono::steady_clock"},
        {"scope", "SE(3) point transform, rectified projection, classification, and output "
                  "vector construction"},
        {"excluded",
         {"fixture_generation", "image_decode", "visualization", "serialization", "queue_wait",
          "replay_pacing"}}}},
      {"build",
       {{"git_commit", std::string(sfr::core::kBuildInfo.git_commit)},
        {"git_dirty", sfr::core::kBuildInfo.git_dirty},
        {"build_type", std::string(sfr::core::kBuildInfo.build_type)},
        {"compiler", std::string(sfr::core::kBuildInfo.compiler)}}},
      {"host",
       {{"os", std::string(sfr::core::kBuildInfo.host_os)},
        {"arch", std::string(sfr::core::kBuildInfo.host_arch)},
        {"cpu", std::string(sfr::core::kBuildInfo.host_cpu)}}},
      {"config",
       {{"points_per_sample", options.point_count},
        {"warmup_iterations", options.warmup_iterations},
        {"measured_iterations", options.measured_iterations},
        {"z_min_m", fixture.projection.minimumDepthMeters()},
        {"image_width_px", fixture.projection.imageWidthPixels()},
        {"image_height_px", fixture.projection.imageHeightPixels()}}},
      {"synthetic_fixture",
       {{"id", kFixtureId},
        {"provenance", "authored deterministic synthetic geometry; no KITTI data"},
        {"generator", "64-bit LCG with exact index-partitioned terminal categories"},
        {"seed_hex", seedHex()},
        {"visible_percent", kVisiblePercent},
        {"outside_image_percent", kOutsidePercent},
        {"behind_or_too_near_percent", 100U - kVisiblePercent - kOutsidePercent},
        {"T_camera_rect_00_lidar",
         {{"rotation_row_major", {{0.0, -1.0, 0.0}, {0.0, 0.0, -1.0}, {1.0, 0.0, 0.0}}},
          {"translation_m", {0.0, 0.0, 0.0}}}},
        {"P_image_camera_rect_00", matrixJson(fixture.projection.matrix())}}},
      {"latency_ms",
       {{"count", stats.count},
        {"min", stats.min_ms},
        {"max", stats.max_ms},
        {"mean", stats.mean_ms},
        {"p50", stats.p50_ms},
        {"p95", stats.p95_ms},
        {"p99", stats.p99_ms}}},
      {"throughput_points_per_sec",
       {{"mean", mean_throughput_points_per_sec}, {"p50", p50_throughput_points_per_sec}}},
      {"counts",
       {{"input_points", counts.input_points},
        {"visible_points", counts.visible_points},
        {"non_finite_input", counts.non_finite_input},
        {"behind_or_too_near", counts.behind_or_too_near},
        {"non_positive_homogeneous_depth", counts.non_positive_homogeneous_depth},
        {"outside_image", counts.outside_image}}},
      {"verification",
       {{"all_warmup_iterations_accounted", true},
        {"all_measured_iterations_accounted", true},
        {"verified_warmup_iterations", options.warmup_iterations},
        {"verified_measured_iterations", options.measured_iterations}}},
  };
}

[[nodiscard]] int run(const BenchmarkOptions& options) {
  const bool optimized_build = sfr::core::kBuildInfo.build_type == "Release" ||
                               sfr::core::kBuildInfo.build_type == "RelWithDebInfo";
  if (!options.smoke_mode && !optimized_build) {
    throw ArgumentError("non-smoke benchmark runs require Release or RelWithDebInfo");
  }
  if (!options.smoke_mode && !options.output_json.empty() && sfr::core::kBuildInfo.git_dirty) {
    throw ArgumentError("release benchmark reports require a clean Git tree");
  }

  const SyntheticFixture fixture = makeSyntheticFixture();
  const sfr::geometry::ProjectionCounts expected = expectedCounts(options.point_count);
  const std::vector<sfr::core::PointXYZI> point_cloud =
      generateDeterministicCloud(options.point_count);

  for (std::size_t iteration = 0; iteration < options.warmup_iterations; ++iteration) {
    const sfr::geometry::PointCloudProjectionResult result = sfr::geometry::projectPointCloud(
        point_cloud, fixture.T_camera_rect_00_lidar, fixture.projection);
    validateCounts(result.counts, expected);
  }

  std::vector<double> sample_durations_ms;
  sample_durations_ms.reserve(options.measured_iterations);
  for (std::size_t iteration = 0; iteration < options.measured_iterations; ++iteration) {
    const auto start = std::chrono::steady_clock::now();
    const sfr::geometry::PointCloudProjectionResult result = sfr::geometry::projectPointCloud(
        point_cloud, fixture.T_camera_rect_00_lidar, fixture.projection);
    const double elapsed_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    sample_durations_ms.push_back(elapsed_ms);
    validateCounts(result.counts, expected);
  }

  const LatencyStats stats = computeStats(std::move(sample_durations_ms));
  const double mean_throughput_points_per_sec =
      static_cast<double>(options.point_count) * 1000.0 / stats.mean_ms;
  const double p50_throughput_points_per_sec =
      static_cast<double>(options.point_count) * 1000.0 / stats.p50_ms;
  const Json report = makeReport(options, fixture, stats, expected, mean_throughput_points_per_sec,
                                 p50_throughput_points_per_sec);

  std::cout << "SFR projection microbenchmark\n"
            << "commit: " << sfr::core::kBuildInfo.git_commit
            << (sfr::core::kBuildInfo.git_dirty ? " (dirty)" : " (clean)") << '\n'
            << "build: " << sfr::core::kBuildInfo.build_type << " | "
            << sfr::core::kBuildInfo.compiler << '\n'
            << "host: " << sfr::core::kBuildInfo.host_os << " | " << sfr::core::kBuildInfo.host_arch
            << " | " << sfr::core::kBuildInfo.host_cpu << '\n'
            << "samples: " << options.measured_iterations << " measured after "
            << options.warmup_iterations << " warm-up; " << options.point_count
            << " points/sample\n"
            << std::fixed << std::setprecision(3) << "latency ms: min=" << stats.min_ms
            << " mean=" << stats.mean_ms << " p50=" << stats.p50_ms << " p95=" << stats.p95_ms
            << " p99=" << stats.p99_ms << " max=" << stats.max_ms << '\n'
            << std::setprecision(0)
            << "throughput points/s: mean=" << mean_throughput_points_per_sec
            << " p50=" << p50_throughput_points_per_sec << '\n'
            << "counts: visible=" << expected.visible_points
            << " behind_or_too_near=" << expected.behind_or_too_near
            << " outside_image=" << expected.outside_image << " input=" << expected.input_points
            << '\n';

  if (!options.output_json.empty()) {
    writeReportAtomically(options.output_json, report, options.overwrite);
    std::cout << "report: " << options.output_json.string() << '\n';
  }
  return kExitSuccess;
}

} // namespace

int main(int argc, char** argv) {
  try {
    return run(parseArguments(argc, argv));
  } catch (const ArgumentError& error) {
    if (std::string_view(error.what()) == "__help_shown__") {
      return kExitSuccess;
    }
    std::cerr << "argument error: " << error.what() << '\n';
    return kExitInvalidArguments;
  } catch (const OutputError& error) {
    std::cerr << "output error: " << error.what() << '\n';
    return kExitOutputFailure;
  } catch (const std::exception& error) {
    std::cerr << "processing error: " << error.what() << '\n';
    return kExitProcessingFailure;
  }
}
