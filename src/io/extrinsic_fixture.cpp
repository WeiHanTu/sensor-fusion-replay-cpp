#include "sfr/io/extrinsic_fixture.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <string_view>
#include <system_error>
#include <tuple>

#include <nlohmann/json.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "sfr/io/io_error.hpp"
#include "sfr/io/kitti_io.hpp"

namespace sfr::io {
namespace {

using Json = nlohmann::json;
using geometry::ObservedPixel;
constexpr std::uintmax_t kMaximumJsonBytes = std::uintmax_t{16U} * 1024U * 1024U;
constexpr std::size_t kMaximumRecords = 100'000U;
constexpr std::size_t kMaximumFrames = 1'000U;
constexpr std::string_view kFixtureVersion = "1.0.0";
constexpr std::string_view kGeneratorVersion = "authored-grid-splitmix64-v1";

[[noreturn]] void malformed(const std::string& message) {
  throw IoError(IoErrorCode::kInvalidCalibration, "synthetic extrinsic fixture: " + message);
}

[[nodiscard]] double finiteNumber(const Json& value) {
  if (!value.is_number()) {
    malformed("expected a finite number");
  }
  const double number = value.get<double>();
  if (!std::isfinite(number)) {
    malformed("non-finite number");
  }
  return number;
}

[[nodiscard]] std::uint64_t unsignedNumber(const Json& value) {
  if (!value.is_number_unsigned()) {
    malformed("ID/count must be an unsigned integer");
  }
  return value.get<std::uint64_t>();
}

void boundedArray(const Json& value, const std::size_t limit = kMaximumRecords) {
  if (!value.is_array() || value.empty() || value.size() > limit) {
    malformed("expected a nonempty bounded array");
  }
}

[[nodiscard]] std::string text(const Json& value) {
  if (!value.is_string() || value.get_ref<const std::string&>().empty()) {
    malformed("expected nonempty text");
  }
  return value.get<std::string>();
}

[[nodiscard]] std::string fingerprint(const Json& value) {
  // Versioned content checksum for fixture integrity, not cryptographic secrecy.
  std::uint64_t hash = 14695981039346656037ULL;
  for (const unsigned char byte : value.dump()) {
    hash ^= byte;
    hash *= 1099511628211ULL;
  }
  std::ostringstream result;
  result << "fnv1a64-json-v1-" << std::hex << std::setfill('0') << std::setw(16) << hash;
  return result.str();
}

[[nodiscard]] std::string sceneIdentity(std::vector<core::PointXYZI> points) {
  std::ranges::sort(points, [](const auto& first, const auto& second) {
    return std::tie(first.x_m, first.y_m, first.z_m, first.reflectance) <
           std::tie(second.x_m, second.y_m, second.z_m, second.reflectance);
  });
  Json scene = Json::array();
  for (const auto& point : points) {
    scene.push_back({point.x_m, point.y_m, point.z_m, point.reflectance});
  }
  return fingerprint(scene);
}

[[nodiscard]] Json readJson(const std::filesystem::path& file) {
  std::error_code error;
  const auto bytes = std::filesystem::file_size(file, error);
  if (error || bytes == 0U || bytes > kMaximumJsonBytes) {
    throw IoError(IoErrorCode::kInvalidFrameFile, "JSON file missing, empty or exceeds 16 MiB");
  }
  std::ifstream input(file);
  if (!input) {
    throw IoError(IoErrorCode::kFileOpen, "cannot open authored JSON file");
  }
  std::vector<std::set<std::string>> keys;
  const auto callback = [&keys](const int depth, const Json::parse_event_t event, Json& parsed) {
    if (depth > 32) {
      malformed("JSON nesting exceeds 32 levels");
    }
    if (event == Json::parse_event_t::object_start) {
      keys.emplace_back();
    } else if (event == Json::parse_event_t::key) {
      if (keys.empty() || !keys.back().insert(parsed.get<std::string>()).second) {
        malformed("duplicate JSON object key");
      }
    } else if (event == Json::parse_event_t::object_end) {
      keys.pop_back();
    }
    return true;
  };
  try {
    return Json::parse(input, callback);
  } catch (const Json::exception& parse_error) {
    malformed(std::string("invalid JSON: ") + parse_error.what());
  }
}

void schema(const Json& value, const std::string_view name) {
  if (value.at("schema_name") != name || value.at("schema_version") != kFixtureVersion) {
    malformed("unsupported schema name/version");
  }
}

[[nodiscard]] std::filesystem::path safePath(const std::filesystem::path& root,
                                             const std::string& relative) {
  const std::filesystem::path path(relative);
  if (path.empty() || path.is_absolute()) {
    malformed("fixture path must be relative");
  }
  for (const auto& component : path) {
    if (component == ".." || component == ".") {
      malformed("unsafe fixture path component");
    }
  }
  const auto resolved = std::filesystem::weakly_canonical(root / path);
  const auto within = resolved.lexically_relative(std::filesystem::weakly_canonical(root));
  if (within.empty() || *within.begin() == "..") {
    malformed("fixture path resolves outside its root");
  }
  return resolved;
}

[[nodiscard]] Json matrixJson(const geometry::RigidTransform& transform) {
  const geometry::Matrix4d matrix = transform.matrix();
  Json values = Json::array();
  for (Eigen::Index row = 0; row < 4; ++row) {
    for (Eigen::Index column = 0; column < 4; ++column) {
      values.push_back(matrix(row, column));
    }
  }
  return values;
}

[[nodiscard]] geometry::RigidTransform readTransform(const Json& values) {
  boundedArray(values);
  if (values.size() != 16U) {
    malformed("transform requires 16 row-major values");
  }
  geometry::Matrix4d matrix;
  for (Eigen::Index row = 0; row < 4; ++row) {
    for (Eigen::Index column = 0; column < 4; ++column) {
      matrix(row, column) = finiteNumber(values.at(static_cast<std::size_t>(row * 4 + column)));
    }
  }
  if (matrix(3, 0) != 0.0 || matrix(3, 1) != 0.0 || matrix(3, 2) != 0.0 || matrix(3, 3) != 1.0) {
    malformed("invalid homogeneous bottom row");
  }
  return geometry::RigidTransform({geometry::FrameId("camera_rect_00"), geometry::FrameId("lidar"),
                                   matrix.block<3, 3>(0, 0), matrix.block<3, 1>(0, 3)});
}

[[nodiscard]] Json countsJson(const geometry::ProjectionCounts& counts) {
  return Json{{"input", counts.input_points},
              {"visible", counts.visible_points},
              {"non_finite", counts.non_finite_input},
              {"behind_or_too_near", counts.behind_or_too_near},
              {"non_positive_homogeneous_depth", counts.non_positive_homogeneous_depth},
              {"outside_image", counts.outside_image}};
}

[[nodiscard]] geometry::ProjectionCounts readCounts(const Json& value) {
  geometry::ProjectionCounts counts{unsignedNumber(value.at("input")),
                                    unsignedNumber(value.at("visible")),
                                    unsignedNumber(value.at("non_finite")),
                                    unsignedNumber(value.at("behind_or_too_near")),
                                    unsignedNumber(value.at("non_positive_homogeneous_depth")),
                                    unsignedNumber(value.at("outside_image"))};
  const std::array<std::uint64_t, 5> parts{
      counts.visible_points, counts.non_finite_input, counts.behind_or_too_near,
      counts.non_positive_homogeneous_depth, counts.outside_image};
  std::uint64_t sum = 0U;
  for (const auto part : parts) {
    if (part > counts.input_points - sum) {
      malformed("generation accounting overflow/mismatch");
    }
    sum += part;
  }
  if (sum != counts.input_points) {
    malformed("generation accounting mismatch");
  }
  return counts;
}

[[nodiscard]] ObservedPixel readPixel(const Json& value) {
  return {finiteNumber(value.at("u_px")), finiteNumber(value.at("v_px"))};
}

[[nodiscard]] const ExtrinsicFixtureFrame& frameInfo(const ExtrinsicFixture& fixture,
                                                     const std::uint64_t frame_id) {
  const auto found = std::ranges::find(fixture.frames, frame_id, &ExtrinsicFixtureFrame::frame_id);
  if (found == fixture.frames.end()) {
    malformed("requested frame ID does not exist");
  }
  return *found;
}

class FixtureStaging final {
public:
  explicit FixtureStaging(std::filesystem::path path) : path_(std::move(path)) {
    if (!std::filesystem::create_directory(path_)) {
      throw IoError(IoErrorCode::kInvalidLayout, "fixture staging already exists");
    }
  }
  FixtureStaging(const FixtureStaging&) = delete;
  FixtureStaging& operator=(const FixtureStaging&) = delete;
  ~FixtureStaging() {
    if (!published_) {
      std::error_code error;
      std::filesystem::remove_all(path_, error);
    }
  }
  void publish() noexcept { published_ = true; }

private:
  std::filesystem::path path_;
  bool published_{false};
};

void writeJson(const std::filesystem::path& file, const Json& value) {
  std::ofstream output(file);
  output.exceptions(std::ios::failbit | std::ios::badbit);
  output << value.dump(2) << '\n';
  output.close();
}

class FixtureRandom final {
public:
  explicit FixtureRandom(const std::uint64_t seed) : state_(seed) {}
  [[nodiscard]] std::uint64_t next() {
    state_ += 0x9e3779b97f4a7c15ULL;
    std::uint64_t value = state_;
    value = (value ^ (value >> 30U)) * 0xbf58476d1ce4e5b9ULL;
    value = (value ^ (value >> 27U)) * 0x94d049bb133111ebULL;
    return value ^ (value >> 31U);
  }
  [[nodiscard]] double uniformSigned() {
    return 2.0 * static_cast<double>(next() >> 11U) / 9007199254740992.0 - 1.0;
  }

private:
  std::uint64_t state_;
};

[[nodiscard]] std::vector<core::PointXYZI> authoredPoints(const std::uint64_t frame_id,
                                                          const AuthoredRecoveryGeometry kind) {
  const std::array<std::vector<double>, 3> depths{
      {{6, 10, 18, 30}, {8, 15, 25}, {7, 12, 22, 40, 50}}};
  const std::array<std::vector<double>, 3> vertical{{{-0.18, -0.06, 0.06, 0.18},
                                                     {-0.20, -0.12, -0.04, 0.04, 0.12, 0.20},
                                                     {-0.16, -0.02, 0.08, 0.21}}};
  const std::array<int, 3> lateral_counts{12, 10, 8};
  const std::array<double, 3> minimum{-0.40, -0.32, -0.45};
  const std::array<double, 3> maximum{0.40, 0.44, 0.30};
  const auto frame = static_cast<std::size_t>(frame_id);
  std::vector<core::PointXYZI> points;
  for (const double depth : depths.at(frame)) {
    for (int lateral = 0; lateral < lateral_counts.at(frame); ++lateral) {
      const double fraction = static_cast<double>(lateral) / (lateral_counts.at(frame) - 1);
      const double x_ratio = minimum.at(frame) + fraction * (maximum.at(frame) - minimum.at(frame));
      for (const double y_ratio : vertical.at(frame)) {
        const bool narrow = kind == AuthoredRecoveryGeometry::kNarrow && frame_id < 2U;
        const double z_m = narrow ? 20.0 : depth;
        const double x_m = (narrow ? -0.005 + fraction * 0.010 : x_ratio) * z_m;
        const double y_m = (narrow ? 0.0 : y_ratio) * z_m;
        points.push_back({z_m, -x_m, -y_m, 0.5F});
      }
    }
  }
  return points;
}

} // namespace

void generateExtrinsicFixture(const std::filesystem::path& output_directory,
                              const ExtrinsicFixtureConfig& config, const bool overwrite) {
  if (!std::isfinite(config.noise_stddev_px) || config.noise_stddev_px < 0.0 ||
      !std::isfinite(config.training_outlier_fraction) || config.training_outlier_fraction < 0.0 ||
      config.training_outlier_fraction > 1.0 ||
      (config.geometry != AuthoredRecoveryGeometry::kRich &&
       config.geometry != AuthoredRecoveryGeometry::kNarrow)) {
    malformed("invalid noise/outlier/geometry configuration");
  }
  const auto output = output_directory.lexically_normal();
  if (output.empty() || output.filename().empty() || output.filename() == "." ||
      output.filename() == ".." || output == output.root_path()) {
    malformed("output must name a non-root fixture directory");
  }
  try {
    geometry::Matrix3d rotation;
    rotation << 0, -1, 0, 0, 0, -1, 1, 0, 0;
    const geometry::RigidTransform nominal({geometry::FrameId("camera_rect_00"),
                                            geometry::FrameId("lidar"), rotation,
                                            config.nominal_translation_m});
    const auto hidden = geometry::applyExtrinsicCorrection(nominal, config.hidden);
    geometry::Matrix34d matrix;
    matrix << 520, 0, 480, -52, 0, 520, 270, 0, 0, 0, 1, 0;
    const geometry::RectifiedProjection projection({matrix, 960, 540, 0.1});
    if (std::filesystem::exists(output) && (!std::filesystem::is_directory(output) || !overwrite)) {
      throw IoError(IoErrorCode::kInvalidLayout, "fixture exists; overwrite must be explicit");
    }
    if (std::filesystem::exists(output) &&
        std::filesystem::directory_iterator(output) != std::filesystem::directory_iterator()) {
      schema(readJson(output / "fixture.json"), "sfr.extrinsic_fixture");
    }
    std::filesystem::create_directories(output.parent_path().empty() ? "." : output.parent_path());
    const auto temporary = std::filesystem::path(output.string() + ".tmp");
    FixtureStaging staging(temporary);
    Json frame_metadata = Json::array();
    Json oracle_pixels = Json::array();
    for (std::uint64_t frame_id = 0U; frame_id < 3U; ++frame_id) {
      const auto points = authoredPoints(frame_id, config.geometry);
      const auto projected = geometry::projectPointCloud(points, hidden, projection);
      Json point_records = Json::array();
      for (std::size_t index = 0U; index < points.size(); ++index) {
        const auto& point = points[index];
        point_records.push_back({{"source_index", 11U + 3U * index},
                                 {"x_m", point.x_m},
                                 {"y_m", point.y_m},
                                 {"z_m", point.z_m},
                                 {"reflectance", point.reflectance}});
      }
      FixtureRandom random(config.seed + frame_id * 0x9e3779b97f4a7c15ULL);
      std::vector<std::pair<std::uint64_t, std::uint64_t>> ranked;
      ranked.reserve(projected.visible_points.size());
      for (const auto& point : projected.visible_points) {
        ranked.emplace_back(random.next(), point.source_index);
      }
      std::ranges::sort(ranked);
      const std::size_t outlier_count =
          frame_id < 2U ? static_cast<std::size_t>(std::floor(config.training_outlier_fraction *
                                                              static_cast<double>(ranked.size())))
                        : 0U;
      std::set<std::uint64_t> outliers;
      for (std::size_t index = 0U; index < outlier_count; ++index) {
        outliers.insert(ranked[index].second);
      }
      Json targets = Json::array();
      cv::Mat image(540, 960, CV_8UC3, cv::Scalar(30, 30, 30));
      cv::putText(image, "AUTHORED SYNTHETIC CORRESPONDENCES | frame " + std::to_string(frame_id),
                  {15, 28}, cv::FONT_HERSHEY_SIMPLEX, 0.55, {220, 220, 220}, 1);
      ObservedPixel noise{};
      ObservedPixel noise_sum{};
      for (std::size_t index = 0U; index < projected.visible_points.size(); ++index) {
        const auto& point = projected.visible_points[index];
        if (index % 2U == 0U) {
          noise = {random.uniformSigned() * std::sqrt(3.0) * config.noise_stddev_px,
                   random.uniformSigned() * std::sqrt(3.0) * config.noise_stddev_px};
          if (index + 1U == projected.visible_points.size()) {
            noise = {};
          }
        } else {
          noise.u_px = -noise.u_px;
          noise.v_px = -noise.v_px;
        }
        noise_sum.u_px += noise.u_px;
        noise_sum.v_px += noise.v_px;
        const bool outlier = outliers.contains(point.source_index);
        const ObservedPixel observed{point.u_px + noise.u_px + (outlier ? 40.0 : 0.0),
                                     point.v_px + noise.v_px + (outlier ? -30.0 : 0.0)};
        if (!std::isfinite(observed.u_px) || !std::isfinite(observed.v_px) || observed.u_px < 0.0 ||
            observed.u_px >= 960 || observed.v_px < 0.0 || observed.v_px >= 540) {
          malformed("corruption left image bounds; no targets were silently discarded");
        }
        const std::uint64_t source_index = 11U + 3U * point.source_index;
        targets.push_back({{"source_index", source_index},
                           {"u_px", observed.u_px},
                           {"v_px", observed.v_px},
                           {"weight", 1.0}});
        oracle_pixels.push_back({{"frame_id", frame_id},
                                 {"source_index", source_index},
                                 {"u_px", point.u_px},
                                 {"v_px", point.v_px},
                                 {"training_outlier", outlier}});
        cv::circle(image,
                   {static_cast<int>(std::lround(observed.u_px)),
                    static_cast<int>(std::lround(observed.v_px))},
                   2, {120, 120, 120}, cv::FILLED);
      }
      if (targets.empty()) {
        malformed("hidden transform produced no visible authored observations");
      }
      const std::string base = "frames/" + std::to_string(frame_id) + "/";
      std::filesystem::create_directories(temporary / base);
      writeJson(temporary / (base + "points.json"), point_records);
      writeJson(temporary / (base + "observations.json"), targets);
      if (!cv::imwrite((temporary / (base + "image.png")).string(), image)) {
        malformed("unable to encode authored image");
      }
      const double count = static_cast<double>(targets.size());
      frame_metadata.push_back(
          {{"frame_id", frame_id},
           {"points_file", base + "points.json"},
           {"observations_file", base + "observations.json"},
           {"image_file", base + "image.png"},
           {"scene_identity", sceneIdentity(points)},
           {"points_identity", fingerprint(point_records)},
           {"observations_identity", fingerprint(targets)},
           {"generation_counts", countsJson(projected.counts)},
           {"outlier_count", outlier_count},
           {"noise_sample_mean",
            {{"u_px", noise_sum.u_px / count}, {"v_px", noise_sum.v_px / count}}}});
    }
    Json projection_values = Json::array();
    for (Eigen::Index row = 0; row < 3; ++row) {
      for (Eigen::Index column = 0; column < 4; ++column) {
        projection_values.push_back(matrix(row, column));
      }
    }
    Json manifest{
        {"schema_name", "sfr.extrinsic_fixture"},
        {"schema_version", kFixtureVersion},
        {"fixture_id", "sfr_extrinsic_authored_v1"},
        {"generator_version", kGeneratorVersion},
        {"provenance", "project_authored_synthetic"},
        {"seed", config.seed},
        {"geometry", config.geometry == AuthoredRecoveryGeometry::kRich ? "rich" : "narrow"},
        {"noise_stddev_px", config.noise_stddev_px},
        {"training_outlier_fraction", config.training_outlier_fraction},
        {"noise_generator", "splitmix64-top53-uniform-antithetic-v1"},
        {"outlier_offset_px", {40.0, -30.0}},
        {"nominal_transform", matrixJson(nominal)},
        {"projection_matrix", projection_values},
        {"width_px", 960U},
        {"height_px", 540U},
        {"z_min_m", 0.1},
        {"frames", frame_metadata}};
    const std::string identity = fingerprint(manifest);
    manifest["content_identity"] = identity;
    writeJson(temporary / "fixture.json", manifest);
    Json oracle{{"schema_name", "sfr.extrinsic_oracle"},
                {"schema_version", kFixtureVersion},
                {"fixture_identity", identity},
                {"hidden_parameters",
                 {{"yaw_camera_y_rad", config.hidden.yaw_camera_y_rad},
                  {"translation_camera_x_m", config.hidden.translation_camera_x_m}}},
                {"hidden_transform", matrixJson(hidden)},
                {"pixels", oracle_pixels}};
    oracle["content_identity"] = fingerprint(oracle);
    writeJson(temporary / "oracle.json", oracle);
    if (std::filesystem::exists(output)) {
      std::filesystem::remove_all(output);
    }
    std::filesystem::rename(temporary, output);
    staging.publish();
  } catch (const std::filesystem::filesystem_error& error) {
    throw IoError(IoErrorCode::kFileOpen,
                  std::string("fixture filesystem failure: ") + error.what());
  } catch (const std::ios_base::failure& error) {
    throw IoError(IoErrorCode::kFileOpen, std::string("fixture write failure: ") + error.what());
  } catch (const cv::Exception& error) {
    throw IoError(IoErrorCode::kInvalidFrameFile,
                  std::string("fixture image failure: ") + error.what());
  } catch (const geometry::GeometryError& error) {
    malformed(error.what());
  }
}

ExtrinsicFixture loadExtrinsicFixture(const std::filesystem::path& root) {
  try {
    Json manifest = readJson(root / "fixture.json");
    schema(manifest, "sfr.extrinsic_fixture");
    const std::string identity = text(manifest.at("content_identity"));
    manifest.erase("content_identity");
    if (fingerprint(manifest) != identity ||
        manifest.at("provenance") != "project_authored_synthetic" ||
        manifest.at("fixture_id") != "sfr_extrinsic_authored_v1" ||
        manifest.at("generator_version") != kGeneratorVersion ||
        manifest.at("noise_generator") != "splitmix64-top53-uniform-antithetic-v1" ||
        manifest.at("outlier_offset_px") != Json::array({40.0, -30.0})) {
      malformed("identity or authored provenance mismatch");
    }
    const std::string geometry_name = text(manifest.at("geometry"));
    if (geometry_name != "rich" && geometry_name != "narrow") {
      malformed("unsupported authored geometry");
    }
    const Json& projection_values = manifest.at("projection_matrix");
    boundedArray(projection_values);
    if (projection_values.size() != 12U) {
      malformed("projection requires 12 values");
    }
    geometry::Matrix34d matrix;
    for (Eigen::Index row = 0; row < 3; ++row) {
      for (Eigen::Index column = 0; column < 4; ++column) {
        matrix(row, column) =
            finiteNumber(projection_values.at(static_cast<std::size_t>(row * 4 + column)));
      }
    }
    const auto width = unsignedNumber(manifest.at("width_px"));
    const auto height = unsignedNumber(manifest.at("height_px"));
    if (width == 0U || height == 0U || width > 10'000U || height > 10'000U ||
        width * height > 10'000'000U) {
      malformed("image dimensions exceed fixture limits");
    }
    ExtrinsicFixture fixture{
        root,
        text(manifest.at("fixture_id")),
        std::string(kFixtureVersion),
        text(manifest.at("generator_version")),
        identity,
        readTransform(manifest.at("nominal_transform")),
        geometry::RectifiedProjection({matrix, static_cast<int>(width), static_cast<int>(height),
                                       finiteNumber(manifest.at("z_min_m"))}),
        geometry_name == "rich" ? AuthoredRecoveryGeometry::kRich
                                : AuthoredRecoveryGeometry::kNarrow,
        unsignedNumber(manifest.at("seed")),
        finiteNumber(manifest.at("noise_stddev_px")),
        finiteNumber(manifest.at("training_outlier_fraction")),
        {}};
    if (fixture.noise_stddev_px < 0.0 || fixture.training_outlier_fraction < 0.0 ||
        fixture.training_outlier_fraction > 1.0) {
      malformed("invalid corruption metadata");
    }
    boundedArray(manifest.at("frames"), kMaximumFrames);
    std::set<std::uint64_t> ids;
    for (const Json& value : manifest.at("frames")) {
      ExtrinsicFixtureFrame frame{
          unsignedNumber(value.at("frame_id")),      text(value.at("points_file")),
          text(value.at("observations_file")),       text(value.at("image_file")),
          text(value.at("scene_identity")),          text(value.at("points_identity")),
          text(value.at("observations_identity")),   readCounts(value.at("generation_counts")),
          unsignedNumber(value.at("outlier_count")), readPixel(value.at("noise_sample_mean"))};
      if (!ids.insert(frame.frame_id).second ||
          frame.generation_counts.input_points > kMaximumRecords ||
          frame.outlier_count > frame.generation_counts.visible_points) {
        malformed("duplicate frame ID or invalid generation counts");
      }
      static_cast<void>(safePath(root, frame.points_file));
      static_cast<void>(safePath(root, frame.observations_file));
      static_cast<void>(safePath(root, frame.image_file));
      fixture.frames.push_back(std::move(frame));
    }
    std::ranges::sort(fixture.frames, {}, &ExtrinsicFixtureFrame::frame_id);
    return fixture;
  } catch (const Json::exception& error) {
    malformed(std::string("manifest fields: ") + error.what());
  } catch (const geometry::GeometryError& error) {
    malformed(error.what());
  } catch (const std::filesystem::filesystem_error& error) {
    malformed(error.what());
  }
}

std::vector<geometry::ExtrinsicObservationFrame>
loadExtrinsicObservationFrames(const ExtrinsicFixture& fixture,
                               const std::span<const std::uint64_t> frame_ids) {
  try {
    if (frame_ids.empty()) {
      malformed("selected frame set is empty");
    }
    std::set<std::uint64_t> requested;
    std::vector<geometry::ExtrinsicObservationFrame> result;
    for (const std::uint64_t frame_id : frame_ids) {
      if (!requested.insert(frame_id).second) {
        malformed("duplicate selected frame ID");
      }
      const auto& info = frameInfo(fixture, frame_id);
      Json points = readJson(safePath(fixture.root, info.points_file));
      Json targets = readJson(safePath(fixture.root, info.observations_file));
      boundedArray(points);
      boundedArray(targets);
      // Canonical serialized record ordering is independent of file array order.
      const auto by_source = [](const Json& first, const Json& second) {
        return unsignedNumber(first.at("source_index")) < unsignedNumber(second.at("source_index"));
      };
      std::sort(points.begin(), points.end(), by_source);
      std::sort(targets.begin(), targets.end(), by_source);
      if (fingerprint(points) != info.points_identity ||
          fingerprint(targets) != info.observations_identity ||
          points.size() != info.generation_counts.input_points ||
          targets.size() != info.generation_counts.visible_points) {
        malformed("point/correspondence identity or count mismatch");
      }
      std::map<std::uint64_t, core::PointXYZI> keyed;
      for (const auto& point : points) {
        const double reflectance = finiteNumber(point.at("reflectance"));
        if (std::abs(reflectance) > std::numeric_limits<float>::max() ||
            !keyed
                 .emplace(unsignedNumber(point.at("source_index")),
                          core::PointXYZI{
                              finiteNumber(point.at("x_m")), finiteNumber(point.at("y_m")),
                              finiteNumber(point.at("z_m")), static_cast<float>(reflectance)})
                 .second) {
          malformed("invalid point or duplicate point source ID");
        }
      }
      std::vector<core::PointXYZI> scene_points;
      for (const auto& [source_id, point] : keyed) {
        static_cast<void>(source_id);
        scene_points.push_back(point);
      }
      if (sceneIdentity(std::move(scene_points)) != info.scene_identity) {
        malformed("scene identity disagrees with point geometry");
      }
      geometry::ExtrinsicObservationFrame frame{frame_id, {}};
      for (const auto& target : targets) {
        const std::uint64_t id = unsignedNumber(target.at("source_index"));
        const auto found = keyed.find(id);
        if (found == keyed.end()) {
          malformed("observation has no matching source point");
        }
        frame.observations.push_back(
            {id, found->second, readPixel(target),
             target.contains("weight") ? finiteNumber(target.at("weight")) : 1.0});
      }
      result.push_back(std::move(frame));
    }
    geometry::validateExtrinsicObservations(result, fixture.projection);
    std::ranges::sort(result, {}, &geometry::ExtrinsicObservationFrame::frame_id);
    return result;
  } catch (const Json::exception& error) {
    malformed(std::string("observation fields: ") + error.what());
  } catch (const geometry::GeometryError& error) {
    malformed(error.what());
  } catch (const std::filesystem::filesystem_error& error) {
    malformed(error.what());
  }
}

void validateExtrinsicSplit(const ExtrinsicFixture& fixture,
                            const std::span<const std::uint64_t> training,
                            const std::span<const std::uint64_t> validation) {
  if (training.size() < 2U || validation.empty()) {
    malformed("split needs at least two train frames and one held-out frame");
  }
  std::set<std::uint64_t> ids;
  std::set<std::string> training_scenes;
  for (const auto id : training) {
    if (!ids.insert(id).second) {
      malformed("duplicate training frame");
    }
    training_scenes.insert(frameInfo(fixture, id).scene_identity);
  }
  for (const auto id : validation) {
    if (!ids.insert(id).second || training_scenes.contains(frameInfo(fixture, id).scene_identity)) {
      malformed("held-out frame overlaps training ID or scene content");
    }
  }
}

cv::Mat loadExtrinsicFixtureImage(const ExtrinsicFixture& fixture, const std::uint64_t frame_id) {
  return loadBgrImage(safePath(fixture.root, frameInfo(fixture, frame_id).image_file),
                      fixture.projection.imageWidthPixels(),
                      fixture.projection.imageHeightPixels());
}

ExtrinsicOracle loadExtrinsicOracle(const ExtrinsicFixture& fixture) {
  try {
    Json value = readJson(safePath(fixture.root, "oracle.json"));
    schema(value, "sfr.extrinsic_oracle");
    const auto oracle_identity = text(value.at("content_identity"));
    value.erase("content_identity");
    if (fingerprint(value) != oracle_identity) {
      malformed("oracle content identity mismatch");
    }
    if (value.at("fixture_identity") != fixture.content_identity) {
      malformed("oracle belongs to a different fixture");
    }
    const Json& parameters = value.at("hidden_parameters");
    ExtrinsicOracle oracle{{finiteNumber(parameters.at("yaw_camera_y_rad")),
                            finiteNumber(parameters.at("translation_camera_x_m"))},
                           readTransform(value.at("hidden_transform")),
                           fixture.content_identity,
                           {}};
    const auto expected =
        geometry::applyExtrinsicCorrection(fixture.T_nominal_camera_rect_00_lidar, oracle.hidden);
    if ((expected.matrix() - oracle.T_hidden_camera_rect_00_lidar.matrix()).norm() > 1e-9) {
      malformed("hidden parameters disagree with stored transform");
    }
    boundedArray(value.at("pixels"));
    std::set<std::pair<std::uint64_t, std::uint64_t>> keys;
    std::map<std::uint64_t, std::uint64_t> frame_counts;
    std::map<std::uint64_t, std::uint64_t> outlier_counts;
    for (const Json& pixel : value.at("pixels")) {
      const auto frame_id = unsignedNumber(pixel.at("frame_id"));
      const auto source_id = unsignedNumber(pixel.at("source_index"));
      static_cast<void>(frameInfo(fixture, frame_id));
      const auto uv = readPixel(pixel);
      if (!keys.emplace(frame_id, source_id).second || !pixel.at("training_outlier").is_boolean() ||
          uv.u_px < 0.0 || uv.v_px < 0.0 || uv.u_px >= fixture.projection.imageWidthPixels() ||
          uv.v_px >= fixture.projection.imageHeightPixels()) {
        malformed("invalid/duplicate oracle target");
      }
      oracle.pixels.push_back({frame_id, source_id, uv, pixel.at("training_outlier").get<bool>()});
      ++frame_counts[frame_id];
      outlier_counts[frame_id] += pixel.at("training_outlier").get<bool>() ? 1U : 0U;
    }
    for (const auto& frame : fixture.frames) {
      if (frame_counts[frame.frame_id] != frame.generation_counts.visible_points ||
          outlier_counts[frame.frame_id] != frame.outlier_count) {
        malformed("oracle coverage/outlier counts disagree with manifest");
      }
    }
    return oracle;
  } catch (const Json::exception& error) {
    malformed(std::string("oracle fields: ") + error.what());
  } catch (const geometry::GeometryError& error) {
    malformed(error.what());
  } catch (const std::filesystem::filesystem_error& error) {
    malformed(error.what());
  }
}

std::vector<geometry::ExtrinsicObservationFrame>
cleanExtrinsicObservations(const std::span<const geometry::ExtrinsicObservationFrame> frames,
                           const ExtrinsicOracle& oracle) {
  std::map<std::pair<std::uint64_t, std::uint64_t>, ObservedPixel> keyed;
  for (const auto& pixel : oracle.pixels) {
    if (!keyed.emplace(std::make_pair(pixel.frame_id, pixel.source_index), pixel.pixel).second) {
      malformed("duplicate oracle correspondence key");
    }
  }
  std::vector<geometry::ExtrinsicObservationFrame> clean(frames.begin(), frames.end());
  for (auto& frame : clean) {
    for (auto& observation : frame.observations) {
      const auto found = keyed.find({frame.frame_id, observation.source_index});
      if (found == keyed.end()) {
        malformed("missing clean oracle correspondence");
      }
      observation.observed_pixel = found->second;
    }
  }
  return clean;
}

} // namespace sfr::io
