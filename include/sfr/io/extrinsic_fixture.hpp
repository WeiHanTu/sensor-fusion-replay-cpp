#ifndef SFR_IO_EXTRINSIC_FIXTURE_HPP_
#define SFR_IO_EXTRINSIC_FIXTURE_HPP_

#include <cstdint>
#include <filesystem>
#include <span>
#include <string>
#include <vector>

#include <opencv2/core/mat.hpp>

#include "sfr/geometry/extrinsic_recovery.hpp"

namespace sfr::io {

enum class AuthoredRecoveryGeometry : std::uint8_t { kRich, kNarrow };

/// Generator-only configuration. Hidden values never occur in estimator inputs.
struct ExtrinsicFixtureConfig final {
  geometry::ExtrinsicCorrection hidden;
  geometry::Vector3d nominal_translation_m{geometry::Vector3d::Zero()};
  AuthoredRecoveryGeometry geometry{AuthoredRecoveryGeometry::kRich};
  std::uint64_t seed{17U};
  double noise_stddev_px{0.0};
  double training_outlier_fraction{0.0};
};

/// Manifest identity, safe relative paths and generation accounting for a scene.
struct ExtrinsicFixtureFrame final {
  std::uint64_t frame_id;
  std::string points_file;
  std::string observations_file;
  std::string image_file;
  std::string scene_identity;
  std::string points_identity;
  std::string observations_identity;
  geometry::ProjectionCounts generation_counts{};
  std::uint64_t outlier_count{0U};
  geometry::ObservedPixel noise_sample_mean{};
};

/// Owned nominal metadata only. Loading this manifest never opens oracle data.
struct ExtrinsicFixture final {
  std::filesystem::path root;
  std::string fixture_id;
  std::string version;
  std::string generator_version;
  std::string content_identity;
  geometry::RigidTransform T_nominal_camera_rect_00_lidar;
  geometry::RectifiedProjection projection;
  AuthoredRecoveryGeometry geometry;
  std::uint64_t seed;
  double noise_stddev_px;
  double training_outlier_fraction;
  std::vector<ExtrinsicFixtureFrame> frames;
};

/// A clean authored UV target retained exclusively for post-search evaluation.
struct OraclePixel final {
  std::uint64_t frame_id;
  std::uint64_t source_index;
  geometry::ObservedPixel pixel;
  bool training_outlier;
};

/// Owned evaluation-only truth, separate from the estimator's C++ signature.
struct ExtrinsicOracle final {
  geometry::ExtrinsicCorrection hidden;
  geometry::RigidTransform T_hidden_camera_rect_00_lidar;
  std::string fixture_identity;
  std::vector<OraclePixel> pixels;
};

/// Generates three authored frames and explicit correspondences via staging.
/// @throws IoError For invalid configuration, path, collision or publication.
void generateExtrinsicFixture(const std::filesystem::path& output_directory,
                              const ExtrinsicFixtureConfig& config, bool overwrite = false);

/// Strictly loads bounded JSON nominal metadata; oracle data remain unopened.
/// @throws IoError For malformed/duplicate keys, unsupported schema or unsafe paths.
[[nodiscard]] ExtrinsicFixture loadExtrinsicFixture(const std::filesystem::path& root);

/// Loads only selected observation files, explicitly joining source IDs to points.
///
/// Checks file identities, numeric contracts and original point bindings. Missing,
/// duplicate or changed bindings fail closed; serialized ordering is irrelevant.
[[nodiscard]] std::vector<geometry::ExtrinsicObservationFrame>
loadExtrinsicObservationFrames(const ExtrinsicFixture& fixture,
                               std::span<const std::uint64_t> frame_ids);

/// Validates nonempty, disjoint roles and distinct scene content using metadata.
void validateExtrinsicSplit(const ExtrinsicFixture& fixture,
                            std::span<const std::uint64_t> training,
                            std::span<const std::uint64_t> validation);

/// Loads an authored display image; it supplies no estimation correspondences.
[[nodiscard]] cv::Mat loadExtrinsicFixtureImage(const ExtrinsicFixture& fixture,
                                                std::uint64_t frame_id);

/// Loads and checks the oracle sidecar for the matching fixture after fitting.
[[nodiscard]] ExtrinsicOracle loadExtrinsicOracle(const ExtrinsicFixture& fixture);

/// Copies observations, replacing UV targets by explicitly joined clean oracle UV.
[[nodiscard]] std::vector<geometry::ExtrinsicObservationFrame>
cleanExtrinsicObservations(std::span<const geometry::ExtrinsicObservationFrame> frames,
                           const ExtrinsicOracle& oracle);

} // namespace sfr::io

#endif // SFR_IO_EXTRINSIC_FIXTURE_HPP_
