#ifndef SFR_VIZ_EXTRINSIC_RECOVERY_ARTIFACT_HPP_
#define SFR_VIZ_EXTRINSIC_RECOVERY_ARTIFACT_HPP_

#include <filesystem>
#include <string>
#include <vector>

#include "sfr/geometry/extrinsic_recovery.hpp"
#include "sfr/viz/projection_artifact.hpp"

namespace sfr::viz {

/// Owned report-only provenance; no private dataset paths or estimator priors.
struct RecoveryFrameProvenance final {
  std::uint64_t frame_id;
  std::string scene_identity;
  std::string points_identity;
  std::string observations_identity;
  geometry::ProjectionCounts generation_counts;
  std::uint64_t outlier_count;
  geometry::ObservedPixel noise_mean;
};

struct RecoveryFixtureProvenance final {
  std::string id;
  std::string version;
  std::string generator_version;
  std::string content_identity;
  std::string geometry;
  std::uint64_t seed;
  double noise_stddev_px;
  double outlier_fraction;
  std::vector<RecoveryFrameProvenance> frames;
};

struct RecoveryFrameImage final {
  std::uint64_t frame_id;
  cv::Mat image_bgr8;
};

/// Borrowed fit/evaluation inputs are used only during publication.
///
/// Oracle is report-only, disclosed after fitting. Comparisons own their keyed
/// residuals; images share OpenCV storage. No references survive this call.
struct ExtrinsicRecoveryArtifactRequest final {
  std::filesystem::path output_root;
  std::filesystem::path fixture_root;
  std::string run_id;
  geometry::RigidTransform nominal;
  geometry::RectifiedProjection projection;
  RecoveryFixtureProvenance fixture;
  geometry::ExtrinsicSearchConfig config;
  geometry::ExtrinsicSearchResult search;
  geometry::ExtrinsicCorrection hidden;
  std::vector<geometry::ExtrinsicObservationFrame> training;
  std::vector<geometry::ExtrinsicObservationFrame> validation;
  geometry::RecoveryComparison train_observed;
  geometry::RecoveryComparison train_clean;
  geometry::RecoveryComparison validation_observed;
  geometry::RecoveryComparison validation_clean;
  std::vector<RecoveryFrameImage> images;
  std::chrono::nanoseconds loading_duration{};
  std::chrono::nanoseconds evaluation_duration{};
  std::chrono::nanoseconds reproducibility_duration{};
  bool deterministic_replay_checked{false};
  bool deterministic_replay_equal{false};
  bool overwrite{false};
};

/// Publishes complete reports/plots/overlays through an exclusively claimed stage.
///
/// Refuses fixture-contained outputs, unrelated nonempty runs and unowned stages.
/// Explicit overwrite removes the old run before rename: not transactional and
/// not crash-durable. Completion of publication is separate from acceptance.
/// @throws ArtifactError For malformed accounting, collisions or I/O/rendering.
[[nodiscard]] std::filesystem::path
writeExtrinsicRecoveryArtifacts(const ExtrinsicRecoveryArtifactRequest& request);

} // namespace sfr::viz
#endif
