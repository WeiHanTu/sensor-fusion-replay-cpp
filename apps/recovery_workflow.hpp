#ifndef SFR_APPS_RECOVERY_WORKFLOW_HPP_
#define SFR_APPS_RECOVERY_WORKFLOW_HPP_

#include <chrono>
#include <tuple>

#include "recovery_arguments.hpp"
#include "sfr/io/extrinsic_fixture.hpp"
#include "sfr/viz/extrinsic_recovery_artifact.hpp"

namespace sfr::cli {

// Composition shared by CLI and campaign, never called by the estimator.
[[nodiscard]] inline viz::ExtrinsicRecoveryArtifactRequest composeRecoveryRun(
    const std::filesystem::path& dataset, const std::vector<std::uint64_t>& training_ids,
    const std::vector<std::uint64_t>& validation_ids, const geometry::ExtrinsicSearchConfig& config,
    const std::filesystem::path& output, const std::string& run_id, const bool overwrite = false) {
  const auto load_start = std::chrono::steady_clock::now();
  const auto fixture = io::loadExtrinsicFixture(dataset);
  io::validateExtrinsicSplit(fixture, training_ids, validation_ids);
  const auto training = io::loadExtrinsicObservationFrames(fixture, training_ids);
  const auto loading_duration = std::chrono::steady_clock::now() - load_start;
  const auto fit = [&]() {
    try {
      return geometry::searchExtrinsicCorrection(fixture.T_nominal_camera_rect_00_lidar,
                                                 fixture.projection, training, config);
    } catch (const geometry::GeometryError& error) {
      throw ArgumentError(std::string("invalid search configuration: ") + error.what());
    }
  };
  const geometry::ExtrinsicSearchResult search = fit(); // freeze before oracle/held out
  const auto repeat_start = std::chrono::steady_clock::now();
  const auto repeated = fit();
  bool equal = search.status == repeated.status && search.loss_sum == repeated.loss_sum &&
               search.consumed_frame_ids == repeated.consumed_frame_ids &&
               search.points_evaluated == repeated.points_evaluated &&
               search.candidates.size() == repeated.candidates.size();
  const auto count_key = [](const geometry::ProjectionCounts& c) {
    return std::tuple(c.input_points, c.visible_points, c.non_finite_input, c.behind_or_too_near,
                      c.non_positive_homogeneous_depth, c.outside_image);
  };
  if (equal) {
    for (std::size_t index = 0U; index < search.candidates.size(); ++index) {
      const auto& a = search.candidates[index];
      const auto& b = repeated.candidates[index];
      equal =
          equal && a.yaw_tick == b.yaw_tick && a.translation_tick == b.translation_tick &&
          a.coarse == b.coarse && a.fine == b.fine &&
          a.evaluation.loss_sum == b.evaluation.loss_sum &&
          count_key(a.evaluation.projection_counts) == count_key(b.evaluation.projection_counts);
    }
  }
  const auto replay_duration = std::chrono::steady_clock::now() - repeat_start;
  const auto evaluation_start = std::chrono::steady_clock::now();
  const auto oracle = io::loadExtrinsicOracle(fixture);
  const auto validation = io::loadExtrinsicObservationFrames(fixture, validation_ids);
  const auto clean_train = io::cleanExtrinsicObservations(training, oracle);
  const auto clean_validation = io::cleanExtrinsicObservations(validation, oracle);
  const auto compare = [&](const auto& frames) {
    return geometry::compareExtrinsicCorrections(fixture.T_nominal_camera_rect_00_lidar,
                                                 fixture.projection, frames, search.best,
                                                 oracle.hidden);
  };
  const auto train_observed = compare(training);
  const auto train_clean = compare(clean_train);
  const auto validation_observed = compare(validation);
  const auto validation_clean = compare(clean_validation);
  std::vector<viz::RecoveryFrameImage> images;
  images.reserve(training_ids.size() + validation_ids.size());
  for (const auto id : training_ids) {
    images.push_back({id, io::loadExtrinsicFixtureImage(fixture, id)});
  }
  for (const auto id : validation_ids) {
    images.push_back({id, io::loadExtrinsicFixtureImage(fixture, id)});
  }
  const auto evaluation_duration = std::chrono::steady_clock::now() - evaluation_start;
  viz::RecoveryFixtureProvenance provenance{
      fixture.fixture_id,
      fixture.version,
      fixture.generator_version,
      fixture.content_identity,
      fixture.geometry == io::AuthoredRecoveryGeometry::kRich ? "rich" : "narrow",
      fixture.seed,
      fixture.noise_stddev_px,
      fixture.training_outlier_fraction,
      {}};
  for (const auto& f : fixture.frames) {
    provenance.frames.push_back({f.frame_id, f.scene_identity, f.points_identity,
                                 f.observations_identity, f.generation_counts, f.outlier_count,
                                 f.noise_sample_mean});
  }
  return {output,
          dataset,
          run_id,
          fixture.T_nominal_camera_rect_00_lidar,
          fixture.projection,
          provenance,
          config,
          search,
          oracle.hidden,
          training,
          validation,
          train_observed,
          train_clean,
          validation_observed,
          validation_clean,
          images,
          loading_duration,
          evaluation_duration,
          replay_duration,
          true,
          equal,
          overwrite};
}

[[nodiscard]] inline int recoveryExitCode(const geometry::ExtrinsicSearchResult& search) {
  return search.status == geometry::RecoverySearchStatus::kGridComplete &&
                 search.identifiability.available && !search.identifiability.poorly_identifiable
             ? 0
             : 4;
}

} // namespace sfr::cli
#endif
