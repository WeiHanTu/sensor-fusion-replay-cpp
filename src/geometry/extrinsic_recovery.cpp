#include "sfr/geometry/extrinsic_recovery.hpp"

#include <Eigen/SVD>
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <string>
#include <tuple>

#include "sfr/geometry/calibration_sensitivity.hpp"

namespace sfr::geometry {
namespace {

[[noreturn]] void invalidRecovery(const std::string& message) {
  throw GeometryError(GeometryErrorCode::kInvalidProjection, message);
}

class CompensatedSum final {
public:
  void add(const double value) {
    const double adjusted = value - compensation_;
    const double updated = sum_ + adjusted;
    compensation_ = (updated - sum_) - adjusted;
    sum_ = updated;
  }
  [[nodiscard]] double value() const noexcept { return sum_; }

private:
  double sum_{0.0};
  double compensation_{0.0};
};

struct PreparedObservation final {
  std::uint64_t frame_id;
  std::uint64_t source_index;
  ObservedPixel target;
  double weight;
};

struct PreparedObservations final {
  std::vector<core::PointXYZI> points;
  std::vector<PreparedObservation> observations;
  std::vector<std::uint64_t> frame_ids;
  double total_weight{0.0};
};

void validateLoss(const RecoveryLossConfig& loss) {
  if (loss.kind != RecoveryLoss::kSquaredL2 && loss.kind != RecoveryLoss::kHuber) {
    invalidRecovery("unknown recovery loss");
  }
  if (!std::isfinite(loss.huber_delta_px) || loss.huber_delta_px <= 0.0) {
    invalidRecovery("Huber delta must be finite and positive");
  }
}

[[nodiscard]] PreparedObservations
prepareObservations(const std::span<const ExtrinsicObservationFrame> frames,
                    const RectifiedProjection& projection) {
  if (frames.empty()) {
    invalidRecovery("recovery requires nonempty observation frames");
  }
  std::vector<ExtrinsicObservationFrame> ordered(frames.begin(), frames.end());
  std::ranges::sort(ordered, {}, &ExtrinsicObservationFrame::frame_id);
  PreparedObservations prepared;
  CompensatedSum total_weight;
  std::optional<std::uint64_t> previous_frame;
  for (ExtrinsicObservationFrame& frame : ordered) {
    if ((previous_frame.has_value() && previous_frame.value() == frame.frame_id) ||
        frame.observations.empty()) {
      invalidRecovery("observation frames must be nonempty with unique IDs");
    }
    previous_frame = frame.frame_id;
    prepared.frame_ids.push_back(frame.frame_id);
    std::ranges::sort(frame.observations, {}, &ExtrinsicObservation::source_index);
    std::optional<std::uint64_t> previous_source;
    bool has_active = false;
    for (const ExtrinsicObservation& observation : frame.observations) {
      const core::PointXYZI& point = observation.point_lidar;
      const ObservedPixel& target = observation.observed_pixel;
      if ((previous_source.has_value() && previous_source.value() == observation.source_index) ||
          !std::isfinite(point.x_m) || !std::isfinite(point.y_m) || !std::isfinite(point.z_m) ||
          !std::isfinite(point.reflectance) || !std::isfinite(target.u_px) ||
          !std::isfinite(target.v_px) || target.u_px < 0.0 || target.v_px < 0.0 ||
          target.u_px >= projection.imageWidthPixels() ||
          target.v_px >= projection.imageHeightPixels() || !std::isfinite(observation.weight) ||
          observation.weight < 0.0) {
        invalidRecovery("invalid observation or duplicate source ID");
      }
      previous_source = observation.source_index;
      has_active = has_active || observation.weight > 0.0;
      total_weight.add(observation.weight);
      prepared.points.push_back(point);
      prepared.observations.push_back(
          {frame.frame_id, observation.source_index, target, observation.weight});
    }
    if (!has_active) {
      invalidRecovery("each observation frame needs a positive-weight observation");
    }
  }
  prepared.total_weight = total_weight.value();
  if (!std::isfinite(prepared.total_weight) || prepared.total_weight <= 0.0) {
    invalidRecovery("total observation weight must be finite and positive");
  }
  return prepared;
}

[[nodiscard]] double radialLoss(const double residual, const RecoveryLossConfig& loss) {
  if (loss.kind == RecoveryLoss::kSquaredL2) {
    return residual * residual;
  }
  if (residual <= loss.huber_delta_px) {
    return 0.5 * residual * residual;
  }
  return loss.huber_delta_px * (residual - 0.5 * loss.huber_delta_px);
}

[[nodiscard]] ExtrinsicCandidateEvaluation
evaluatePrepared(const PreparedObservations& prepared,
                 const RigidTransform& T_candidate_camera_rect_00_lidar,
                 const RectifiedProjection& projection, const ExtrinsicCorrection& correction,
                 const RecoveryLossConfig& loss) {
  const PointCloudProjectionResult projected =
      projectPointCloud(prepared.points, T_candidate_camera_rect_00_lidar, projection);
  ExtrinsicCandidateEvaluation result;
  result.correction = correction;
  result.projection_counts = projected.counts;
  result.total_weight = prepared.total_weight;
  std::vector<bool> visible(prepared.points.size(), false);
  CompensatedSum objective;
  for (const core::ProjectedPoint& point : projected.visible_points) {
    const auto index = static_cast<std::size_t>(point.source_index);
    if (index >= prepared.observations.size() || visible[index]) {
      invalidRecovery("projection returned an invalid input-span source index");
    }
    visible[index] = true;
    const PreparedObservation& observation = prepared.observations[index];
    if (observation.weight == 0.0) {
      continue;
    }
    const double residual =
        std::hypot(observation.target.u_px - point.u_px, observation.target.v_px - point.v_px);
    const double contribution = observation.weight * radialLoss(residual, loss);
    if (!std::isfinite(contribution)) {
      result.numeric_failure = true;
      continue;
    }
    objective.add(contribution);
  }
  for (std::size_t index = 0U; index < prepared.observations.size(); ++index) {
    if (prepared.observations[index].weight == 0.0) {
      ++result.zero_weight;
    } else {
      ++result.positive_weight;
      if (visible[index]) {
        ++result.positive_weight_visible;
      } else {
        ++result.positive_weight_invalid;
      }
    }
  }
  const double sum = objective.value();
  result.numeric_failure = result.numeric_failure || !std::isfinite(sum);
  if (result.positive_weight_invalid == 0U && !result.numeric_failure) {
    result.loss_sum = sum;
    result.loss_mean = sum / prepared.total_weight;
  }
  return result;
}

struct AxisTicks final {
  std::int64_t maximum;
  std::int64_t coarse_stride;
};

[[nodiscard]] std::int64_t integralTicks(const double value) {
  if (!std::isfinite(value) || value < 1.0 || value > 100000000.0 ||
      std::abs(value - std::round(value)) > 1e-9) {
    invalidRecovery("grid ratios/spans must be integral within 1e-9 and bounded");
  }
  return static_cast<std::int64_t>(std::llround(value));
}

[[nodiscard]] AxisTicks validateAxis(const RecoveryAxisGrid& axis) {
  if (!std::isfinite(axis.minimum) || !std::isfinite(axis.maximum) ||
      !std::isfinite(axis.coarse_step) || !std::isfinite(axis.fine_step) ||
      axis.minimum >= axis.maximum || axis.fine_step <= 0.0 || axis.coarse_step < axis.fine_step) {
    invalidRecovery("grid bounds/steps must be finite, ordered and positive");
  }
  const auto stride = integralTicks(axis.coarse_step / axis.fine_step);
  const auto coarse_span = integralTicks((axis.maximum - axis.minimum) / axis.coarse_step);
  if (stride > 100000000 / coarse_span) {
    invalidRecovery("grid has too many fine ticks");
  }
  return {coarse_span * stride, stride};
}

[[nodiscard]] ExtrinsicCorrection correctionAt(const ExtrinsicSearchConfig& config,
                                               const std::int64_t yaw,
                                               const std::int64_t translation) {
  return {config.yaw.minimum + static_cast<double>(yaw) * config.yaw.fine_step,
          config.translation.minimum +
              static_cast<double>(translation) * config.translation.fine_step};
}

[[nodiscard]] std::optional<std::vector<ObservedPixel>>
activePixels(const PreparedObservations& prepared, const RigidTransform& nominal,
             const RectifiedProjection& projection, const ExtrinsicCorrection& correction) {
  const auto cloud =
      projectPointCloud(prepared.points, applyExtrinsicCorrection(nominal, correction), projection);
  std::vector<ObservedPixel> pixels(prepared.points.size());
  std::vector<bool> seen(prepared.points.size(), false);
  for (const auto& point : cloud.visible_points) {
    const auto index = static_cast<std::size_t>(point.source_index);
    pixels[index] = {point.u_px, point.v_px};
    seen[index] = true;
  }
  for (std::size_t index = 0U; index < seen.size(); ++index) {
    if (!seen[index] && prepared.observations[index].weight > 0.0) {
      return std::nullopt;
    }
  }
  return pixels;
}

[[nodiscard]] RecoveryIdentifiability diagnose(const PreparedObservations& prepared,
                                               const RigidTransform& nominal,
                                               const RectifiedProjection& projection,
                                               const ExtrinsicCorrection& best,
                                               const ExtrinsicSearchConfig& config) {
  RecoveryIdentifiability result;
  const auto base = activePixels(prepared, nominal, projection, best);
  result.points_evaluated += prepared.points.size();
  if (!base.has_value()) {
    return result;
  }
  Eigen::MatrixXd jacobian(static_cast<Eigen::Index>(2U * prepared.points.size()), 2);
  jacobian.setZero();
  for (int axis_index = 0; axis_index < 2; ++axis_index) {
    const auto& axis = axis_index == 0 ? config.yaw : config.translation;
    const double center = axis_index == 0 ? best.yaw_camera_y_rad : best.translation_camera_x_m;
    auto minus = best;
    auto plus = best;
    if (axis_index == 0) {
      minus.yaw_camera_y_rad -= axis.fine_step;
      plus.yaw_camera_y_rad += axis.fine_step;
    } else {
      minus.translation_camera_x_m -= axis.fine_step;
      plus.translation_camera_x_m += axis.fine_step;
    }
    std::optional<std::vector<ObservedPixel>> low;
    std::optional<std::vector<ObservedPixel>> high;
    if (center - axis.fine_step >= axis.minimum - 1e-15) {
      low = activePixels(prepared, nominal, projection, minus);
      result.points_evaluated += prepared.points.size();
    }
    if (center + axis.fine_step <= axis.maximum + 1e-15) {
      high = activePixels(prepared, nominal, projection, plus);
      result.points_evaluated += prepared.points.size();
    }
    if (!low.has_value() && !high.has_value()) {
      return result;
    }
    const bool central = low.has_value() && high.has_value();
    result.used_one_sided_difference = result.used_one_sided_difference || !central;
    const auto& a = low.has_value() ? low.value() : base.value();
    const auto& b = high.has_value() ? high.value() : base.value();
    const double scale = (axis.maximum - axis.minimum) / ((central ? 2.0 : 1.0) * axis.fine_step);
    for (std::size_t index = 0U; index < prepared.points.size(); ++index) {
      const double weight_scale = std::sqrt(prepared.observations[index].weight) * scale;
      jacobian(static_cast<Eigen::Index>(2U * index), axis_index) =
          (b[index].u_px - a[index].u_px) * weight_scale;
      jacobian(static_cast<Eigen::Index>(2U * index + 1U), axis_index) =
          (b[index].v_px - a[index].v_px) * weight_scale;
    }
  }
  if (!jacobian.allFinite()) {
    return result;
  }
  const Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian);
  result.sigma_max = svd.singularValues()(0);
  result.sigma_min = svd.singularValues()(1);
  result.rank = static_cast<std::uint8_t>(
      (result.sigma_max > 0.0 ? 1U : 0U) +
      (result.sigma_min > config.rank_relative_tolerance * result.sigma_max ? 1U : 0U));
  if (result.sigma_min > 0.0) {
    const double condition = result.sigma_max / result.sigma_min;
    if (std::isfinite(condition)) {
      result.condition = condition;
    }
  }
  result.available = true;
  result.poorly_identifiable = result.rank < 2U || !result.condition.has_value() ||
                               result.condition.value() > config.condition_limit;
  return result;
}

} // namespace

void validateExtrinsicObservations(const std::span<const ExtrinsicObservationFrame> frames,
                                   const RectifiedProjection& projection) {
  static_cast<void>(prepareObservations(frames, projection));
}

RigidTransform applyExtrinsicCorrection(const RigidTransform& T_nominal_camera_rect_00_lidar,
                                        const ExtrinsicCorrection& correction) {
  return leftApplyCameraXTranslation(
      leftApplyCameraYaw(T_nominal_camera_rect_00_lidar, correction.yaw_camera_y_rad),
      correction.translation_camera_x_m);
}

ExtrinsicCandidateEvaluation
evaluateExtrinsicCandidate(const RigidTransform& T_nominal_camera_rect_00_lidar,
                           const RectifiedProjection& projection,
                           const std::span<const ExtrinsicObservationFrame> frames,
                           const ExtrinsicCorrection& correction, const RecoveryLossConfig& loss) {
  validateLoss(loss);
  const PreparedObservations prepared = prepareObservations(frames, projection);
  return evaluatePrepared(prepared,
                          applyExtrinsicCorrection(T_nominal_camera_rect_00_lidar, correction),
                          projection, correction, loss);
}

bool betterExtrinsicCandidate(const ExtrinsicCandidateEvaluation& left,
                              const ExtrinsicCandidateEvaluation& right) {
  if (!left.loss_sum.has_value()) {
    return false;
  }
  if (!right.loss_sum.has_value()) {
    return true;
  }
  const auto key = [](const ExtrinsicCandidateEvaluation& candidate) {
    return std::tuple(candidate.loss_sum.value(), std::abs(candidate.correction.yaw_camera_y_rad),
                      std::abs(candidate.correction.translation_camera_x_m),
                      candidate.correction.yaw_camera_y_rad,
                      candidate.correction.translation_camera_x_m);
  };
  return key(left) < key(right);
}

ExtrinsicSearchResult
searchExtrinsicCorrection(const RigidTransform& nominal, const RectifiedProjection& projection,
                          const std::span<const ExtrinsicObservationFrame> training,
                          const ExtrinsicSearchConfig& config) {
  validateLoss(config.loss);
  const auto yaw = validateAxis(config.yaw);
  const auto translation = validateAxis(config.translation);
  if (training.size() < 2U || !std::isfinite(config.rank_relative_tolerance) ||
      config.rank_relative_tolerance <= 0.0 || config.rank_relative_tolerance >= 1.0 ||
      !std::isfinite(config.condition_limit) || config.condition_limit <= 1.0) {
    invalidRecovery("search needs two training frames and valid diagnostic thresholds");
  }
  static_cast<void>(applyExtrinsicCorrection(nominal, {}));
  const auto coarse_count =
      static_cast<std::uint64_t>(yaw.maximum / yaw.coarse_stride + 1) *
      static_cast<std::uint64_t>(translation.maximum / translation.coarse_stride + 1);
  const auto fine_count =
      static_cast<std::uint64_t>(std::min(yaw.maximum + 1, 2 * yaw.coarse_stride + 1)) *
      static_cast<std::uint64_t>(
          std::min(translation.maximum + 1, 2 * translation.coarse_stride + 1));
  // Upper bound includes cached intersections: conservative refusal is explicit.
  const auto maximum_attempts = coarse_count + fine_count;
  std::uint64_t observations = 0U;
  for (const auto& frame : training) {
    if (frame.observations.size() > config.max_point_projections - observations) {
      invalidRecovery("observation count exceeds point-projection budget");
    }
    observations += frame.observations.size();
  }
  if (maximum_attempts > config.max_candidates ||
      observations > config.max_point_projections / maximum_attempts) {
    invalidRecovery("configured search exceeds candidate or point-projection budget");
  }
  const auto prepared = prepareObservations(training, projection);
  ExtrinsicSearchResult result;
  result.consumed_frame_ids = prepared.frame_ids;
  result.total_weight = prepared.total_weight;
  using Key = std::pair<std::int64_t, std::int64_t>;
  std::map<Key, RecoveryCandidateRow> rows;
  std::optional<Key> best_key;
  const auto attempt = [&](const Key& key, const bool coarse) {
    auto found = rows.find(key);
    if (found == rows.end()) {
      auto evaluation = evaluatePrepared(
          prepared, applyExtrinsicCorrection(nominal, correctionAt(config, key.first, key.second)),
          projection, correctionAt(config, key.first, key.second), config.loss);
      found =
          rows.emplace(key, RecoveryCandidateRow{key.first, key.second, false, false, evaluation})
              .first;
      result.points_evaluated += prepared.points.size();
    }
    found->second.coarse = found->second.coarse || coarse;
    found->second.fine = found->second.fine || !coarse;
    if (found->second.evaluation.loss_sum.has_value() &&
        (!best_key.has_value() || betterExtrinsicCandidate(found->second.evaluation,
                                                           rows.at(best_key.value()).evaluation))) {
      best_key = key;
    }
  };
  const auto start = std::chrono::steady_clock::now();
  for (std::int64_t y = 0; y <= yaw.maximum; y += yaw.coarse_stride) {
    for (std::int64_t x = 0; x <= translation.maximum; x += translation.coarse_stride) {
      attempt({y, x}, true);
    }
  }
  if (best_key.has_value()) {
    const Key center = best_key.value();
    const Key low{std::max<std::int64_t>(0, center.first - yaw.coarse_stride),
                  std::max<std::int64_t>(0, center.second - translation.coarse_stride)};
    const Key high{std::min(yaw.maximum, center.first + yaw.coarse_stride),
                   std::min(translation.maximum, center.second + translation.coarse_stride)};
    for (auto y = low.first; y <= high.first; ++y) {
      for (auto x = low.second; x <= high.second; ++x) {
        attempt({y, x}, false);
      }
    }
    const auto best = best_key.value();
    const auto& evaluation = rows.at(best).evaluation;
    result.best = evaluation.correction;
    result.loss_sum = evaluation.loss_sum;
    result.loss_mean = evaluation.loss_mean;
    result.yaw_bound_hit = best.first == 0 || best.first == yaw.maximum;
    result.translation_bound_hit = best.second == 0 || best.second == translation.maximum;
    result.refinement_edge_hit =
        (!result.yaw_bound_hit && (best.first == low.first || best.first == high.first)) ||
        (!result.translation_bound_hit &&
         (best.second == low.second || best.second == high.second));
    result.status = result.yaw_bound_hit || result.translation_bound_hit
                        ? RecoverySearchStatus::kBoundLimited
                        : (result.refinement_edge_hit ? RecoverySearchStatus::kRefinementLimited
                                                      : RecoverySearchStatus::kGridComplete);
  }
  result.search_duration = std::chrono::steady_clock::now() - start;
  for (auto& [key, row] : rows) {
    static_cast<void>(key);
    result.candidates.push_back(row);
  }
  if (result.best.has_value()) {
    const auto diagnostic_start = std::chrono::steady_clock::now();
    result.identifiability = diagnose(prepared, nominal, projection, result.best.value(), config);
    result.diagnostics_duration = std::chrono::steady_clock::now() - diagnostic_start;
  }
  return result;
}

namespace {

void addStatus(ProjectionCounts& counts, const ProjectionStatus status) {
  ++counts.input_points;
  switch (status) {
  case ProjectionStatus::kVisible:
    ++counts.visible_points;
    break;
  case ProjectionStatus::kNonFiniteInput:
    ++counts.non_finite_input;
    break;
  case ProjectionStatus::kBehindOrTooNear:
    ++counts.behind_or_too_near;
    break;
  case ProjectionStatus::kNonPositiveHomogeneousDepth:
    ++counts.non_positive_homogeneous_depth;
    break;
  case ProjectionStatus::kOutsideImage:
    ++counts.outside_image;
    break;
  }
}

void summarize(RecoveryResidualMetrics& metrics, std::vector<double> errors) {
  if (errors.empty()) {
    return;
  }
  std::ranges::sort(errors);
  const auto percentile = [&](const double p) {
    const auto rank = static_cast<std::size_t>(std::ceil(p * static_cast<double>(errors.size())));
    return errors[std::clamp(rank, std::size_t{1U}, errors.size()) - 1U];
  };
  metrics.median_px = percentile(0.5);
  metrics.p95_px = percentile(0.95);
  // Scaling prevents overflow of squared large but finite residuals.
  const double largest = errors.back();
  CompensatedSum squares;
  if (largest > 0.0) {
    for (const double error : errors) {
      squares.add((error / largest) * (error / largest));
    }
  }
  metrics.rmse_px = largest * std::sqrt(squares.value() / static_cast<double>(errors.size()));
}

[[nodiscard]] RecoveryTransformMetrics transformMetrics(const PreparedObservations& prepared,
                                                        const RigidTransform& transform,
                                                        const RectifiedProjection& projection) {
  RecoveryTransformMetrics result;
  const auto cloud = projectPointCloud(prepared.points, transform, projection);
  result.pooled.counts = cloud.counts;
  std::vector<std::optional<ObservedPixel>> visible(prepared.points.size());
  for (const auto& pixel : cloud.visible_points) {
    visible[static_cast<std::size_t>(pixel.source_index)] = ObservedPixel{pixel.u_px, pixel.v_px};
  }
  std::map<std::uint64_t, std::vector<double>> frame_errors;
  std::map<std::uint64_t, RecoveryResidualMetrics> frame_metrics;
  std::vector<double> pooled_errors;
  for (std::size_t index = 0U; index < prepared.points.size(); ++index) {
    const auto& observation = prepared.observations[index];
    auto& metrics = frame_metrics[observation.frame_id];
    metrics.frame_id = observation.frame_id;
    RecoveryResidual residual{observation.frame_id, observation.source_index,   observation.weight,
                              observation.target,   ProjectionStatus::kVisible, visible[index],
                              std::nullopt};
    const auto& pixel = visible[index];
    if (pixel.has_value()) {
      residual.radial_error_px = std::hypot(observation.target.u_px - pixel.value().u_px,
                                            observation.target.v_px - pixel.value().v_px);
      pooled_errors.push_back(residual.radial_error_px.value());
      frame_errors[observation.frame_id].push_back(residual.radial_error_px.value());
    } else {
      const auto& point = prepared.points[index];
      residual.status =
          projection.project(transform.transformPoint({point.x_m, point.y_m, point.z_m})).status;
    }
    addStatus(metrics.counts, residual.status);
    if (observation.weight == 0.0) {
      ++metrics.zero_weight;
      ++result.pooled.zero_weight;
    }
    result.residuals.push_back(residual);
  }
  summarize(result.pooled, std::move(pooled_errors));
  for (auto& [id, metrics] : frame_metrics) {
    summarize(metrics, std::move(frame_errors[id]));
    result.frames.push_back(metrics);
  }
  return result;
}

} // namespace

RecoveryComparison
compareExtrinsicCorrections(const RigidTransform& nominal, const RectifiedProjection& projection,
                            const std::span<const ExtrinsicObservationFrame> frames,
                            const std::optional<ExtrinsicCorrection>& recovered,
                            const ExtrinsicCorrection& oracle) {
  const auto prepared = prepareObservations(frames, projection);
  RecoveryComparison result;
  result.nominal = transformMetrics(prepared, applyExtrinsicCorrection(nominal, {}), projection);
  result.oracle = transformMetrics(prepared, applyExtrinsicCorrection(nominal, oracle), projection);
  result.points_evaluated = 2U * prepared.points.size();
  if (recovered.has_value()) {
    result.recovered = transformMetrics(
        prepared, applyExtrinsicCorrection(nominal, recovered.value()), projection);
    result.points_evaluated += prepared.points.size();
  }
  // Invisible classification requires one additional primitive projection.
  result.points_evaluated +=
      result.nominal.pooled.counts.input_points - result.nominal.pooled.counts.visible_points;
  result.points_evaluated +=
      result.oracle.pooled.counts.input_points - result.oracle.pooled.counts.visible_points;
  if (result.recovered.has_value()) {
    result.points_evaluated += result.recovered.value().pooled.counts.input_points -
                               result.recovered.value().pooled.counts.visible_points;
  }
  for (std::size_t index = 0U; index < prepared.points.size(); ++index) {
    if (result.nominal.residuals[index].predicted.has_value() &&
        result.oracle.residuals[index].predicted.has_value() &&
        (!result.recovered.has_value() ||
         result.recovered.value().residuals[index].predicted.has_value())) {
      const auto mark = [&](RecoveryTransformMetrics& table) {
        ++table.pooled.common_visible;
        for (auto& frame : table.frames) {
          if (frame.frame_id == prepared.observations[index].frame_id) {
            ++frame.common_visible;
          }
        }
      };
      mark(result.nominal);
      mark(result.oracle);
      if (result.recovered.has_value()) {
        mark(result.recovered.value());
      }
    }
  }
  return result;
}

RecoveryAcceptance assessExtrinsicRecovery(const ExtrinsicSearchResult& result,
                                           const ExtrinsicSearchConfig& config,
                                           const ExtrinsicCorrection& hidden,
                                           const RecoveryComparison& validation) {
  RecoveryAcceptance gate;
  gate.search_and_identifiability_gate = result.status == RecoverySearchStatus::kGridComplete &&
                                         result.identifiability.available &&
                                         !result.identifiability.poorly_identifiable;
  if (!result.best.has_value() || !validation.recovered.has_value()) {
    return gate;
  }
  const double yaw_error = result.best.value().yaw_camera_y_rad - hidden.yaw_camera_y_rad;
  const double tx_error =
      result.best.value().translation_camera_x_m - hidden.translation_camera_x_m;
  gate.signed_yaw_error_deg = yaw_error * 180.0 / std::numbers::pi;
  gate.signed_translation_error_m = tx_error;
  gate.parameter_gate = std::abs(yaw_error) <= config.yaw.fine_step / 2.0 + 1e-12 &&
                        std::abs(tx_error) <= config.translation.fine_step / 2.0 + 1e-12;
  const auto full = [](const RecoveryTransformMetrics& table) {
    return table.pooled.counts.input_points > 0U &&
           table.pooled.counts.input_points == table.pooled.counts.visible_points;
  };
  gate.full_coverage =
      full(validation.nominal) && full(validation.recovered.value()) && full(validation.oracle);
  const auto& corrected = validation.recovered.value().pooled;
  const auto& baseline = validation.nominal.pooled;
  if (gate.full_coverage && corrected.rmse_px.has_value() && corrected.median_px.has_value() &&
      corrected.p95_px.has_value() && baseline.median_px.has_value() &&
      baseline.p95_px.has_value() && baseline.rmse_px.has_value()) {
    if (hidden.yaw_camera_y_rad == 0.0 && hidden.translation_camera_x_m == 0.0) {
      gate.held_out_gate = corrected.rmse_px.value() <= 1e-9;
    } else {
      gate.held_out_gate = corrected.median_px.value() <= 0.1 * baseline.median_px.value() &&
                           corrected.p95_px.value() <= 0.1 * baseline.p95_px.value() &&
                           corrected.rmse_px.value() <= 0.1 * baseline.rmse_px.value();
    }
  }
  gate.passed = gate.parameter_gate && gate.full_coverage && gate.held_out_gate &&
                gate.search_and_identifiability_gate;
  return gate;
}

} // namespace sfr::geometry
