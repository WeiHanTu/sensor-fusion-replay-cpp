#include "sfr/geometry/extrinsic_recovery.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <string>

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

} // namespace sfr::geometry
