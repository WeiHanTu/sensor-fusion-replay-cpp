#ifndef SFR_GEOMETRY_EXTRINSIC_RECOVERY_HPP_
#define SFR_GEOMETRY_EXTRINSIC_RECOVERY_HPP_

#include <cstdint>
#include <optional>
#include <span>
#include <vector>

#include "sfr/geometry/point_cloud_projection.hpp"

namespace sfr::geometry {

/// Authored target pixel only; oracle camera depth is deliberately excluded.
struct ObservedPixel final {
  double u_px;
  double v_px;
};

/// Explicit binding of a stable source ID to an authored point and target pixel.
struct ExtrinsicObservation final {
  std::uint64_t source_index;
  core::PointXYZI point_lidar;
  ObservedPixel observed_pixel;
  double weight{1.0};
};

/// Owned observations for one authored scene; source IDs are unique within it.
struct ExtrinsicObservationFrame final {
  std::uint64_t frame_id;
  std::vector<ExtrinsicObservation> observations;
};

/// Left-applied camera +y rotation followed by camera +x translation.
struct ExtrinsicCorrection final {
  double yaw_camera_y_rad{0.0};
  double translation_camera_x_m{0.0};
};

enum class RecoveryLoss : std::uint8_t { kSquaredL2, kHuber };

/// Huber transition is radial pixel error; weights remain fixed for a search.
struct RecoveryLossConfig final {
  RecoveryLoss kind{RecoveryLoss::kHuber};
  double huber_delta_px{2.0};
};

/// Complete classification of a fixed observation set under one candidate.
///
/// Absent loss means rejected candidate, not a zero residual. Projection counts
/// include explicit zero-weight masks; only positive weights affect the loss.
struct ExtrinsicCandidateEvaluation final {
  ExtrinsicCorrection correction;
  ProjectionCounts projection_counts{};
  std::uint64_t positive_weight{0U};
  std::uint64_t zero_weight{0U};
  std::uint64_t positive_weight_visible{0U};
  std::uint64_t positive_weight_invalid{0U};
  double total_weight{0.0};
  bool numeric_failure{false};
  std::optional<double> loss_sum;
  std::optional<double> loss_mean;
};

/// Validates finite values, image bounds, unique frame/source IDs and weights.
///
/// Each frame must be nonempty with positive active weight. Inputs are borrowed;
/// no references survive the call. Throws GeometryError for malformed input.
void validateExtrinsicObservations(std::span<const ExtrinsicObservationFrame> frames,
                                   const RectifiedProjection& projection);

/// Constructs Delta * T_nominal using yaw first, camera-x translation second.
/// @throws GeometryError For non-finite correction or wrong nominal endpoints.
[[nodiscard]] RigidTransform
applyExtrinsicCorrection(const RigidTransform& T_nominal_camera_rect_00_lidar,
                         const ExtrinsicCorrection& correction);

/// Evaluates every observation in canonical frame/source order.
///
/// Inputs are borrowed and copied into a canonical input-span/source-key map.
/// Invisible positive-weight observations reject the entire candidate. No point
/// is silently omitted; arithmetic failure is reported separately from visibility.
/// @throws GeometryError For invalid observations, loss or transform convention.
[[nodiscard]] ExtrinsicCandidateEvaluation evaluateExtrinsicCandidate(
    const RigidTransform& T_nominal_camera_rect_00_lidar, const RectifiedProjection& projection,
    std::span<const ExtrinsicObservationFrame> frames, const ExtrinsicCorrection& correction,
    const RecoveryLossConfig& loss = {});

} // namespace sfr::geometry

#endif // SFR_GEOMETRY_EXTRINSIC_RECOVERY_HPP_
