#ifndef SFR_GEOMETRY_EXTRINSIC_RECOVERY_HPP_
#define SFR_GEOMETRY_EXTRINSIC_RECOVERY_HPP_

#include <chrono>
#include <cstdint>
#include <numbers>
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

/// Inclusive bounds and lattice steps in radians (yaw) or meters (translation).
struct RecoveryAxisGrid final {
  double minimum;
  double maximum;
  double coarse_step;
  double fine_step;
};

/// Resource limits apply before allocation; diagnostics use training data only.
struct ExtrinsicSearchConfig final {
  RecoveryAxisGrid yaw{-2.0 * std::numbers::pi / 180.0, 2.0 * std::numbers::pi / 180.0,
                       0.1 * std::numbers::pi / 180.0, 0.01 * std::numbers::pi / 180.0};
  RecoveryAxisGrid translation{-0.10, 0.10, 0.005, 0.0005};
  RecoveryLossConfig loss;
  std::uint64_t max_candidates{100000U};
  std::uint64_t max_point_projections{100000000U};
  double rank_relative_tolerance{1e-6};
  double condition_limit{1e4};
};

enum class RecoverySearchStatus : std::uint8_t {
  kGridComplete,
  kBoundLimited,
  kRefinementLimited,
  kNoValidCandidate
};

/// Local span-scaled weighted projection Jacobian, not statistical confidence.
struct RecoveryIdentifiability final {
  bool available{false};
  bool used_one_sided_difference{false};
  std::uint8_t rank{0U};
  double sigma_max{0.0};
  double sigma_min{0.0};
  std::optional<double> condition;
  bool poorly_identifiable{true};
  std::uint64_t points_evaluated{0U};
};

/// Fine-lattice key with all stage membership and candidate accounting retained.
struct RecoveryCandidateRow final {
  std::int64_t yaw_tick;
  std::int64_t translation_tick;
  bool coarse{false};
  bool fine{false};
  ExtrinsicCandidateEvaluation evaluation;
};

/// Owned immutable-by-convention search snapshot; no hidden/validation inputs.
struct ExtrinsicSearchResult final {
  RecoverySearchStatus status{RecoverySearchStatus::kNoValidCandidate};
  std::optional<ExtrinsicCorrection> best;
  std::optional<double> loss_sum;
  std::optional<double> loss_mean;
  double total_weight{0.0};
  bool yaw_bound_hit{false};
  bool translation_bound_hit{false};
  bool refinement_edge_hit{false};
  std::vector<std::uint64_t> consumed_frame_ids;
  std::vector<RecoveryCandidateRow> candidates;
  std::uint64_t points_evaluated{0U};
  std::chrono::nanoseconds search_duration{};
  std::chrono::nanoseconds diagnostics_duration{};
  RecoveryIdentifiability identifiability;
};

/// Loss, absolute yaw/translation, then signed yaw/translation; exact ties only.
[[nodiscard]] bool betterExtrinsicCandidate(const ExtrinsicCandidateEvaluation& left,
                                            const ExtrinsicCandidateEvaluation& right);

/// Exhausts coarse grid and one clipped fine rectangle with deterministic ties.
///
/// Requires at least two training frames. Borrows inputs, canonicalizes once and
/// allocates candidate/residual storage. Finite-grid completion is not continuous
/// convergence or a universal half-grid accuracy guarantee. Timed search excludes
/// preparation and local SVD diagnostics. Throws GeometryError for invalid grids,
/// budgets, observations, or nominal convention, before candidate evaluation.
[[nodiscard]] ExtrinsicSearchResult searchExtrinsicCorrection(
    const RigidTransform& T_nominal_camera_rect_00_lidar, const RectifiedProjection& projection,
    std::span<const ExtrinsicObservationFrame> training, const ExtrinsicSearchConfig& config = {});

/// One keyed diagnostic residual. Invisible predictions have no pixel/error.
struct RecoveryResidual final {
  std::uint64_t frame_id;
  std::uint64_t source_index;
  double weight;
  ObservedPixel target;
  ProjectionStatus status;
  std::optional<ObservedPixel> predicted;
  std::optional<double> radial_error_px;
};

/// Unweighted all-retained-observation metrics; incomplete coverage is conditional.
struct RecoveryResidualMetrics final {
  std::uint64_t frame_id{0U}; // meaningful only for per-frame rows
  ProjectionCounts counts{};
  std::uint64_t zero_weight{0U};
  std::uint64_t common_visible{0U};
  std::optional<double> median_px;
  std::optional<double> p95_px;
  std::optional<double> rmse_px;
};

struct RecoveryTransformMetrics final {
  RecoveryResidualMetrics pooled;
  std::vector<RecoveryResidualMetrics> frames;
  std::vector<RecoveryResidual> residuals;
};

/// Nominal/recovered/oracle evaluation, never used as a search input or prior.
struct RecoveryComparison final {
  RecoveryTransformMetrics nominal;
  std::optional<RecoveryTransformMetrics> recovered;
  RecoveryTransformMetrics oracle;
  std::uint64_t points_evaluated{0U};
};

/// Evaluates fixed transforms on borrowed keyed data and returns owned diagnostics.
///
/// Uses the existing projection contracts; primary errors include outliers and
/// zero-weight masks. Null metrics mean no visible samples, not perfect fit.
/// Canonical keys define the common-visible intersection across available models.
/// Throws GeometryError for malformed observations/transform convention.
[[nodiscard]] RecoveryComparison
compareExtrinsicCorrections(const RigidTransform& nominal, const RectifiedProjection& projection,
                            std::span<const ExtrinsicObservationFrame> frames,
                            const std::optional<ExtrinsicCorrection>& recovered,
                            const ExtrinsicCorrection& oracle);

/// Frozen noiseless gate; noisy/diagnostic campaigns retain these pass/miss fields.
struct RecoveryAcceptance final {
  std::optional<double> signed_yaw_error_deg;
  std::optional<double> signed_translation_error_m;
  bool parameter_gate{false};
  bool full_coverage{false};
  bool held_out_gate{false};
  bool search_and_identifiability_gate{false};
  bool passed{false};
};

/// Post-fit only: half-fine-step parameter errors and frozen held-out residual gates.
/// Nonzero truth requires >=90% improvement in median/p95/RMSE. Zero truth uses
/// RMSE <=1e-9 px. No limited/unidentifiable result can pass this gate.
[[nodiscard]] RecoveryAcceptance
assessExtrinsicRecovery(const ExtrinsicSearchResult& result, const ExtrinsicSearchConfig& config,
                        const ExtrinsicCorrection& hidden,
                        const RecoveryComparison& clean_validation);

} // namespace sfr::geometry

#endif // SFR_GEOMETRY_EXTRINSIC_RECOVERY_HPP_
