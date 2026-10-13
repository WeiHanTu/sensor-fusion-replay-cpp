# Presentation-only extraction from completed raw reports, in frozen campaign order.
# Usage: jq -s --slurpfile campaign ROOT/campaign.json -f THIS_FILE ROOT/runs/*/report.json
def metrics:
  {
    median_px: [.nominal.pooled.median_px, .recovered.pooled.median_px, .oracle.pooled.median_px],
    p95_px: [.nominal.pooled.p95_px, .recovered.pooled.p95_px, .oracle.pooled.p95_px],
    rmse_px: [.nominal.pooled.rmse_px, .recovered.pooled.rmse_px, .oracle.pooled.rmse_px],
    visible: [.nominal.pooled.counts.visible, .recovered.pooled.counts.visible, .oracle.pooled.counts.visible],
    invalid: [.nominal.pooled.counts.invalid, .recovered.pooled.counts.invalid, .oracle.pooled.counts.invalid],
    common: [.nominal.pooled.common_visible, .recovered.pooled.common_visible, .oracle.pooled.common_visible]
  };
. as $reports | $campaign[0] as $campaign |
[$campaign.cases[].case_id as $id | $reports[] | select(.run_id == $id)] as $ordered |
if (($ordered | length) != ($campaign.cases | length)) or
   (any($campaign.cases[]; .case_id as $id | ([$reports[] | select(.run_id == $id)] | length) != 1)) or
   (any($ordered[]; .schema_name != "sfr.extrinsic_recovery" or .schema_version != "1.0.0" or
     .status != "complete" or .build != $campaign.build))
then error("missing/duplicate/incompatible run reports") else
{
  schema_name: "sfr.extrinsic_recovery_evidence",
  schema_version: "1.0.0",
  build: $campaign.build,
  host: $campaign.host,
  required_gates_passed: $campaign.required_gates_passed,
  B_noise: $campaign.B_noise,
  C_outliers: $campaign.C_outliers,
  claim_boundary: "Authored synthetic known-correspondence two-parameter recovery only; not physical/KITTI calibration, HIL, sim-to-real or production.",
  fixture_version: $ordered[0].fixture.version,
  generator_version: $ordered[0].fixture.generator_version,
  train_frames: [0,1], held_out_frames: [2], training_points: 372, held_out_points: 160,
  metric_array_model_order: ["nominal","recovered","oracle"],
  projection_matrix: $ordered[0].model.projection_matrix,
  nominal_rotation: $ordered[0].model.nominal_transform.rotation,
  performance_scope: {
    explicit_warmup: 0, timed_search_samples_per_case: 1, percentiles: "not_computed",
    search_ms_scope: "coarse/fine candidate evaluation and selection; canonical preparation and result-table materialization excluded",
    visibility_definition: "positive-depth in-image projection validity, not physical occlusion or independent real correspondences",
    replay: "separate verification pass excluded from primary search",
    interpretation: "Elapsed diagnostics, not a steady-state latency benchmark"
  },
  cases: [$ordered[] | {
    case_id: .run_id, fixture_identity: .fixture.content_identity,
    seed: .fixture.seed, geometry: .fixture.geometry,
    noise_stddev_px: .fixture.noise.theoretical_stddev_px,
    outlier_fraction: .fixture.outliers.training_fraction,
    nominal_translation_m: [.model.nominal_transform.translation_m[][0]],
    hidden: .model.hidden_parameters, recovered: .model.recovered_parameters,
    search: {
      config: .search.config, loss: .search.loss_name, status: .search.result_status,
      evaluations: .search.evaluations, points_evaluated: .search.points_evaluated,
      global_bound_hits: .search.global_bound_hits, refinement_edge_hit: .search.refinement_edge_hit,
      identifiability: .search.identifiability
    },
    parameter_errors: .parameter_errors,
    train_observed: (.metrics.train.observed | metrics),
    held_out_observed: (.metrics.validation.observed | metrics),
    held_out_clean: (.metrics.validation.clean | metrics),
    acceptance: .acceptance, reproducible: .reproducibility.equal
  }]
} end
