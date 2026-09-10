# Implementation and Verification Plan

## 1. Current state

Planning baseline on 2026-09-09:

- Before this work, the local workspace directory was empty. It now contains the
  planning contract and a local `v0.1` checkpoint through the synthetic
  calibration-sensitivity experiment.
- The local repository is initialized on `main` with the intended `origin` and
  local checkpoint commits. The public GitHub repository was confirmed empty
  before local initialization; no push has been performed.
- Frame-labelled rigid transforms, a deterministic cycle-free frame graph, a
  full `3x4` rectified projection primitive, and strict synthetic KITTI
  calibration/data loaders are implemented and locally tested. The one-frame
  projection CLI now emits an atomic depth-overlay/JSON/JSONL run directory on
  authored synthetic input.
- AppleClang 16 cannot run its ASan runtime on the current macOS 26.6 host.
  Equivalent clean Ubuntu 24.04 container runs passed GCC 13 debug tests and
  Clang 18 ASan+UBSan tests. Hosted GitHub Actions has not run.
- Synthetic projection and calibration-sensitivity outputs have been generated
  and inspected. The authored projection microbenchmark and its CLI/schema
  regression tests are implemented. A clean-commit Apple M4 Pro release report
  is retained under `docs/results/` and interpreted in
  `docs/v0.1_verification.md`. No private-data smoke run, push, tag, or release
  claim exists yet. Authored synthetic output proves fixture contracts; it is
  not real-data or publication-quality portfolio evidence.

Status vocabulary:

- `[ ]` not started
- `[~]` in progress, not accepted
- `[x]` implemented and verified with recorded evidence
- `[!]` blocked or failed; reason must follow the item

Do not mark an item `[x]` based only on code review. Run its stated checks.

## 2. Delivery strategy

The critical path is `v0.1`. Stop after a clean, demonstrable `v0.1` if an
interview deadline is within one week. Do not trade geometry tests and release
evidence for partially implemented perception/runtime features.

Estimated focused effort:

| Release | Estimate | Outcome |
| --- | ---: | --- |
| Phase 0 + `v0.1 Geometry` | 5-7 days | Complete public C++/geometry evidence |
| `v0.2 Perception` | 5-7 days | Deterministic geometric obstacle analysis |
| `v0.3 Runtime` | 5-7 days | Measured sync/backpressure/drop behavior |
| `v0.4 ROS 2` | Parked | No work until a spec revision |

Each phase ends with a release gate. Later work begins only after the preceding
gate passes or a failure is explicitly preserved and the version is not tagged.

## 3. Phase 0 — repository and evidence foundation (0.5-1 day)

### 0.1 Initialize without destroying remote state

- [x] Reconfirm the remote is empty.
- [x] Run `git init -b main` in the exact workspace.
- [x] Add `origin` as
  `https://github.com/WeiHanTu/sensor-fusion-replay-cpp.git`.
- [x] Add a conservative `.gitignore` before any dataset is placed nearby.
- [x] Choose Apache-2.0 for the project-authored code and documentation, with its
  explicit patent grant. The repository does not license or redistribute KITTI.
- [x] Create a local checkpoint commit after the implemented baseline checks
  pass. A local commit is not publication; do not push, tag, or release until
  the project-code license is accepted.

Verification:

```bash
git status --short --branch
git remote -v
git diff --check
git ls-files | rg '(^|/)(data|datasets|artifacts|build)(/|$)' && exit 1 || true
```

Exit gate: clean repository metadata, correct remote, accepted project-code
license, no dataset/generated content tracked, and a deliberate owner-approved
push. The local checkpoint alone does not pass this gate.

### 0.2 Freeze reproducible inputs

- [ ] Select one KITTI Raw synced drive for local manual testing and record only
  its public sequence ID, never local absolute paths.
- [ ] Verify it has `image_02`, `velodyne_points`, their timestamp files, and the
  matching daily camera/Velodyne calibration.
- [ ] Record the private smoke-test frame ID/range in a local ignored config.
- [x] Record the intended reference benchmark host: macOS Darwin 25.6.0,
  arm64, Apple M4 Pro.
- [ ] Decide that public visuals remain synthetic unless redistribution
  permission for real-data screenshots is documented.

Exit gate: the private input exists locally, remains ignored, and its layout is
manually checked. No automatic KITTI downloader is added.

## 4. `v0.1 Geometry` (5-7 days total)

### 1.1 Build, style, and CI skeleton (0.5-1 day)

- [x] Add root `CMakeLists.txt` and `CMakePresets.json` with `dev`, `release`,
  and `asan-ubsan` configure/build/test presets.
- [x] Add target-scoped warning helper and sanitizer helper under `cmake/`.
- [x] Add reproducible dependency discovery/acquisition. Eigen, OpenCV,
  nlohmann/json, and GoogleTest use system-first discovery with pinned fallback
  where the dependency is safe to fetch.
- [x] Add `sfr_geometry` plus public tests; prove include/build boundaries.
  Installation/export boundaries remain deferred until a consumable SDK exists.
- [x] Add `.clang-format`, `.clang-tidy`, `format`, `format-check`, and
  `clang-tidy` targets.
- [~] Add Ubuntu GitHub Actions jobs for GCC, Clang, and ASan/UBSan. Workflow is
  written; hosted execution is unverified because nothing has been pushed.
- [x] Add a minimal README capability table that separates locally verified,
  in-progress, and planned work.

Verification:

```bash
cmake --preset dev
cmake --build --preset dev --parallel
ctest --preset dev --output-on-failure
cmake --preset asan-ubsan
cmake --build --preset asan-ubsan --parallel
ctest --preset asan-ubsan --output-on-failure
cmake --build --preset dev --target format-check
cmake --build --preset dev --target clang-tidy
```

Exit gate: all commands work from a clean checkout and CI passes on both
compilers. If macOS differs, document the exact dependency setup rather than
adding platform conditionals blindly.

### 1.2 Strong geometry primitives (1 day)

- [x] Implement fixed-size aliases, frame IDs/tags, `RigidTransform`, validation,
  inverse, composition, and point/batch application.
- [x] Implement `FrameGraph` with inverse traversal, deterministic lookup,
  duplicate/inconsistent-edge rejection, and missing-path semantics.
- [x] Implement the full `3x4` rectified projection primitive without raster or
  OpenCV code. Calibration-chain composition remains in the I/O slice.
- [x] Write the frame convention/projection document and Mermaid frame tree;
  mark world/vehicle edges unresolved.
- [x] Add hand-computable tests for identity, inverse, composition order, invalid
  rotations, frame mismatch, positive depth, bounds, and numeric tolerance.

Verification:

```bash
ctest --preset dev -R 'geometry|frame_graph|projection' --output-on-failure
ctest --preset asan-ubsan -R 'geometry|frame_graph|projection' --output-on-failure
```

Exit gate: all Section 14 `v0.1` geometry tolerances in `spec.md` pass. Review
names specifically for source/target reversal; a plausible overlay is not a
substitute.

### 1.3 Strict synthetic fixtures and KITTI I/O (1-1.5 days)

- [x] Author `tests/fixtures/synthetic/README.md` with provenance and independent
  expected values.
- [~] Create a tiny project-owned image, valid XYZI binary frame, calibration,
  and timestamps. Calibration is checked in; binary/image/timestamp variants are
  generated hermetically by tests. Full synchronizer/perception fixture cases
  remain pending their owning phases.
- [x] Add a staged authored synthetic fixture generator for the public demo;
  verify the generated image, point cloud, calibration, timestamps, explicit
  overwrite, and staging ownership through the strict loaders.
- [x] Implement typed parse errors and strict key/value parser.
- [x] Implement KITTI daily calibration loading and rotation/dimension checks.
- [x] Implement numeric frame enumeration, BGR8 image loading, XYZI loading,
  finite-point filtering, and timestamp parsing on one shared civil-time scale.
  Replay-origin normalization belongs to `v0.3` scheduling.
- [x] Implement layout validation that rejects unsynced/unrectified or mismatched
  sequence structures.
- [x] Add allocation/file-size caps and actionable diagnostics.
- [~] Test every failure mode without accessing network or private KITTI. Core
  parser, ordering, decode, finite-value, dimension, monotonicity, and count
  errors are covered; explicit resource-limit tests remain.

Verification:

```bash
ctest --preset dev -R 'calibration|kitti_io|timestamp|fixture' --output-on-failure
ctest --preset asan-ubsan -R 'calibration|kitti_io|timestamp|fixture' --output-on-failure
```

Exit gate: valid fixture parses; every malformed fixture fails with the expected
typed category; tests remain hermetic.

### 1.4 Projection pipeline and artifacts (1 day)

- [x] Compose the exact KITTI chain
  `P_rect_02 * R_rect_00 * T_camera_raw_00_lidar`.
- [x] Implement rejection-reason counts and common `ProjectedPoint` output.
- [x] Implement deterministic z-buffering and depth coloring with the colormap
  and range recorded in the sidecar report.
- [x] Implement atomic run-directory/report writing and schema validation tests.
- [x] Implement `project_kitti` CLI, help/defaults, exit codes, overwrite guard,
  and synthetic integration test.
- [x] Generate and inspect a synthetic overlay; check expected pixels before
  accepting its appearance.
- [ ] Run one private KITTI frame locally, inspect the overlay, and record only
  command shape/result status plus non-sensitive metadata.

Verification:

```bash
./build/dev/project_kitti --help
ctest --preset dev \
  -R 'project_kitti|projection_artifact|overlay' \
  --output-on-failure
```

The integration test authors its complete synthetic drive in a temporary
directory. Do not document the text-only `tests/fixtures/synthetic` directory as
a directly runnable drive.

Exit gate: synthetic output pixels and JSON match independent expectations;
private real-data output has been visually inspected but is still untracked.

### 1.5 Calibration sensitivity experiment (0.5-1 day)

- [x] Implement explicit left-applied camera-frame perturbations.
- [x] Implement common-visible-set median/p95 displacement and appearance/
  disappearance counters.
- [x] Implement `analyze_calibration` CLI and output panel layout.
- [x] Test signs/axes with a hand-computed synthetic point, including empty common
  set failure.
- [x] Generate baseline plus four yaw and four translation panels on synthetic
  data and inspect labels/legend.
- [ ] Run the same experiment locally on the chosen private frame.
- [x] Document `50*tan(1 deg)` as an analytic illustration, explicitly separate
  from measured pixel displacement.

Verification:

```bash
./build/dev/analyze_calibration --help
ctest --preset dev \
  -R 'calibration_sensitivity|sensitivity_artifact|panel|analyze_calibration' \
  --output-on-failure
```

Exit gate: nine individual images, combined comparison, and JSON report satisfy
the `v0.1` sensitivity contract; all perturbation metadata is unambiguous. The
synthetic side passes; the private KITTI run remains open.

### 1.6 Benchmark, documentation, and `v0.1` release (0.5-1 day)

- [x] Add projection microbenchmark with at least 100,000 deterministic points,
  warm-up, at least 100 measured iterations, and machine-readable output.
- [x] Run only an optimized build; capture commit/dirty state/compiler/host/config.
  The retained report references clean implementation commit `7432516` and an
  Apple M4 Pro Release build.
- [~] Complete README build/test/private-data/example/benchmark instructions.
  Benchmark usage, scope, and the canonical clean report link are present.
  Fresh-checkout execution and private-data sections remain open.
- [~] Publish synthetic overlay, sensitivity comparison, and frame tree only.
  The reproducible generator and publishing script are implemented; final
  release-build images and README placement are pending.
- [x] Add a capability/evidence table: implemented and verified items only.
- [ ] Run every `v0.1` acceptance item from `spec.md` on a release candidate.
- [x] Review for misleading words: real-time, fusion, accuracy, production,
  detection, collision avoidance, and safety.
- [ ] Tag `v0.1.0` only after the tree is clean and evidence is attached/linked.

Required record:

```text
commit:
tree clean:
GCC CI:
Clang CI:
ASan/UBSan:
format-check:
clang-tidy:
synthetic integration:
private KITTI smoke test (not public data):
visual inspection:
benchmark report:
known limitations:
```

The current, explicitly incomplete record is
[`docs/v0.1_verification.md`](docs/v0.1_verification.md).

Exit gate: every `v0.1` item in Section 14 of `spec.md` passes. Otherwise do not
tag and do not claim `v0.1` complete.

## 5. `v0.2 Perception` (5-7 days)

### 2.1 Deterministic voxel and ROI filtering (1 day)

- [ ] Implement validated ROI and voxel configs.
- [ ] Implement floor-based signed voxel keys, overflow checks, centroids, mean
  reflectance, and sorted output.
- [ ] Test boundaries, negative coordinates, invalid leaf sizes, ordering, and
  exact point accounting.
- [ ] Add a benchmark stage counter without claiming speedup before measurement.

Exit gate: deterministic output is byte-identical across repeated test runs and
all input points have one filter disposition.

### 2.2 Constrained ground RANSAC (1-1.5 days)

- [ ] Implement seeded triplet sampling, degeneracy checks, plane normalization,
  tilt/distance constraints, and deterministic tie-breaking.
- [ ] Return structured success/failure, parameters, seed, inliers, and residual.
- [ ] Test exact synthetic plane, outliers, excessive tilt, insufficient inliers,
  degeneracy, and repeatability.
- [ ] Keep no-ground behavior conservative: retain points and report failure.

Exit gate: the independent synthetic ground fixture passes; repeated same-seed
runs match; failure never silently deletes points.

### 2.3 Euclidean clustering and proximity (1.5 days)

- [ ] Implement deterministic neighbor search and connected components without
  PCL.
- [ ] Test tolerance boundary, chain connectivity, min/max rejection, no
  duplicates, deterministic IDs, and full point accounting.
- [ ] Compute centroid/AABB and corridor intersection.
- [ ] Implement closest positive AABB-x clearance and configurable display bands.
- [ ] Test centroid-outside/AABB-intersects cases and no-cluster output.

Exit gate: known fixture memberships and clearances match independent expected
values; output makes no semantic or time-to-collision claim.

### 2.4 Pipeline integration and `v0.2` release (1.5-2 days)

- [ ] Compose filter -> ground -> cluster -> proximity stages.
- [ ] Extend overlays, `frames.jsonl`, `run_summary.json`, and schema tests.
- [ ] Run a documented private KITTI frame range and manually inspect failure and
  success cases; keep raw/generated dataset material untracked.
- [ ] Benchmark each stage in a release build and publish data with provenance.
- [ ] Run all `v0.1` and `v0.2` gates, sanitizer/static checks, and diff audit.
- [ ] Tag `v0.2.0` only after all acceptance checks pass.

Exit gate: Section 14 `v0.2` acceptance passes, including point accounting and
honest geometric labeling.

## 6. `v0.3 Runtime` (5-7 days)

### 3.1 Closeable bounded queue (1-1.5 days)

- [ ] Implement capacity validation, FIFO storage, blocking push, drop-oldest
  push result, pop, idempotent close, and counters.
- [ ] Define ownership and lifetime of queued frame payloads; measure before
  choosing copies versus moves/shared immutable storage.
- [ ] Test capacity 1/N, multi-producer/consumer contention, close-before-wait,
  close-while-waiting, drain, FIFO, eviction ID, and accounting.
- [ ] Avoid arbitrary sleeps; use barriers/latches and bounded watchdogs.

Exit gate: repeated stress tests, ASan/UBSan, and initial TSan pass; no waiter can
be stranded after closure.

### 3.2 Timestamp synchronizer (1-1.5 days)

- [ ] Implement per-stream ordered buffers, nearest matching, inclusive
  tolerance, deterministic tie-breaking, watermark expiry, and end flush.
- [ ] Emit signed/absolute deltas and explicit unpaired reasons.
- [ ] Test exact/boundary/outside pairs, positive/negative skew, ties, no reuse,
  burst arrival order, expiry, and stream end.
- [ ] Property-test small generated sequences against a slow reference matcher if
  feasible; seed and bound all generation.

Exit gate: deterministic expected pairings and one terminal outcome per frame.

### 3.3 Replay scheduler and injection (1 day)

- [ ] Introduce clock/sleeper interfaces and fake clock.
- [ ] Implement unpaced, realtime, and positive scaled modes using relative
  normalized source timestamps.
- [ ] Implement periodic source drop, fixed/seeded delay, timestamp skew, and
  processing delay with recorded configuration.
- [ ] Test scheduling and injection with fake time; separate original/effective
  timestamps and delivery times.

Exit gate: deterministic tests contain no real waiting and each injected event
maps to exactly one explicit category.

### 3.4 Metrics, accounting, and replay app (1-1.5 days)

- [ ] Implement nearest-rank aggregation with empty/singleton/overflow handling.
- [ ] Instrument decode, queue, synchronization, geometry, perception,
  visualization, serialization, processing, and sojourn boundaries.
- [ ] Implement checked per-stream/per-pair accounting equations and refuse
  `status=complete` when they fail.
- [ ] Implement `replay_sequence`, shutdown/error propagation, output guards, and
  schemas.
- [ ] Add deterministic saturation integration tests for block and drop-oldest.

Exit gate: tests prove capacity, closure, metric definitions, and accounting in
normal, drop, delay, skew, overload, and failure paths.

### 3.5 Performance campaign and `v0.3` release (1-1.5 days)

- [ ] Select warm-up and collect at least 1,000 measured pairs for baseline and
  one overload scenario on the declared host.
- [ ] Record cycling/cache warning if the source sequence is repeated.
- [ ] Capture stage p50/p95/p99/min/max/mean, counts, queue high-water marks,
  synchronization deltas, and all terminal reasons.
- [ ] Evaluate the predeclared `processing_total p95 < 100 ms` 10 Hz goal.
  Report pass or miss; do not redefine the goal after seeing the data.
- [ ] Run full GCC/Clang CI, formatting, clang-tidy, ASan/UBSan, TSan, public
  integration, private smoke, and documentation command audit.
- [ ] Update evidence/capability table and known limitations.
- [ ] Tag `v0.3.0` only after Section 14 acceptance passes.

Exit gate: reproducible runtime evidence, exact accounting, sanitizer-clean
shutdown, and an honest 10 Hz goal result.

## 7. `v0.4 ROS 2` — parked

- [ ] Do nothing until a new spec names the ROS 2 distribution, message schemas,
  timestamp mapping, tf2 ownership, QoS cases, rosbag fixture, CI strategy, and
  acceptance measurements.
- [ ] When activated, keep adapters outside the core libraries and preserve
  offline tests.

Starting ROS 2 early is scope failure, not progress.

## 8. Cross-phase release checklist

Use this checklist for every release candidate:

- [ ] `intend.md`, `spec.md`, `plan.md`, implementation, and README agree.
- [ ] No requirement was silently weakened after a failed result.
- [ ] Clean configure/build/test succeeds with supported GCC and Clang.
- [ ] Relevant sanitizers pass.
- [ ] Format check and clang-tidy pass without blanket suppressions.
- [ ] Synthetic end-to-end command succeeds offline.
- [ ] Private KITTI smoke command succeeds where required; data remains ignored.
- [ ] Generated visuals were opened and inspected.
- [ ] Machine-readable outputs validate and accounting equations hold.
- [ ] Benchmark is optimized, clean-commit, fully attributed, and non-flaky.
- [ ] README commands were executed as written.
- [ ] `git diff --check` passes and full diff contains no local path, secret,
  dataset, archive, binary, build product, or unrelated edit.
- [ ] Capability claims say exactly what was implemented and verified.
- [ ] Known limitations and failed/missed performance goals remain visible.

## 9. Risk register and mitigations

| Risk | Consequence | Mitigation / decision |
| --- | --- | --- |
| Transform direction or rectification error | Plausible but wrong overlay | `T_target_source`, independent numeric fixtures, exact chain tests |
| Treating `P_rect_02` as only intrinsic `K` | Camera-baseline projection error | Preserve full `3x4` projection matrix and test it |
| Visual-only validation | No numeric correctness evidence | Hand-computed projections and perturbation metrics |
| KITTI redistribution | Public-repo licensing problem | Private data only; synthetic public visuals; permission gate |
| Dataset/login blocks CI | Unreliable public build | Hermetic authored fixture; private test opt-in |
| PCL/ROS dependency explosion | Project becomes integration work | Forbidden through `v0.3` without spec revision |
| Flaky timing tests | False failures or false confidence | Fake clocks/barriers; no absolute CI performance gate |
| Misleading p99 on tiny samples | Weak performance claim | At least 1,000 measured pairs and sample count disclosure |
| Cached repeated frames | Inflated throughput | Record cycles and cache warning; keep stage metrics |
| Queue shutdown deadlock | Runtime hangs | Closeable queue contract, waiter tests, TSan, bounded watchdog |
| Silent frame loss | Untrustworthy runtime report | Mutually exclusive terminal reasons and checked equations |
| RANSAC nondeterminism | Irreproducible artifacts | Explicit seed, tie-breaks, sorted output |
| “Hazard” overclaim | Implied safety capability | Geometric clearance only; explicit non-safety language |
| Premature optimization | Obscures design without evidence | Profile release build first; preserve baseline reports |

## 10. Recommended commit sequence

Keep commits reviewable and bisectable:

1. `docs: define intent specification and delivery gates`
2. `build: add C++20 CMake presets quality gates and CI`
3. `feat(geometry): add validated transforms and frame graph`
4. `feat(io): add strict KITTI calibration and frame loaders`
5. `feat(projection): add rectified projection and overlay reports`
6. `feat(calibration): add controlled sensitivity analysis`
7. `docs: publish verified v0.1 evidence`
8. `feat(perception): add deterministic voxel and ground filtering`
9. `feat(perception): add clustering and corridor clearance`
10. `docs: publish verified v0.2 evidence`
11. `feat(runtime): add closeable bounded queues`
12. `feat(runtime): add timestamp synchronization and replay injection`
13. `feat(metrics): add accounting and latency reports`
14. `docs: publish verified v0.3 evidence`

Do not force this sequence when a smaller split is clearer, but never combine a
whole release into one opaque commit.
