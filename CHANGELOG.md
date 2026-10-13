# Change log

This log separates unreleased maintenance from the immutable geometry release.
Exact verification belongs to commit-linked evidence records, not this summary.

## Unreleased

### Added

- Controlled synthetic camera-yaw/camera-x-translation recovery: explicit keyed
  observations, deterministic bounded search, L2/Huber, held-out evaluation,
  local identifiability diagnostics and staged artifacts.
- Frozen 21-case A–E campaign and retained success/failure evidence; source
  `e876725` has 135/135 C++ checks on declared supported configurations.
- Public API contracts and execution/dependency/invariant-to-test maps.
- Contributor onboarding, reporting templates and offline repository checks.

### Changed

- Landing page separates implemented capabilities, first run and evidence limits.
- CMake minimum corrected to 3.28 to match features already used; scientific
  behavior, schemas and acceptance thresholds are unchanged.
- Dependency fallback and Action refs use immutable commits. CI uses Ubuntu 24.04,
  retains bounded test diagnostics and exercises the public Release quick start.
- Sanitizer targets use `-O1` with Debug assertions, symbols, ASan/UBSan and the
  complete test matrix intact; no acceptance deadline or threshold was relaxed.

### Known limitations

- Recovery is synthetic known-correspondence evidence, not real/KITTI calibration.
- Off-grid, weak-geometry and bound-limited failures remain visible.
- Native macOS sanitizer initialization fails on the recorded host; supported
  Ubuntu sanitizer verification is independent.
- Perception and asynchronous replay/runtime remain unimplemented.

## v0.1.0 — geometry baseline

Immutable tag at `88da6dd`: validated transforms/frame graph, strict KITTI loading,
rectified projection/accounting, depth overlays, sensitivity, synthetic demo,
projection microbenchmark and build/test/static/sanitizer gates.
See [historical verification](docs/v0.1_verification.md) and
[published release](https://github.com/WeiHanTu/sensor-fusion-replay-cpp/releases/tag/v0.1.0).
