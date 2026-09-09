# Repository Operating Contract

This file governs every change in this repository. Read it together with
[`intend.md`](intend.md), [`spec.md`](spec.md), and [`plan.md`](plan.md) before
editing code.

## Mission

Build a small, complete, C++20-first offline sensor replay system that produces
defensible evidence of geometry, runtime behavior, testing, and performance.
This is a portfolio-quality engineering project, not an autonomous-driving
stack and not a dependency-integration demo.

Be direct. Do not describe planned, mocked, synthetic, or locally observed
behavior as implemented, validated, production-ready, or safe.

## Authority and change control

When requirements conflict, use this order:

1. `intend.md`: product purpose, evidence limits, success, and non-goals.
2. `spec.md`: required behavior, interfaces, invariants, and acceptance gates.
3. `plan.md`: implementation order and release checklist.
4. Code and comments.

If implementation needs to violate `spec.md`, update the spec in the same
change and explain why. Do not silently weaken an acceptance criterion. Keep
the four documents synchronized with architectural or scope changes.

`intend.md` is intentionally named to match the repository request. Do not
create a competing `intent.md` unless the repository is migrated in one atomic
change and every reference is updated.

## Scope discipline

- Finish and tag one version before starting the next: `v0.1 Geometry`, then
  `v0.2 Perception`, then `v0.3 Runtime`.
- `v0.4 ROS 2` is parked. Do not add ROS 2, rosbag, tf2, CUDA, TensorRT, a neural
  detector, or a GUI before all `v0.3` gates pass.
- Eigen and OpenCV are the only required runtime dependencies through `v0.1`.
  A small JSON library is allowed for machine-readable reports. GoogleTest is
  test-only. PCL requires a measured justification and a spec change.
- Prefer a transparent implementation over a library call when the algorithm
  is itself part of the claimed C++ evidence. Do not reimplement image codecs,
  linear algebra, or JSON serialization.
- Treat the hazard result as a geometric proximity observation. Never call it
  object recognition, collision prediction, tracking, or a safety system.

## Evidence rules

- Repository contents prove only what can be built or reproduced from the
  checked-out commit using documented commands.
- A passing synthetic test proves a contract on its fixture; it does not prove
  correctness on all KITTI scenes.
- A visual overlay is a sanity check, not calibration ground truth.
- Calibration sensitivity reports change relative to the supplied calibration;
  they do not estimate calibration accuracy.
- Performance numbers must include commit, build type, compiler, hardware/OS,
  configuration, warm-up, sample count, and percentile definition.
- Never invent benchmark values or copy stale values into the README. Generate
  them from a release build and preserve the report that supports the claim.
- Keep algorithm processing time, queue wait time, replay pacing time, and
  end-to-end sojourn time as separate metrics.
- Label sample output as `synthetic` or `KITTI` and `measured` or `illustrative`.

## Dataset and licensing rules

- Never commit KITTI images, point clouds, calibration files, timestamps,
  tracklets, account credentials, download cookies, or archives.
- Never automate around KITTI authentication or redistribute material from the
  KITTI site. Users obtain data themselves and pass a local path.
- `.gitignore` must exclude `data/`, `datasets/`, `artifacts/`, generated replay
  output, archives, and common KITTI directory names.
- Unit/integration fixtures must be authored synthetic data small enough to
  audit. Include provenance in each fixture README.
- Public documentation may link to KITTI and show synthetic project-owned
  output. Do not publish KITTI-derived screenshots unless redistribution rights
  have been checked and recorded.
- The repository must fail with an actionable message when the private dataset
  path is absent; downloading data is not application behavior.

## C++ and API rules

- Use C++20 and target-based CMake. No global include directories, compile
  flags, or link flags.
- Production code must compile with `-Wall -Wextra -Wpedantic -Werror` on the
  supported GCC/Clang CI configuration.
- Use RAII and value semantics by default. No owning raw pointers and no raw
  `new`/`delete`. Express exclusive ownership with `std::unique_ptr` only when
  value ownership is unsuitable.
- Apply `const` correctness. Use `std::optional` only for expected absence, not
  to hide parse or invariant failures.
- Use `std::chrono` types at timing boundaries. Do not pass unitless integer
  timestamps or durations.
- Use fixed-width integer types in serialized schemas and counters.
- A transform named `T_target_source` maps coordinates from `source` into
  `target`. Do not introduce ambiguous names such as `extrinsic` or `transform`
  without frame names.
- Validate rotations, finite values, dimensions, file sizes, timestamp order,
  and positive camera depth at boundaries. Fail closed on malformed calibration
  or input rather than emitting plausible-looking output.
- Library code must not call `std::exit`, print to stdout/stderr, read global
  environment implicitly, or depend on the current working directory.
- Catch errors once at the CLI boundary, emit a concise diagnostic, and return a
  documented nonzero exit code.
- Concurrency code must have explicit shutdown and wake-up semantics. A blocked
  producer or consumer must be releasable by queue closure.
- Deterministic algorithms and tests must accept or record a seed. Never depend
  on unordered container iteration for serialized or golden output.

## Architecture boundaries

- `sfr_geometry`: frame transforms, frame graph, and projection math. Eigen only.
- `sfr_io`: KITTI layout, calibration, timestamps, images, and point clouds.
- `sfr_perception`: filtering, ground removal, clustering, proximity analysis.
- `sfr_runtime`: queues, synchronization, replay scheduling, injection, metrics.
- `sfr_viz`: overlays and artifact rendering. OpenCV-facing visualization only.
- `apps/*`: argument parsing and composition. Business logic does not live here.

Dependencies flow inward toward data types and geometry. Runtime orchestration
may depend on the other libraries; geometry must not depend on I/O, runtime,
visualization, or CLI code.

## Testing rules

- Every bug fix gets a regression test that fails before the fix.
- Geometry tests use hand-computable synthetic cases and numeric tolerances with
  units. Avoid self-fulfilling tests that compute expected values through the
  production path.
- Parser tests cover missing keys, duplicate keys, wrong element counts,
  non-finite values, truncated point records, malformed timestamps, and
  non-monotonic timestamps.
- Concurrency tests use deterministic barriers/fake clocks where possible, not
  arbitrary sleeps. Time-bounded deadlock tests may use conservative deadlines.
- Queue accounting must prove every accepted item was processed or assigned one
  explicit drop reason.
- Percentile tests use fixed samples and the nearest-rank definition in
  `spec.md`.
- Tests never require a KITTI login or private dataset. Private-data smoke tests
  are opt-in and cannot be required by CI.
- Run sanitizer tests for memory and undefined behavior. Add ThreadSanitizer
  before accepting `v0.3`.

## Required verification

The repository must converge on these stable commands; do not document a
command until it works:

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

Before declaring a version complete:

1. Run the relevant configure, build, test, sanitizer, format, and static-analysis
   commands from a clean build directory.
2. Run the phase's acceptance scenario and retain its machine-readable report.
3. Inspect generated images rather than assuming successful file creation means
   correct rendering.
4. Check `git diff --check` and review the full diff for accidental datasets,
   binaries, absolute local paths, secrets, and unrelated changes.
5. Record exact results in the release notes/README. If a check was not run, say
   `not run`; do not imply it passed.

## Repository hygiene

- Keep commits narrow and ordered by `plan.md`. Do not mix refactors with
  behavioral changes.
- Do not commit build directories, IDE metadata, downloaded data, profiling
  captures, or unreviewed generated output.
- Generated public artifacts must be reproducible by a documented command.
- Preserve user changes in a dirty worktree. Do not reset, discard, or rewrite
  unrelated work.
- Do not weaken warnings, tests, sanitizers, or gates to make CI green. Fix the
  defect or document a narrowly scoped, justified exception.

## Definition of done

A task is done only when code, tests, documentation, and acceptance evidence
agree. A version is done only when every gate for that version in `spec.md` and
`plan.md` is checked with captured evidence. “Compiles on my machine,” an image
that looks plausible, or a benchmark without provenance is not done.
