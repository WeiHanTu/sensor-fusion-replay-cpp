# Reviewability Maintenance Verification

Date: 2026-10-06. This is maintenance after `v0.1.0`, not a new feature release.
The [historical release record](v0.1_verification.md) remains unchanged in meaning.

## Source and scope

Fresh local builds and synthetic artifacts below use clean implementation commit
`3c48c50521fdcb47fc14a16751afa663556fcc8c`. Subsequent evidence-document commits
do not retroactively become the tested local build commit. Hosted CI, when run,
is bound to its own exact head SHA. `v0.1.0` remains at `88da6dd`.

- [Walkthrough](code_walkthrough.md): fixed entry/core/proof stations, bilingual
  opening cues, 5/15-minute routes, execution/dependency diagrams, invariant/test
  map, error path, and evidence boundaries.
- Public geometry APIs describe frames/units, ownership/lifetime, ordering,
  failure handling and allocation. No pipeline framework or new dependencies.
- Immutable, validated projection results reject inconsistent status/payloads;
  visible processing no longer fabricates default coordinates.
- Additive summary metadata describes existing partial timing and publication
  guarantees. Schema `1.0.0` metric meanings have not been silently changed.
- A synthetic CLI test covers duplicate calibration → diagnostic/exit 3 → no
  final output. No perception/runtime capabilities have been added.

## Local checks

Host: Apple M4 Pro, arm64, Darwin 25.6.0. Compiler: AppleClang
16.0.0.16000026. Both fresh build directories were newly configured from the
clean implementation commit with pinned GoogleTest acquisition.

| Check | Observed result | Boundary |
| --- | --- | --- |
| Fresh Debug configure/build/CTest | 85/85 passed | `build/review-dev`; warnings-as-errors |
| Fresh Release configure/build/CTest | 85/85 passed | `build/review-release`; warnings-as-errors |
| Standard dev preset build/CTest | 85/85 passed | Existing `build/dev` reconfigured at the same commit |
| Walkthrough focused test command | 24/24 passed | Defined projection/calibration/CLI route |
| format-check | Passed | Standard dev target |
| Initial clang-tidy | Failed, then corrected | Unchecked optional access and ineffective move; targeted checks on both corrected files passed; full rerun pending |
| Native ASan+UBSan | Failed at test discovery | Runtime `sanitizer_malloc_mac.inc:189`, `!asan_init_is_running`; not a sanitizer pass |
| Docker sanitizer fallback | Unavailable | Daemon not running; no container tests claimed |
| Synthetic walkthrough commands | Passed | Fixture generator plus both geometry CLIs; no KITTI |
| Artifact checks | Passed | Summary/frame status, timing/publication metadata and projection/z-buffer equations |
| Visual inspection | Passed | Overlay and nine-panel comparison opened; labels and geometry inspected |
| Navigation | Passed | 28 local link targets exist; 10 named route/test symbols located |
| High-level Mermaid rendering | Not run yet | Source is present; rendering is distinct from link checks |
| Human 15-minute rehearsal | Not run | Requires an unfamiliar engineer; static checks do not prove usability |
| Private KITTI smoke rerun | Not run | Historical private-data evidence is not relabeled as fresh |

85 configured checks comprise 80 GoogleTest cases, four CLI help checks, and one
benchmark smoke. Counts are not a claim of exhaustive correctness.

Commands run from repository root:

```bash
cmake --preset dev -B build/review-dev
cmake --build build/review-dev --parallel 4
ctest --test-dir build/review-dev --output-on-failure
cmake --preset release -B build/review-release
cmake --build build/review-release --parallel 4
ctest --test-dir build/review-release --output-on-failure
cmake --preset dev
cmake --build --preset dev --parallel 4
ctest --preset dev --output-on-failure
cmake --build --preset dev --target format-check
cmake --build --preset dev --target clang-tidy
ctest --preset dev -R 'PointCloudProjectionTest|ProjectionTest|CalibrationTest|ProjectKittiCliTest' --output-on-failure
cmake --preset asan-ubsan -B build/review-asan
cmake --build build/review-asan --parallel 4
```

The final sanitizer build command failed; sanitizer CTest was consequently not
run. No warning/sanitizer settings were weakened. Walkthrough generator and CLI
commands were executed exactly as listed in [the guide](code_walkthrough.md#reproduce-proof-without-kitti).

## Regression evidence

Before the geometry fix, three tests ran and failed because construction threw
nothing: `RejectsVisibleResultWithoutPayload`,
`RejectsRejectionResultWithPayload`, and `RejectsUnknownResultStatus`.
After the fix, all 13 projection/point-cloud tests passed. This is an API
consistency defect, not a claim that KITTI previously produced corrupted images.

Both `DescribesTimingAndPublicationBoundaries` tests failed before report metadata
was added (`measurement` absent). Both passed afterward, including exact scope,
exclusion/publication fields and the unchanged measured-stage sum. The duplicate
calibration CLI test passed; it strengthens proof of an existing failure path.

## Retained local artifacts

Generated outputs remain ignored, not part of the public reproducibility contract:

- `artifacts/walkthrough_fixture`: project-authored synthetic KITTI-shaped input.
- `artifacts/walkthrough_projection`: one successful projection run.
- `artifacts/walkthrough_sensitivity`: baseline/eight perturbations/comparison.
- `artifacts/benchmarks/projection_benchmark_reviewability.json`: clean Release
  benchmark at the implementation commit; 10 warm-ups and 100 measured
  iterations of 100,000 points; all 110 iterations accounted.
- Benchmark SHA-256:
  `06b28bcde914f0d4ec289b00c66aadf7a2cdfafce99a4c8b4a8420c118729136`.

The new local benchmark is a harness verification run, not a published speedup
or regression conclusion. Other developer checks were active during this run;
there was no controlled performance comparison. The retained public benchmark
at `185e287` remains explicitly historical.

## Publication and remaining work

The static-analysis follow-up adds an explicit optional guard and copies the
trivially-copyable payload. Full clean-build verification of that follow-up is
pending in this checkpoint; the table above belongs to the initial implementation
commit, not silently to the follow-up.

Push/hosted CI status must be checked against the exact final commit; the old
release's passing CI cannot accept these changes. Human timed rehearsal remains
pending even when commands and CI pass. Crash-durable storage, transactional
overwrite, runtime metrics, synchronization, queues and perception remain outside
this maintenance change.
