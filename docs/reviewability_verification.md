# Reviewability Maintenance Verification

Date: 2026-10-06. This is maintenance after `v0.1.0`, not a new feature release.
The [historical release record](v0.1_verification.md) remains unchanged in meaning.

## Source and scope

Fresh local builds and synthetic artifacts below use clean implementation commit
`3c48c50521fdcb47fc14a16751afa663556fcc8c`. Subsequent evidence-document commits
do not retroactively become the tested local build commit. Hosted CI, when run,
is bound to its own exact head SHA. `v0.1.0` remains at `88da6dd`.

- [Architecture](architecture.md): execution/dependency diagrams, ownership,
  invariant/test map, error path, and evidence boundaries. Public documentation
  presents engineering design; interview/rehearsal notes remain local and ignored.
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
| Focused test command | 24/24 passed | Projection/calibration/CLI contracts |
| format-check | Passed | Standard dev target |
| Initial clang-tidy | Failed, then corrected | Unchecked optional access and ineffective move; targeted checks passed; full follow-up result recorded below |
| Native ASan+UBSan | Failed at test discovery | Runtime `sanitizer_malloc_mac.inc:189`, `!asan_init_is_running`; not a sanitizer pass |
| Docker sanitizer fallback | Unavailable | Daemon not running; no container tests claimed |
| Synthetic demo commands | Passed | Fixture generator plus both geometry CLIs; no KITTI |
| Artifact checks | Passed | Summary/frame status, timing/publication metadata and projection/z-buffer equations |
| Visual inspection | Passed | Overlay and nine-panel comparison opened; labels and geometry inspected |
| Navigation | Passed | 28 local link targets exist; 10 named route/test symbols located |
| Original high-level Mermaid rendering | Passed | Execution/dependency graphs rendered at `1dceeca` in GitHub Preview; not a check of later README rendering |
| Independent documentation usability evaluation | Not run | Static checks do not prove usability |
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
run. No warning/sanitizer settings were weakened. Synthetic generator and CLI
commands used the retained local paths recorded below; the public architecture
document provides the same workflow with separate output directories.

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

## Final implementation verification

Clean source: `1dceeca5eef3315afa6ed5081247881ac173089a`, including the
static-analysis follow-up. New `build/review-final-dev` and
`build/review-final-release` directories each configured, built and passed 85/85
checks. Debug format-check and the full 33-translation-unit clang-tidy target
passed. Dependency/system warnings were filtered by the existing configuration;
this is not a zero-warning claim. No suppressions were added.

```bash
cmake --preset dev -B build/review-final-dev
cmake --build build/review-final-dev --parallel 4
ctest --test-dir build/review-final-dev --output-on-failure
cmake --build build/review-final-dev --target format-check
cmake --build build/review-final-dev --target clang-tidy
cmake --preset release -B build/review-final-release
cmake --build build/review-final-release --parallel 4
ctest --test-dir build/review-final-release --output-on-failure
./build/review-final-dev/generate_synthetic_kitti --output-dir artifacts/walkthrough_final_fixture
./build/review-final-dev/project_kitti --dataset-root artifacts/walkthrough_final_fixture --drive synthetic_drive_sync --frame 0 --output-dir artifacts/walkthrough_final_projection
./build/review-final-dev/analyze_calibration --dataset-root artifacts/walkthrough_final_fixture --drive synthetic_drive_sync --frame 0 --output-dir artifacts/walkthrough_final_sensitivity
./build/review-final-release/benchmark_projection --output-json artifacts/benchmarks/projection_benchmark_reviewability_final.json
```

All commands above passed. Final synthetic overlays/panels were opened and
inspected again. Summary/frame status, clean source provenance, timing/publication
metadata, projection/z-buffer equations and all 110 benchmark iteration counts
were checked. Final benchmark SHA-256:
`46c02df81129046b594dc2e0f7f261211ecdf750ce37200c883a0a61e547ca24`.
This was also an uncontrolled harness verification run, not a performance
comparison. Both benchmark files remain ignored local artifacts.

## Hosted implementation checks

[GitHub Actions run 37552909043](https://github.com/WeiHanTu/sensor-fusion-replay-cpp/actions/runs/37552909043)
completed successfully at exact commit `1dceeca5eef3315afa6ed5081247881ac173089a`.
Completed logs confirm 85/85 tests in each job: GCC 13.3.0 Debug, Clang 18.1.3
Debug, and Clang 18.1.3 ASan+UBSan. Formatting passed in both compiler jobs;
clang-tidy passed in the Clang job. This verifies the implementation checkpoint,
not later documentation revisions. The native macOS sanitizer failure remains
a separate observed limitation.

## Public architecture presentation checks

The presentation revision changes documentation only; no C++/CMake changes.
On the working tree based on `1dceeca`, `cmake --preset dev`, the documented
parallel build, full CTest (85/85), focused CTest (24/24), and format-check passed.
All 47 relative link targets in the edited public documents exist. The
`architecture.md` generator/projection/sensitivity commands passed with the
`artifacts/architecture_*` directories; both images were opened and inspected,
and complete summary status plus measurement/publication metadata were checked.
These are development checks with a dirty documentation tree, not a new release
or performance claim. Full local Release/tidy and hosted sanitizer results above
remain attributed to their clean implementation checkpoint, not this revision.
The removed public guide is retained locally under the ignored `artifacts/`
directory. No Git history rewrite is part of this presentation revision.

## Publication and remaining work

The static-analysis follow-up adds an explicit optional guard and copies the
trivially-copyable payload. The initial table above belongs to `3c48c50`; final
implementation verification is separately attributed above.

Independent documentation usability evaluation remains not run even when
commands and CI pass. Crash-durable storage, transactional
overwrite, runtime metrics, synchronization, queues and perception remain outside
this maintenance change.
