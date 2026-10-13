# Contributing

Start with [setup](docs/getting_started.md), then read [AGENTS.md](AGENTS.md),
[intend.md](intend.md), [spec.md](spec.md) and the relevant [plan](plan.md) section.
The authority order is intent → specification → plan → implementation.

## Change discipline

- Keep changes small and scoped. Do not mix a new algorithm with cosmetic refactors.
- Update the specification before changing frame conventions, schema semantics
  or acceptance gates; explain the trade-off, not just the new behavior.
- Use `T_target_source`, meters/radians internally and explicit pixel units.
  Retain the full projection matrix and left-applied recovery convention.
- Put algorithms in the owning library, not the CLI. Preserve geometry's
  Eigen-only boundary, RAII/value ownership and strict warnings.
- Every bug fix needs a regression that fails before the fix. Derive expected
  geometry independently; generator/estimator agreement alone is weak evidence.
- Keep seeds, correspondence IDs, invalid-point accounting and failure statuses
  explicit. Never tune a gate after seeing results to manufacture acceptance.
- Register every new C++ header/source in the root format/tidy file list.
  Keep [architecture/source/test maps](docs/architecture.md) synchronized.

Perception/runtime are planned releases, not invitations to add ROS, CUDA,
neural detection, MuJoCo or a GUI. Propose scope changes before implementation.
No stable SDK/ABI or binary distribution is promised.

## Verification

Run from the repository root:

```bash
python3 -m unittest discover -s tools/tests -v
python3 tools/check_repository.py
cmake --preset dev
cmake --build --preset dev --parallel
ctest --preset dev --output-on-failure
cmake --build --preset dev --target format-check
cmake --build --preset dev --target clang-tidy
cmake --preset release
cmake --build --preset release --parallel
ctest --preset release --output-on-failure
```

Supported Ubuntu sanitizer check:

```bash
cmake --preset asan-ubsan
cmake --build --preset asan-ubsan --parallel
ctest --preset asan-ubsan --output-on-failure
```

The sanitizer preset retains Debug assertions/symbols and uses `-O1` with both
instruments. Native macOS initialization failure is recorded, not waived or called
a pass. Ubuntu CI is the supported sanitizer gate. No checks require KITTI.
Python 3.10+ is standard-library contributor tooling only.

For a clean independent tree, configure with `-B build/<new-name>`, then build
and run CTest using that directory. Do not erase an existing checkout. Reconfigure
after commits: report provenance is captured at configure time. Run the
[quick start](README.md#quick-start--no-kitti-account-or-data), inspect its PNGs,
and retain reports under ignored `artifacts/`. Absolute wall time is not a CI
performance gate. The repository checker is deliberately limited to the Markdown
syntax used here; it does not fetch remote URLs or render diagrams.

## Before submitting

Use the PR template: state scope, changed contracts, tests actually run, failures,
claim limits and relevant report provenance. Write `not run` where appropriate.
For bugs include a small **authored synthetic** reproduction, exact commit,
compiler/build/platform, command and expected versus actual result.

Run `git diff --check` and review the complete diff. Never submit KITTI data,
calibration/timestamps, derived screenshots, archives, build products, credentials
or private absolute paths. The checker enforces known forbidden path patterns;
it is not a secret scanner or proof of data provenance. Public images must have
authored provenance and a documented regeneration command. Private interview
notes stay ignored.

Original contributions are under the project's [Apache-2.0 license](LICENSE).
Do not copy third-party code without preserving its license and attribution.
Report possible vulnerabilities using [SECURITY.md](SECURITY.md), not a public
issue containing an exploit or private data.
