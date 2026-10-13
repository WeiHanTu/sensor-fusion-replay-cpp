# Repository delivery maintenance

Scope: onboarding, contribution/review guidance, offline repository contracts and
CI reproducibility/diagnostics. No geometry/recovery algorithm, output schema,
scientific gate, perception/runtime feature or historical release tag changed.

## Design

- The README is an implemented-system landing page, not a cumulative release log.
  Setup/usage, architecture, contributor guidance and commit-bound evidence have
  distinct documents linked through the documentation index.
- The offline standard-library Python checker covers the inline Markdown/ATX
  heading/fenced-code syntax used here, local file/anchor existence, C++ quality
  source registration, known forbidden data/output paths, CMake/preset consistency,
  pinned refs and README/CI synthetic CLI command consistency.
- Checker tests use tiny independent fixtures, including malformed navigation,
  symlink escape, moving refs, missing source registration and command drift.
  The checker is not a full Markdown/YAML parser, secret scanner, diagram renderer,
  license/provenance audit, or unfamiliar-engineer usability test.
- CMake 3.28 matches features already used. Fallback dependencies and Actions are
  pinned to official upstream commits; Ubuntu 24.04, GCC 13 and Clang 18 are named.
  System packages/runner images still update; no hermetic-build claim is made.
- The existing 135-test C++ suite, full A–E matrix, 600-second campaign deadline,
  warning/static gates and sanitizer instrumentation remain. A Release job executes
  the README's no-KITTI commands. CI retains only generated public synthetic reports,
  JUnit and CTest logs for 14 days, including available failure diagnostics.

## Verification status

Local checks on 2026-10-13 used a worktree based on `09eee0f` with this maintenance
applied (`git_dirty=true`), not a clean-commit performance measurement. AppleClang
16.0.0.16000026, CMake 4.2.1, Apple M4 Pro/arm64/Darwin 25.6.0:

| Check | Result |
| --- | --- |
| Fresh dev configure/build/CTest | 135/135 passed (`build/professional-dev`) |
| Fresh Release configure/build/CTest | 135/135 passed (`build/professional-release`) |
| format-check/full 42-TU clang-tidy | Passed; no actionable project diagnostics |
| Checker tests and repository contracts | 16/16 passed; 25 Markdown files checked |
| Workflow YAML and preset parsing | Passed locally; hosted execution is separate |
| Forced dependency fallback | Exact Eigen 3.4.0/JSON 3.11.3 commits acquired and built; benchmark smoke and accepted recovery passed with tests disabled; zero upstream CTest entries |
| Synthetic first-run CLIs | Passed with the fresh Release binaries; report complete and accepted, train 0/1 and held-out 2 |
| Artifact inspection | Projection PNG, recovery loss surface and recovered held-out overlay opened and inspected |
| Hosted CI at `75e3ab9` | All five jobs passed; GCC Debug/Release, Clang Debug and ASan/UBSan each 135/135; checker 16/16; format/full Clang static gate passed |
| README Release workflow in CI | Exact four synthetic CLI commands executed; report accepted with clean commit provenance and held-out boundary checked |
| Published diagnostics | All four downloaded C++ job artifacts contain JUnit XML with 135 tests and zero failures; logs and authored reports retained for 14 days |
| GitHub presentation | Two-path Mermaid diagram, quick-start layout and About description/topics inspected publicly; not human usability proof |

The checker detects the original baseline's CMake mismatch. A new checker
regression first failed for a document symlink outside the checkout, then passed
after rejection was moved before reading the document. This protects the tool's
local read boundary, not the C++ parsers or a general security claim.
CI artifact auditing also exposed a JUnit relative-path error: the first run
wrote XML into a nested build path, while uploading logs/reports but missing XML.
A new path-contract regression failed before correction, then passed. CTest now
uses the binary-directory-relative basename, with a required nonempty-file check
before publication. The first run is not claimed to have complete diagnostics.
Raw build/CTest/static logs and first-run reports remain under ignored `artifacts/`.
Fresh local checks are not inferred from the baseline's earlier passing CI.

Final implemented infrastructure:
`75e3ab9227caf41687ef32dedc5bbfbf45698316`;
[completed exact-commit CI](https://github.com/WeiHanTu/sensor-fusion-replay-cpp/actions/runs/38367494150).
The first `b5d0c03` run also passed its C++/static/checker gates, but its published
artifact lacked JUnit; that distinction remains above. The later documentation
closure does not relabel itself as the implementation tested in this run.
The immutable `v0.1.0` tag is unchanged; no GitHub Release entry exists or was
created by this maintenance. Public metadata changes affected only description
and topics, not visibility, permissions or branch protection.

Native macOS sanitizer initialization was not retried for this infrastructure-only
change; the recorded limitation remains. Supported Ubuntu sanitizer CI is required.
Private KITTI smoke and independent human usability evaluation are not run for
this maintenance. Existing historical evidence files/images remain unchanged.
