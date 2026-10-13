# Getting started

The public workflow requires no KITTI account, private dataset, Python runtime,
ROS, GPU or display server. Run commands from the repository root. The
[README quick start](../README.md#quick-start--no-kitti-account-or-data) is the
shortest complete path; this page explains setup, artifacts and private input.

## Prerequisites

| Component | Supported setup |
| --- | --- |
| CMake | 3.28+; presets use Unix Makefiles |
| C++ | C++20, GCC/Clang on Ubuntu 24.04; AppleClang on macOS |
| Libraries | Eigen 3.4+, OpenCV 4.5+ or 5.x, nlohmann/json 3.11+ |
| Tests | GoogleTest 1.15+; fallback 1.15.2 |
| Contributor tooling | clang-format, clang-tidy; Python 3.10+ for offline repository checks only |

Ubuntu 24.04 (development setup):

```bash
sudo apt-get update
sudo apt-get install --yes cmake g++ clang clang-format clang-tidy python3 \
  libeigen3-dev libopencv-dev nlohmann-json3-dev libgtest-dev
```

macOS with Homebrew and Xcode Command Line Tools installed:

```bash
brew install cmake eigen googletest llvm nlohmann-json opencv
```

CMake searches Homebrew LLVM for format/tidy. AppleClang remains the normal
compiler; installing LLVM does not require switching the whole toolchain.
Use `CMAKE_PREFIX_PATH` or the package's `*_DIR` if dependencies are outside
normal search prefixes. OpenCV is required locally, not downloaded by the build.
A fresh configuration may require network access for the fixed-commit fallbacks;
see [dependency acquisition](dependencies.md).

```bash
git clone https://github.com/WeiHanTu/sensor-fusion-replay-cpp.git
cd sensor-fusion-replay-cpp
```

## Build and inspect results

Run the [README commands](../README.md#quick-start--no-kitti-account-or-data).
Release is used for the first experiment so the full campaign does not run at
unoptimized Debug speed. Debug and sanitizer workflows are in
[CONTRIBUTING.md](../CONTRIBUTING.md#verification).

Projection creates one unique child under `artifacts/quickstart_projection/`:

```text
<printed-run-directory>/
├── run_summary.json
├── frames.jsonl
└── overlays/0000000000.png
```

Recovery creates `artifacts/quickstart_recovery/joint/`:

```text
report.json
loss_surface.json
residuals.json
loss_surface.png
residual_comparison.png
overlays/<frame-id>/{nominal,recovered,oracle}.png
```

Open the PNGs. Inspect report counts, `build.git_commit`, `build.git_dirty`,
search status, bound flags, identifiability and `acceptance.passed`. File creation
or CLI exit 0 alone does not establish recovery. No-estimate runs retain
diagnostics and explicitly omit recovered overlays.

Fixtures and exact named runs refuse implicit overwrite. For repeat execution,
choose new fixture/output paths and a new run ID. `--overwrite` deliberately
replaces the exact owned target, not crash-durable rollback; preserve a report
before replacing it. Prefer unique runs.

## Other entry points

| Executable | Purpose |
| --- | --- |
| `project_kitti` | One-frame projection and depth overlay |
| `analyze_calibration` | Baseline plus eight specified perturbations |
| `generate_synthetic_kitti` | Small project-authored KITTI-shaped fixture |
| `generate_extrinsic_fixture` | Three authored scenes; keyed observations and separate oracle |
| `estimate_extrinsics` | Bounded yaw/translation fit followed by held-out evaluation |
| `run_extrinsic_campaign` | Full frozen 21-case A–E experiment |
| `benchmark_projection` | Synthetic geometry-only microbenchmark |

All support `--help`, the authority for current flag defaults. Recovery commands
are in [the experiment document](extrinsic_recovery.md). The
[benchmark document](benchmark_walkthrough.md) separates measurement from I/O,
visualization and end-to-end latency. Formal measurements require a clean
committed Release build, reconfigured after the commit.

## Private KITTI input

Download authorized KITTI Raw **synced/rectified** data yourself; the application
does not download or bypass authentication. The local daily root must contain:

```text
<daily-root>/
├── calib_cam_to_cam.txt
├── calib_velo_to_cam.txt
└── <date>_drive_<id>_sync/
    ├── image_02/{data/*.png,timestamps.txt}
    └── velodyne_points/{data/*.bin,timestamps.txt}
```

```bash
./build/release/project_kitti \
  --dataset-root /path/to/2011_09_26 --drive 2011_09_26_drive_0005_sync \
  --frame 0 --output-dir artifacts/private_projection
./build/release/analyze_calibration \
  --dataset-root /path/to/2011_09_26 --drive 2011_09_26_drive_0005_sync \
  --frame 0 --output-dir artifacts/private_sensitivity
```

These are command shapes, not freshly verified private-data results. The
[historical geometry record](v0.1_verification.md) describes the earlier private
frame-0 smoke. Keep inputs, derived images and reports private. The synthetic
recovery CLI does **not** accept KITTI as a correspondence dataset.

## Troubleshooting

| Symptom | Action |
| --- | --- |
| Unsupported preset / CMake version | Install CMake 3.28+; check `cmake --version` |
| OpenCV not found | Install development packages and point CMake at their config directory |
| FetchContent network failure | Install a compatible system package or restore network access; no dataset login is involved |
| Existing fixture/run | Choose a new path/ID; overwrite only when replacement is intended |
| Exit 2 / 3 / 5 | Invalid configuration / malformed input / output failure; read the diagnostic |
| Recovery exit 4 | A diagnostic run may exist: inspect bound/refinement/identifiability/no-valid status |
| macOS ASan startup CHECK | Recorded runtime limitation; not a sanitizer pass. Use supported Ubuntu CI |
| Slow Debug campaign | The full matrix is intentional; use Release for experiments, retain Debug/sanitizer review gates |

Build metadata is captured at **configure time**. Reconfigure after edits or a
commit before relying on provenance. For an independent fresh build, choose a
new build directory rather than deleting a checkout or retained evidence.
