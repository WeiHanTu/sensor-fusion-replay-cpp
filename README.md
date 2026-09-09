# sensor-fusion-replay-cpp

A C++20-first offline camera/LiDAR replay project for inspectable geometry,
runtime, testing, and performance evidence.

> Status: early `v0.1 Geometry` implementation. This is not a released,
> production, real-time, or safety system.

The product contract is in [`intend.md`](intend.md), the normative architecture
and acceptance gates are in [`spec.md`](spec.md), and ordered work is in
[`plan.md`](plan.md).

## Capability status

| Capability | Status | Evidence |
| --- | --- | --- |
| C++20/CMake quality skeleton | Locally verified | Warnings-as-errors build, format and clang-tidy |
| Frame-labelled SE(3) transforms | Locally verified | Independent unit tests |
| Deterministic frame graph | Locally verified | Forward/inverse/path-invariant tests |
| Rectified 3x4 projection primitive | Locally verified | Hand-computed projection/boundary tests |
| Strict KITTI calibration/data loading | Locally verified on synthetic inputs | Typed parser/I/O negative tests |
| Point-cloud projection and rejection accounting | Locally verified on synthetic inputs | Hand-computed and integration tests |
| Depth-colored z-buffer overlay | Locally verified on synthetic inputs | Deterministic pixel tests |
| Atomic JSON/JSONL run artifacts | Locally verified on synthetic inputs | Schema/accounting/overwrite tests |
| `project_kitti` CLI | Locally verified on synthetic inputs | End-to-end process and exit-code tests |
| Calibration sensitivity | Locally verified on synthetic inputs | Hand-computed sign/axis tests, JSON checks, and inspected nine-panel output |
| Geometric perception | Planned (`v0.2`) | None |
| Replay/backpressure metrics | Planned (`v0.3`) | None |

“Locally verified” currently means 65/65 public tests passed in macOS debug and
release builds with AppleClang 16 and OpenCV 5.0. The same 65/65 tests passed
with GCC 13 and OpenCV 4.6, then Clang 18 with ASan+UBSan, in a clean Ubuntu
24.04 container. Format and clang-tidy passed locally. Hosted GitHub Actions has
not run. This state has a local checkpoint commit but has not been pushed,
tagged, or released. A project-code license has not been selected.

## Prerequisites

Required tools and libraries are CMake 3.24+, a C++20 compiler, Eigen 3.4+,
OpenCV 4.5+ or 5.x, and nlohmann/json 3.11+. GoogleTest 1.15.2 is fetched at
configure time when a compatible system target is unavailable.

Ubuntu 24.04:

```bash
sudo apt-get update
sudo apt-get install --yes \
  cmake clang-format clang-tidy libeigen3-dev libgtest-dev \
  libopencv-dev nlohmann-json3-dev
```

macOS with Homebrew:

```bash
brew install cmake eigen googletest llvm nlohmann-json opencv
```

## Verified development commands

```bash
cmake --preset dev
cmake --build --preset dev --parallel
ctest --preset dev --output-on-failure
cmake --build --preset dev --target format-check
cmake --build --preset dev --target clang-tidy
```

Project one frame from an authorized local KITTI Raw synced drive:

```bash
./build/dev/project_kitti \
  --dataset-root /path/to/2011_09_26 \
  --drive 2011_09_26_drive_0005_sync \
  --frame 0 \
  --output-dir artifacts/projection
```

The command creates a unique child run directory containing
`run_summary.json`, `frames.jsonl`, and `overlays/<frame>.png`. Run
`./build/dev/project_kitti --help` for validated defaults and optional flags.
This command shape is tested with authored synthetic inputs; no private KITTI
smoke run has been performed yet.

Run the fixed calibration-sensitivity suite on the same kind of private input:

```bash
./build/dev/analyze_calibration \
  --dataset-root /path/to/2011_09_26 \
  --drive 2011_09_26_drive_0005_sync \
  --frame 0 \
  --output-dir artifacts/sensitivity
```

It writes a baseline, four camera-`+y` yaw panels, four camera-`+x`
translation panels, a combined comparison, and a JSON report. Perturbations are
left-applied in `camera_rect_00`; the report gives common-visible-set median/p95
pixel displacement plus appeared/disappeared counts. The synthetic comparison
was opened and inspected. No private KITTI sensitivity run has been performed.
The reported `50*tan(1 deg)` value is an analytic illustration, not a measured
calibration error or pixel-accuracy claim.

The `asan-ubsan` preset is intended for the Ubuntu CI toolchain. AppleClang 16's
ASan runtime aborts during test discovery on the current macOS 26.6 host; the
same tests pass under Clang 18 ASan+UBSan in Ubuntu 24.04. This host limitation
must not be presented as a native sanitizer pass.

No KITTI data is distributed by this repository. Obtain authorized data
separately from the [KITTI Raw Data](https://www.cvlibs.net/datasets/kitti/raw_data.php)
page and keep it outside version control.
