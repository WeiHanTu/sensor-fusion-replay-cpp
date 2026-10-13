# Dependencies and reproducibility

The application is C++20; Python is used only for contributor/CI repository
checks. There is no ROS, PCL, CUDA, MuJoCo, solver framework or GUI dependency.

## Acquisition contract

| Dependency | Minimum / scope | Missing-system-package fallback |
| --- | --- | --- |
| CMake | 3.28; build tool | Install locally; never fetched by CMake |
| Eigen | 3.4; geometry, public headers | 3.4.0 commit `3147391d946bb4b6c68edd901f2add6ac1f31f8c` |
| OpenCV | 4.5+ or 5.x; core/imgcodecs/imgproc | None; a development package is required |
| nlohmann/json | 3.11; fixture/report serialization | 3.11.3 commit `9cca280a4d0ccf0c08f47a99aa71d1b0e52f8d03` |
| GoogleTest | 1.15; tests only | 1.15.2 commit `b514bdc898e2951020cbdca1304b75f5950d1f59` |

System-first discovery is intentional. Compatible installed package versions
may differ from fallback versions; the table is **not** a complete lockfile or
bit-for-bit environment guarantee. FetchContent uses exact upstream commits,
not moving tags. Full-commit Git acquisition is not a shallow-tag clone and may
take longer. Eigen's header-only fallback never imports its upstream test tree.
Configure offline only when compatible system targets or populated dependencies
are already available. `SFR_BUILD_TESTS=OFF` omits GoogleTest/public tests, not
runtime libraries or examples.

The supported CI runner is `ubuntu-24.04`; GCC and Clang are both tested.
Actions are pinned to verified commits from their official repositories.
APT packages and hosted runner images still receive updates. This is a bounded,
auditable setup—not hermetic reproducibility or a vulnerability guarantee.

## Why CMake 3.28

The old minimum (3.24) was inconsistent with existing features: preset schema 6
requires 3.25, and FetchContent `EXCLUDE_FROM_ALL` was added in 3.28. The root
minimum and presets now agree at 3.28. See the official
[preset version table](https://cmake.org/cmake/help/latest/manual/cmake-presets.7.html#versions)
and [FetchContent reference](https://cmake.org/cmake/help/latest/module/FetchContent.html).
This corrects a support declaration; no scientific acceptance threshold changes.

## License boundaries

Original code/documentation use [Apache-2.0](../LICENSE). Upstream dependencies
retain their own licenses and notices; they are not relicensed by this repository:

- [Eigen license text](https://gitlab.com/libeigen/eigen/-/blob/3.4.0/COPYING.MPL2).
- [OpenCV license](https://github.com/opencv/opencv/blob/4.x/LICENSE).
- [nlohmann/json license](https://github.com/nlohmann/json/blob/v3.11.3/LICENSE.MIT).
- [GoogleTest license](https://github.com/google/googletest/blob/v1.15.2/LICENSE).

No third-party source is vendored and no binary package is distributed. A future
binary/vendor distribution requires its own upstream-license/notice audit.
KITTI has a separate data/license boundary: no KITTI input or derived public
image is included. Synthetic fixture provenance is in
[the fixture README](../tests/fixtures/synthetic/README.md) and
[recovery fixture README](../tests/fixtures/synthetic/extrinsic_recovery/README.md).

## Evidence and build provenance

Reports record commit/dirty state, build type, compiler and host at configure
time, not continuously during execution. Reconfigure after edits or commits.
Future reports may extend dependency-version metadata; current reports do not
claim to serialize every installed package version. Preserve exact raw reports
and describe measured/excluded work rather than copying old numbers to a new HEAD.
