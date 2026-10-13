# Product Intent

## Status

This document defines intended behavior and evidence boundaries. Current
implementation and verification status is recorded in `README.md` and
`plan.md`; future-scope language below is not a claim of completed behavior.

## Product statement

`sensor-fusion-replay-cpp` is a small, C++20-first offline replay pipeline for
paired camera and LiDAR data. It loads a user-provided KITTI Raw synced sequence
and daily calibration, transforms Velodyne points into the rectified camera
reference frame, projects them into a color image, performs deliberately simple
geometric obstacle analysis, and exposes timing, synchronization, dropping, and
backpressure behavior under controlled replay faults.

The point is not to build an autonomous stack. The point is to make four kinds
of engineering evidence easy to inspect and reproduce:

1. Sensor geometry and explicit frame conventions.
2. Modern C++ ownership, interfaces, build discipline, and failure handling.
3. Numeric, parser, integration, and concurrent-runtime tests.
4. Honest performance and overload measurements with provenance.

One repository cannot establish senior-level C++ experience. It can remove the
more basic failure mode: having no inspectable C++ evidence at all.

## Intended users and use cases

The primary user is an engineer or interviewer evaluating the implementation.
The secondary user is the repository author replaying a locally downloaded
KITTI sequence.

The critical workflows are:

- Build and run all public tests on Ubuntu or macOS without KITTI data.
- Point one CLI at a private KITTI Raw synced drive and generate a depth-colored
  camera/LiDAR overlay plus a machine-readable projection report.
- Generate baseline-versus-perturbed calibration comparisons for known yaw and
  lateral-translation perturbations.
- Recover bounded camera +y yaw and camera +x translation corrections from
  explicitly authored synthetic correspondences; evaluate frozen parameters on
  completely held-out authored frames and expose geometry/noise/outlier limits.
- Run deterministic filtering, ground removal, clustering, and nearest-corridor
  proximity analysis without claiming semantic perception.
- Replay paired streams at controlled rates and show what happens when input is
  delayed, dropped, skewed, or faster than the consumer.
- Reproduce p50/p95/p99 measurements and trace every lost frame to one reason.

## Product principles

### Small and complete beats broad and unfinished

Each phase must be releasable and demonstrable on its own. A correct `v0.1`
with tests and artifacts is more valuable than a half-integrated ROS/CUDA stack.

### Frame semantics are part of the type contract

The code and output use `T_target_source`: the transform maps a point expressed
in `source` into `target`. Camera/Velodyne axis conventions, rectification, units,
and perturbation frames are explicit. Ambiguity is a defect, even when the image
looks plausible.

### Failure is visible

Malformed calibration, truncated LiDAR data, mismatched file/timestamp counts,
non-monotonic timestamps, non-finite values, and unwritable outputs terminate
with actionable errors. The pipeline must not silently skip corrupt inputs.

### Measurements require provenance

Every report carries the configuration, sample count, clock domain, build and
host metadata needed to interpret it. Offline replay pacing is not reported as
algorithm latency. Synthetic fault injection is labeled as synthetic.
One-frame geometry reports label their partial stage-sum and publication
boundaries; these are not end-to-end runtime measurements or crash-safe storage.

### Private datasets stay private

The repository provides loaders, synthetic fixtures, and commands, not KITTI
data or account automation. Public sample images must be project-owned
synthetic artifacts unless permission for other material is recorded.

## Delivery scope

### `v0.1 Geometry` — mandatory first release

- C++20, target-based CMake, presets, warnings-as-errors, formatting, static
  analysis, GoogleTest, ASan, UBSan, and Ubuntu CI.
- `SE(3)` rigid transforms with validation, inverse, composition, and batch point
  application.
- Explicit frame graph and documented frame tree:
  `world -> vehicle -> lidar -> camera_rect_00 -> image_02`.
  `world` and `vehicle` are explanatory until an OXTS adapter exists; only known
  calibrated edges may be evaluated.
- Strict parsers for KITTI daily camera/Velodyne calibration, timestamps, color
  camera 02 images, and Velodyne XYZI binary frames.
- Correct projection chain for synced/rectified camera 02.
- Depth-colored overlay and numeric projection summary.
- Baseline-versus-perturbed calibration panels for yaw
  `-1.0, -0.5, +0.5, +1.0 deg` and camera-x translation
  `-0.05, -0.01, +0.01, +0.05 m`.
- Unit/integration tests using authored synthetic data; optional private KITTI
  smoke test.
- A reproducible release-build projection benchmark with p50/p95/p99 and full
  provenance.

### Synthetic extrinsic recovery V1 — requested geometry extension

Status: design review, not implemented. This is separate from the `v0.2` and
`v0.3` capability releases and uses the existing geometry/libraries.

The observable is a stored correspondence between an authored LiDAR-frame 3D
point and a synthetic target `(u,v)` pixel, keyed by frame/source identity. A
fixture generator creates targets with a hidden, left-applied camera correction.
The estimator receives the nominal transform and training observations; an
evaluation harness retains hidden parameters and held-out frames.

V1 estimates two parameters only, using configurable bounded coarse-to-fine
search and squared L2/Huber reprojection loss. At least two geometrically distinct
training frames and a third held-out frame are required. Candidate visibility,
weights, numeric failure, search bounds, and identifiability are explicit.
The primary evidence combines signed/absolute parameter errors with held-out
reprojection improvement. Training loss or an overlay alone cannot establish
recovery. Noiseless, noise, outlier, geometry, and bounds campaigns must preserve
failed cases alongside successes. Detailed gates are in `spec.md` Section 17;
the design rationale is in [docs/extrinsic_recovery.md](docs/extrinsic_recovery.md).

No MuJoCo/dynamics code, full 6-DoF solver, correspondence extraction from raw
images, KITTI recovery, HIL, or sim-to-real claim belongs in this extension.

### `v0.2 Perception` — mandatory second release

- Deterministic voxel centroid filtering.
- Seeded, constrained ground-plane RANSAC with explicit failure output.
- Deterministic Euclidean clustering over a configured LiDAR-frame region of
  interest, without PCL by default.
- Axis-aligned cluster bounds and closest positive longitudinal clearance inside
  a configured path corridor.
- Per-frame JSONL and overlay output. The result is geometric proximity only.

### `v0.3 Runtime` — mandatory third release

- Closeable bounded queues with blocking and drop-oldest policies.
- One-to-one, nearest-timestamp camera/LiDAR synchronization with a tolerance and
  deterministic tie-breaking.
- Real-time-rate, faster-than-real-time, and unpaced offline replay modes.
- Deterministic delay, source-drop, and timestamp-skew injection.
- Separate counters for source drops, synchronization expiry, queue-policy
  drops, processing failures, and successful output.
- Per-stage processing, queue wait, end-to-end sojourn, and synchronization-delta
  p50/p95/p99 metrics.
- Saturation tests, clean shutdown tests, ASan/UBSan, and ThreadSanitizer.

### `v0.4 ROS 2` — optional and explicitly parked

Only after `v0.3` is complete: ROS 2/rosbag input adapter, tf2 bridge, QoS and
diagnostics. It must remain an adapter around the tested core, not a rewrite.

## Success criteria

The project succeeds when all of the following are true:

- A fresh Ubuntu CI runner builds and passes public tests without private data.
- The documented private-data command produces the overlay, sensitivity panel,
  per-frame record, and run summary required by `spec.md`.
- A reviewer can trace the projection formula, frames, units, and calibration
  keys from documentation into tests and implementation.
- Public architecture documentation supports technical discussion by mapping
  data flow, ownership, failure handling, and invariants to implementation/tests.
  Interview scripts and timed rehearsal notes remain local, not public artifacts.
- Recovery V1 meets its predeclared synthetic gates using stored correspondences,
  an estimator without oracle/held-out access, complete candidate accounting,
  and held-out metrics; weak geometry and bound-pinned results remain visible.
- Every input frame accepted by the replay runtime is accounted for exactly once
  as processed or as one explicit terminal failure/drop reason.
- Queue occupancy never exceeds configured capacity, and closure cannot strand
  a blocked producer or consumer.
- Performance tables link to machine-readable reports and state host, build,
  configuration, warm-up, samples, and percentile method.
- The README's build, test, example, and benchmark commands are executed in the
  release candidate and match the checked-out code.
- No private KITTI content, credentials, generated bulk output, or unjustified
  safety/production claim is committed.

Phase-specific numeric and artifact gates live in `spec.md`.

## Evidence boundaries

When implemented and verified, this repository may support claims such as:

- “Implemented a C++20 offline camera/LiDAR replay pipeline with explicit SE(3)
  transforms, KITTI calibration parsing, rectified projection, and synthetic
  numeric tests.”
- “Measured queueing and processing latency under deterministic delay/drop
  injection and reported p50/p95/p99 with complete frame accounting.”
- “Implemented deterministic geometric filtering/clustering and corridor
  clearance reporting.”
- “A project-authored synthetic experiment recovers bounded camera-yaw and
  camera-x-translation corrections from known 3D-to-2D correspondences and
  evaluates them on held-out authored frames.” Only after Section 17 passes.

It does not support claims of:

- Production deployment, real-time operating-system behavior, hard real-time
  guarantees, or automotive safety certification.
- Physical sensor calibration, KITTI extrinsic recovery, or proof that KITTI
  calibration is accurate. Controlled synthetic parameter recovery is the sole
  estimation exception; it is not calibration from arbitrary camera images.
- Semantic object detection, tracking, prediction, SLAM, localization, or sensor
  fusion in the state-estimation sense.
- Physical robot/heavy-machinery validation, sim-to-real transfer, collision
  avoidance, or field reliability.
- General perception accuracy. There is no labeled evaluation protocol in scope.
- Senior C++ ownership based on this project alone.

## Non-goals through `v0.3`

- Online calibration or bundle adjustment.
- Full 6-DoF extrinsic recovery, HIL, and MuJoCo/dynamics system identification.
- Camera undistortion for unsynced/unrectified KITTI data.
- GPS/IMU ingestion, world-pose estimation, motion compensation, or mapping.
- Neural networks, YOLO, TensorRT, CUDA, OpenCL, or GPU optimization.
- ROS 2, rosbag, tf2, DDS, or middleware benchmarking.
- PCL, unless profiling proves the in-repo implementation is the wrong tradeoff.
- A desktop/web GUI, cloud service, database, or live sensor driver.
- 3D oriented boxes, semantic labels, multi-frame tracking, velocity or
  time-to-collision estimation.
- Windows support, binary packages, or a stable third-party SDK/ABI.
- Bundling, mirroring, or downloading KITTI from the application.

## Dataset assumptions and legal boundary

The supported real-data path is KITTI Raw `*_sync` (synced and rectified), color
camera `image_02`, and `velodyne_points`, with the matching daily
`calib_cam_to_cam.txt` and `calib_velo_to_cam.txt`. KITTI states that these
streams are captured/synchronized at 10 Hz, provides timestamps, and stores
Velodyne frames as binary float matrices. The official paper defines camera
axes as x-right/y-down/z-forward and Velodyne axes as
x-forward/y-left/z-up.

KITTI currently requires login for raw downloads. Its site terms restrict
copying and republication of site materials. Therefore this project does not
ship data, credentials, a downloader, calibration dumps, or KITTI-derived public
screenshots by default. This is a conservative engineering boundary, not legal
advice.

Primary references:

- [KITTI Raw Data](https://www.cvlibs.net/datasets/kitti/raw_data.php)
- [KITTI sensor setup](https://www.cvlibs.net/datasets/kitti/setup.php)
- [Vision meets Robotics: The KITTI Dataset](https://www.cvlibs.net/publications/Geiger2013IJRR.pdf)
- [KITTI Terms of Service](https://www.cvlibs.net/datasets/kitti/terms_of_service.php)

## Recorded release decisions

- Original code and documentation use Apache-2.0; KITTI is not relicensed.
- Private acceptance used `2011_09_26_drive_0005_sync`, frame 0. CI remains
  independent of private data.
- The retained geometry benchmark records Apple M4 Pro, arm64, Darwin 25.6.0.
  Measurements are workload- and commit-specific, not portable speed guarantees.
- Public visuals remain authored synthetic output. No permission to publish
  KITTI-derived screenshots is claimed.
