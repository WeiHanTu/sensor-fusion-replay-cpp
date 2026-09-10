# Technical Specification

## 1. Status and normative language

This is the implementation contract. Implementation status is recorded in
`README.md` and `plan.md`; unchecked requirements here are not claims of
completed behavior.

`MUST`, `MUST NOT`, `SHOULD`, and `MAY` are normative. Release gates cannot be
waived by changing a README sentence. Any changed requirement must be reviewed
in this file and reflected in `intend.md`, `plan.md`, tests, and release evidence.

## 2. Supported system

### 2.1 Platforms and toolchain

- Language: C++20 without compiler-specific language extensions.
- Build: target-based CMake, minimum version 3.24, with checked-in
  `CMakePresets.json`.
- Required CI: Ubuntu with one supported GCC configuration and one supported
  Clang configuration.
- Required developer platform: macOS with AppleClang, documented but not
  initially required as hosted CI.
- Required warnings: `-Wall -Wextra -Wpedantic -Werror` for project targets.
- Required dynamic checks: ASan+UBSan through `v0.2`; ASan+UBSan and TSan for
  `v0.3` concurrency code. Sanitizer configurations are separate.

### 2.2 Dependencies

| Dependency | Scope | Rule |
| --- | --- | --- |
| Eigen3 | Runtime | Geometry/math only; no I/O ownership hidden in Eigen types. |
| OpenCV core/imgcodecs/imgproc | Runtime | Image loading and rendering; no GUI requirement. |
| nlohmann_json or equivalent small JSON library | Runtime | Reports only; pin/acquire reproducibly. |
| GoogleTest | Tests | No production dependency. |
| Google Benchmark | Benchmarks | Not currently required; the audited `v0.1` harness uses `std::chrono::steady_clock`. |

PCL, ROS 2, CUDA, TensorRT, and neural inference dependencies MUST NOT be added
through `v0.3` without a spec revision and measured justification.

## 3. Repository and target architecture

Required repository shape at `v0.3`:

```text
sensor-fusion-replay-cpp/
├── AGENTS.md
├── CMakeLists.txt
├── CMakePresets.json
├── intend.md
├── spec.md
├── plan.md
├── cmake/
├── include/sfr/
│   ├── core/
│   ├── geometry/
│   ├── camera/
│   ├── calibration/
│   ├── io/
│   ├── perception/
│   ├── runtime/
│   └── viz/
├── src/
├── apps/
│   ├── project_kitti.cpp
│   ├── analyze_calibration.cpp
│   └── replay_sequence.cpp
├── tests/
│   ├── fixtures/synthetic/
│   ├── unit/
│   └── integration/
├── benchmarks/
├── examples/
├── docs/
└── .github/workflows/ci.yml
```

Required CMake targets:

| Target | Responsibility | May depend on |
| --- | --- | --- |
| `sfr_geometry` | SE(3), named frames, frame graph, projection primitives | Eigen |
| `sfr_io` | KITTI layout/calibration/timestamp/image/point loaders | geometry, OpenCV |
| `sfr_perception` | voxel, ground, clusters, corridor proximity | geometry |
| `sfr_runtime` | queues, sync, injection, scheduling, metrics | geometry, io, perception |
| `sfr_viz` | overlay and panel rendering | geometry, perception, OpenCV |
| `project_kitti` | one-frame/range projection CLI | io, geometry, viz |
| `analyze_calibration` | controlled perturbation CLI | io, geometry, viz |
| `benchmark_projection` | authored projection microbenchmark and JSON report | geometry, JSON, build info |
| `generate_synthetic_kitti` | authored one-frame public demo fixture | OpenCV only |
| `replay_sequence` | sequence/perception/runtime CLI | all libraries |

Apps MUST contain composition and argument/error handling only. Core algorithms
MUST be callable from tests without launching a process.

## 4. Coordinate and geometry contract

### 4.1 Naming and units

- `T_target_source` maps a homogeneous point from `source` coordinates to
  `target` coordinates: `p_target = T_target_source * p_source`.
- Length is meters internally. Angles are radians internally and degrees only at
  CLI/report boundaries. Image coordinates are pixels.
- All matrices use explicit Eigen scalar/shape aliases. Public APIs MUST NOT use
  dynamically sized matrices for fixed-size transforms or projection matrices.
- Velodyne frame: x forward, y left, z up.
- KITTI camera frame: x right, y down, z forward.
- Image frame: u right, v down, origin at the top-left pixel center convention
  used by the projection output.

### 4.2 `RigidTransform`

The `RigidTransform` value contains `R_target_source` (`3x3 double`) and
`t_target_source_m` (`3x1 double`). Construction MUST reject:

- non-finite values;
- `abs(det(R) - 1) > 1e-6`;
- `||R*R^T - I||_F > 1e-6`.

Required operations:

- identity;
- inverse;
- frame-compatible composition;
- transform one point and a point span/batch;
- obtain a `4x4` homogeneous matrix for reporting/testing.

Frame identity MUST be explicit through strong frame tags or runtime frame IDs.
Composition with mismatched endpoints MUST fail at compile time when statically
known, otherwise with a typed error.

### 4.3 `FrameGraph`

- Edges are directed transforms and may be traversed through an inverse.
- Frame names are unique and non-empty.
- Inserting a duplicate directed edge or a transform inconsistent with an
  existing path MUST fail; silently replacing an edge is forbidden.
- Lookup returns the composed shortest path only when exactly one deterministic
  result exists. A missing path is expected absence (`std::optional`).
- `v0.1` evaluable frames are `lidar`, `camera_raw_00`, and
  `camera_rect_00`. `image_02` is a projection domain, not a Euclidean SE(3)
  frame.
- Documentation MAY show `world -> vehicle -> lidar -> camera_rect_00 ->
  image_02`, but unavailable world/vehicle transforms MUST be visibly marked
  unresolved. The system MUST NOT fabricate them.

### 4.4 KITTI calibration and projection

The parser consumes matching daily files:

- `calib_velo_to_cam.txt`: `R` (9 doubles) and `T` (3 doubles), defining
  `T_camera_raw_00_lidar`.
- `calib_cam_to_cam.txt`: `R_rect_00` (9 doubles), `P_rect_02` (12 doubles),
  and `S_rect_02` (2 integers).

Missing/duplicate keys, wrong counts, non-finite values, invalid rotations, or
non-positive image dimensions MUST be errors. Unknown keys MAY be retained or
ignored but MUST NOT change interpretation of required keys.

For homogeneous LiDAR point `X_lidar = [x,y,z,1]^T`, projection to rectified
color camera 02 is:

```text
X_raw_00  = T_camera_raw_00_lidar * X_lidar
X_rect_00 = R_rect_00_4x4 * X_raw_00
q         = P_rect_02 * X_rect_00
u         = q.x / q.z
v         = q.y / q.z
```

The point is renderable only if all values are finite, `X_rect_00.z > z_min_m`,
`q.z > 0`, and `0 <= u < width`, `0 <= v < height`. The default `z_min_m` is
`0.1 m` and is configurable. Rejection counts MUST be reported separately for
non-finite input, behind/too-near camera, non-positive homogeneous depth, and
outside image.

If multiple points round to the same pixel, visualization MUST retain the
nearest positive camera-depth point (z-buffer rule). Pixel conversion MUST use
one documented rule consistently; default is nearest integer after the
continuous bounds check.

Depth coloring MUST use camera forward depth, a documented clamped range
`[depth_min_m, depth_max_m]`, and one named OpenCV colormap. The legend and range
MUST appear in the output or sidecar report.

### 4.5 Calibration sensitivity

The baseline is the loaded `T_camera_rect_00_lidar`. The default perturbation is
left-applied in rectified camera coordinates:

```text
T'_camera_rect_00_lidar = Delta_camera * T_camera_rect_00_lidar
```

- Rotation experiment: yaw about camera +y at
  `-1.0, -0.5, +0.5, +1.0 deg`.
- Translation experiment: camera +x at
  `-0.05, -0.01, +0.01, +0.05 m`.
- CLI flags MAY select another explicit axis, but reports MUST record axis,
  sign, units, and left/right application convention.

Each comparison reports the number of points valid in both baseline and
perturbed images plus median and p95 pixel displacement on that common set.
Disappearing/reappearing points are separate counts. Empty common sets are a
reported failure, not zero displacement.

The documented `50*tan(1 deg) ~= 0.87 m` example is an analytic angular-offset
illustration, not measured KITTI error and not a pixel reprojection guarantee.

## 5. Input contract

### 5.1 Supported KITTI layout

Only a KITTI Raw synced/rectified drive is supported through `v0.3`:

```text
<dataset-root>/
├── calib_cam_to_cam.txt
├── calib_velo_to_cam.txt
└── <date>_drive_<drive>_sync/
    ├── image_02/
    │   ├── data/*.png
    │   └── timestamps.txt
    └── velodyne_points/
        ├── data/*.bin
        └── timestamps.txt
```

The CLI MAY accept the daily directory plus drive name instead of this exact
root, but resolved paths and sequence ID MUST be recorded. Unsynced/unrectified
input MUST be rejected with an actionable message.

### 5.2 LiDAR records

- Each `.bin` is a contiguous sequence of little-endian IEEE-754 `float32`
  records `(x_m, y_m, z_m, reflectance)`.
- File size MUST be nonzero and divisible by 16 bytes.
- Non-finite points MUST not enter geometry; their count is reported. A frame
  with no finite points is an error.
- Public in-memory representation is `PointXYZI {double x_m, y_m, z_m;
  float reflectance;}` or a documented precision-equivalent structure.

### 5.3 Images and frame identity

- Images are decoded as BGR8 by OpenCV and converted deliberately where needed.
- Empty images and dimensions inconsistent with `S_rect_02` are errors by
  default. An explicit compatibility flag MAY allow a recorded dimension
  override.
- Numeric filename stem is the `FrameId` (`uint64_t`). Lexical ordering alone is
  forbidden.
- Duplicate frame IDs are errors.

### 5.4 Timestamps

- Preserve the original timestamp text for diagnostics and parse its civil date
  and time at nanosecond precision into `SensorTimestamp`, a strong
  `std::chrono::nanoseconds`-based type. Camera and LiDAR values MUST remain on
  the same timezone-free civil-time scale so their initial offset is preserved;
  do not label that scale UTC or local time when the input does not say so.
- Normalize replay scheduling for both streams against one shared origin: the
  earliest valid timestamp across the selected camera and LiDAR inputs. Never
  normalize each stream independently, because that would erase initial sensor
  skew.
- Timestamp lines and frame files MUST have a one-to-one count and index mapping.
- Each stream MUST be strictly monotonic. Equal or decreasing timestamps are
  errors.
- Any dropped/skewed timestamp created by fault injection is derived metadata;
  the original value remains available in the frame record.

## 6. Core schemas

Names may change only if semantics remain one-to-one and the spec is updated.

```text
ImageFrame
  frame_id: uint64
  original_timestamp: SensorTimestamp
  effective_timestamp: SensorTimestamp
  image_bgr8: owned/ref-counted cv::Mat
  source_path: filesystem::path

LidarFrame
  frame_id: uint64
  original_timestamp: SensorTimestamp
  effective_timestamp: SensorTimestamp
  points: vector<PointXYZI>
  non_finite_points: uint64
  source_path: filesystem::path

SynchronizedFrame
  pair_id: uint64
  image: ImageFrame
  lidar: LidarFrame
  signed_delta: lidar.effective_timestamp - image.effective_timestamp
  paired_at: SteadyTimePoint

ProjectedPoint
  source_index: uint32 or uint64
  u_px, v_px: double
  depth_camera_m: double
  reflectance: float

Cluster
  cluster_id: uint64
  point_indices: vector<uint32 or uint64>
  centroid_lidar_m: Vec3
  aabb_min_lidar_m, aabb_max_lidar_m: Vec3

ProximityObservation
  pair_id: uint64
  nearest_cluster_id: optional<uint64>
  longitudinal_clearance_m: optional<double>
  band: none | clear | warning | critical
  corridor_half_width_m: double
  thresholds_m: recorded values
```

Source paths MUST NOT be serialized as absolute paths in public artifacts.
Reports use dataset/sequence IDs and relative frame names.

## 7. Perception contract (`v0.2`)

### 7.1 Region of interest

Filtering operates in LiDAR coordinates. Defaults, all configurable and recorded:

- `x in [0.0, 50.0] m`
- `y in [-10.0, 10.0] m`
- `z in [-3.0, 3.0] m`

Boundary inclusion MUST be documented and tested; default is closed intervals.

### 7.2 Voxel filter

- Default leaf size: `0.20 m` per axis; each axis MUST be positive.
- Voxel key uses `floor(coordinate / leaf_size)` with overflow checks.
- Output point is the arithmetic centroid; reflectance is the arithmetic mean.
- Output order is lexicographically sorted voxel key order and deterministic.
- The report records input, finite, ROI, and output counts.

### 7.3 Ground removal

- Algorithm: seeded 3-point RANSAC in LiDAR coordinates.
- Default distance threshold: `0.20 m`; maximum ground-normal tilt from LiDAR
  +z: `15 deg`; minimum inlier fraction: `0.20`; iterations: recorded.
- Degenerate triples are rejected. Tie-break: more inliers, then lower mean
  inlier residual, then earliest iteration.
- Output includes normalized plane coefficients, inlier count/fraction,
  residual, and seed.
- If no plane meets constraints, return `ground_not_found`; do not remove points
  or manufacture a horizontal plane.

### 7.4 Euclidean clustering

- Operates on deterministic non-ground voxel centroids.
- Default distance tolerance: `0.80 m`; minimum size: `5`; maximum size:
  `50,000`; all configurable and validated.
- A grid/spatial index MAY accelerate neighbor lookup, but cluster membership and
  IDs MUST be deterministic. IDs sort by minimum source point index.
- Every candidate point belongs to exactly zero or one retained cluster; points
  in rejected small/large components are counted explicitly.

### 7.5 Proximity observation

- Default corridor is `x > 0` and `|y| <= 1.5 m`, expanded by cluster AABB
  intersection rather than centroid-only membership.
- Longitudinal clearance is the smallest positive AABB x face in the corridor.
- Default bands: `critical < 5 m`, `warning < 15 m`, otherwise `clear`; no
  intersecting cluster is `none`.
- Bands are configurable display heuristics, not calibrated risk or safety
  decisions. No time-to-collision is emitted.

## 8. Runtime contract (`v0.3`)

### 8.1 `BoundedQueue<T>`

Required policies:

- `block`: producer waits while full.
- `drop_oldest`: accepting a new item atomically evicts exactly the oldest item.

Invariants:

- Configured capacity is at least 1 and occupancy never exceeds it.
- FIFO order is preserved among non-evicted items.
- `close()` is idempotent, wakes all waiters, prevents future pushes, and allows
  queued items to drain before `pop()` returns closed/empty.
- Destruction cannot leave a thread blocked; owners join worker threads.
- Each push reports accepted, rejected-closed, or accepted-with-evicted-ID.
- Counters satisfy the accounting equations in Section 11.

### 8.2 Synchronizer

- Policy: one-to-one nearest effective timestamp within inclusive tolerance.
- Default tolerance: `50 ms`, configurable and recorded.
- Matching minimizes absolute delta among currently eligible candidates.
  Ties choose the earlier effective timestamp, then lower frame ID.
- Frames are never reused. Input per stream is monotonic.
- Watermarks expire a frame only when no future frame from the opposite monotonic
  stream can fall within tolerance. End-of-stream flushes remaining unpaired
  frames with explicit `sync_expired` reasons.
- Output pairs are ordered by `max(image_ts, lidar_ts)` then pair ID.

### 8.3 Replay scheduling

Modes:

- `unpaced`: process as fast as possible.
- `realtime`: replay relative source times against an injected steady clock.
- `scaled`: relative source timing divided by positive speed factor.

Scheduling MUST use `std::chrono::steady_clock` or an injected fake clock, never
wall-clock time. Falling behind does not rewrite source timestamps. The report
records scheduled time, actual enqueue time, and lateness.

### 8.4 Fault injection

All injection is optional, deterministic, and recorded:

- periodic source drop (`every_n`, stream, starting frame);
- fixed or seeded-distribution delivery delay per stream;
- fixed timestamp skew per stream;
- configurable processing delay for overload testing.

Source-drop injection, queue eviction, synchronization expiry, corrupt-input
failure, and processing failure MUST be distinct terminal reasons. Seeded modes
record algorithm, seed, and parameters.

### 8.5 Time metrics

Use steady-clock durations for runtime measurements:

- `source_decode`: load/decode duration per stream.
- `queue_wait`: accepted enqueue to successful dequeue.
- `sync_wait`: first arrival at synchronizer to pair/expiry.
- `geometry`, `perception`, `visualization`, and `serialization`: stage time.
- `processing_total`: dequeue of pair to completed result, excluding replay
  pacing.
- `sojourn_total`: first accepted input enqueue to terminal result.
- `sync_abs_delta`: absolute effective source timestamp difference, not wall time.

No metric may be silently negative. Overflow/clock inversion is a run error.

## 9. CLI contract

Every CLI MUST support `--help`, reject unknown arguments, print errors to
stderr, and use these exit categories:

- `0`: success;
- `2`: invalid arguments/configuration;
- `3`: input/layout/parse failure;
- `4`: processing/invariant failure;
- `5`: output/report write failure.

Minimum commands:

```bash
project_kitti \
  --dataset-root <private-daily-root> \
  --drive <date>_drive_<id>_sync \
  --frame <numeric-id> \
  --output-dir <dir>

analyze_calibration \
  --dataset-root <private-daily-root> \
  --drive <date>_drive_<id>_sync \
  --frame <numeric-id> \
  --output-dir <dir>

replay_sequence \
  --dataset-root <private-daily-root> \
  --drive <date>_drive_<id>_sync \
  --mode unpaced \
  --output-dir <dir>
```

Defaults MUST be printed by `--help` and serialized. Refuse to overwrite an
existing nonempty run directory unless `--overwrite` is explicit. That flag may
replace only files within the exact resolved run directory, never the dataset.

## 10. Output schemas

### 10.1 Run directory

```text
<output-dir>/<run-id>/
├── run_summary.json
├── frames.jsonl
├── overlays/                 # optional by mode
├── calibration_sensitivity/  # analyze_calibration only
└── errors.jsonl              # only if frame-level continuation is enabled
```

`run-id` is unique and filesystem-safe. Tests MAY inject a deterministic ID.
Writes MUST use a temporary sibling followed by an atomic rename where the
filesystem permits, so a partial `run_summary.json` is never presented as final.

### 10.2 `run_summary.json`

Required top-level fields:

```json
{
  "schema_version": "1.0.0",
  "status": "complete",
  "run_id": "...",
  "capability_version": "v0.1",
  "dataset": {
    "kind": "kitti_raw_sync",
    "sequence_id": "...",
    "camera": "image_02"
  },
  "build": {
    "git_commit": "...",
    "git_dirty": false,
    "build_type": "Release",
    "compiler": "..."
  },
  "host": {
    "os": "...",
    "arch": "...",
    "cpu": "..."
  },
  "config": {},
  "counts": {},
  "latency_ms": {},
  "percentile_method": "nearest_rank",
  "warmup_frames": 0,
  "measured_frames": 0
}
```

Unknown additive fields are allowed. Required fields MUST NOT change meaning
without a schema-version change. `status=complete` is written only after all
requested outputs close successfully. Dirty builds are allowed for development
but MUST be conspicuous and MUST NOT support release benchmark claims.

Each latency series contains `count`, `min`, `max`, `mean`, `p50`, `p95`, and
`p99`, all finite or explicit `null` when count is zero. Nearest-rank percentile
for sorted `N` samples uses one-based index `ceil(p*N)`, clamped to `[1,N]`.

### 10.3 `frames.jsonl`

Each line is one terminal pair/result and MUST include schema version, pair and
source frame IDs, original/effective timestamps, signed sync delta, terminal
status/reason, stage durations, point counts, and optional proximity observation.
JSON Lines permit streaming; a truncated final line makes the run incomplete.

## 11. Accounting invariants

All counters are `uint64_t` with checked increment/aggregation.

For each source stream:

```text
discovered = source_injected_drop
           + source_parse_failure
           + enqueue_rejected_closed
           + queue_policy_drop
           + sync_expired
           + paired
```

If fail-fast mode terminates before discovering all files, `discovered` applies
only to enumerated inputs and run status is failed. A frame MUST appear in one
and only one terminal category.

For pairs:

```text
paired = processing_failure + output_failure + completed
```

Queue counters also satisfy:

```text
accepted_pushes = successful_pops + evicted_drop_oldest + drained_at_shutdown
```

`drained_at_shutdown` means items explicitly removed and terminally accounted,
not abandoned memory. Every complete run MUST validate these equations before
writing `status=complete`.

## 12. Testing specification

### 12.1 Public synthetic fixture

The repository MUST contain a small authored fixture with:

- calibration whose transform and projection are hand-computable;
- at least one image generated by project code or stored with clear provenance;
- one valid LiDAR binary plus malformed/truncated variants;
- valid, missing, duplicate, non-finite, and wrong-count calibration cases;
- timestamp sequences covering exact match, boundary tolerance, tie, expiry,
  duplicate, and decreasing cases;
- a synthetic ground plane and clusters with known membership/clearance.

Fixture expected values MUST be independent of the production implementation.

### 12.2 Required test groups

- `geometry`: identity/inverse/composition, invalid rotations, frame mismatch,
  positive-depth projection, z-buffer, bounds.
- `calibration_io`: strict keys/counts/finite values/dimensions.
- `kitti_io`: numeric ordering, point record size, image dimensions, timestamps.
- `sensitivity`: left-applied axis perturbations and independently computed pixel
  displacement.
- `perception`: voxel boundaries/negative coordinates, seeded ground success and
  failure, deterministic clusters, corridor AABB logic.
- `queue`: FIFO, both policies, capacity, close-before/while-waiting, accounting.
- `synchronizer`: tolerance inclusivity, nearest choice, tie-break, no reuse,
  watermarks, end flush.
- `runtime`: fake-clock pacing, each injection reason, saturation, clean shutdown,
  full accounting.
- `metrics`: exact nearest-rank percentiles, empty/singleton series, overflow.
- `cli_integration`: valid synthetic run and each exit-code category.

Tests MUST be order-independent and repeatable. No public test may need network
or private data.

The public demo generator MUST label its output as authored synthetic data,
write through an exclusively claimed staging directory, refuse implicit
overwrite, and produce a fixture that the same strict loaders and CLIs consume.
Generated demo images MUST be reproduced from a documented release-build command
and MUST NOT contain or derive from KITTI data.

## 13. Performance specification

- Benchmark only optimized (`Release` or documented `RelWithDebInfo`) builds.
- Warm-up samples are excluded and counted separately.
- Projection benchmark uses at least 100,000 finite synthetic points and reports
  input points/s plus rejection counts.
- The projection fixture MUST record an authored-fixture identifier, deterministic
  generator and seed, category distribution, `T_camera_rect_00_lidar`, full
  `P_image_camera_rect_00`, image dimensions, and minimum camera depth.
- Every warm-up and measured projection iteration MUST satisfy the terminal
  accounting equation and the fixture's exact expected category counts. A report
  MUST state how many iterations were checked; one sampled iteration is
  insufficient evidence for the full run.
- Projection timing MUST use `std::chrono::steady_clock` and state the measured
  boundary. Fixture generation, validation, image decode, visualization,
  serialization, queue wait, and replay pacing are excluded from this geometry
  microbenchmark and MUST NOT be implied by its throughput.
- End-to-end runtime evidence uses at least 1,000 measured pairs after warm-up.
  Cycling a shorter sequence is allowed only if the report records cycles and
  warns about cache effects.
- Report p50/p95/p99 by the specified nearest-rank method; include min/max/mean
  and sample count.
- CI MUST run benchmarks as smoke tests but MUST NOT fail on absolute wall-time
  thresholds on shared runners.
- The release goal is `processing_total p95 < 100 ms` (10 Hz capacity) on one
  declared reference host with visualization and artifact writing separately
  stated. If missed, report the miss; do not tune away work or hide stages.
- A regression gate MAY compare identical host/config/commit baselines; a
  difference above 15% requires investigation, not automatic claims of cause.

## 14. Release acceptance

### `v0.1 Geometry`

- All `v0.1` public tests and ASan/UBSan pass on a clean tree.
- CI passes GCC and Clang warnings-as-errors configurations.
- Synthetic projection matches independent expected pixels within `1e-9` before
  raster rounding; round-trip SE(3) point error is at most `1e-9 m` for test
  fixtures.
- Private KITTI smoke run parses at least one image/LiDAR pair and produces a
  visually inspected depth overlay. This local check is recorded but its data is
  not committed.
- Sensitivity command emits baseline, eight perturbed panels, a combined
  comparison image, and JSON metrics with nonempty common point sets.
- Frame-tree documentation distinguishes calibrated, rectification, projection,
  and unresolved conceptual edges.
- Release benchmark report has at least 100,000 input points per sample, at least
  100 measured iterations, and complete provenance.
- README commands have been run from a fresh clone/check-out environment.

### `v0.2 Perception`

- All `v0.1` gates remain green; all deterministic perception tests pass.
- Synthetic fixture recovers the expected ground/non-ground partition and
  cluster membership under declared tolerances.
- Every filtered point is accounted as ROI-excluded, ground, retained cluster,
  rejected-small, or rejected-large.
- Private KITTI run emits overlays and JSONL for a documented frame range;
  outputs are manually inspected and clearly labeled geometric/non-semantic.

### `v0.3 Runtime`

- All earlier gates remain green; ASan/UBSan and TSan pass their supported
  configurations.
- Deterministic overload tests exercise `block` and `drop_oldest`, never exceed
  capacity, terminate cleanly, and satisfy all accounting equations.
- Synchronization tests cover positive/negative boundary deltas, tie-breaking,
  expiry, end-of-stream, injected skew, and source drops.
- A release run records at least 1,000 measured pairs for baseline and one
  overload scenario, with p50/p95/p99 and all terminal counters.
- The declared-host 10 Hz goal is reported as pass or miss without changing the
  gate definition after observing results.

### `v0.4 ROS 2`

No acceptance criteria are active until a future spec revision defines the ROS
distribution, message mappings, QoS scenarios, and rosbag evidence.

## 15. Documentation contract

The eventual README MUST include:

- one-sentence scope and an explicit non-safety disclaimer;
- dependency, configure, build, test, sanitizer, format, analysis, example, and
  benchmark commands that have actually been run;
- supported private KITTI directory layout without credentials/download bypass;
- transform notation, axis conventions, projection equation, and frame tree;
- synthetic baseline and calibration-sensitivity visuals;
- machine-readable benchmark links and no unsupported headline numbers;
- a capability matrix marking `planned`, `implemented`, and `verified` honestly;
- project code license, KITTI attribution/link, and dataset non-redistribution
  statement.

## 16. Security and robustness

- Treat dataset paths and files as untrusted local input.
- Check sizes before allocation and arithmetic before multiplication/addition.
- Set configurable upper bounds for image pixels, points per frame, frame count,
  output bytes, and queue capacity; report limit failures.
- Resolve output paths and refuse traversal outside the selected output root.
- Never log secrets, cookies, user home paths, or full private dataset paths in
  public reports.
- Fuzzing calibration, timestamp, and LiDAR parsers is recommended after `v0.3`,
  but not a release gate through `v0.3`.
