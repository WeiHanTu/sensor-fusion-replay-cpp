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

The requested synthetic extrinsic-recovery extension has additional proposed
files/targets in Section 17. It reuses these library boundaries and does not
activate perception, runtime, or ROS targets.

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

Projection results MUST reject construction with an unknown status, a visible
status without a point, or a rejection status with a point. Status/payload are
immutable after construction; consumers MUST NOT substitute default image
coordinates when a required visible payload is absent.

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

This restriction applies to the KITTI replay CLIs. The separate Section 17
experiment accepts only its authored synthetic correspondence schema; it MUST
NOT infer observations from this KITTI layout.

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
For a new run this is staged publication, not crash durability: `v0.1` performs
no fsync. Explicit overwrite removes the previous exact run before rename and
MUST NOT be described as gap-free transactional replacement or rollback. An
overwrite publication failure can lose the previous run. Prefer unique run IDs.

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

The one-frame `v0.1` writers retain their existing schema `1.0.0` stage meanings:

- `source_decode` includes calibration/layout loading, selection, image and
  point-cloud loading. It is not just image decoding.
- `geometry` includes transform/project/classify; sensitivity additionally
  includes all perturbation projections and common-set comparisons.
- `visualization` covers overlay rendering. Sensitivity panel labeling/grid
  construction are measured in its serialization boundary instead.
- Projection `serialization` measures overlay PNG encoding and writing only.
  Sensitivity `serialization` includes panel labeling, PNG encoding/writing,
  grid construction, and sensitivity `report.json` construction/writing.
- `processing_total` is the sum of those measured stages; both writers exclude
  `frames.jsonl`, `run_summary.json`, output preparation and final publication.
  It MUST NOT be equated to runtime Section 8.5 processing or end-to-end sojourn.
- With one sample all percentiles coincide; this is not a tail-latency study.

New one-frame summaries MUST include additive `measurement` fields: `clock`,
`serialization_scope`, `processing_total_scope`, `end_to_end=false`, and
`excluded` listing output preparation, JSONL/summary writing, directory
publication, queue wait and replay pacing. Additive `publication` fields record
`new_run=staged_sibling_rename`,
`overwrite=remove_existing_then_rename`, and `crash_durable=false`. These clarify
existing scope; they do not redefine timing or claim crash-safe overwrite.

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

The README MUST show a compact implemented-system diagram and link to
`docs/architecture.md` for execution/dependency maps, data ownership,
invariant-to-enforcement/test mapping, a complete error path, and reproducible
synthetic commands. Public documentation MUST explain engineering behavior,
not interview scripts or timed presentation routes; rehearsal notes stay local
and ignored. This presentation change retains the technical review requirements.
Documentation MUST distinguish historical versus fresh verification and
deterministic decisions versus variable run metadata. Public core API comments
MUST explain frames/units, borrowed or owned state, ordering, failures, and
relevant allocation costs. Static checks do not establish human usability.

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

## 17. Controlled synthetic extrinsic recovery V1

Status: design approved on 2026-10-12; implementation and final verification
are in progress. This section is
the requested additive geometry experiment contract. It does not retag
`v0.1.0`, rename `v0.2 Perception`, or add dynamics/MuJoCo dependencies.
The owner approved the written observable/convention/split/search/output design
before implementation. Passing draft runs are not published release evidence.
Rationale and review navigation are in
[docs/extrinsic_recovery.md](docs/extrinsic_recovery.md).

### 17.1 Observable and estimator boundary

The observable MUST be an explicit authored 3D-to-2D correspondence. Raw images,
LiDAR points, nominal projected pixels, and visual agreement alone MUST NOT be
treated as independent correspondence ground truth.

Proposed owned input types (names may change with equivalent semantics):

```text
ObservedPixel { u_px: double, v_px: double }
ExtrinsicObservation {
  source_index: uint64
  point_lidar: core::PointXYZI
  observed_pixel: ObservedPixel
  weight: double = 1.0
}
ExtrinsicObservationFrame {
  frame_id: uint64
  observations: vector<ExtrinsicObservation>
}
ExtrinsicCorrection {
  yaw_camera_y_rad: double
  translation_camera_x_m: double
}
```

`ImageProjection` contains camera depth. The target type MUST contain only `(u,v)`
so hidden camera depth cannot leak to the estimator. Depth from a candidate
projection remains internal geometry output. IDs are fixed-width in serialized
schemas, and `(frame_id, source_index)` is the stable observation key.

Each source index MUST bind exactly one authored point within its frame. Input
loading explicitly joins keyed point and observation records, rejects duplicate
IDs, missing references, and point-binding mismatches, and sorts by frame/source
ID before accumulation. Array position alone MUST NOT establish correspondence.
When `projectPointCloud` returns an input-span index, a validated explicit
span-index-to-source-ID map translates it back; loader compaction is not identity.

All point coordinates/reflectance and target pixels MUST be finite. Target pixels
MUST be within the declared image. Weight defaults to 1.0 when absent; a supplied
weight MUST be finite and non-negative. Zero weight is an explicit mask, counted
separately. Training MUST contain at least two frames with positive active weight
in each and finite positive total weight. Malformed observations fail closed.

The search API accepts only nominal transform, projection, training frames, and
search/loss configuration. It MUST NOT accept hidden parameters/transform,
oracle pixels/depth, noise/outlier labels, validation frames, or evaluation gates.
The harness freezes and records the result before loading held-out observations
and oracle data. Disjoint frame IDs, per-frame content identities, and exact
consumed training IDs are recorded. This is an auditable software boundary, not
a cryptographic secret or double-blind experiment.

### 17.2 Parameter convention

```text
theta = [yaw_camera_y_rad, translation_camera_x_m]
Delta_camera(theta) = [ R_y(yaw), (translation_x, 0, 0)^T; 0, 1 ]
T_candidate_camera_rect_00_lidar
  = Delta_camera(theta) * T_nominal_camera_rect_00_lidar

T_candidate = leftApplyCameraXTranslation(
                leftApplyCameraYaw(T_nominal, yaw), translation_x)
```

Yaw is about camera +y in radians; translation is along camera +x in meters.
Apply yaw first, translation second. Reversing these calls rotates the
translation vector and is a different model. Nominal endpoints MUST be `lidar`
to `camera_rect_00`. Retain the complete `3x4` projection matrix and current
depth/bounds/status contracts. No additional degree of freedom is estimated.

### 17.3 Authored fixture and split

Use a dedicated fixture `sfr_extrinsic_authored_v1`; preserve the existing
one-frame KITTI-shaped demo and its golden outputs. This new multi-frame fixture
contains JSON point/correspondence records and authored images, not KITTI files.
The generator uses hidden `Delta_camera` to project authored 3D points, retains
visible observations, then applies optional deterministic pixel corruption.
Generator accounting reports authored, hidden-visible/retained, and rejected
counts by existing projection reasons. Retained targets stay fixed for fitting.

Default nominal rotation maps camera x=-lidar y, camera y=-lidar z,
camera z=lidar x; nominal translation is zero. Projection is authored:
`P = [[520,0,480,-52],[0,520,270,0],[0,0,1,0]]`, image `960x540`,
`z_min=0.1 m`. The nonzero fourth column exercises the full matrix. Point
definitions are stated in nominal camera coordinates and converted into LiDAR
coordinates using this documented mapping.

Predeclared rich scenes (counts below are authored points before visibility):

| Frame | Role | Camera depths (m) | Lateral x/z coverage | Structure / density |
| --- | --- | --- | --- | --- |
| 0 | Train | 6, 10, 18, 30 | [-0.40, +0.40] | Four depth planes; 12 lateral x 4 vertical samples; 192 points |
| 1 | Train | 8, 15, 25 | [-0.32, +0.44] | Offset vertical panels; 10 lateral x 6 vertical samples; 180 points |
| 2 | Held out | 7, 12, 22, 40, 50 | [-0.45, +0.30] | Alternating stepped structures; 8 lateral x 4 vertical samples; 160 points |

Vertical y/z samples are respectively `[-0.18,-0.06,0.06,0.18]`,
`[-0.20,-0.12,-0.04,0.04,0.12,0.20]`, and
`[-0.16,-0.02,0.08,0.21]`. The default rich geometry has visibility margins;
visibility stress cases use separate fixtures, not post-fit filtering.
No random point split substitutes for frame 2. Additional splits are explicit
configurations; duplicates, overlap, missing IDs, empty roles, or identical
scene-content reuse across roles MUST be rejected.

`fixture.json` contains schema/version, authored provenance, generator version,
seed, frame paths/content identities, nominal transform, full P/dimensions,
noise/outlier settings, and generation counts. Point/observation files store
double coordinates to avoid float32 quantization confounding exact-recovery gates.
`oracle.json` separately holds hidden parameters/transform and clean targets,
corruption labels, and the associated fixture identity. The estimator loader
does not read that file. The harness checks its identity after fitting.

Noise is zero-mean pixel noise with a documented deterministic integer PRNG and
mapping. Default noise campaign uses bounded uniform per-coordinate noise with
theoretical standard deviation `1 px` and paired antithetic draws per frame;
the realized sample mean is also reported. Do not rely on implementation-defined
`std::*_distribution` output. Corruption that leaves image bounds is a fixture
generation failure; do not clamp, discard, or resample observations silently.

### 17.4 Loss and candidate accounting

For each positive-weight observation, `r_i = observed_uv_i - predicted_uv_i`
and `s_i = hypot(r_u, r_v)` in pixels:

```text
L2:       rho(s) = s^2
Huber:    rho(s) = 0.5*s^2                      if s <= delta_px
                   delta_px*(s-0.5*delta_px)     otherwise
loss_sum(theta) = sum_i weight_i * rho(s_i)
loss_mean(theta) = loss_sum / sum_i weight_i
```

`delta_px` is finite, positive configuration; default `2 px`. No adaptive delta,
weight tuning, or outlier rejection using oracle labels is allowed. L2 and Huber
loss values use different scaling and MUST NOT be compared as accuracy metrics.
The fixed weighted sum selects candidates; weights default to 1.0 and do not
equalize frame density. Report both sum and mean plus effective total weight.

Evaluate every observation for every candidate, including explicit zero-weight
masks. If any positive-weight observation is non-visible, reject the whole
candidate. Record all visibility reasons; do not stop counting after the first
invalid point. Non-finite residual/loss arithmetic also rejects the candidate
with a distinct numeric-failure flag. Rejected candidates have `loss=null` in
JSON, never an omitted row or serialized infinity. All-invalid search returns
`no_valid_candidate` with no fabricated parameter estimate.

For every candidate, checked counters MUST satisfy:

```text
observations = visible + non_finite + behind_or_too_near
             + non_positive_homogeneous_depth + outside_image
observations = positive_weight + zero_weight
positive_weight = positive_weight_visible + positive_weight_invalid
```

The search also accounts for attempted/valid/rejected evaluations and points
evaluated. Masking is explicit input configuration, never candidate-dependent.

### 17.5 Deterministic coarse-to-fine search

| Configuration | Yaw (degrees at CLI/report boundary) | Translation x (m) |
| --- | --- | --- |
| Global inclusive bounds | [-2.0, +2.0] | [-0.10, +0.10] |
| Coarse step | 0.10 | 0.005 |
| Fine step | 0.01 | 0.0005 |
| Refinement half-width | One coarse step | One coarse step |

All values are configuration. Fine steps are positive and no larger than coarse
steps. Coarse/fine ratios and bound spans on the coarse lattice MUST be integral
within a documented `1e-9` dimensionless validation tolerance. Enumerate integer
ticks anchored at the lower bounds, derive coordinates by multiplication, and
never accumulate floating step values. Invalid/overflowing settings fail before
allocation. Configured default limits are 100,000 candidates and
100,000,000 point projections per search; reject a request exceeding either.
Preflight conservatively budgets coarse plus maximum fine attempts before
deduplication; reported evaluations/projections count only actual unique visits.

Evaluate the complete coarse grid, select its best valid candidate, then evaluate
a fine rectangle of +/- one coarse step around it, clipped to global bounds.
Cache duplicate tick pairs; report unique evaluations and stage membership. The
defaults visit 1,681 coarse points and at most 441 fine-grid locations. Every
candidate uses training only. Candidate records sort by yaw tick then translation
tick; accumulation sorts by frame/source ID and uses compensated summation.

Order valid candidates by finite `loss_sum`, then absolute yaw, absolute
translation, signed yaw, signed translation. Use exact computed loss equality
for tie-breaking; no epsilon comparator that breaks strict ordering.

Return correction, loss, consumed training IDs, counts, all evaluated grid rows,
and separate flags: `global_bound_hit`, `refinement_edge_hit`, and numerical
identifiability diagnostics. Status is `grid_complete`, `bound_limited`,
`refinement_limited`, or `no_valid_candidate` (global-bound status takes precedence
over refinement-edge status). Exhausting this finite search is not continuous
optimization convergence or proof of the global fine-grid minimum.

Rich noiseless campaign cases must additionally match a test-only exhaustive
fine-grid reference optimum under the same comparator. Reference checks are
outside the timed production search and use reduced fixtures where needed.
If the best fine candidate touches an artificial local window edge, return
`refinement_limited`; V1 does not silently widen/restart or claim convergence.

### 17.6 Identifiability and evaluation

Yaw changes horizontal projection roughly with focal length; camera-x translation
scales roughly with inverse depth. Repeated points at one depth/near the optical
axis can give a valley of equivalent or near-equivalent corrections. Nominal
intrinsics, unestimated extrinsic axes, correspondence accuracy, and frame
definitions are assumed correct. Only this two-parameter model is tested.

Training-only diagnostics use the weighted 2D residual Jacobian at the selected
candidate, finite differences at the configured fine steps, and Eigen singular
values. Scale Jacobian columns by their respective global parameter spans
(yaw in radians, translation in meters). State central/one-sided differencing
and mark diagnostics unavailable if valid perturbations cannot be evaluated.
Numerical rank uses `sigma_j > 1e-6 * sigma_max`; rank < 2 or scaled condition
number > `1e4` is `poorly_identifiable`. These frozen diagnostic defaults are
configurable and recorded. They are local sensitivity diagnostics, not statistical
confidence or proof of unique calibration. An unavailable diagnostic cannot
support a positive identifiability claim. Plot evaluated loss surfaces as well.

Report nominal, recovered, and oracle residual metrics separately for train and
held out, pooled and per frame: nearest-rank median/p95 of radial pixel error,
RMSE `sqrt(mean(s_i^2))`, visible/invalid counts and reasons, and the common set
across all three transforms. Also report metrics against clean oracle targets
in a separately labeled evaluation table. Oracle parameters are disclosed only
after selection and are never treated as observed depth or a fitting prior.

Primary residual metrics are unweighted over all retained observations, including
injected outliers and zero-weight records. Report the explicit weight mask
separately; any active-only secondary table states its own counts. If coverage is
incomplete, visible-only metrics are labeled conditional, empty series are `null`,
and full-coverage recovery acceptance fails. Comparison on a smaller common set
alone cannot pass.
Candidate residuals, loss tables, and held-out vectors retain stable keys.

Parameter errors are recovered minus hidden: signed/absolute yaw degrees and
translation meters. Report per-axis/global bound flags. Time search with
`steady_clock`, excluding fixture generation/loading, oracle/held-out evaluation,
rendering, and publication; time those separately. Report unique candidates,
point projections, toolchain/host, and whether deterministic replay was checked.
Cross-toolchain floating results are tolerance comparisons, not a byte-identity
promise. Timings, run IDs, and host/build metadata are variable.

### 17.7 Frozen experiment campaign and gates

| Experiment | Predeclared configuration | Required evidence / gate |
| --- | --- | --- |
| A: Clean | Rich split 0,1 / 2; L2; theta `(0,0)`, `(+/-0.70 deg,0)`, `(0,+/-0.040 m)`, `(+0.70 deg,-0.040 m)`, `(-0.60 deg,+0.035 m)` | Each parameter error <= half fine step; no bound/local-edge hit; all counts/visibility valid |
| B: Noise | Rich joint `(+0.70 deg,-0.040 m)`; 1 px noise; seeds 17,29,43,71,101; Huber 2 px; no outliers | Per-seed signed/absolute errors and held-out metrics; median and min/max errors across every seed; failures retained |
| C: Outliers | Same joint correction; no pixel noise; seed 17; 10% of retained training observations, per-frame `floor(0.10*N)`; pixel offset `(+40,-30)`; paired L2/Huber inputs | Huber has lower clean held-out RMSE and lower normalized parameter error than L2 on the predeclared rich fixture; observed/outlier-inclusive metrics also reported |
| D: Geometry | Same correction/counts; rich versus training depth 20 m, x/z in [-0.005,+0.005], y/z=0; exact repeated-axis fixture as rank-one unit case; rich frame 2 held out | Show 2D loss surfaces, singular values, conditioning and parameter errors; exact degeneracy flags poor identifiability, never claimed successful recovery |
| E: Bounds/nominal | Restrict each bound separately around joint truth `(+0.70 deg,-0.040 m)`: yaw [-0.50,+0.50] deg or tx [-0.020,+0.020] m; nominal translation variant `(0.12,-0.04,0.20) m`; off-grid truth `(+0.733 deg,+0.0268 m)` | Preserve bound/failure statuses; verify left-application with non-identity nominal; report off-grid half-step target pass/miss without post-hoc changes |

For nonzero A cases, frozen held-out median, p95 and RMSE must each improve by
at least 90% relative to nominal, on complete coverage. For zero correction,
expect zero within numeric tolerance and held-out RMSE <= `1e-9 px`; ratio
improvement is undefined and serialized `null`. Parameter half-step tolerances
are `0.005 deg` and `0.00025 m`, with only `1e-12` unit-specific comparison slack.
C's normalized error is `sqrt((yaw_error_deg/4)^2 + (tx_error_m/0.20)^2)` under
the default spans. Held-out data in C stays uncorrupted; no oracle inlier labels
are used for optimization. D's near-degenerate recovery is diagnostic evidence,
not required to meet A's recovery gate. E's off-grid half-step result is always
reported; an off-grid miss prevents claiming that bound as a general guarantee.

Finite grid resolution by itself does not guarantee half-step error in each
parameter for coupled, off-grid, noisy, or degenerate data. Acceptance above is
fixture-specific. B/D/E have no invented success threshold for expected degradation;
all must produce finite/explicit-null metrics, honest statuses, and exact counts.
Any missed required gate blocks the verified V1 claim; changing a gate needs an
explicit spec revision explaining new evidence, not just a convenient threshold.

### 17.8 Fixture/run schema and artifacts

Use new schema names `sfr.extrinsic_fixture`, `sfr.extrinsic_oracle`, and
`sfr.extrinsic_recovery`, initially version `1.0.0`. Existing projection/sensitivity
schema meanings remain unchanged. Parsers reject duplicate JSON keys, unsupported
versions, non-finite/wrong-type values, unknown mandatory semantics, unsafe
relative paths, missing/duplicate references, and oracle/fixture identity mismatch.
Manifest, point/observation files and oracle use versioned canonical-JSON FNV-1a
content identities for accidental-change detection, not cryptographic authenticity.
Oracle identity and per-frame target/outlier counts are checked after fitting.

Required fixture layout (generated into ignored local output):

```text
<dataset-root>/fixture.json
<dataset-root>/oracle.json                 # harness only, opened after selection
<dataset-root>/frames/<id>/points.json
<dataset-root>/frames/<id>/observations.json
<dataset-root>/frames/<id>/image.png        # authored background/target display
```

Recovery reports MUST contain these groups:

- `schema_name`, `schema_version`, `status`, `run_id`, `claim_boundary`.
- `build {git_commit, git_dirty, build_type, compiler}` and
  `host {os, arch, cpu}` using existing configure-time provenance.
- `fixture {id, version, content_identity, generator_version, seed,
  provenance, noise, outliers, generation_counts}`.
- `model {parameter_order, units, application, nominal_transform,
  nominal_correction, hidden_parameters, hidden_transform, recovered_parameters,
  recovered_transform, projection_matrix, dimensions, z_min}`. Nominal correction
  is `(0,0)` relative to its own supplied transform; it is not a decomposition of
  that transform into two assumed absolute extrinsic coordinates.
- `split {train_frame_ids, validation_frame_ids, consumed_train_ids,
  frame_content_identities, disjoint_checked, held_out_loaded_after_search}`.
- `search {config, loss_name, huber_delta_px, loss_sum, loss_mean,
  total_weight, best_parameters, result_status, global_bound_hits,
  refinement_edge_hit, evaluations, points_evaluated, identifiability}`.
- `parameter_errors`, `metrics {train, validation}` with per-frame and pooled
  nominal/recovered/oracle tables, observed and clean-target tables, common sets,
  explicit masks, visibility reasons and metric sample counts.
- `timing {clock, search_ms, evaluation_ms, rendering_ms, staging_write_ms,
  publication_scope}`, `reproducibility`, `accounting`, `acceptance`, `artifacts`.

Full attempted candidate rows go to `loss_surface.json`: integer ticks, physical
parameters, coarse/fine membership, finite loss or `null`, rejection reason,
weights and projection counts. Plot coarse and fine regions separately with
axis units, loss family/delta, visibility mask, and selected estimate; never
interpolate missing rejected cells as good fits. Oracle annotation is harness
output only. Residual plot uses fixed identities and visibly marks invalid data.

Publish a unique staged run with `report.json`, `loss_surface.json`,
`residuals.json`, `loss_surface.png`, `residual_comparison.png`, and
`overlays/<frame_id>/{nominal,recovered,oracle}.png` for every selected frame.
When no estimate exists, report its absence and mark recovered artifacts
unavailable; still preserve the failure report and surface. Paths are relative.
`status=complete` means checked outputs closed before publication, separately
from `acceptance.passed`. Report completed-but-unaccepted experiments honestly.

Reuse the staged sibling/RAII ownership convention and existing output-error
categories. Reject existing nonempty exact runs unless overwrite is explicit;
preexisting staging is never removed. Preserve the documented remove-then-rename
overwrite risk and absence of crash durability. Output targets must not resolve
inside the input fixture. Timings embedded in the report exclude final report
write/publication as explicitly recorded; CLI may measure total wall time separately.

### 17.9 Architecture, CLI, and tests

Proposed files/targets:

- `include/sfr/geometry/extrinsic_recovery.hpp`,
  `src/geometry/extrinsic_recovery.cpp`: typed inputs, validation, candidate
  evaluation/search, residual metrics and diagnostics; Eigen-only `sfr_geometry`.
- `include/sfr/io/extrinsic_fixture.hpp`, `src/io/extrinsic_fixture.cpp`:
  authored fixture generation/loading, keyed joins, oracle loading and identities.
  JSON joins `sfr_io` privately; geometry stays independent of JSON/OpenCV.
- `include/sfr/viz/extrinsic_recovery_artifact.hpp`,
  `src/viz/extrinsic_recovery_artifact.cpp`: OpenCV plots/overlays and staged reports.
- `apps/estimate_extrinsics.cpp`, `examples/generate_extrinsic_fixture.cpp`,
  and a C++ campaign driver under `examples/`: argument parsing/composition.
- Public unit/parser/artifact tests and `tests/integration/estimate_extrinsics_cli_test.cpp`;
  fixture README stores authored provenance and independent expected values.

Proposed CLI interface, not yet an executable command:

```text
estimate_extrinsics --dataset-root <authored-fixture-root>
  --train-frames 0,1 --validation-frames 2
  --parameters yaw_camera_y,translation_camera_x
  --loss huber --huber-delta-px 2
  --yaw-min-deg -2 --yaw-max-deg 2 --yaw-coarse-step-deg 0.10 --yaw-fine-step-deg 0.01
  --tx-min-m -0.10 --tx-max-m 0.10 --tx-coarse-step-m 0.005 --tx-fine-step-m 0.0005
  --output-dir <ignored-output-root> [--run-id <safe-id>] [--overwrite]
```

Help documents defaults, fixed V1 parameter set and scientific failure statuses.
Exit categories remain 0/2/3/4/5. Bound/refinement limitation, no valid candidate,
or poor/unavailable identifiability returns 4 after publishing a diagnostic run;
valid grid completion returns 0 even when oracle evaluation reports a missed
acceptance gate. The campaign driver checks acceptance fields explicitly. Input
and configuration errors return 3/2 before publishing; output failure returns 5.

Required tests cover zero/positive/negative/axis-only/joint recovery; non-identity
nominal and independent composition/sign arithmetic; deterministic ties; invalid
grids/resource limits; non-finite observations/weights; duplicate/missing IDs and
shuffled keyed records; complete candidate visibility/numeric accounting;
bound/local-edge/all-invalid results; fixed-seed noise; authored Huber versus L2;
rank-one/weak geometry; and exhaustive-reference comparison. Leakage tests alter
held-out pixels/IDs and oracle values while keeping train/nominal/config fixed:
the selected parameters and candidate table MUST remain unchanged; overlapping
split/input-integrity violations MUST fail.

Schema/artifact/CLI tests cover required fields, checked counts, scientific
failure reports, publication/overwrite/staging ownership, unsafe paths, malformed
fixtures, unsupported parameters, help, and every exit category. Independent
hand-derived pixel/loss cases are required: a generator and estimator sharing a
projection implementation alone cannot detect a shared sign/frame mistake.

V1 completion requires all existing/new tests, clean GCC/Clang CI, ASan/UBSan,
format/tidy, inspected generated visuals, and retained clean-commit Release A-E
reports with every required gate and failure accounted for. No absolute CI
wall-time gate is added. Full 6-DoF, nonlinear solvers, real-data objectives,
MuJoCo/dynamics, and interview material in public files remain outside V1.
