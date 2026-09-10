# Projection Microbenchmark Walkthrough

## Claim boundary

`benchmark_projection` is a geometry microbenchmark. It measures the time to
transform one authored finite LiDAR point cloud into the rectified camera frame,
project it through a full `3x4` matrix, classify every point, and construct the
visible-point output vector.

It does **not** measure KITTI file or image decoding, overlay rendering, JSON
serialization, queue wait, replay pacing, synchronization, or end-to-end
latency. Its throughput is not a real-time, dataset, perception, or safety
claim.

## Authored deterministic fixture

The report identifies the fixture as
`authored_projection_benchmark_v1`. It contains no KITTI data. A fixed 64-bit
linear congruential generator with seed `0x123456789abcdef0` supplies repeatable
depth, pixel, and reflectance values. Point indices are partitioned before
generation so the expected terminal counts are exact:

- 70% project inside the `1240 x 376` image;
- 15% have valid positive depth but project outside the image;
- 15% are behind the camera or at/below the `0.1 m` minimum depth.

The LiDAR-to-camera rotation implements the declared axis mapping with no
translation:

```text
camera x = -lidar y
camera y = -lidar z
camera z =  lidar x
```

The authored projection matrix is:

```text
[700,   0, 620, 35.00]
[  0, 700, 188,  0.25]
[  0,   0,   1,  0.00]
```

Visible and outside-image points are generated from desired image coordinates
and converted algebraically back to LiDAR coordinates. This avoids borrowing
camera parameters or distributions from a private dataset. The JSON report
records the seed, generator description, exact transform, projection matrix,
image dimensions, and minimum depth needed to audit the fixture.

## Measurement and accounting

The harness uses `std::chrono::steady_clock`. The default release run performs
10 unmeasured warm-up iterations followed by 100 measured iterations, each with
100,000 points. Fixture construction is outside the timed region. Classification
validation and report serialization occur after timing.

Every warm-up and measured iteration must satisfy both:

```text
input = visible
      + non_finite_input
      + behind_or_too_near
      + non_positive_homogeneous_depth
      + outside_image
```

and the fixture's exact category counts. A mismatch aborts the benchmark rather
than producing plausible-looking metrics. The report records the number of
warm-up and measured iterations that passed these checks.

Latency samples are sorted and summarized with the nearest-rank definition:

```text
rank = clamp(ceil(p * N), 1, N)
percentile(p) = sorted_samples[rank - 1]
```

The report includes sample count, min, max, mean, p50, p95, and p99 latency.
It reports aggregate point throughput derived from total points divided by total
measured time, plus the rate corresponding to p50 latency. The latter is named
`at_p50_latency`; it is not misrepresented as a percentile of independently
sorted throughput samples. Throughput describes only the timed geometry boundary
above.

## Running it

For an ordinary local report:

```bash
cmake --preset release
cmake --build --preset release --parallel
./build/release/benchmark_projection \
  --output-json artifacts/benchmarks/projection_benchmark_current.json \
  --overwrite
```

Non-smoke runs require `Release` or `RelWithDebInfo`. A non-smoke JSON report is
also rejected when the build metadata says the tree was dirty at CMake configure
time. Therefore a release evidence run follows this order:

1. commit the benchmark implementation;
2. confirm the tree is clean;
3. configure and build the release preset from that clean commit;
4. run the default benchmark and preserve its JSON report;
5. review the report before making any performance claim.

`--smoke` uses 1,000 points, one warm-up, and two measured iterations. It exists
for hermetic CLI/schema testing and may run in a dirty debug build. Smoke timing
is not release evidence. `--overwrite` replaces only the exact report path; it
is never implicit.

## Report interpretation

The JSON schema version is `1.1.0`. Build and host fields provide commit, dirty
state, build type, compiler, OS, architecture, and CPU. Configuration and
fixture fields bind the results to the exact workload. Verification fields prove
that all declared iterations were checked.

A retained report supports only this statement: on the recorded host and build,
the recorded commit processed the authored geometry fixture with the reported
latency and accounting. It does not prove KITTI correctness, calibration
accuracy, deterministic wall time, or `v0.1` release completion.
