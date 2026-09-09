# Frame and Projection Conventions

Status: geometry contract for `v0.1`; world/vehicle pose support is not
implemented.

## Transform notation

`T_target_source` maps coordinates expressed in `source` into `target`:

```text
p_target = T_target_source * p_source
```

Composition is written in application order from right to left:

```text
T_camera_lidar = T_camera_vehicle * T_vehicle_lidar
```

Lengths are meters. Angles are radians inside the library. KITTI's Velodyne
frame uses x-forward, y-left, z-up. Its camera frame uses x-right, y-down,
z-forward.

## Frame tree

```mermaid
flowchart LR
  W[world] -. "unresolved: no OXTS adapter" .-> V[vehicle]
  V -. "unresolved: no vehicle pose edge" .-> L[lidar / Velodyne]
  L -->|"T_camera_raw_00_lidar from R,T"| C0[camera_raw_00]
  C0 -->|"R_rect_00 rectification"| CR[camera_rect_00]
  CR -->|"P_rect_02, perspective projection"| I[image_02]
```

The first two edges are explanatory only. `image_02` is a 2D projection domain,
not an SE(3) coordinate frame. The frame graph must not fabricate unresolved
transforms.

## KITTI color-camera projection

For homogeneous Velodyne point `X_lidar = [x, y, z, 1]^T`:

```text
X_raw_00  = T_camera_raw_00_lidar * X_lidar
X_rect_00 = R_rect_00_4x4 * X_raw_00
q         = P_rect_02 * X_rect_00
u         = q.x / q.z
v         = q.y / q.z
```

`P_rect_02` remains a full 3x4 matrix. Collapsing it to a 3x3 intrinsic matrix
would discard its translation/baseline term.

A projected point first passes continuous bounds `0 <= u < width` and
`0 <= v < height`. Rasterization rounds to the nearest integer and clamps the
result to the valid raster boundary; this makes values just below the exclusive
upper edge deterministic. When multiple points select one pixel, only the
nearest positive rectified-camera z depth is rendered. Depth color uses OpenCV
`COLORMAP_TURBO` over the configured clamped range, which is also recorded in
the JSON report.

## Calibration sensitivity convention

Sensitivity perturbations are left-applied in rectified-camera coordinates:

```text
T'_camera_rect_00_lidar = Delta_camera * T_camera_rect_00_lidar
```

The fixed `v0.1` suite rotates about camera `+y` at `-1.0`, `-0.5`, `+0.5`, and
`+1.0 deg`, then translates along camera `+x` at `-0.05`, `-0.01`, `+0.01`, and
`+0.05 m`. Positive camera `+x` is image-right. For a point on the optical axis,
a positive camera `+y` yaw also moves its projection image-right under this
active left-applied convention.

Pixel-displacement statistics use only source points visible in both the
baseline and perturbed images. Appearance and disappearance are counted
separately. `50*tan(1 deg) ~= 0.87 m` is only an analytic angular-offset
illustration; it is not a measured KITTI calibration error or a pixel
reprojection guarantee.

Primary references:

- [KITTI Raw Data](https://www.cvlibs.net/datasets/kitti/raw_data.php)
- [Vision meets Robotics: The KITTI Dataset](https://www.cvlibs.net/publications/Geiger2013IJRR.pdf)
