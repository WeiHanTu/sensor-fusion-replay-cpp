# Synthetic Fixture Provenance

Every value in this directory is authored for this repository. No KITTI image,
point cloud, calibration dump, timestamp file, or derived screenshot is stored
here.

The calibration uses the documented KITTI axis mapping with zero translation:

```text
camera x = -lidar y
camera y = -lidar z
camera z =  lidar x
```

`R_rect_00` is identity. `P_rect_02` is a deliberately small full `3x4`
projection with `fx=fy=2`, `cx=4`, `cy=3`, and a nonzero x translation term of
`1`. The declared synthetic image size is `8x6` pixels. Values are simple enough
to calculate independently by hand.

Binary point frames and PNG images are generated in temporary directories by
the C++ tests, so this text-only fixture remains auditable.
