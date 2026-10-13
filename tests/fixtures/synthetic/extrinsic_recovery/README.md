# Authored extrinsic-recovery fixture provenance

Every point, projection matrix, pixel, image and perturbation in this experiment
is authored for this project. There is no KITTI or MuJoCo data. Runtime fixtures
are generated into temporary test directories or ignored `artifacts/` output.

The three rich scenes and independent frame split are defined in `spec.md` §17.
Camera coordinates map to LiDAR `(z,-x,-y)`. The full projection matrix has
`fx=fy=520`, `cx=480`, `cy=270`, and `P(0,3)=-52`.

For the first frame-0 point, camera `(x,y,z)=(-2.4,-1.08,6)` maps to LiDAR
`(6,2.4,1.08)`. With zero correction, the independently derived target is
`u=480-208-52/6`, `v=270-93.6`. Unit tests use these literal expressions instead
of producing the expected pixel through the projection implementation.

Stable point IDs are explicit records (11,14,17,...), not implicit array offsets.
The generator stores keyed targets under a hidden correction and keeps oracle
parameters/clean targets in a separate file opened only after estimation.
SplitMix64 integer arithmetic and a top-53-bit uniform mapping produce paired
antithetic pixel noise; configured outliers are selected by sorted seeded ranks.
Content identities are versioned FNV-1a checksums of canonical JSON, not
cryptographic authentication or a claim of secret hidden calibration.
