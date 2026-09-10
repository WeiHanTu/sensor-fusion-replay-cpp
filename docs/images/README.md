# Public image provenance

Both PNG files in this directory are generated entirely from project-authored
synthetic inputs. They contain no KITTI data and do not claim real-scene or
calibration accuracy.

Generation command from repository root:

```bash
cmake --preset release
cmake --build --preset release --parallel
./examples/generate_public_artifacts.sh build/release docs/images
```

Inspected on 2026-09-10 from clean implementation commit
`185e287ca41e1696d0752570b6dba36780639af3`:

- `synthetic_projection.png`:
  `0c76352b73a2a855f1ae0ca495cc17a0710f8913b6d4055be3e9643e2289584d`
- `synthetic_calibration_sensitivity.png`:
  `0df456bfe743a8a5eb0ff9d86af54e89645b112a2acd1c529352b266e4a02e4a`

The hashes cover the exact checked-in renderings; OpenCV version or image-codec
changes may produce different bytes without changing the intended geometry.
