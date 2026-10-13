# Documentation

Start with the task you need to perform. Historical verification records describe
their exact commits; they are not evidence for untested later changes.

| Task | Document |
| --- | --- |
| Build and run without KITTI; use private KITTI locally | [Getting started](getting_started.md) |
| Trace execution, ownership, failures and tests | [Architecture](architecture.md) |
| Understand frames, units and the full projection matrix | [Frame conventions](frame_conventions.md) |
| Reproduce bounded synthetic recovery and its A–E matrix | [Extrinsic recovery](extrinsic_recovery.md) |
| Interpret projection microbenchmark measurements | [Benchmark methodology](benchmark_walkthrough.md) |
| Inspect dependency versions, acquisition and license boundaries | [Dependencies](dependencies.md) |
| Change the repository and run its gates | [Contributing](../CONTRIBUTING.md) |

## Evidence records

- [Geometry release](v0.1_verification.md): immutable `v0.1.0` baseline.
- [API/reviewability maintenance](reviewability_verification.md): post-release changes.
- [Synthetic recovery verification](extrinsic_recovery_verification.md) and
  [all-case JSON](evidence/extrinsic_recovery_v1.json): includes diagnostic failures.
- [Public image provenance](images/README.md): authored projection/sensitivity output.
- [Repository maintenance](repository_maintenance.md): onboarding/CI/checker scope and verification.

## Contracts versus roadmap

[intend.md](../intend.md) defines purpose and claim boundaries;
[spec.md](../spec.md) defines required behavior and acceptance;
[plan.md](../plan.md) records ordered work and completion evidence.
Planned perception/runtime requirements are not implemented capabilities.
The [change log](../CHANGELOG.md) summarizes shipped and unreleased changes.
