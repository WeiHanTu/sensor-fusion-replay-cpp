## Scope

What changed and why? Link the relevant specification/plan section. Separate
implementation from refactoring and state any contract/schema change.

## Verification

Record exact commands/results; replace unchecked items with `not run` and a reason.

- [ ] Regression fails before a bug fix and passes afterward (or not applicable).
- [ ] C++ dev/Release tests; supported ASan/UBSan.
- [ ] format-check and full clang-tidy.
- [ ] Repository checker and its independent tests.
- [ ] Relevant synthetic CLI/artifact scenario; PNGs opened and inspected.
- [ ] `git diff --check` and full diff review.

## Evidence and boundaries

Commit/build/configuration for any result, measured/excluded work, known failures
and limitations. Historical evidence is not a current measurement.

- [ ] Intent/spec/plan/architecture and changed APIs agree.
- [ ] No KITTI content, derived screenshots, credentials, private paths, generated
      bulk output or private interview material included.
- [ ] No acceptance threshold weakened or unsupported calibration/safety claim added.
