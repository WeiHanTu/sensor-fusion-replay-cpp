# Security and input handling

This is a local offline engineering experiment, not a hardened service or safety
system. Only current `main` receives maintenance; historical tags are evidence,
not a promise of backported security fixes or a stable ABI.

## Reporting a possible vulnerability

Use GitHub's **Report a vulnerability** button on this repository's Security tab
if private reporting is available. Its availability is not guaranteed by this
document. If unavailable, open a minimal issue asking the maintainer for a
private reporting channel, without exploit details, sensitive inputs or secrets.
Do not post a weaponized reproduction publicly. No response-time guarantee is
offered. A normal correctness bug with a harmless authored fixture can use the
public bug template.

Include the exact commit, toolchain/platform, affected parser or output boundary,
expected impact and a minimal project-authored reproduction through the agreed
channel. Never attach KITTI content, cookies, tokens or private dataset paths.

## Operational boundaries

- Treat calibration, JSON, timestamps, image/cloud files and output paths as
  untrusted. Size/path checks reduce risk; they are not a parser-security proof.
- Image decoding and math/JSON dependencies retain their own vulnerabilities and
  licenses. Keep system packages current; pinned fallbacks are not auto-updated.
- Prefer unique output runs. Explicit overwrite removes the old exact run before
  rename; it is not crash-durable, gap-free or rollback-safe.
- ASan/UBSan and static analysis catch classes of defects, not all vulnerabilities.
  No full fuzz campaign, sandboxed decoder, hard-real-time guarantee or safety
  certification is claimed.
