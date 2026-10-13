#!/usr/bin/env python3
"""Offline checks for this repository's deliberately small documentation syntax.

Not a general Markdown/YAML parser, secret scanner, provenance audit or renderer.
Reads tracked and non-ignored new files; never modifies the checkout or fetches URLs.
"""

import argparse
import json
from pathlib import Path
import re
import subprocess
import sys
from urllib.parse import unquote, urlsplit


CPP_SUFFIXES = {".h", ".hpp", ".cc", ".cpp", ".cxx"}
CPP_ROOTS = {"include", "src", "apps", "tests", "benchmarks", "examples"}
LINK = re.compile(r"!?\[[^\]\n]*\]\((?:<([^>]+)>|([^\s)]+))\)")


def markdown_body(text):
    """Remove fenced code while preserving line numbers; report open fences."""
    lines = []
    fence = None
    for line in text.splitlines():
        marker = re.match(r"^\s{0,3}(`{3,}|~{3,})(.*)$", line)
        if fence:
            if marker and marker[1][0] == fence[0] and len(marker[1]) >= len(fence):
                if not marker[2].strip():
                    fence = None
            lines.append("")
        elif marker:
            fence = marker[1]
            lines.append("")
        else:
            lines.append(line)
    return "\n".join(lines), fence is not None


def heading_anchors(text):
    """GitHub-style anchors for ATX headings used here, including duplicate suffixes."""
    body, _ = markdown_body(text)
    anchors = set()
    for line in body.splitlines():
        heading = re.match(r"^#{1,6}\s+(.+?)(?:\s+#+)?$", line)
        if not heading:
            continue
        label = LINK.sub(lambda match: re.search(r"\[([^]]+)\]", match[0])[1], heading[1])
        label = re.sub(r"<[^>]*>", "", label).lower()
        slug = re.sub(r"[^\w\- ]", "", label).replace(" ", "-")
        anchor = slug
        suffix = 0
        while anchor in anchors:
            suffix += 1
            anchor = f"{slug}-{suffix}"
        anchors.add(anchor)
    return anchors


def check_markdown(root, files):
    errors = []
    texts = {}
    for name in files:
        path = root / name
        if not path.resolve().is_relative_to(root):
            errors.append(f"{name}: document escapes repository")
            continue
        texts[name] = path.read_text(encoding="utf-8")
    anchors = {name: heading_anchors(text) for name, text in texts.items()}
    for name, text in texts.items():
        body, open_fence = markdown_body(text)
        if open_fence:
            errors.append(f"{name}: unclosed code fence")
        # Inline-code examples are not links. Headings retain their code labels above.
        body = re.sub(r"(`+).*?\1", "", body)
        for match in LINK.finditer(body):
            target = urlsplit(match[1] or match[2])
            if target.scheme in {"https", "http", "mailto"}:
                continue
            line = body.count("\n", 0, match.start()) + 1
            location = f"{name}:{line}"
            if target.scheme or target.netloc or target.path.startswith("/"):
                errors.append(f"{location}: non-portable link {match[0]}")
                continue
            path = (root / name).parent / unquote(target.path) if target.path else root / name
            path = path.resolve()
            if not path.is_relative_to(root):
                errors.append(f"{location}: link escapes repository")
                continue
            relative = path.relative_to(root).as_posix()
            if not path.is_file():
                errors.append(f"{location}: missing link target {relative}")
            elif target.fragment and relative in anchors:
                if unquote(target.fragment) not in anchors[relative]:
                    errors.append(f"{location}: missing anchor {relative}#{target.fragment}")
            elif target.fragment:
                errors.append(f"{location}: unsupported fragment on non-Markdown target")
        if re.search(r"/(?:Users|home)/[^\s/]+/", text):
            errors.append(f"{name}: private absolute home path in public documentation")
    return errors


def check_cpp_coverage(cmake, files):
    block = cmake.split("sfr_add_developer_tools(", 1)[-1]
    registered = set(re.findall(r"^\s+([\w./-]+\.(?:h|hpp|cc|cpp|cxx))\)?\s*$", block, re.M))
    actual = {name for name in files if Path(name).suffix in CPP_SUFFIXES
              and Path(name).parts[0] in CPP_ROOTS}
    return ([f"C++ quality target missing: {name}" for name in sorted(actual - registered)]
            + [f"C++ quality target stale: {name}" for name in sorted(registered - actual)])


def check_build_contract(cmake, presets, workflow):
    errors = []
    match = re.search(r"cmake_minimum_required\(VERSION (\d+)\.(\d+)(?:\.(\d+))?\)", cmake)
    if not match:
        return ["CMake minimum declaration missing"]
    minimum = tuple(int(value or 0) for value in match.groups())
    declared = presets.get("cmakeMinimumRequired", {})
    preset_minimum = tuple(declared.get(key, 0) for key in ("major", "minor", "patch"))
    if minimum != preset_minimum:
        errors.append("CMake root/preset minimums disagree")
    if presets.get("version") != 6:
        errors.append("preset schema changed: review checker/version compatibility")
    if minimum < (3, 25, 0):
        errors.append("preset schema 6 requires CMake 3.25+")
    if "EXCLUDE_FROM_ALL" in cmake and minimum < (3, 28, 0):
        errors.append("FetchContent EXCLUDE_FROM_ALL requires CMake 3.28+")
    for preset in presets.get("testPresets", []):
        if preset.get("execution", {}).get("noTestsAction") != "error":
            errors.append(f"test preset {preset['name']}: empty suite must fail")
    for tag in re.findall(r"\bGIT_TAG\s+(\S+)", cmake):
        if not re.fullmatch(r"[0-9a-f]{40}", tag):
            errors.append(f"mutable dependency ref: {tag}")
    if "GIT_SHALLOW TRUE" in cmake:
        errors.append("shallow-tag cloning cannot guarantee arbitrary pinned commits")
    for action in re.findall(r"uses:\s*([^\s#]+)", workflow):
        if not re.fullmatch(r"actions/[\w-]+@[0-9a-f]{40}", action):
            errors.append(f"Action needs reviewed immutable commit: {action}")
    for runner in re.findall(r"runs-on:\s*([^\s#]+)", workflow):
        if runner != "ubuntu-24.04":
            errors.append(f"unreviewed CI runner: {runner}")
    for output in re.findall(r"--output-junit\s+(\S+)", workflow):
        if output != "test-results.xml":
            errors.append("CTest JUnit path must be test-results.xml, relative to its binary directory")
    return errors


def forbidden_path(name):
    path = Path(name)
    if path.parts[0] in {"build", "artifacts", "data", "datasets", "output", "outputs", "out"}:
        return True
    if name.endswith((".zip", ".tar", ".tar.gz", ".tgz", ".7z")):
        return True
    if any("_drive_" in part and part.endswith("_sync") or part == "velodyne_points"
           for part in path.parts):
        return True
    if path.name in {"calib_cam_to_cam.txt", "calib_velo_to_cam.txt", "calib_imu_to_velo.txt",
                     "timestamps.txt", "tracklet_labels.xml"}:
        return not name.startswith("tests/fixtures/synthetic/")
    if any(part in {"image_00", "image_01", "image_02", "image_03"} for part in path.parts):
        return "data" in path.parts
    return path.name == "CMakeUserPresets.json" or "__pycache__" in path.parts


def shell_commands(text):
    """Normalize simple continued lines, not shell syntax or arbitrary scripts."""
    joined = re.sub(r"\\\n\s*", " ", text)
    return {" ".join(line.split()) for line in joined.splitlines()
            if line.strip().startswith("./build/release/")}


def check_quickstart(readme, workflow):
    section = readme.split("## Quick start", 1)[-1].split("## Recovery results", 1)[0]
    commands = shell_commands(section)
    if len(commands) != 4:
        return ["README quick start must retain the four reviewed synthetic CLI commands"]
    missing = commands - shell_commands(workflow)
    return [f"CI quick start differs from README: {command}" for command in sorted(missing)]


def repository_files(root):
    result = subprocess.run(["git", "ls-files", "--cached", "--others", "--exclude-standard", "-z"],
                            cwd=root, check=True, capture_output=True)
    return sorted({name for name in result.stdout.decode("utf-8").split("\0")
                   if name and (root / name).is_file()})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[1])
    root = parser.parse_args().root.resolve()
    try:
        files = repository_files(root)
        cmake = (root / "CMakeLists.txt").read_text(encoding="utf-8")
        workflow = (root / ".github/workflows/ci.yml").read_text(encoding="utf-8")
        presets = json.loads((root / "CMakePresets.json").read_text(encoding="utf-8"))
        readme = (root / "README.md").read_text(encoding="utf-8")
        markdown = [name for name in files if name.endswith(".md")]
        errors = check_markdown(root, markdown) + check_cpp_coverage(cmake, files)
        errors += check_build_contract(cmake, presets, workflow)
        errors += check_quickstart(readme, workflow)
        errors += [f"forbidden repository path: {name}" for name in files if forbidden_path(name)]
    except (OSError, ValueError, subprocess.CalledProcessError) as error:
        print(f"repository check failed: {error}", file=sys.stderr)
        return 2
    if errors:
        print("\n".join(errors), file=sys.stderr)
        return 1
    print(f"Repository contracts passed: {len(markdown)} Markdown files; "
          "local navigation, C++ coverage, build/CI refs, quick start and known path hygiene.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
