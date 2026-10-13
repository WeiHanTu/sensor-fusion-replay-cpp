"""Independent tiny fixtures for the offline repository checker."""

import importlib.util
from pathlib import Path
import tempfile
import unittest

SPEC = importlib.util.spec_from_file_location(
    "checker", Path(__file__).resolve().parents[1] / "check_repository.py")
CHECKER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(CHECKER)


class MarkdownTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name).resolve()

    def write(self, name, text):
        path = self.root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text, encoding="utf-8")

    def test_valid_relative_self_and_encoded_links(self):
        self.write("README.md", "# Root\n[Self](#root)\n[Other](<docs/two words.md#units-m>)")
        self.write("docs/two words.md", "# Units `m`\n[Back](../README.md#root)")
        self.assertEqual([], CHECKER.check_markdown(self.root, ["README.md", "docs/two words.md"]))

    def test_missing_file_and_anchor(self):
        self.write("README.md", "# Root\n[X](missing.md)\n[Y](#missing)")
        errors = CHECKER.check_markdown(self.root, ["README.md"])
        self.assertEqual(2, len(errors))
        self.assertIn("README.md:2", errors[0])
        self.assertIn("missing anchor", errors[1])

    def test_duplicate_heading_suffixes_and_punctuation(self):
        anchors = CHECKER.heading_anchors("# API `T_target_source`\n## API `T_target_source`\n# A — B")
        self.assertEqual({"api-t_target_source", "api-t_target_source-1", "a--b"}, anchors)

    def test_fenced_and_inline_examples_are_not_links(self):
        self.write("README.md", "```text\n[Bad](missing.md)\n```\n`[Bad](missing.md)`\n")
        self.assertEqual([], CHECKER.check_markdown(self.root, ["README.md"]))

    def test_unclosed_fence(self):
        self.write("README.md", "~~~text\n[Bad](missing.md)\n")
        self.assertEqual(["README.md: unclosed code fence"],
                         CHECKER.check_markdown(self.root, ["README.md"]))

    def test_different_marker_does_not_close_fence(self):
        body, opened = CHECKER.markdown_body("```text\n~~~\n[X](missing)\n```")
        self.assertFalse(opened)
        self.assertNotIn("missing", body)

    def test_escape_absolute_and_remote_links(self):
        self.write("README.md", "[A](../../escape)\n[B](/private/x)\n"
                   "[C](https://example.invalid/not-fetched#x)\n[D](mailto:example@example.invalid)")
        errors = CHECKER.check_markdown(self.root, ["README.md"])
        self.assertEqual(2, len(errors))
        self.assertIn("escapes", errors[0])
        self.assertIn("non-portable", errors[1])

    def test_symlink_escape(self):
        (self.root / "outside.md").symlink_to(self.root.parent / "outside.md")
        self.write("README.md", "[X](outside.md)")
        self.assertIn("escapes", CHECKER.check_markdown(self.root, ["README.md"])[0])

    def test_private_home_path(self):
        self.write("README.md", "Private path: /Users/example/private/data")
        self.assertIn("private absolute", CHECKER.check_markdown(self.root, ["README.md"])[0])

    def test_document_symlink_is_rejected_before_reading_outside_content(self):
        with tempfile.TemporaryDirectory() as outside:
            target = Path(outside) / "external.md"
            target.write_text("# Outside\n", encoding="utf-8")
            (self.root / "linked.md").symlink_to(target)
            errors = CHECKER.check_markdown(self.root, ["linked.md"])
            self.assertEqual(["linked.md: document escapes repository"], errors)


class ContractTests(unittest.TestCase):
    def test_cpp_registration_missing_stale_and_complete(self):
        cmake = "sfr_add_developer_tools(\n  FILES\n  src/a.cpp\n  include/a.hpp)\n"
        self.assertEqual([], CHECKER.check_cpp_coverage(cmake, ["src/a.cpp", "include/a.hpp"]))
        errors = CHECKER.check_cpp_coverage(cmake, ["src/a.cpp", "tests/new.cpp"])
        self.assertEqual(["C++ quality target missing: tests/new.cpp",
                          "C++ quality target stale: include/a.hpp"], errors)

    def test_build_versions_and_immutable_refs(self):
        sha = "a" * 40
        cmake = f"cmake_minimum_required(VERSION 3.28)\nEXCLUDE_FROM_ALL\nGIT_TAG {sha}"
        presets = {"version": 6, "cmakeMinimumRequired": {"major": 3, "minor": 28},
                   "testPresets": [{"name": "dev", "execution": {"noTestsAction": "error"}}]}
        workflow = f"runs-on: ubuntu-24.04\nuses: actions/checkout@{sha}"
        self.assertEqual([], CHECKER.check_build_contract(cmake, presets, workflow))
        errors = CHECKER.check_build_contract(cmake.replace("3.28", "3.24"), presets,
                                             "runs-on: ubuntu-latest\nuses: actions/checkout@v7")
        self.assertEqual(5, len(errors))

    def test_mutable_dependency_empty_suite_and_shallow_clone(self):
        cmake = "cmake_minimum_required(VERSION 3.28)\nGIT_TAG v1\nGIT_SHALLOW TRUE"
        presets = {"version": 6, "cmakeMinimumRequired": {"major": 3, "minor": 28},
                   "testPresets": [{"name": "dev"}]}
        errors = CHECKER.check_build_contract(cmake, presets, "runs-on: ubuntu-24.04")
        self.assertEqual(3, len(errors))

    def test_data_hygiene_preserves_only_authored_calibration_exception(self):
        forbidden = ["build/a", "artifacts/x.json", "data/a", "datasets/a", "a.zip",
                     "data_drive_01_sync/a", "calib_cam_to_cam.txt", "timestamps.txt",
                     "velodyne_points/a.bin", "image_02/data/a.png", "CMakeUserPresets.json"]
        for name in forbidden:
            with self.subTest(name=name):
                self.assertTrue(CHECKER.forbidden_path(name))
        for name in ["tests/fixtures/synthetic/calib_cam_to_cam.txt", "src/io/kitti_io.cpp",
                     "docs/images/synthetic_projection.png"]:
            with self.subTest(name=name):
                self.assertFalse(CHECKER.forbidden_path(name))

    def test_quickstart_matches_continued_commands_and_detects_drift(self):
        commands = "\n".join(f"./build/release/tool{i} \\\n  --output-dir artifacts/x{i}" for i in range(4))
        self.assertEqual([], CHECKER.check_quickstart("## Quick start\n" + commands, commands))
        errors = CHECKER.check_quickstart("## Quick start\n" + commands,
                                         commands.replace("artifacts/x3", "artifacts/other"))
        self.assertEqual(1, len(errors))
        self.assertIn("differs", errors[0])


if __name__ == "__main__":
    unittest.main()
