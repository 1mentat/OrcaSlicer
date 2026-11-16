#!/usr/bin/env python3
"""Focused tests for CatchAddTestsPreprocess and its CMake integration."""

import json
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


TEST_DIRECTORY = Path(__file__).resolve().parent
HERE = (TEST_DIRECTORY if (TEST_DIRECTORY / "CatchAddTestsPreprocess.py").is_file()
        else TEST_DIRECTORY.parents[1] / "tests/catch2/extras")
PREPROCESSOR = HERE / "CatchAddTestsPreprocess.py"
SCRIPT = HERE / "CatchAddTests.cmake"


class CatchPreprocessorTests(unittest.TestCase):
    def test_rejects_invalid_listing_schema(self):
        invalid = [
            ({"version": True, "listings": {"tests": []}}, "unsupported Catch output version"),
            ({"version": 1, "listings": {"tests": {}}}, "must be an array"),
            ({"version": 1, "listings": {"tests": [{"name": 3, "tags": []}]}}, "no string name"),
            ({"version": 1, "listings": {"tests": [{"name": "bad tags", "tags": [3]}]}}, "invalid tags"),
        ]
        with tempfile.TemporaryDirectory() as temp:
            source = Path(temp) / "listing.json"
            for payload, message in invalid:
                with self.subTest(payload=payload):
                    source.write_text(json.dumps(payload), encoding="utf-8")
                    result = subprocess.run(
                        [sys.executable, str(PREPROCESSOR), str(source)],
                        capture_output=True, text=True,
                    )
                    self.assertEqual(result.returncode, 1)
                    self.assertIn(message, result.stderr)
                    self.assertEqual(result.stdout, "")

    @unittest.skipUnless(shutil.which("cmake") and shutil.which("ctest"), "CMake and CTest are required")
    def test_special_names_tags_and_paths_are_literal(self):
        with tempfile.TemporaryDirectory(prefix="catch output with spaces ") as temp:
            root = Path(temp)
            working = root / "working directory with spaces"
            working.mkdir()
            sentinel = root / "injected"
            malicious_name = f'literal ]==] terminator )\nfile(WRITE "{sentinel}" "executed")\n# ☃ slash\\ and comma,'
            # These are literal Catch selectors, distinct from registered names:
            # brackets, backslashes and commas must not become filter syntax.
            expected_selectors = {
                malicious_name: (
                    'literal \\]==\\] terminator )\n'
                    f'file(WRITE "{sentinel}" "executed")\n'
                    '# ☃ slash\\\\ and comma\\,'
                ),
                "plain name": "plain name",
                "slash\\comma, name": "slash\\\\comma\\, name",
                "ends ]] here": "ends \\]\\] here",
            }
            tags = ["unicode-☃", "semi;colon", "bracket [tag]", "slash\\comma,"]
            payload = {
                "version": 1,
                "listings": {"tests": [
                    {"name": malicious_name, "tags": tags},
                    {"name": "plain name", "tags": []},
                    {"name": "slash\\comma, name", "tags": ["slash\\comma,"]},
                    {"name": "ends ]] here", "tags": []},
                ]},
            }
            payload_path = root / "payload.json"
            payload_path.write_text(json.dumps(payload), encoding="utf-8")
            listing_exe = root / "fake Catch listing.py"
            listing_exe.write_text(
                "from pathlib import Path\n"
                "import sys\n"
                "sys.stdout.write(Path(__file__).with_name('payload.json').read_text())\n",
                encoding="utf-8",
            )

            ctest_file = root / "generated CTest with spaces.cmake"
            generated = subprocess.run([
                shutil.which("cmake"),
                f"-DTEST_EXECUTABLE={listing_exe}",
                f"-DTEST_EXECUTOR={sys.executable}",
                f"-DTEST_WORKING_DIR={working}",
                "-DTEST_SPEC=~[Parallel]",
                "-DTEST_LIST=discovered_tests",
                "-DADD_TAGS_AS_LABELS=TRUE",
                "-DTEST_OUTPUT_DIR=reports with spaces",
                f"-DCTEST_FILE={ctest_file}",
                "-P", str(SCRIPT),
            ], capture_output=True, text=True)
            self.assertEqual(generated.returncode, 0, generated.stdout + generated.stderr)
            self.assertEqual(ctest_file.read_text(encoding="utf-8").count("add_test("),
                             len(payload["listings"]["tests"]))

            build = root / "consumer build"
            build.mkdir()
            # Production CTest files include the generated script directly.
            # Configuring another CMake project would regenerate its commands
            # through an extra escaping layer, with version-dependent results.
            (build / "CTestTestfile.cmake").write_text(
                f"include([=[{ctest_file}]=])\n",
                encoding="utf-8",
            )
            shown = subprocess.run(
                [shutil.which("ctest"), "--test-dir", str(build), "--show-only=json-v1"],
                capture_output=True, text=True,
            )
            self.assertEqual(shown.returncode, 0, shown.stdout + shown.stderr)
            self.assertFalse(sentinel.exists(), "test-name contents were executed as CMake code")
            tests = json.loads(shown.stdout)["tests"]
            by_name = {test["name"]: test for test in tests}
            expected_names = [item["name"] for item in payload["listings"]["tests"]]
            self.assertEqual(len(tests), len(expected_names))
            self.assertEqual(set(by_name), set(expected_names))

            for item in payload["listings"]["tests"]:
                test = by_name[item["name"]]
                command = test["command"]
                self.assertEqual(command[:2], [sys.executable, str(listing_exe)])
                self.assertEqual(command[2], expected_selectors[item["name"]])
                self.assertTrue(command[3].startswith("--out "))
                output_path = Path(command[3][len("--out "):])
                self.assertEqual(output_path.parent, working / "reports with spaces")
                self.assertRegex(output_path.name, r"^[A-Za-z0-9_]+$")
                properties = {prop["name"]: prop["value"] for prop in test["properties"]}
                self.assertEqual(properties["WORKING_DIRECTORY"], str(working))
                actual_labels = properties.get("LABELS", [])
                self.assertEqual(len(actual_labels), len(item["tags"]))
                self.assertEqual(set(actual_labels), set(item["tags"]))


if __name__ == "__main__":
    unittest.main(verbosity=2)
