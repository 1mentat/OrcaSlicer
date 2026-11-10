import subprocess
import tempfile
import unittest
from pathlib import Path

from scripts.validation_artifact_lint import lint_tracked_build_outputs, lint_upload_retention


class ValidationArtifactLintTests(unittest.TestCase):
    def test_tracked_build_output_is_rejected_but_geometry_and_vendored_library_are_kept(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            (root / "build/obj").mkdir(parents=True)
            (root / "tests/data").mkdir(parents=True)
            (root / "deps/GMP/gmp/lib/win-x64").mkdir(parents=True)
            (root / "build/evidence").mkdir(parents=True)
            (root / "validation-artifacts-v2").mkdir(parents=True)
            (root / "OrcaSlicer.dSYM/Contents").mkdir(parents=True)
            (root / "build/obj/slicer.o").write_bytes(b"\xcf\xfa\xed\xfecompiled")
            (root / "tests/data/cube.obj").write_text("v 0 0 0\n", encoding="utf-8")
            (root / "tests/data/compiled.obj").write_bytes(b"\x64\x86\x01\x00coff object")
            (root / "profiles.profraw").write_bytes(b"llvm profile data")
            (root / "build/evidence/report.json").write_text("{}\n", encoding="utf-8")
            (root / "validation-artifacts-v2/report.json").write_text("{}\n", encoding="utf-8")
            (root / "OrcaSlicer.dSYM/Contents/Info.plist").write_text("plist", encoding="utf-8")
            (root / "tests/data/unknown.obj").write_bytes(b"opaque object payload")
            (root / "deps/GMP/gmp/lib/win-x64/libgmp-10.lib").write_bytes(b"!<arch>\nfixture")
            subprocess.run(["git", "init", "-q", str(root)], check=True)
            subprocess.run(["git", "-C", str(root), "add", "build", "tests", "deps", "profiles.profraw", "validation-artifacts-v2", "OrcaSlicer.dSYM"], check=True)

            findings = lint_tracked_build_outputs(root)

            self.assertEqual(len(findings), 7)
            self.assertTrue(any("build/obj/slicer.o" in item for item in findings))
            self.assertTrue(any("tests/data/compiled.obj" in item for item in findings))
            self.assertTrue(any("profiles.profraw" in item for item in findings))
            self.assertTrue(any("build/evidence/report.json" in item for item in findings))
            self.assertTrue(any("validation-artifacts-v2/report.json" in item for item in findings))
            self.assertTrue(any("OrcaSlicer.dSYM" in item for item in findings))
            self.assertTrue(any("unknown.obj" in item for item in findings))

    def test_upload_retention_requires_positive_integer_for_github_and_forgejo(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            workflow = root / "ci.yml"
            workflow.write_text(
                """jobs:
  test:
    steps:
      - uses: actions/upload-artifact@v7
        name: GitHub upload
        with:
          name: test
          retention-days: 5
      - uses: actions/download-artifact@v7
        with:
          name: this cannot satisfy an upload
      - uses: https://code.forgejo.org/actions/upload-artifact@v4
        name: Forgejo upload
        with:
          name: logs
          retention-days: 0
      - name: Missing retention
        uses: actions/upload-artifact@v7
        with:
          name: other
      - name: Expression retention
        uses: actions/upload-artifact@v7
        with:
          name: expression
          retention-days: ${{ inputs.days }}
      - name: Unrelated run text
        run: echo retention-days: 20
      - name: Block scalar cannot set retention
        uses: actions/upload-artifact@v7
        with:
          name: scalar
          path: |
            retention-days: 14
  second_job:
    steps:
      - uses: actions/upload-artifact@v7
        name: Second job missing retention
        with:
          name: separate
  inline_steps_job:
    steps: [{uses: actions/upload-artifact@v7}]
  aliased_steps_job:
    steps: *shared_steps
""",
                encoding="utf-8",
            )

            findings = lint_upload_retention([workflow])

            self.assertEqual(len(findings), 7)
            self.assertIn("Forgejo upload", findings[0])
            self.assertIn("Missing retention", findings[1])
            self.assertIn("Expression retention", findings[2])
            self.assertIn("Block scalar cannot set retention", findings[3])
            self.assertIn("Second job missing retention", findings[4])
            self.assertIn("unsupported inline or aliased steps syntax", findings[5])
            self.assertIn("unsupported inline or aliased steps syntax", findings[6])


if __name__ == "__main__":
    unittest.main()
