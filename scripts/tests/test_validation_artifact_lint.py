import json
import subprocess
import tempfile
import unittest
from pathlib import Path

import yaml

from scripts.validation_artifact_lint import (
    lint_producer_registry,
    lint_tracked_build_outputs,
    lint_upload_retention,
)


class ValidationArtifactLintTests(unittest.TestCase):
    @staticmethod
    def _registry_repo(root: Path) -> tuple[list[Path], Path, Path]:
        scripts = root / "scripts"
        action_dir = root / ".github/actions/validation-upload"
        workflows_dir = root / ".github/workflows"
        scripts.mkdir(parents=True)
        action_dir.mkdir(parents=True)
        workflows_dir.mkdir(parents=True)
        rows = []
        producer_specs = {
            "settings-inventory": ("scripts/settings_inventory.py", "--out"),
            "mutation-validation": ("scripts/slicer_mutation.py", "--out"),
            "pr-validation": ("scripts/slicer_pr_validation.py", "--output-dir"),
            "module-feedback": ("scripts/slicer_module_feedback.py", "--out"),
        }
        for producer_id, (relative, output_option) in producer_specs.items():
            source = root / relative
            source.write_text(
                "def main(argv=None):\n    return 0\n\n"
                "if __name__ == '__main__':\n"
                "    from validation_runner import managed_main\n"
                f"    raise SystemExit(managed_main(main, output_option={output_option!r}, default_name='result'))\n",
                encoding="utf-8",
            )
            rows.append({
                "id": producer_id, "path": relative, "kind": "cli",
                "lifecycle": "validation-runner", "output_options": [output_option],
                "payload_env": "VALIDATION_ARTIFACT_DIR",
            })
        rows.append({
            "id": "public-validation-upload", "path": ".github/actions/validation-upload/action.yml",
            "kind": "github-upload", "lifecycle": "ephemeral-runner",
            "retention_days": 14, "max_bytes": 4294967296,
        })
        registry = root / ".github/validation-producers.json"
        registry.write_text(json.dumps({"schema_version": 1, "producers": rows}), encoding="utf-8")

        input_names = {
            "name": {"required": True}, "path": {"required": True},
            "retention-days": {"default": "14"}, "max-bytes": {"default": "4294967296"},
            "if-no-files-found": {"default": "warn"}, "overwrite": {"default": "false"},
            "compression-level": {"default": "6"}, "include-hidden-files": {"default": "false"},
        }
        env = {
            "VALIDATION_UPLOAD_NAME": "${{ inputs.name }}",
            "VALIDATION_UPLOAD_PATH": "${{ inputs.path }}",
            "VALIDATION_UPLOAD_RETENTION_DAYS": "${{ inputs.retention-days }}",
            "VALIDATION_UPLOAD_MAX_BYTES": "${{ inputs.max-bytes }}",
            "VALIDATION_UPLOAD_IF_NO_FILES_FOUND": "${{ inputs.if-no-files-found }}",
            "VALIDATION_UPLOAD_OVERWRITE": "${{ inputs.overwrite }}",
            "VALIDATION_UPLOAD_COMPRESSION_LEVEL": "${{ inputs.compression-level }}",
            "VALIDATION_UPLOAD_INCLUDE_HIDDEN_FILES": "${{ inputs.include-hidden-files }}",
        }
        upload_with = {
            "name": "${{ inputs.name }}", "path": "${{ inputs.path }}",
            "retention-days": "${{ inputs.retention-days }}",
            "if-no-files-found": "${{ inputs.if-no-files-found }}",
            "overwrite": "${{ inputs.overwrite }}",
            "compression-level": "${{ inputs.compression-level }}",
            "include-hidden-files": "${{ inputs.include-hidden-files }}",
        }
        action = {
            "inputs": input_names,
            "runs": {"using": "composite", "steps": [
                {"name": "Preflight", "shell": "bash", "env": env,
                 "run": 'python "$GITHUB_ACTION_PATH/../../../scripts/validation_upload.py"'},
                {"name": "Upload", "uses": "actions/upload-artifact@cf430e030ddbb5b0abf93d22962f4752f3646cd9",
                 "with": upload_with},
            ]},
        }
        action_path = action_dir / "action.yml"
        action_path.write_text(yaml.safe_dump(action, sort_keys=False), encoding="utf-8")
        (scripts / "validation_upload.py").write_text(
            "MAX_BYTES = 4 * 1024**3\ndef preflight(env):\n    return {}\n", encoding="utf-8")
        workflow_path = workflows_dir / "call.yml"
        workflow = {"jobs": {"caller": {"steps": [
            {"uses": "./.github/actions/validation-upload", "with": {
                "name": "result", "path": "result/",
                "retention-days": "${{ inputs.retention-days }}",
                "max-bytes": "${{ inputs.max-bytes }}",
            }},
            {"uses": "./.github/actions/validation-upload", "with": {"name": "default", "path": "result/"}},
        ]}}}
        workflow_path.write_text(yaml.safe_dump(workflow, sort_keys=False), encoding="utf-8")
        return [workflow_path], registry, action_path

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
        run: 'echo retention-days: 20'
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
    steps: &shared_steps [{uses: actions/upload-artifact@v7}]
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
            self.assertIn("inline_steps_job", findings[5])
            self.assertIn("aliased_steps_job", findings[6])

    def test_valid_yaml_layouts_and_script_text_are_preserved(self):
        with tempfile.TemporaryDirectory() as temporary:
            workflow = Path(temporary) / "ci.yaml"
            workflow.write_text("""jobs:
  normal:
    steps: &uploads # Comment on a mapping key is ordinary YAML.
    - uses: actions/upload-artifact@v7
      with: {name: proof, retention-days: '14'}
    - run: |
        echo 'steps: [{uses: actions/upload-artifact@v7}]'
  reused:
    steps: *uploads
  flow:
    steps: [{uses: 'https://code.forgejo.org/actions/upload-artifact@v4', with: {retention-days: 3}}]
""", encoding="utf-8")
            self.assertEqual(lint_upload_retention([workflow]), [])

    def test_malformed_yaml_and_custom_objects_fail_closed(self):
        with tempfile.TemporaryDirectory() as temporary:
            workflow = Path(temporary) / "ci.yml"
            for text in ("jobs: [", "!!python/name:os.system ''"):
                with self.subTest(text=text):
                    workflow.write_text(text, encoding="utf-8")
                    findings = lint_upload_retention([workflow])
                    self.assertEqual(len(findings), 1)
                    self.assertIn("cannot safely parse workflow", findings[0])

    def test_workflows_and_allowlisted_inputs_cannot_follow_outside_symlinks(self):
        with tempfile.TemporaryDirectory() as temporary:
            base = Path(temporary)
            root = base / "repo"
            root.mkdir()
            outside = base / "outside.yml"
            outside.write_text("jobs: {}\n", encoding="utf-8")
            workflow = root / "ci.yml"
            library = root / "deps/GMP/gmp/lib/win-x64/libgmp-10.lib"
            library.parent.mkdir(parents=True)
            try:
                workflow.symlink_to(outside)
                library.symlink_to(outside)
            except OSError as error:
                self.skipTest(f"symlink creation unavailable: {error}")
            self.assertEqual(len(lint_upload_retention([workflow], root)), 1)
            self.assertEqual(len(lint_tracked_build_outputs(root, [library.relative_to(root).as_posix()])), 1)

    def test_registry_requires_all_real_executable_producer_guards(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            workflows, registry, _ = self._registry_repo(root)
            self.assertEqual(lint_producer_registry(root, workflows), [])

            document = json.loads(registry.read_text(encoding="utf-8"))
            document["producers"] = [row for row in document["producers"] if row["id"] != "pr-validation"]
            registry.write_text(json.dumps(document), encoding="utf-8")
            findings = lint_producer_registry(root, workflows)
            self.assertTrue(any("required producer 'pr-validation' is missing" in item for item in findings))

            document["producers"].append({
                "id": "pr-validation", "path": "scripts/slicer_pr_validation.py",
                "kind": "cli", "lifecycle": "validation-runner",
                "output_options": ["--output-dir"], "payload_env": "VALIDATION_ARTIFACT_DIR",
            })
            registry.write_text(json.dumps(document), encoding="utf-8")
            source = root / "scripts/slicer_pr_validation.py"
            source.write_text('"""managed_main output_option=--output-dir"""\ndef main():\n    return 0\n', encoding="utf-8")
            findings = lint_producer_registry(root, workflows)
            self.assertTrue(any("must import managed_main" in item for item in findings))

            source.write_text(
                "def main(argv=None):\n    return 0\ndef fake():\n    return 0\n\n"
                "if __name__ == '__main__':\n"
                "    from validation_runner import managed_main\n"
                "    raise SystemExit(managed_main(fake, output_option='--out', default_name='result'))\n",
                encoding="utf-8",
            )
            findings = lint_producer_registry(root, workflows)
            self.assertTrue(any("must pass its executable main function" in item for item in findings))

            source.write_text(
                "def main(argv=None):\n    return 0\n\n"
                "if __name__ == '__main__':\n"
                "    from validation_runner import managed_main\n"
                "    raise SystemExit(managed_main(main, output_option='--out', default_name='result'))\n",
                encoding="utf-8",
            )
            findings = lint_producer_registry(root, workflows)
            self.assertTrue(any("must bind output_option='--output-dir'" in item for item in findings))

    def test_registry_rejects_traversal_duplicate_and_malformed_entries(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            workflows, registry, _ = self._registry_repo(root)
            document = json.loads(registry.read_text(encoding="utf-8"))

            document["producers"][0]["path"] = "../outside.py"
            registry.write_text(json.dumps(document), encoding="utf-8")
            findings = lint_producer_registry(root, workflows)
            self.assertTrue(any("path must be confined" in item for item in findings))

            document = json.loads(json.dumps(document))
            document["producers"][0]["path"] = "scripts/settings_inventory.py"
            document["producers"].append(document["producers"][0])
            registry.write_text(json.dumps(document), encoding="utf-8")
            findings = lint_producer_registry(root, workflows)
            self.assertTrue(any("duplicate producer id 'settings-inventory'" in item for item in findings))

            registry.write_text('{"schema_version": 1, "schema_version": 1, "producers": []}', encoding="utf-8")
            findings = lint_producer_registry(root, workflows)
            self.assertTrue(any("cannot safely parse registry" in item for item in findings))

    def test_registry_is_required_after_lifecycle_integration_but_not_before(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            workflows_dir = root / ".github/workflows"
            workflows_dir.mkdir(parents=True)
            workflow = workflows_dir / "baseline.yml"
            workflow.write_text("jobs: {}\n", encoding="utf-8")
            self.assertEqual(lint_producer_registry(root, [workflow]), [])

            (root / "scripts").mkdir()
            (root / "scripts/validation_runner.py").write_text("# lifecycle runtime\n", encoding="utf-8")
            findings = lint_producer_registry(root, [workflow])
            self.assertTrue(any("required when validation runner or upload wrapper is present" in item
                                for item in findings))

    def test_upload_wrapper_preflight_is_required_unskippable_and_pinned(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            workflows, _, action_path = self._registry_repo(root)
            self.assertEqual(lint_producer_registry(root, workflows), [])

            action = yaml.safe_load(action_path.read_text(encoding="utf-8"))
            action["runs"]["steps"].pop(0)
            action_path.write_text(yaml.safe_dump(action, sort_keys=False), encoding="utf-8")
            findings = lint_producer_registry(root, workflows)
            self.assertTrue(any("requires exactly one validation_upload.py preflight" in item for item in findings))

            action = yaml.safe_load(action_path.read_text(encoding="utf-8"))
            action["runs"]["steps"].insert(0, {
                "name": "Preflight", "shell": "bash", "if": "false", "continue-on-error": True,
                "env": {}, "run": 'python "$GITHUB_ACTION_PATH/../../../scripts/validation_upload.py"',
            })
            action_path.write_text(yaml.safe_dump(action, sort_keys=False), encoding="utf-8")
            findings = lint_producer_registry(root, workflows)
            self.assertTrue(any("cannot be conditionally skipped" in item for item in findings))
            self.assertTrue(any("cannot continue after an error" in item for item in findings))

            action = yaml.safe_load(action_path.read_text(encoding="utf-8"))
            for condition in ("always()", "!success()", "${{ !success() }}"):
                action["runs"]["steps"][1]["if"] = condition
                action_path.write_text(yaml.safe_dump(action, sort_keys=False), encoding="utf-8")
                findings = lint_producer_registry(root, workflows)
                self.assertTrue(any("upload step cannot run after a failed preflight" in item for item in findings))

            action = yaml.safe_load(action_path.read_text(encoding="utf-8"))
            action["runs"]["steps"][1].pop("if")
            action["runs"]["steps"][1]["uses"] = "actions/upload-artifact@v7"
            action_path.write_text(yaml.safe_dump(action, sort_keys=False), encoding="utf-8")
            findings = lint_producer_registry(root, workflows)
            self.assertTrue(any("pinned to a full commit SHA" in item for item in findings))

    def test_wrapper_defaults_and_nested_caller_values_are_checked(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            workflows, _, action_path = self._registry_repo(root)
            self.assertEqual(lint_producer_registry(root, workflows), [])

            action = yaml.safe_load(action_path.read_text(encoding="utf-8"))
            action["inputs"]["max-bytes"]["default"] = "8589934592"
            action_path.write_text(yaml.safe_dump(action, sort_keys=False), encoding="utf-8")
            findings = lint_producer_registry(root, workflows)
            self.assertTrue(any("input 'max-bytes' default must match registry value" in item for item in findings))

            action = yaml.safe_load(action_path.read_text(encoding="utf-8"))
            action["inputs"]["max-bytes"]["default"] = "4294967296"
            action_path.write_text(yaml.safe_dump(action, sort_keys=False), encoding="utf-8")
            workflow = yaml.safe_load(workflows[0].read_text(encoding="utf-8"))
            caller = workflow["jobs"]["caller"]["steps"][0]["with"]
            caller["retention-days"] = 31
            caller["max-bytes"] = 4294967297
            workflows[0].write_text(yaml.safe_dump(workflow, sort_keys=False), encoding="utf-8")
            findings = lint_producer_registry(root, workflows)
            self.assertTrue(any("retention-days must be 1..30" in item for item in findings))
            self.assertTrue(any("max-bytes must be 1..4294967296" in item for item in findings))

            caller["retention-days"] = "${{ inputs.max-bytes }}"
            caller["max-bytes"] = "${{ inputs.retention-days }}"
            workflows[0].write_text(yaml.safe_dump(workflow, sort_keys=False), encoding="utf-8")
            findings = lint_producer_registry(root, workflows)
            self.assertTrue(any("retention-days must be 1..30" in item for item in findings))
            self.assertTrue(any("max-bytes must be 1..4294967296" in item for item in findings))


if __name__ == "__main__":
    unittest.main()
