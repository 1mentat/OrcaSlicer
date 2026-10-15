#!/usr/bin/env python3
"""Tests for source-owned settings variance accounting."""

from pathlib import Path
import json
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import settings_inventory  # noqa: E402


ROOT = Path(__file__).resolve().parents[2]


class SettingsInventoryTests(unittest.TestCase):
    def test_comment_stripper_preserves_literals_and_line_numbers(self):
        source = 'const char *url = "https://example.test/*keep*/"; // remove\n/* remove\n   two lines */\nthis->add("live", coBool);\n'
        stripped = settings_inventory._strip_cpp_comments(source)
        self.assertIn('"https://example.test/*keep*/"', stripped)
        self.assertNotIn("remove", stripped)
        self.assertEqual(stripped.count("\n"), source.count("\n"))
        self.assertEqual(stripped.splitlines()[3], 'this->add("live", coBool);')

    def test_runtime_audit_exposes_omitted_and_spurious_settings(self):
        with tempfile.TemporaryDirectory() as directory:
            registry = Path(directory) / "registry.json"
            registry.write_text(json.dumps({
                "schema_version": 1, "source": "PrintConfigDef::options",
                "settings_count": 2, "options": [{"key": "shared"}, {"key": "generated"}],
            }))
            audit = settings_inventory.audit_runtime_registry(
                {"options": [{"key": "shared"}, {"key": "comment_only"}]}, registry)
            self.assertFalse(audit["matches"])
            self.assertEqual(audit["missing_from_source_inventory"], ["generated"])
            self.assertEqual(audit["absent_from_runtime_registry"], ["comment_only"])
            self.assertTrue(settings_inventory.audit_runtime_registry(
                {"options": [{"key": "shared"}, {"key": "generated"}]}, registry)["matches"])

    def test_source_parser_includes_generated_and_inherited_options(self):
        inventory = settings_inventory.parse_print_config(ROOT / "src/libslic3r/PrintConfig.cpp")
        self.assertGreater(inventory["option_count"], 800)
        self.assertGreater(inventory["registration_count"], inventory["option_count"])
        self.assertEqual(
            {item["key"] for item in inventory["duplicates"]},
            {"preset_names", "retract_lift_above", "retract_lift_below"},
        )
        options = {item["key"]: item for item in inventory["options"]}
        self.assertNotIn("adaptive_layer_height", options)
        self.assertNotIn("spaghetti_detector", options)
        self.assertEqual(options["bottom_surface_pattern"]["enum_value_count"], 9)
        self.assertEqual(options["bottom_surface_pattern"]["enum_values"],
                         options["top_surface_pattern"]["enum_values"])
        self.assertEqual(options["filament_retraction_length"]["type"], "coFloats")
        self.assertEqual(options["gcode_remap_x"]["enum_value_count"], 9)
        for family in ("speed", "acceleration", "jerk"):
            for axis in "xyze":
                option = options[f"machine_max_{family}_{axis}"]
                self.assertEqual(option["type"], "coFloats")
                self.assertEqual(option["origin"], "literal_axis_loop")

    def test_report_marks_only_declared_dimensions_as_evidence(self):
        report = settings_inventory.build_report(
            root=ROOT,
            source_path=ROOT / "src/libslic3r/PrintConfig.cpp",
            evidence_path=ROOT / "tests/settings_coverage.json",
        )
        self.assertEqual(report["option_count"], len(report["options"]))
        self.assertEqual(sum(report["counts"].values()), report["option_count"])
        options = {item["key"]: item for item in report["options"]}
        self.assertEqual(options["sparse_infill_pattern"]["status"], "declared_covered")
        self.assertEqual(options["layer_height"]["status"], "declared_partial")
        self.assertEqual(options["gcode_comments"]["status"], "declared_covered")
        self.assertEqual(options["printer_technology"]["status"], "declared_partial")
        self.assertIn("Missing required dimensions: enum_values.", options["printer_technology"]["reason"])
        self.assertEqual(options["supports_enable"]["status"], "declared_partial")
        self.assertIn("boolean_values", options["supports_enable"]["reason"])
        uncovered = [option for option in options.values() if option["status"] == "uncovered"]
        self.assertTrue(uncovered)
        self.assertTrue(all("do not establish behavior" in option["reason"] for option in uncovered))
        self.assertFalse(report["execution_verified"])

    def test_schema_roundtrip_is_an_explicit_non_pipeline_dimension(self):
        self.assertIn("schema_roundtrip", settings_inventory._DIMENSIONS)
        with tempfile.TemporaryDirectory() as directory:
            evidence = Path(directory) / "evidence.json"
            evidence.write_text(json.dumps({
                "schema_version": 1,
                "entries": {
                    "printer_technology": {
                        "test": "tests/libslic3r/test_config.cpp",
                        "tag": "[Config]",
                        "dimensions": ["schema_roundtrip"],
                        "reason": "The registry-wide config contract round-trips its serialized default.",
                    },
                },
            }))
            report = settings_inventory.build_report(
                root=ROOT,
                source_path=ROOT / "src/libslic3r/PrintConfig.cpp",
                evidence_path=evidence,
            )
        option = next(item for item in report["options"] if item["key"] == "printer_technology")
        self.assertEqual(option["evidence_dimensions"], ["schema_roundtrip"])
        self.assertEqual(option["status"], "declared_partial")
        self.assertIn("pipeline_oracle", option["reason"])


if __name__ == "__main__":
    unittest.main()
