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
        with tempfile.TemporaryDirectory() as directory:
            evidence = Path(directory) / "evidence.json"
            evidence.write_text(json.dumps({
                "schema_version": 1,
                "default_uncovered_reason": (
                    "No dedicated settings-variance evidence is declared for this registered option; "
                    "full_print_config() construction, schema serialization, and source coverage do not establish behavior."
                ),
                "entries": {
                    "sparse_infill_pattern": {
                        "test": "tests/fff_print/test_settings_variance.cpp",
                        "tag": "[SettingsVariance]",
                        "dimensions": ["enum_values", "pipeline_oracle"],
                        "reason": "Synthetic fixture declares enum and pipeline evidence.",
                    },
                    "layer_height": {
                        "test": "tests/fff_print/test_layer_height_variance.cpp",
                        "tag": "[LayerHeightVariance]",
                        "dimensions": ["representative_values", "pipeline_oracle"],
                        "reason": "Synthetic fixture declares representative and pipeline evidence.",
                    },
                    "gcode_comments": {
                        "test": "tests/fff_print/test_gcode_settings_contracts.cpp",
                        "tag": "[GCodeSettings]",
                        "dimensions": ["boolean_values", "pipeline_oracle"],
                        "reason": "Synthetic fixture declares boolean and pipeline evidence.",
                    },
                    "printer_technology": {
                        "test": "tests/sla_print/test_sla_pipeline_contracts.cpp",
                        "tag": "[SLAPrint][SLAPipeline]",
                        "dimensions": ["representative_values", "pipeline_oracle"],
                        "reason": "Synthetic fixture declares representative and pipeline evidence.",
                    },
                    "supports_enable": {
                        "test": "tests/sla_print/test_sla_pipeline_contracts.cpp",
                        "tag": "[SLAPrint][SLAFeatures][SLASupportsEnableVariance]",
                        "dimensions": ["boolean_values", "pipeline_oracle"],
                        "reason": "Synthetic fixture declares boolean and pipeline evidence.",
                    },
                },
            }), encoding="utf-8")
            report = settings_inventory.build_report(
                root=ROOT,
                source_path=ROOT / "src/libslic3r/PrintConfig.cpp",
                evidence_path=evidence,
            )
        self.assertEqual(report["option_count"], len(report["options"]))
        self.assertEqual(sum(report["counts"].values()), report["option_count"])
        options = {item["key"]: item for item in report["options"]}
        self.assertEqual(options["sparse_infill_pattern"]["status"], "declared_covered")
        self.assertEqual(options["layer_height"]["status"], "declared_partial")
        self.assertEqual(options["gcode_comments"]["status"], "declared_covered")
        self.assertEqual(options["printer_technology"]["status"], "declared_partial")
        self.assertIn("Missing required dimensions: enum_values.", options["printer_technology"]["reason"])
        self.assertEqual(options["supports_enable"]["status"], "declared_covered")
        self.assertEqual(options["supports_enable"]["required_dimensions"],
                         ["boolean_values", "pipeline_oracle"])
        self.assertIn("boolean_values", options["supports_enable"]["evidence_dimensions"])
        self.assertIn("pipeline_oracle", options["supports_enable"]["evidence_dimensions"])
        uncovered = [option for option in options.values() if option["status"] == "uncovered"]
        self.assertTrue(uncovered)
        self.assertTrue(all("do not establish behavior" in option["reason"] for option in uncovered))
        self.assertFalse(report["execution_verified"])

    def test_boolean_settings_require_values_beyond_pipeline_evidence(self):
        with tempfile.TemporaryDirectory() as directory:
            evidence = Path(directory) / "evidence.json"
            evidence.write_text(json.dumps({"schema_version": 1, "entries": {
                "supports_enable": {
                    "test": "tests/sla_print/test_sla_pipeline_contracts.cpp",
                    "tag": "[SLAPrint][SLAFeatures][SLASupportsEnableVariance]",
                    "dimensions": ["pipeline_oracle"],
                    "reason": "Synthetic pipeline-only evidence.",
                },
            }}), encoding="utf-8")
            partial = settings_inventory.build_report(
                root=ROOT,
                source_path=ROOT / "src/libslic3r/PrintConfig.cpp",
                evidence_path=evidence,
            )
        supports_enable = next(item for item in partial["options"]
                               if item["key"] == "supports_enable")
        self.assertEqual(supports_enable["status"], "declared_partial")
        self.assertIn("Missing required dimensions: boolean_values.",
                      supports_enable["reason"])

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

    def test_nullable_settings_require_behavior_evidence_beyond_schema_and_values(self):
        cases = {
            "enable_overhang_speed": ["boolean_values", "pipeline_oracle", "schema_roundtrip"],
            "filament_retract_when_changing_layer": [
                "boolean_values", "pipeline_oracle", "nullable_behavior"],
            "filament_retraction_length": [
                "representative_values", "pipeline_oracle", "nullable_behavior"],
        }
        with tempfile.TemporaryDirectory() as directory:
            evidence = Path(directory) / "evidence.json"
            evidence.write_text(json.dumps({"entries": {
                key: {"test": "tests/fff_print/test_wipe.cpp", "tag": "[Wipe]",
                      "dimensions": dimensions, "reason": "Synthetic dimension claim."}
                for key, dimensions in cases.items()
            }}), encoding="utf-8")
            report = settings_inventory.build_report(
                root=ROOT, source_path=ROOT / "src/libslic3r/PrintConfig.cpp",
                evidence_path=evidence)
        options = {item["key"]: item for item in report["options"]}
        missing_nil = options["enable_overhang_speed"]
        self.assertEqual(missing_nil["status"], "declared_partial")
        self.assertIn("Missing required dimensions: nullable_behavior.", missing_nil["reason"])
        self.assertEqual(options["filament_retract_when_changing_layer"]["status"], "declared_covered")
        numeric = options["filament_retraction_length"]
        self.assertEqual(numeric["status"], "declared_partial")
        self.assertEqual(numeric["required_dimensions"], [
            "boundary_values", "pipeline_oracle", "nullable_behavior"])
        self.assertIn("Missing required dimensions: boundary_values.", numeric["reason"])
        self.assertEqual(report["counts"]["declared_covered"], 1)
        self.assertEqual(report["counts"]["declared_partial"], 2)
        self.assertEqual(sum(report["counts"].values()), report["option_count"])


class SettingsMetadataTests(unittest.TestCase):
    def test_source_parser_does_not_cross_registration_lambda(self):
        source = """
void PrintConfigDef::init_common_params() {
    this->add("tilt", coFloat);
    auto add_generated = [this](const char *key) {
        auto def = this->add(key, coEnum);
        def->enum_values = {"generated"};
    };
    auto top = def = this->add("top", coInt);
    def->enum_values.push_back("0");
    def = this->add("bottom", coInt);
    def->enum_values.push_back("-1");
    append(def->enum_values, top->enum_values);
    def->nullable = true;
    this->add("localized", coEnums);
    def->enum_values.push_back(L("Standard"));
}
void PrintConfigDef::handle_legacy(
"""
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "PrintConfig.cpp"
            path.write_text(source, encoding="utf-8")
            options = {
                item["key"]: item
                for item in settings_inventory.parse_print_config(path)["options"]
            }
        self.assertEqual(options["tilt"]["enum_values"], [])
        self.assertEqual(options["localized"]["enum_values"], ["Standard"])
        self.assertEqual(options["bottom"]["enum_values"], ["-1", "0"])
        self.assertTrue(options["bottom"]["nullable"])

    def test_range_parser_tracks_expression_bounds_and_stops_before_generated_overrides(self):
        source = """
const std::vector<std::string> filament_extruder_override_keys = {
    "filament_bounded", "filament_unbounded"
};
void PrintConfigDef::init_common_params() {
    auto def = this->add("bounded", coFloats);
    def->min = 0;
    def->set_default_value(new ConfigOptionFloats{1});
    def = this->add("expression_bounded", coFloat);
    def->max = configured_limit;
    def->set_default_value(new ConfigOptionFloat(1));
    def = this->add("unbounded", coFloats);
    def->set_default_value(new ConfigOptionFloats{1});
    for (auto& opt_key : filament_extruder_override_keys) {
        def = this->add_nullable(opt_key, coFloats);
        def->min = it_opt->second.min;
        def->max = it_opt->second.max;
    }
    def = this->add("nullable_unbounded", coFloats);
    def->nullable = true;
    def->set_default_value(new ConfigOptionFloatsNullable{1});
}
void PrintConfigDef::handle_legacy(
"""
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source_path = root / "PrintConfig.cpp"
            source_path.write_text(source, encoding="utf-8")
            options = {
                item["key"]: item
                for item in settings_inventory.parse_print_config(source_path)["options"]
            }
        self.assertEqual(options["bounded"]["range_declared"], ["min"])
        self.assertEqual(options["expression_bounded"]["range_declared"], ["max"])
        self.assertEqual(options["unbounded"]["range_declared"], [])
        self.assertEqual(options["filament_bounded"]["range_declared"], ["min"])
        self.assertTrue(options["filament_bounded"]["nullable"])
        self.assertEqual(options["filament_unbounded"]["range_declared"], [])
        self.assertTrue(options["filament_unbounded"]["nullable"])
        self.assertEqual(options["nullable_unbounded"]["range_declared"], [])

    def test_unbounded_scalar_numeric_evidence_uses_representatives(self):
        for option_type in ("coInt", "coFloat", "coPercent", "coFloatOrPercent"):
            with self.subTest(option_type=option_type):
                self.assertEqual(settings_inventory._required_dimensions(
                    option_type, range_declared=[]),
                    ["representative_values", "pipeline_oracle"])
                self.assertEqual(settings_inventory._required_dimensions(
                    option_type, nullable=True, range_declared=[]),
                    ["representative_values", "pipeline_oracle", "nullable_behavior"])
                # Absent range metadata is unknown, not evidence of no bounds.
                self.assertEqual(settings_inventory._required_dimensions(
                    option_type, range_declared=None), ["boundary_values", "pipeline_oracle"])

    def test_numeric_vectors_keep_boundaries_required_without_registered_bounds(self):
        for option_type in ("coInts", "coFloats", "coPercents", "coFloatsOrPercents"):
            with self.subTest(option_type=option_type):
                self.assertEqual(settings_inventory._required_dimensions(
                    option_type, range_declared=[]), ["boundary_values", "pipeline_oracle"])
                self.assertEqual(settings_inventory._required_dimensions(
                    option_type, nullable=True, range_declared=[]),
                    ["boundary_values", "pipeline_oracle", "nullable_behavior"])
                self.assertEqual(settings_inventory._required_dimensions(
                    option_type, range_declared=None), ["boundary_values", "pipeline_oracle"])

    def test_range_aware_report_keeps_bounds_nullable_and_pipeline_gaps_distinct(self):
        source = """
const std::vector<std::string> filament_extruder_override_keys = {
    "filament_bounded", "filament_unbounded"
};
void PrintConfigDef::init_common_params() {
    auto def = this->add("bounded", coFloats);
    def->min = 0;
    def->set_default_value(new ConfigOptionFloats{1});
    def = this->add("expression_bounded", coFloat);
    def->max = configured_limit;
    def->set_default_value(new ConfigOptionFloat(1));
    def = this->add("unbounded", coFloats);
    def->set_default_value(new ConfigOptionFloats{1});
    for (auto& opt_key : filament_extruder_override_keys) {
        def = this->add_nullable(opt_key, coFloats);
        def->min = it_opt->second.min;
        def->max = it_opt->second.max;
    }
    def = this->add("nullable_unbounded", coFloats);
    def->nullable = true;
    def->set_default_value(new ConfigOptionFloatsNullable{1});
}
void PrintConfigDef::handle_legacy(
"""
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "tests").mkdir()
            (root / "tests" / "fixture.cpp").write_text("[RangeAudit]", encoding="utf-8")
            source_path = root / "PrintConfig.cpp"
            source_path.write_text(source, encoding="utf-8")
            evidence_path = root / "evidence.json"
            evidence_path.write_text(json.dumps({"entries": {
                key: {"test": "tests/fixture.cpp", "tag": "[RangeAudit]",
                      "dimensions": dimensions, "reason": "Synthetic range audit evidence."}
                for key, dimensions in {
                    "bounded": ["representative_values", "pipeline_oracle"],
                    "expression_bounded": ["representative_values", "pipeline_oracle"],
                    "unbounded": ["representative_values", "pipeline_oracle"],
                    "filament_bounded": ["boundary_values", "representative_values",
                                         "pipeline_oracle", "nullable_behavior"],
                    "filament_unbounded": ["representative_values", "pipeline_oracle"],
                    "nullable_unbounded": ["representative_values", "pipeline_oracle"],
                }.items()
            }}), encoding="utf-8")
            report = settings_inventory.build_report(
                root=root, source_path=source_path, evidence_path=evidence_path)
        options = {item["key"]: item for item in report["options"]}
        self.assertEqual(options["bounded"]["status"], "declared_partial")
        self.assertIn("Missing required dimensions: boundary_values.",
                      options["bounded"]["reason"])
        self.assertEqual(options["expression_bounded"]["status"], "declared_partial")
        self.assertIn("Missing required dimensions: boundary_values.",
                      options["expression_bounded"]["reason"])
        self.assertEqual(options["unbounded"]["status"], "declared_partial")
        self.assertIn("Missing required dimensions: boundary_values.",
                      options["unbounded"]["reason"])
        self.assertEqual(options["filament_bounded"]["status"], "declared_covered")
        self.assertEqual(options["filament_unbounded"]["status"], "declared_partial")
        self.assertIn("Missing required dimensions: boundary_values, nullable_behavior.",
                      options["filament_unbounded"]["reason"])
        self.assertEqual(options["nullable_unbounded"]["status"], "declared_partial")
        self.assertIn("Missing required dimensions: boundary_values, nullable_behavior.",
                      options["nullable_unbounded"]["reason"])

    def test_real_source_range_metadata_does_not_inherit_or_miss_bounds(self):
        options = {
            item["key"]: item
            for item in settings_inventory.parse_print_config(
                ROOT / "src/libslic3r/PrintConfig.cpp")["options"]
        }
        self.assertEqual(options["absolute_correction"]["range_declared"], [])
        self.assertEqual(options["z_offset"]["range_declared"], [])
        self.assertEqual(options["standby_temperature_delta"]["range_declared"],
                         ["max", "min"])
        self.assertEqual(options["nozzle_temperature_initial_layer"]["range_declared"],
                         ["max", "min"])
        # The following direct numeric option precedes a generated nullable
        # override loop. That loop's inherited bounds must not leak backward.
        self.assertEqual(options["filament_dev_drying_cooling_temperature"]["range_declared"], [])
        self.assertEqual(options["filament_retraction_length"]["range_declared"],
                         options["retraction_length"]["range_declared"])

    def test_real_source_captures_literal_metadata_and_direct_nullable(self):
        source = ROOT / "src/libslic3r/PrintConfig.cpp"
        options = {
            item["key"]: item
            for item in settings_inventory.parse_print_config(source)["options"]
        }
        self.assertEqual(options["belt_frame_tilt_angle"]["enum_values"], [])
        self.assertEqual(
            options["nozzle_volume_type"]["enum_values"],
            ["Standard", "High Flow", "Hybrid", "TPU High Flow", "E3D High Flow", "Extra High Flow"],
        )
        self.assertEqual(
            options["support_interface_bottom_layers"]["enum_values"],
            ["-1", "0", "1", "2", "3"],
        )
        self.assertTrue(options["bridge_acceleration"]["nullable"])
        # MaterialType::all() is runtime-owned and must remain explicitly incomplete.
        self.assertEqual(options["filament_type"]["enum_values"], [])

    def test_key_only_registry_is_not_broadly_verified(self):
        report = {"options": [{"key": "shared", "enum_values": [], "nullable": False,
                                "default_declared": True}]}
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "registry.json"
            path.write_text(json.dumps({
                "schema_version": 1,
                "source": "PrintConfigDef::options",
                "settings_count": 1,
                "options": [{"key": "shared"}],
            }), encoding="utf-8")
            audit = settings_inventory.audit_runtime_registry(report, path)
        settings_inventory.apply_runtime_registry(report, audit)
        self.assertTrue(report["runtime_registry_keyset_verified"])
        self.assertFalse(report["runtime_registry_metadata_reconciled"])
        self.assertFalse(report["runtime_registry_verified"])
        self.assertFalse(report["metadata_complete"])

    def test_validated_registry_supplies_effective_metadata_and_records_differences(self):
        report = {"options": [{"key": "shared", "type": "coFloat", "enum_values": ["wrong"],
                                "nullable": False, "default_declared": True,
                                "evidence_dimensions": ["boundary_values", "pipeline_oracle"],
                                "evidence": {"test": "fixture.cpp", "tag": "[Fixture]"},
                                "evidence_reason": "Numeric boundary behavior is exercised."}]}
        settings_inventory._refresh_evidence_status(report)
        self.assertEqual(report["counts"], {"declared_covered": 1})
        registry = {
            "schema_version": 1,
            "source": "PrintConfigDef::options",
            "settings_count": 1,
            "options": [{
                "key": "shared", "type_id": 1, "nullable": True, "has_default": False,
                "enum_values": ["right"], "canonical_enum_values": ["right"],
            }],
        }
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "registry.json"
            type_header = Path(directory) / "Config.hpp"
            type_header.write_text("enum ConfigOptionType {\n coFloat = 1,\n coBool = 8\n};\n",
                                   encoding="utf-8")
            path.write_text(json.dumps(registry), encoding="utf-8")
            audit = settings_inventory.audit_runtime_registry(report, path,
                                                                  type_header=type_header)
        settings_inventory.apply_runtime_registry(report, audit)
        self.assertTrue(report["runtime_registry_verified"])
        option = report["options"][0]
        self.assertEqual(option["enum_values"], ["wrong"])
        self.assertEqual(option["effective_enum_values"], ["right"])
        self.assertEqual(option["effective_canonical_enum_values"], ["right"])
        self.assertTrue(option["effective_nullable"])
        self.assertFalse(option["effective_default_declared"])
        self.assertEqual({item["field"] for item in report["metadata_mismatches"]},
                         {"enum_values", "nullable", "default_declared"})
        self.assertEqual(option["status"], "declared_partial")
        self.assertEqual(report["counts"], {"declared_partial": 1})
        self.assertEqual(report["counts_by_type"], {"coFloat": {"declared_partial": 1}})
        self.assertIn("nullable_behavior", option["required_dimensions"])
        # Reconciliation must not duplicate missing-dimension explanations.
        settings_inventory.apply_runtime_registry(report, audit)
        self.assertEqual(option["reason"].count("Missing required dimensions:"), 1)

        audit["metadata_by_key"]["shared"]["nullable"] = False
        settings_inventory.apply_runtime_registry(report, audit)
        self.assertEqual(option["status"], "declared_covered")
        self.assertEqual(report["counts"], {"declared_covered": 1})
        self.assertNotIn("nullable_behavior", option["required_dimensions"])
        self.assertEqual(option["reason"], option["evidence_reason"])

    def test_malformed_metadata_is_rejected(self):
        report = {"options": [{"key": "shared", "type": "coFloat"}]}
        registry = {
            "schema_version": 1,
            "source": "PrintConfigDef::options",
            "settings_count": 1,
            "options": [{
                "key": "shared", "type_id": 1, "nullable": False, "has_default": True,
                "enum_values": ["ok", 7], "canonical_enum_values": [],
            }],
        }
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "registry.json"
            type_header = Path(directory) / "Config.hpp"
            type_header.write_text("enum ConfigOptionType {\n coFloat = 1\n};\n",
                                   encoding="utf-8")
            path.write_text(json.dumps(registry), encoding="utf-8")
            with self.assertRaises(settings_inventory.InventoryError):
                settings_inventory.audit_runtime_registry(report, path,
                                                              type_header=type_header)

    def test_keyset_mismatch_never_exposes_effective_metadata(self):
        report = {"options": [{"key": "shared", "type": "coFloat",
                                "enum_values": [], "nullable": False,
                                "default_declared": True}]}
        registry = {
            "schema_version": 1,
            "source": "PrintConfigDef::options",
            "settings_count": 2,
            "options": [{
                "key": "shared", "type_id": 1, "nullable": False, "has_default": True,
                "enum_values": [], "canonical_enum_values": [],
            }, {
                "key": "extra", "type_id": 1, "nullable": False, "has_default": True,
                "enum_values": ["untrusted"], "canonical_enum_values": ["untrusted"],
            }],
        }
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "registry.json"
            type_header = Path(directory) / "Config.hpp"
            type_header.write_text("enum ConfigOptionType {\n coFloat = 1\n};\n",
                                   encoding="utf-8")
            path.write_text(json.dumps(registry), encoding="utf-8")
            audit = settings_inventory.audit_runtime_registry(report, path,
                                                                  type_header=type_header)
        settings_inventory.apply_runtime_registry(report, audit)
        self.assertFalse(report["runtime_registry_verified"])
        self.assertFalse(report["metadata_complete"])
        self.assertNotIn("effective_enum_values", report["options"][0])

    def test_wrong_type_id_fails_against_config_option_type_domain(self):
        report = {"options": [{"key": "shared", "type": "coFloat"}]}
        registry = {
            "schema_version": 1,
            "source": "PrintConfigDef::options",
            "settings_count": 1,
            "options": [{
                "key": "shared", "type_id": 8, "nullable": False, "has_default": True,
                "enum_values": [], "canonical_enum_values": [],
            }],
        }
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "registry.json"
            type_header = Path(directory) / "Config.hpp"
            type_header.write_text("enum ConfigOptionType {\n coFloat = 1,\n coBool = 8\n};\n",
                                   encoding="utf-8")
            path.write_text(json.dumps(registry), encoding="utf-8")
            with self.assertRaises(settings_inventory.InventoryError):
                settings_inventory.audit_runtime_registry(report, path,
                                                              type_header=type_header)


if __name__ == "__main__":
    unittest.main()
