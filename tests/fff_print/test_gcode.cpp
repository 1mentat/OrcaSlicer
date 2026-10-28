#include "libslic3r/CustomGCode.hpp"
#include <catch2/matchers/catch_matchers_string.hpp>
#include "libslic3r/GCode/AdaptivePAProcessor.hpp"
#include <cstdlib>
#include <cstdio>
#include <cctype>
#include <sstream>
#include <memory>
#include <limits>
#include "libslic3r/GCode.hpp"
#include "test_gcode_transform_bridge.hpp"
#include "libslic3r/GCode/WipeTower.hpp"
#include "libslic3r/calib.hpp"
#include <catch2/catch_all.hpp>
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/Config.hpp"
#include "libslic3r/GCodeWriter.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/Layer.hpp"
#include <map>
#include <set>
#include <string_view>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include "libslic3r/PrintBase.hpp"
#include "libslic3r/libslic3r.h"
#include <ios>
#include <iterator>
#include "libslic3r/GCode/GCodeProcessor.hpp"
#include "libslic3r/GCodeReader.hpp"
#include "test_utils.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <initializer_list>
#include <iomanip>
#include <optional>
#include <utility>
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include <catch2/catch_message.hpp>
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/ModelArrange.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/Preset.hpp"
#include "libslic3r/PresetBundle.hpp"
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>

#include "libslic3r/GCode/PrintExtents.hpp"
#include "libslic3r/Polyline.hpp"
#include <boost/beast/core/detail/base64.hpp>

#include "test_helpers.hpp"
#include "test_gcode_contract_helpers.hpp"

#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/Point.hpp"
#include <string>
#include <vector>

using namespace Slic3r;

TEST_CASE("Klipper object labels name each copy without the characters Klipper cannot parse", "[GCode]")
{
    const auto [name, label] = GENERATE(table<std::string, std::string>({
        {"my part (2)", "my_part_2"},
        {"(cube)", "cube"},
    }));
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({{"gcode_flavor", "klipper"}, {"exclude_object", "1"}});
    Print print;
    Model model;
    Test::init_print(std::vector<TriangleMesh>{Test::cube(20.)}, print, model, config, nullptr, false, 2);
    model.objects.front()->name = name;
    arrange_objects(model, BoundingBox{Point::new_scale(0., 0.), Point::new_scale(500., 500.)},
                    ArrangeParams{scaled(min_object_distance(config))});
    print.apply(model, config);

    const std::string gcode = Test::gcode(print);
    for (const char *copy : {"0", "1"}) {
        const std::string instance_label = label + "_id_0_copy_" + copy;
        INFO(instance_label);
        CHECK(gcode.find("EXCLUDE_OBJECT_DEFINE NAME=" + instance_label + " ") != std::string::npos);
        CHECK(gcode.find("EXCLUDE_OBJECT_START NAME=" + instance_label + "\n") != std::string::npos);
    }
}

namespace gcode_print_variance_contracts {

namespace {

TriangleMesh short_cube()
{
    TriangleMesh shape = make_cube(10.0, 10.0, 10.0);
    shape.scale(Vec3f(1.0f, 1.0f, 0.4f));
    return shape;
}

DynamicPrintConfig base_config()
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        { "layer_height",               0.2 },
        { "initial_layer_print_height", 0.2 },
        { "wall_loops",                 1 },
        { "top_shell_layers",           1 },
        { "bottom_shell_layers",        1 },
        { "top_shell_thickness",        0.0 },
        { "bottom_shell_thickness",     0.0 },
        { "sparse_infill_density",      "20%" },
        { "line_width",                 0.5 },
        { "initial_layer_line_width",   0.5 },
        { "outer_wall_line_width",      0.5 },
        { "inner_wall_line_width",      0.5 },
        { "sparse_infill_line_width",   0.5 },
        { "internal_solid_infill_line_width", 0.5 },
        { "top_surface_line_width",     0.5 },
        { "skin_infill_line_width",     0.5 },
        { "skeleton_infill_line_width", 0.5 },
        { "bridge_line_width",          0.35 },
        { "skirt_loops",                0 },
        { "brim_type",                  "no_brim" },
        { "raft_layers",                0 },
        { "gcode_flavor",               "marlin2" },
        { "machine_start_gcode",         "" },
        { "machine_end_gcode",           "" },
        { "file_start_gcode",            "" },
        { "before_layer_change_gcode",   "" },
        { "layer_change_gcode",          "G92 E0\n" },
        { "printing_by_object_gcode",   "" },
        { "print_sequence",              "by layer" },
        { "gcode_label_objects",         false },
        { "exclude_object",              false },
        { "gcode_comments",              true },
        { "z_hop",                       0.0 },
        { "emit_machine_limits_to_gcode", false },
        { "retraction_length",           0.8 },
        { "retraction_minimum_travel",   0.0 },
        { "machine_max_acceleration_x",  "500,500" },
        { "machine_max_acceleration_y",  "500,500" },
        { "machine_max_acceleration_z",  "100,100" },
        { "machine_max_acceleration_e",  "5000,5000" },
        { "machine_max_acceleration_extruding",  "1200,1200" },
        { "machine_max_acceleration_retracting", "1400,1400" },
        { "machine_max_acceleration_travel", "1600,1600" },
        { "machine_max_speed_x",          "100,100" },
        { "machine_max_speed_y",          "100,100" },
        { "machine_max_speed_z",          "10,10" },
        { "machine_max_speed_e",          "50,50" },
        { "machine_max_jerk_x",           "8,8" },
        { "machine_max_jerk_y",           "8,8" },
        { "machine_max_jerk_z",           "0.4,0.4" },
        { "machine_max_jerk_e",           "5,5" },
        { "machine_max_junction_deviation", "0.02,0.02" },
    });
    return config;
}

void initialize(Print &print, Model &model, const DynamicPrintConfig &config, bool two_objects)
{
    auto add_object = [&model, &print](TriangleMesh mesh) {
        ModelObject *object = model.add_object();
        object->name += "object.stl";
        object->add_volume(std::move(mesh));
        object->add_instance();
        object->ensure_on_bed();
        print.auto_assign_extruders(object);
    };
    add_object(short_cube());
    if (two_objects) {
        TriangleMesh second = short_cube();
        second.translate(60.0, 0.0, 0.0);
        add_object(std::move(second));
    }
    print.apply(model, config);
    require_valid(print);
}

ExportedPrint fresh_export(const DynamicPrintConfig &config, bool two_objects = false)
{
    Print print;
    Model model;
    initialize(print, model, config, two_objects);
    return export_print(print);
}

ExportedPrint incremental_export(const DynamicPrintConfig &baseline, const DynamicPrintConfig &changed,
                                 bool two_objects = false)
{
    Print print;
    Model model;
    initialize(print, model, baseline, two_objects);
    REQUIRE_FALSE(export_print(print).text.empty());
    const PrintBase::ApplyStatus status = print.apply(model, changed);
    REQUIRE(status != PrintBase::APPLY_STATUS_UNCHANGED);
    require_valid(print);
    return export_print(print);
}

size_t executable_command_count(const std::string &text, const DynamicPrintConfig &config,
                                const std::string &command, const std::string &marker)
{
    size_t count = 0;
    GCodeReader reader;
    reader.apply_config(config);
    reader.parse_buffer(text, [&count, &command, &marker](GCodeReader &, const GCodeReader::GCodeLine &line) {
        if (line.cmd_is(command.c_str()) && line.raw().find(marker) != std::string::npos)
            ++count;
    });
    return count;
}

std::optional<size_t> first_executable_command(const std::string &text, const DynamicPrintConfig &config,
                                                const std::string &command, const std::string &marker)
{
    std::optional<size_t> position;
    size_t cursor = 0;
    GCodeReader reader;
    reader.apply_config(config);
    reader.parse_buffer(text, [&position, &cursor, &text, &command, &marker](GCodeReader &, const GCodeReader::GCodeLine &line) {
        if (position.has_value() || !line.cmd_is(command.c_str()) || line.raw().find(marker) == std::string::npos)
            return;
        const size_t found = text.find(line.raw(), cursor);
        if (found != std::string::npos)
            position = found;
    });
    return position;
}

std::optional<size_t> last_executable_command(const std::string &text, const DynamicPrintConfig &config,
                                               const std::string &command)
{
    std::optional<size_t> position;
    size_t cursor = 0;
    GCodeReader reader;
    reader.apply_config(config);
    reader.parse_buffer(text, [&position, &cursor, &text, &command](GCodeReader &, const GCodeReader::GCodeLine &line) {
        if (!line.cmd_is(command.c_str()))
            return;
        const size_t found = text.find(line.raw(), cursor);
        if (found != std::string::npos) {
            position = found;
            cursor = found + line.raw().size();
        }
    });
    return position;
}


struct NegativeXYMotionEvidence
{
    size_t negative_e_xy_moves = 0;
    bool   finite = true;
};

NegativeXYMotionEvidence negative_xy_motion_evidence(const std::string &text,
                                                     const DynamicPrintConfig &config)
{
    NegativeXYMotionEvidence evidence;
    GCodeReader reader;
    reader.apply_config(config);
    reader.parse_buffer(text, [&evidence](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        if (!(line.cmd_is("G0") || line.cmd_is("G1") || line.cmd_is("G2") || line.cmd_is("G3")) ||
            !line.has_e())
            return;

        const double xy = line.dist_XY(self);
        const double de = line.dist_E(self);
        if (!std::isfinite(xy) || !std::isfinite(de)) {
            evidence.finite = false;
            return;
        }
        if (xy > EPSILON && de < -EPSILON)
            ++evidence.negative_e_xy_moves;
    });
    return evidence;
}

void require_hook_order(const std::string &text, const DynamicPrintConfig &config, size_t layers)
{
    std::vector<std::string> hooks;
    GCodeReader reader;
    reader.apply_config(config);
    reader.parse_buffer(text, [&hooks](GCodeReader &, const GCodeReader::GCodeLine &line) {
        if (line.cmd_is("M117") &&
            (line.raw().find("GP_BEFORE-") != std::string::npos ||
             line.raw().find("GP_LAYER-") != std::string::npos))
            hooks.push_back(line.raw());
    });
    REQUIRE(hooks.size() == 2 * layers);
    for (size_t layer = 0; layer < layers; ++layer) {
        INFO("layer " << layer);
        CHECK(hooks[2 * layer] == "M117 GP_BEFORE-" + std::to_string(layer));
        CHECK(hooks[2 * layer + 1] == "M117 GP_LAYER-" + std::to_string(layer));
    }
}

} // namespace

TEST_CASE("Lifecycle G-code hooks preserve layer ordering and placeholders", "[GCodePrintVariance]")
{
    DynamicPrintConfig config = base_config();
    config.set_deserialize_strict({
        { "machine_start_gcode",       "M117 GP_START\n" },
        { "machine_end_gcode",         "M117 GP_END\n" },
        { "before_layer_change_gcode", "M117 GP_BEFORE-[layer_num]\n" },
        { "layer_change_gcode",        "G92 E0\nM117 GP_LAYER-[layer_num]\n" },
    });
    const ExportedPrint exported = fresh_export(config);
    const size_t layers = count_text(exported.text, ";LAYER_CHANGE");
    REQUIRE(layers > 1);
    REQUIRE(executable_command_count(exported.text, config, "M117", "GP_START") == 1);
    REQUIRE(executable_command_count(exported.text, config, "M117", "GP_END") == 1);
    REQUIRE(executable_command_count(exported.text, config, "M117", "GP_BEFORE-") == layers);
    REQUIRE(executable_command_count(exported.text, config, "M117", "GP_LAYER-") == layers);
    const auto start = first_executable_command(exported.text, config, "M117", "GP_START");
    const auto end = first_executable_command(exported.text, config, "M117", "GP_END");
    const auto last_motion = last_executable_command(exported.text, config, "G1");
    REQUIRE(start.has_value());
    REQUIRE(end.has_value());
    REQUIRE(last_motion.has_value());
    REQUIRE(*start < exported.text.find(";LAYER_CHANGE"));
    REQUIRE(*end > *last_motion);
    require_hook_order(exported.text, config, layers);
}

TEST_CASE("Custom lifecycle hooks change only the intended emitted commands", "[GCodePrintVariance]")
{
    const DynamicPrintConfig baseline = base_config();
    DynamicPrintConfig changed = baseline;
    changed.set_deserialize_strict({
        { "machine_start_gcode",       "M117 GP_START\n" },
        { "machine_end_gcode",         "M117 GP_END\n" },
        { "before_layer_change_gcode", "M117 GP_BEFORE\n" },
        { "layer_change_gcode",        "G92 E0\nM117 GP_LAYER\n" },
    });
    const ExportedPrint fresh = fresh_export(changed);
    const ExportedPrint incremental = incremental_export(baseline, changed);
    const MotionEvidence fresh_motion = motion_evidence(fresh.text, changed);
    const MotionEvidence incremental_motion = motion_evidence(incremental.text, changed);
    require_same_material(fresh_motion, incremental_motion);
    REQUIRE(fresh_motion.extrusion_moves > 0);
    for (const char *marker : {"GP_START", "GP_END"}) {
        CHECK(executable_command_count(fresh.text, changed, "M117", marker) == 1);
        CHECK(executable_command_count(incremental.text, changed, "M117", marker) == 1);
    }
    REQUIRE(executable_command_count(fresh.text, changed, "M117", "GP_LAYER") > 1);
    REQUIRE(executable_command_count(fresh.text, changed, "M117", "GP_BEFORE") > 1);
    REQUIRE(executable_command_count(incremental.text, changed, "M117", "GP_LAYER") ==
            executable_command_count(fresh.text, changed, "M117", "GP_LAYER"));
    REQUIRE(executable_command_count(incremental.text, changed, "M117", "GP_BEFORE") ==
            executable_command_count(fresh.text, changed, "M117", "GP_BEFORE"));
}

TEST_CASE("Printing by object emits its boundary hook once between objects", "[GCodePrintVariance]")
{
    DynamicPrintConfig by_layer = base_config();
    by_layer.set_deserialize_strict({
        { "printing_by_object_gcode", "M117 GP_BETWEEN\n" },
        { "print_sequence",          "by layer" },
    });
    const ExportedPrint layered = fresh_export(by_layer, true);
    REQUIRE(executable_command_count(layered.text, by_layer, "M117", "GP_BETWEEN") == 0);

    DynamicPrintConfig by_object = by_layer;
    by_object.set_deserialize_strict({ { "print_sequence", "by object" } });
    const ExportedPrint sequential = fresh_export(by_object, true);
    REQUIRE(executable_command_count(sequential.text, by_object, "M117", "GP_BETWEEN") == 1);
    REQUIRE(executable_command_count(sequential.text, by_object, "M117", "GP_BETWEEN") < count_text(sequential.text, ";LAYER_CHANGE"));
    require_same_material(motion_evidence(layered.text, by_layer), motion_evidence(sequential.text, by_object));
}

TEST_CASE("Sequential boundary placeholders identify the finished object", "[GCodePrintVariance]")
{
    DynamicPrintConfig config = base_config();
    config.set_deserialize_strict({
        { "print_sequence",          "by object" },
        { "printing_by_object_gcode", "M117 GP_OBJECT-[current_object_idx]\n" },
    });
    const ExportedPrint exported = fresh_export(config, true);
    REQUIRE(executable_command_count(exported.text, config, "M117", "GP_OBJECT-1") == 1);
    REQUIRE(executable_command_count(exported.text, config, "M117", "GP_OBJECT-0") == 0);
    const auto boundary = first_executable_command(exported.text, config, "M117", "GP_OBJECT-1");
    const auto last_motion = last_executable_command(exported.text, config, "G1");
    REQUIRE(boundary.has_value());
    REQUIRE(last_motion.has_value());
    REQUIRE(*boundary < *last_motion);
}

TEST_CASE("Object labels are optional metadata and leave deposited material unchanged", "[GCodePrintVariance]")
{
    DynamicPrintConfig disabled = base_config();
    disabled.set_deserialize_strict({ { "gcode_label_objects", false } });
    const ExportedPrint no_labels = fresh_export(disabled, true);
    REQUIRE(count_text(no_labels.text, "; printing object ") == 0);

    DynamicPrintConfig enabled = disabled;
    enabled.set_deserialize_strict({ { "gcode_label_objects", true } });
    const ExportedPrint labels = fresh_export(enabled, true);
    REQUIRE(count_text(labels.text, "; printing object ") > 0);
    REQUIRE(count_text(labels.text, "; stop printing object ") > 0);
    require_same_material(motion_evidence(no_labels.text, disabled), motion_evidence(labels.text, enabled));
}

TEST_CASE("Exclude-object commands are gated independently from object comments", "[GCodePrintVariance]")
{
    DynamicPrintConfig disabled = base_config();
    disabled.set_deserialize_strict({
        { "gcode_label_objects", false },
        { "exclude_object",      false },
    });
    Print disabled_print;
    Model disabled_model;
    initialize(disabled_print, disabled_model, disabled, true);
    const ExportedPrint no_exclude = export_print(disabled_print);
    REQUIRE_FALSE(no_exclude.label_object_enabled);
    REQUIRE(executable_command_count(no_exclude.text, disabled, "M486", "") == 0);

    DynamicPrintConfig enabled = disabled;
    enabled.set_deserialize_strict({ { "exclude_object", true } });
    const ExportedPrint exclude = fresh_export(enabled, true);
    REQUIRE(exclude.label_object_enabled);
    REQUIRE(executable_command_count(exclude.text, enabled, "M486", "") > 0);
    REQUIRE(executable_command_count(exclude.text, enabled, "M486", "S-1") > 0);
    require_same_material(motion_evidence(no_exclude.text, disabled), motion_evidence(exclude.text, enabled));
}

TEST_CASE("Machine envelope emission is a semantic opt-in", "[GCodePrintVariance]")
{
    DynamicPrintConfig disabled = base_config();
    disabled.set_deserialize_strict({ { "emit_machine_limits_to_gcode", false } });
    const ExportedPrint no_limits = fresh_export(disabled);
    REQUIRE(executable_command_count(no_limits.text, disabled, "M201", "") == 0);
    REQUIRE(executable_command_count(no_limits.text, disabled, "M203", "") == 0);

    DynamicPrintConfig enabled = disabled;
    enabled.set_deserialize_strict({ { "emit_machine_limits_to_gcode", true } });
    const ExportedPrint limits = fresh_export(enabled);
    REQUIRE(executable_command_count(limits.text, enabled, "M201", "") > 0);
    REQUIRE(executable_command_count(limits.text, enabled, "M203", "") > 0);
    REQUIRE(executable_command_count(limits.text, enabled, "M204", "") > 0);
    REQUIRE(executable_command_count(limits.text, enabled, "M205", "") > 0);
    require_same_material(motion_evidence(no_limits.text, disabled), motion_evidence(limits.text, enabled));
}

TEST_CASE("Machine envelope selects junction deviation only when enabled and positive",
          "[GCodePrintVariance][MachineEnvelope]")
{
    struct EnvelopeRow {
        const char *name;
        bool        emit;
        const char *junction_deviation;
        double      expected_junction;
    };

    const DynamicPrintConfig reference_config = base_config();
    const ExportedPrint reference = fresh_export(reference_config);
    const MotionEvidence reference_motion = motion_evidence(reference.text, reference_config);
    REQUIRE(reference_motion.extrusion_moves > 0);

    const std::array<EnvelopeRow, 3> rows = {{
        {"disabled gate suppresses the envelope", false, "0.25,0.25", 0.0},
        {"enabled zero deviation keeps classic jerk", true, "0,0", 0.0},
        {"enabled positive deviation emits M205 J", true, "0.25,0.25", 0.25},
    }};

    for (const EnvelopeRow &row : rows) {
        INFO(row.name);
        DynamicPrintConfig config = base_config();
        config.set_deserialize_strict({
            {"emit_machine_limits_to_gcode", row.emit},
            {"default_junction_deviation", 0.0},
            {"machine_max_junction_deviation", row.junction_deviation},
            {"machine_max_jerk_x", "11,11"},
            {"machine_max_jerk_y", "13,13"},
        });
        const ExportedPrint output = fresh_export(config);
        require_same_material(reference_motion, motion_evidence(output.text, config));

        std::array<std::vector<std::array<float, 4>>, 3> limits;
        std::vector<float> junctions;
        const std::array<char, 4> axes = {{'X', 'Y', 'Z', 'E'}};
        GCodeReader reader;
        reader.apply_config(config);
        reader.parse_buffer(output.text, [&](GCodeReader &, const GCodeReader::GCodeLine &line) {
            if (line.cmd_is("M205")) {
                float value = 0.f;
                if (line.has_value('J', value))
                    junctions.push_back(value);
            }
            int index = -1;
            if (line.cmd_is("M201"))
                index = 0;
            else if (line.cmd_is("M203"))
                index = 1;
            else if (line.cmd_is("M205") && line.has('Z') && line.has('E'))
                index = 2;
            if (index < 0)
                return;
            std::array<float, 4> values{};
            for (size_t axis = 0; axis < axes.size(); ++axis) {
                REQUIRE(line.has_value(axes[axis], values[axis]));
                REQUIRE(std::isfinite(values[axis]));
            }
            limits[size_t(index)].push_back(values);
        });

        if (!row.emit) {
            CHECK(limits[0].empty());
            CHECK(limits[1].empty());
            CHECK(limits[2].empty());
        } else {
            // These values are explicitly configured by base_config() and the row overrides.
            const std::array<std::array<double, 4>, 3> expected = {{
                {{500., 500., 100., 5000.}},
                {{100., 100., 10., 50.}},
                {{11., 13., 0.4, 5.}},
            }};
            for (size_t command = 0; command < limits.size(); ++command) {
                REQUIRE(limits[command].size() == 1);
                for (size_t axis = 0; axis < axes.size(); ++axis) {
                    INFO("command index " << command << ", axis " << axes[axis]);
                    CHECK_THAT(limits[command][0][axis],
                               Catch::Matchers::WithinAbs(expected[command][axis], 1e-5));
                }
            }
        }

        if (row.expected_junction > 0.) {
            REQUIRE(junctions.size() == 1);
            CHECK_THAT(junctions.front(), Catch::Matchers::WithinAbs(row.expected_junction, 1e-6));
        } else {
            CHECK(junctions.empty());
        }
    }
}

TEST_CASE("Machine envelope serializes registered numeric boundaries and preserves deposited motion",
          "[GCode][GCodePrintVariance][MachineEnvelope][GCodeExportWorkflows]")
{
    struct ParsedEnvelope {
        std::vector<std::array<float, 4>> acceleration;
        std::vector<std::array<float, 4>> speed;
        std::vector<std::array<float, 3>> mode_acceleration;
        std::vector<std::array<float, 4>> jerk;
        std::vector<float> junction_deviation;
    };

    const auto parse_envelope = [](const ExportedPrint &output, const DynamicPrintConfig &config) {
        ParsedEnvelope parsed;
        GCodeReader reader;
        reader.apply_config(config);
        reader.parse_buffer(output.text, [&parsed](GCodeReader &, const GCodeReader::GCodeLine &line) {
            const auto read_four = [&line](const std::array<char, 4> &words) {
                std::array<float, 4> values{};
                for (size_t i = 0; i < words.size(); ++i) {
                    REQUIRE(line.has_value(words[i], values[i]));
                    REQUIRE(std::isfinite(values[i]));
                }
                return values;
            };
            if (line.cmd_is("M201"))
                parsed.acceleration.push_back(read_four({{'X', 'Y', 'Z', 'E'}}));
            else if (line.cmd_is("M203"))
                parsed.speed.push_back(read_four({{'X', 'Y', 'Z', 'E'}}));
            else if (line.cmd_is("M204") && line.has('P') && line.has('R') && line.has('T')) {
                // GCodeWriter can emit separate P-only/T-only changes later; the full P/R/T
                // command is the machine-envelope preamble whose three source settings are tested.
                std::array<float, 3> values{};
                const std::array<char, 3> words{{'P', 'R', 'T'}};
                for (size_t i = 0; i < values.size(); ++i) {
                    REQUIRE(line.has_value(words[i], values[i]));
                    REQUIRE(std::isfinite(values[i]));
                }
                parsed.mode_acceleration.push_back(values);
            } else if (line.cmd_is("M205")) {
                float junction = 0.f;
                if (line.has_value('J', junction)) {
                    REQUIRE(std::isfinite(junction));
                    parsed.junction_deviation.push_back(junction);
                }
                if (line.has('Z') && line.has('E'))
                    parsed.jerk.push_back(read_four({{'X', 'Y', 'Z', 'E'}}));
            }
        });
        return parsed;
    };

    const auto check_four = [](const std::vector<std::array<float, 4>> &actual,
                               const std::array<float, 4> &expected) {
        REQUIRE(actual.size() == 1);
        for (size_t i = 0; i < expected.size(); ++i)
            CHECK_THAT(actual.front()[i], Catch::Matchers::WithinAbs(expected[i], 1e-5f));
    };
    const auto check_three = [](const std::vector<std::array<float, 3>> &actual,
                                const std::array<float, 3> &expected) {
        REQUIRE(actual.size() == 1);
        for (size_t i = 0; i < expected.size(); ++i)
            CHECK_THAT(actual.front()[i], Catch::Matchers::WithinAbs(expected[i], 1e-5f));
    };

    DynamicPrintConfig reference_config = base_config();
    const ExportedPrint reference = fresh_export(reference_config);
    const MotionEvidence reference_motion = motion_evidence(reference.text, reference_config);
    REQUIRE(reference_motion.extrusion_moves > 0);
    REQUIRE(reference_motion.xy_path > 0.0);
    REQUIRE(reference_motion.filament > 0.0);

    // All fifteen speed/jerk/acceleration options have a registered minimum of zero
    // and no registered maximum. Junction deviation alone also has a finite 0.3 maximum.
    DynamicPrintConfig minimum_config = base_config();
    minimum_config.set_deserialize_strict({
        {"emit_machine_limits_to_gcode", true},
        {"machine_max_acceleration_x", "0,0"},
        {"machine_max_acceleration_y", "0,0"},
        {"machine_max_acceleration_z", "0,0"},
        {"machine_max_acceleration_e", "0,0"},
        {"machine_max_acceleration_extruding", "0,0"},
        {"machine_max_acceleration_retracting", "0,0"},
        {"machine_max_acceleration_travel", "0,0"},
        {"machine_max_speed_x", "0,0"},
        {"machine_max_speed_y", "0,0"},
        {"machine_max_speed_z", "0,0"},
        {"machine_max_speed_e", "0,0"},
        {"machine_max_jerk_x", "0,0"},
        {"machine_max_jerk_y", "0,0"},
        {"machine_max_jerk_z", "0,0"},
        {"machine_max_jerk_e", "0,0"},
        {"machine_max_junction_deviation", "0,0"},
    });
    const ExportedPrint minimum_output = fresh_export(minimum_config);
    const MotionEvidence minimum_motion = motion_evidence(minimum_output.text, minimum_config);
    require_same_material(reference_motion, minimum_motion);
    const ParsedEnvelope minimum = parse_envelope(minimum_output, minimum_config);
    const std::array<float, 4> zero_four{{0.f, 0.f, 0.f, 0.f}};
    const std::array<float, 3> zero_three{{0.f, 0.f, 0.f}};
    check_four(minimum.acceleration, zero_four);
    check_four(minimum.speed, zero_four);
    check_three(minimum.mode_acceleration, zero_three);
    check_four(minimum.jerk, zero_four);
    CHECK(minimum.junction_deviation.empty());

    // Distinct positive axis values catch X/Y/Z/E mixups. These are representative
    // values, not upper-bound claims; the inclusive registered JD maximum is tested here.
    DynamicPrintConfig upper_junction_config = base_config();
    upper_junction_config.set_deserialize_strict({
        {"emit_machine_limits_to_gcode", true},
        {"machine_max_acceleration_x", "101,101"},
        {"machine_max_acceleration_y", "202,202"},
        {"machine_max_acceleration_z", "303,303"},
        {"machine_max_acceleration_e", "404,404"},
        {"machine_max_acceleration_extruding", "505,505"},
        {"machine_max_acceleration_retracting", "606,606"},
        {"machine_max_acceleration_travel", "707,707"},
        {"machine_max_speed_x", "11,11"},
        {"machine_max_speed_y", "22,22"},
        {"machine_max_speed_z", "33,33"},
        {"machine_max_speed_e", "44,44"},
        {"machine_max_jerk_x", "1.11,1.11"},
        {"machine_max_jerk_y", "2.22,2.22"},
        {"machine_max_jerk_z", "0.33,0.33"},
        {"machine_max_jerk_e", "4.44,4.44"},
        {"machine_max_junction_deviation", "0.3,0.3"},
    });
    const ExportedPrint upper_junction_output = fresh_export(upper_junction_config);
    const MotionEvidence upper_junction_motion = motion_evidence(upper_junction_output.text, upper_junction_config);
    require_same_material(reference_motion, upper_junction_motion);
    const ParsedEnvelope upper_junction = parse_envelope(upper_junction_output, upper_junction_config);
    check_four(upper_junction.acceleration, {{101.f, 202.f, 303.f, 404.f}});
    check_four(upper_junction.speed, {{11.f, 22.f, 33.f, 44.f}});
    check_three(upper_junction.mode_acceleration, {{505.f, 606.f, 707.f}});
    check_four(upper_junction.jerk, {{1.11f, 2.22f, 0.33f, 4.44f}});
    REQUIRE(upper_junction.junction_deviation.size() == 1);
    CHECK_THAT(upper_junction.junction_deviation.front(), Catch::Matchers::WithinAbs(0.3f, 1e-5f));
}

TEST_CASE("Z hop changes travel height while preserving material paths", "[GCodePrintVariance]")
{
    DynamicPrintConfig no_hop = base_config();
    no_hop.set_deserialize_strict({ { "z_hop", 0.0 } });
    const ExportedPrint flat = fresh_export(no_hop);
    const MotionEvidence flat_motion = motion_evidence(flat.text, no_hop);

    DynamicPrintConfig hopped = no_hop;
    hopped.set_deserialize_strict({ { "z_hop", 1.0 } });
    const ExportedPrint lifted = fresh_export(hopped);
    const MotionEvidence lifted_motion = motion_evidence(lifted.text, hopped);
    require_same_material(flat_motion, lifted_motion);
    REQUIRE(lifted_motion.z_only_moves > flat_motion.z_only_moves);
}

TriangleMesh support_fixture()
{
    TriangleMesh base = make_cube(40., 40., 2.);
    TriangleMesh stem = make_cube(8., 8., 12.);
    stem.translate(Vec3f(16.f, 16.f, 1.f));
    TriangleMesh cap = make_cube(40., 40., 2.);
    cap.translate(Vec3f(0.f, 0.f, 12.f));
    base.merge(stem);
    base.merge(cap);
    return base;
}

struct ZHopMotionEvidence
{
    size_t retractions = 0;
    size_t unretractions = 0;
    size_t same_layer_excursions = 0;
    size_t same_layer_z_only_lift_moves = 0;
    size_t same_layer_diagonal_lift_moves = 0;
    size_t same_layer_maximum_diagonal_run = 0;
    size_t same_layer_overhang_spiral_excursions = 0;
    bool   finite = true;
};

std::string_view zhop_trim_comment(std::string_view comment)
{
    while (!comment.empty() && std::isspace(static_cast<unsigned char>(comment.front())))
        comment.remove_prefix(1);
    while (!comment.empty() && std::isspace(static_cast<unsigned char>(comment.back())))
        comment.remove_suffix(1);
    return comment;
}

bool zhop_is_layer_marker(std::string_view comment)
{
    comment = zhop_trim_comment(comment);
    return comment.rfind("LAYER_CHANGE", 0) == 0 || comment.rfind("CHANGE_LAYER", 0) == 0;
}

bool zhop_is_overhang_role(std::string_view role)
{
    return role.find("Overhang") != std::string_view::npos;
}

ZHopMotionEvidence zhop_motion_evidence(const std::string &text, const DynamicPrintConfig &config)
{
    constexpr size_t max_gcode_bytes = 4u * 1024u * 1024u;
    constexpr size_t max_gcode_lines = 100000u;
    constexpr double plane_tolerance = 5e-4;
    REQUIRE(text.size() <= max_gcode_bytes);
    REQUIRE(static_cast<size_t>(std::count(text.begin(), text.end(), '\n')) <= max_gcode_lines);

    ZHopMotionEvidence evidence;
    bool retracted = false;
    bool crossed_layer = false;
    bool overhang_role_before_retraction = false;
    double start_plane = 0.0;
    size_t excursion_z_only = 0;
    size_t excursion_diagonal = 0;
    size_t excursion_max_diagonal_run = 0;
    size_t diagonal_run = 0;
    std::string active_role;
    GCodeReader reader;
    reader.apply_config(config);
    reader.parse_buffer(text, [&evidence, &retracted, &crossed_layer, &overhang_role_before_retraction,
                               &start_plane, &excursion_z_only, &excursion_diagonal,
                               &excursion_max_diagonal_run, &diagonal_run, &active_role](GCodeReader &self,
                                   const GCodeReader::GCodeLine &line) {
        const std::string_view comment = zhop_trim_comment(line.comment());
        if (zhop_is_layer_marker(comment)) {
            if (retracted)
                crossed_layer = true;
        }
        if (comment.rfind("TYPE:", 0) == 0)
            active_role.assign(zhop_trim_comment(comment.substr(5)));
        else if (comment.rfind("FEATURE:", 0) == 0)
            active_role.assign(zhop_trim_comment(comment.substr(8)));

        const bool motion = line.cmd_is("G0") || line.cmd_is("G1") || line.cmd_is("G2") || line.cmd_is("G3");
        if (!motion)
            return;

        if (!std::isfinite(self.x()) || !std::isfinite(self.y()) || !std::isfinite(self.z()) ||
            !std::isfinite(self.e()) || (line.has_x() && !std::isfinite(line.x())) ||
            (line.has_y() && !std::isfinite(line.y())) || (line.has_z() && !std::isfinite(line.z())) ||
            (line.has_e() && !std::isfinite(line.e()))) {
            evidence.finite = false;
            return;
        }

        if (line.has_e()) {
            const double delta_e = line.dist_E(self);
            if (!std::isfinite(delta_e)) {
                evidence.finite = false;
                return;
            }
            if (delta_e < -EPSILON) {
                ++evidence.retractions;
                if (!retracted) {
                    retracted = true;
                    crossed_layer = false;
                    overhang_role_before_retraction = zhop_is_overhang_role(active_role);
                    start_plane = self.z();
                    excursion_z_only = 0;
                    excursion_diagonal = 0;
                    excursion_max_diagonal_run = 0;
                    diagonal_run = 0;
                }
                return;
            }
            if (delta_e > EPSILON) {
                if (retracted) {
                    ++evidence.unretractions;
                    const double end_plane = self.z();
                    const bool same_layer = !crossed_layer && std::isfinite(start_plane) &&
                                            std::isfinite(end_plane) &&
                                            std::abs(end_plane - start_plane) <= plane_tolerance;
                    if (same_layer) {
                        ++evidence.same_layer_excursions;
                        evidence.same_layer_z_only_lift_moves += excursion_z_only;
                        evidence.same_layer_diagonal_lift_moves += excursion_diagonal;
                        evidence.same_layer_maximum_diagonal_run =
                            std::max(evidence.same_layer_maximum_diagonal_run, excursion_max_diagonal_run);
                        if (overhang_role_before_retraction && excursion_max_diagonal_run >= 4)
                            ++evidence.same_layer_overhang_spiral_excursions;
                    }
                    retracted = false;
                    diagonal_run = 0;
                }
                return;
            }
        }

        if (!retracted)
            return;
        const double xy = line.dist_XY(self);
        const double dz = line.dist_Z(self);
        if (!std::isfinite(xy) || !std::isfinite(dz)) {
            evidence.finite = false;
            return;
        }
        // Only upward Z motion is a lift.  Downward motion and a layer-marker
        // transition are retained for the same-layer gate, but never counted as
        // an upward hop witness.
        if (dz <= EPSILON) {
            diagonal_run = 0;
            return;
        }
        if (xy <= EPSILON) {
            ++excursion_z_only;
            diagonal_run = 0;
        } else {
            ++excursion_diagonal;
            ++diagonal_run;
            excursion_max_diagonal_run = std::max(excursion_max_diagonal_run, diagonal_run);
        }
    });
    REQUIRE(evidence.finite);
    return evidence;
}

DynamicPrintConfig zhop_type_config(const ZHopType type)
{
    DynamicPrintConfig config = base_config();
    config.set_deserialize_strict({
        { "z_hop",                    0.4 },
        { "travel_slope",             3.0 },
        { "retraction_minimum_travel", 0.0 },
        { "retract_when_changing_layer", false },
        { "retract_lift_above",       0.0 },
        { "retract_lift_below",       0.0 },
        { "retract_lift_enforce",     "All Surfaces" },
        { "reduce_crossing_wall",     false },
        { "use_firmware_retraction",  false },
        { "wipe",                     false },
        { "enable_arc_fitting",       false },
        { "resolution",               0.0 },
    });
    config.set_key_value("z_hop_types", new ConfigOptionEnumsGeneric{type});
    return config;
}

TEST_CASE("Public Z-hop modes preserve deposition while changing lift motion",
          "[GCode][ZHopType][GCodePrintVariance]")
{
    DynamicPrintConfig no_hop = zhop_type_config(ZHopType::zhtNormal);
    no_hop.set_deserialize_strict({ { "z_hop", 0.0 } });
    const ExportedPrint no_hop_output = fresh_export(no_hop, true);
    const MotionEvidence no_hop_material = motion_evidence(no_hop_output.text, no_hop);
    REQUIRE(no_hop_material.extrusion_moves > 0);
    const ZHopMotionEvidence no_hop_lift = zhop_motion_evidence(no_hop_output.text, no_hop);
    REQUIRE(no_hop_lift.retractions > 0);
    REQUIRE(no_hop_lift.same_layer_excursions > 0);
    CHECK(no_hop_lift.same_layer_z_only_lift_moves == 0);
    CHECK(no_hop_lift.same_layer_diagonal_lift_moves == 0);

    const std::array<std::pair<ZHopType, const char *>, 3> explicit_modes{{
        { ZHopType::zhtNormal, "normal" },
        { ZHopType::zhtSlope,  "slope" },
        { ZHopType::zhtSpiral, "spiral" },
    }};
    for (const auto &[type, name] : explicit_modes) {
        INFO("explicit z-hop mode " << name);
        const DynamicPrintConfig config = zhop_type_config(type);
        const ExportedPrint output = fresh_export(config, true);
        const MotionEvidence material = motion_evidence(output.text, config);
        const ZHopMotionEvidence lift = zhop_motion_evidence(output.text, config);
        REQUIRE(lift.retractions > 0);
        REQUIRE(lift.unretractions > 0);
        REQUIRE(lift.same_layer_excursions > 0);
        REQUIRE(lift.same_layer_z_only_lift_moves + lift.same_layer_diagonal_lift_moves > 0);
        require_same_material(no_hop_material, material);
        if (type == ZHopType::zhtNormal) {
            REQUIRE(lift.same_layer_z_only_lift_moves > 0);
            CHECK(lift.same_layer_diagonal_lift_moves == 0);
        } else if (type == ZHopType::zhtSlope) {
            REQUIRE(lift.same_layer_diagonal_lift_moves > 0);
            CHECK(lift.same_layer_maximum_diagonal_run == 1);
        } else {
            // The writer's spiral approximation has at least eight bounded segments;
            // this avoids relying on the exact segment count while distinguishing it
            // from the single diagonal slope move.
            REQUIRE(lift.same_layer_maximum_diagonal_run >= 4);
        }
    }
}

TEST_CASE("Auto Z-hop uses spiral motion for an overhang travel on one layer",
          "[GCode][ZHopType][GCodePrintVariance]")
{
    DynamicPrintConfig auto_config = zhop_type_config(ZHopType::zhtAuto);
    // The custom-mesh helper centers this fixture at the origin. Leave room for
    // the complete spiral so the printable-area fallback cannot mask Auto routing.
    auto_config.set_deserialize_strict({
        { "printable_area",             "-100x-100,200x-100,200x200,-100x200" },
        { "enable_support",             true },
        { "support_type",               "normal(auto)" },
        { "support_on_build_plate_only", false },
        { "support_base_pattern",       "rectilinear" },
    });
    const ExportedPrint auto_output = fresh_custom_mesh_export(auto_config, support_fixture());
    const MotionEvidence auto_material = motion_evidence(auto_output.text, auto_config);
    const ZHopMotionEvidence auto_lift = zhop_motion_evidence(auto_output.text, auto_config);
    REQUIRE(auto_material.extrusion_moves > 0);
    REQUIRE(auto_lift.retractions > 0);
    REQUIRE(auto_lift.unretractions > 0);
    REQUIRE(auto_lift.same_layer_excursions > 0);
    REQUIRE(auto_lift.same_layer_overhang_spiral_excursions > 0);

    DynamicPrintConfig auto_no_hop = auto_config;
    auto_no_hop.set_deserialize_strict({ { "z_hop", 0.0 } });
    const ExportedPrint no_hop_output = fresh_custom_mesh_export(auto_no_hop, support_fixture());
    const MotionEvidence no_hop_material = motion_evidence(no_hop_output.text, auto_no_hop);
    const ZHopMotionEvidence no_hop_lift = zhop_motion_evidence(no_hop_output.text, auto_no_hop);
    require_same_material(no_hop_material, auto_material);
    CHECK(no_hop_lift.same_layer_overhang_spiral_excursions == 0);
}


size_t temperature_commands(const std::string &text, const DynamicPrintConfig &config,
                            const std::string &command, float expected)
{
    size_t count = 0;
    GCodeReader reader;
    reader.apply_config(config);
    reader.parse_buffer(text, [&](GCodeReader &, const GCodeReader::GCodeLine &line) {
        float value = 0.f;
        if (line.cmd_is(command.c_str()) && line.has_value('S', value) && std::abs(value - expected) < 0.01f)
            ++count;
    });
    return count;
}

size_t rising_extrusions(const std::string &text, const DynamicPrintConfig &config)
{
    size_t count = 0;
    GCodeReader reader;
    reader.apply_config(config);
    reader.parse_buffer(text, [&](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        if (line.cmd_is("G1") && line.extruding(self) && line.dist_XY(self) > EPSILON && line.dist_Z(self) > EPSILON)
            ++count;
    });
    return count;
}

TEST_CASE("File and machine lifecycle hooks preserve source order", "[GCode][GCodeExportWorkflows]")
{
    auto config = base_config();
    config.set_deserialize_strict({
        {"file_start_gcode", "M117 FILE_MARK\n"},
        {"machine_start_gcode", "M117 MACHINE_MARK\n"},
        {"machine_end_gcode", "M117 END_MARK\n"},
    });
    const auto out = fresh_export(config);
    const auto file = first_executable_command(out.text, config, "M117", "FILE_MARK");
    const auto machine = first_executable_command(out.text, config, "M117", "MACHINE_MARK");
    const auto body = out.text.find(";LAYER_CHANGE");
    const auto end = first_executable_command(out.text, config, "M117", "END_MARK");
    REQUIRE(file.has_value());
    REQUIRE(machine.has_value());
    REQUIRE(body != std::string::npos);
    REQUIRE(end.has_value());
    CHECK(*file < *machine);
    CHECK(*machine < body);
    CHECK(*end > body);
}

TEST_CASE("Custom start temperature is respected without a duplicate initial nozzle command", "[GCode][GCodeExportWorkflows]")
{
    auto config = base_config();
    config.set_deserialize_strict({
        {"machine_start_gcode", "M104 S177\nM140 S41\n"},
        {"nozzle_temperature_initial_layer", 211},
        {"hot_plate_temp_initial_layer", 61},
    });
    const auto out = fresh_export(config);
    REQUIRE(temperature_commands(out.text, config, "M104", 177.f) >= 1);
    CHECK(temperature_commands(out.text, config, "M104", 211.f) == 0);
    REQUIRE(temperature_commands(out.text, config, "M140", 41.f) >= 1);
    CHECK(temperature_commands(out.text, config, "M190", 61.f) == 0);
}

TEST_CASE("Custom layer placeholders expose layer index and height", "[GCode][GCodeExportWorkflows]")
{
    auto config = base_config();
    config.set_deserialize_strict({
        {"before_layer_change_gcode", "G92 E0\nM117 IDX-[layer_num]-Z-[layer_z]\n"},
        {"layer_change_gcode", "M117 CHANGED-[layer_num]\n"},
    });
    const auto out = fresh_export(config);
    const size_t layers = count_text(out.text, ";LAYER_CHANGE");
    REQUIRE(layers > 2);
    REQUIRE(executable_command_count(out.text, config, "M117", "IDX-") == layers);
    REQUIRE(executable_command_count(out.text, config, "M117", "-Z-") == layers);
    REQUIRE(executable_command_count(out.text, config, "M117", "CHANGED-") == layers);
    CHECK(out.text.find("M117 IDX-0-Z-0.2") != std::string::npos);
}

struct BedTemperatureFamily {
    const char *name;
    const char *initial_key;
    const char *ordinary_key;
    int          nominal_initial;
    int          nominal_ordinary;
    int          maximum_initial;
    int          maximum_ordinary;
};

const std::array<BedTemperatureFamily, 6> &bed_temperature_families()
{
    static const std::array<BedTemperatureFamily, 6> families = {{
        {"Cool Plate",         "cool_plate_temp_initial_layer",         "cool_plate_temp",         61, 51, 120, 300},
        {"Engineering Plate",  "eng_plate_temp_initial_layer",          "eng_plate_temp",          62, 52, 300, 300},
        {"High Temp Plate",    "hot_plate_temp_initial_layer",           "hot_plate_temp",          63, 53, 300, 300},
        {"Textured PEI Plate", "textured_plate_temp_initial_layer",      "textured_plate_temp",     64, 54, 300, 300},
        {"Textured Cool Plate","textured_cool_plate_temp_initial_layer", "textured_cool_plate_temp",65, 55, 120, 300},
        {"Supertack Plate",    "supertack_plate_temp_initial_layer",     "supertack_plate_temp",    66, 56, 120, 120},
    }};
    return families;
}

struct ParsedBedCommand {
    double temperature;
    size_t layer;
    bool   wait;
};

std::vector<ParsedBedCommand> parse_bed_commands(const std::string &text, const DynamicPrintConfig &config)
{
    std::vector<ParsedBedCommand> commands;
    size_t layer_markers = 0;
    GCodeReader reader;
    reader.apply_config(config);
    reader.parse_buffer(text, [&](GCodeReader &, const GCodeReader::GCodeLine &line) {
        if (line.raw().rfind(";LAYER_CHANGE", 0) == 0)
            ++layer_markers;
        const bool wait = line.cmd_is("M190");
        if (!wait && !line.cmd_is("M140"))
            return;
        float temperature = -1.f;
        REQUIRE(line.has_value('S', temperature));
        commands.push_back({temperature, layer_markers, wait});
    });
    return commands;
}

TEST_CASE("Every selected bed family emits its initial and later bed temperatures", "[GCode][GCodeExportWorkflows][BedTemperaturePipeline]")
{
    const auto &families = bed_temperature_families();
    const size_t family_index = GENERATE(size_t(0), size_t(1), size_t(2), size_t(3), size_t(4), size_t(5));
    const bool multiple_beds = GENERATE(false, true);
    const BedTemperatureFamily &bed = families[family_index];
    INFO("bed " << bed.name << ", support_multi_bed_types " << multiple_beds);

    DynamicPrintConfig config = base_config();
    for (const BedTemperatureFamily &family : families) {
        // Keep the multi-bed validation control valid for every selected family.
        config.set_deserialize_strict(family.initial_key, "70");
        config.set_deserialize_strict(family.ordinary_key, "50");
    }
    config.set_deserialize_strict({
        {"curr_bed_type", bed.name},
        {"support_multi_bed_types", multiple_beds},
        {"bed_temperature_formula", "by_first_filament"},
        {"machine_start_gcode", ""},
        {"nozzle_temperature_initial_layer", 208},
        {"nozzle_temperature", 202},
    });
    config.set_deserialize_strict(bed.initial_key, std::to_string(bed.nominal_initial));
    config.set_deserialize_strict(bed.ordinary_key, std::to_string(bed.nominal_ordinary));

    const auto exported = fresh_custom_mesh_export(config, make_cube(5., 5., 0.4));
    const auto commands = parse_bed_commands(exported.text, config);
    REQUIRE(commands.size() == 2);
    CHECK(commands[0].wait);
    CHECK(commands[0].layer == 0);
    CHECK_THAT(commands[0].temperature, Catch::Matchers::WithinAbs(bed.nominal_initial, 1e-4));
    CHECK_FALSE(commands[1].wait);
    CHECK(commands[1].layer == 2);
    CHECK_THAT(commands[1].temperature, Catch::Matchers::WithinAbs(bed.nominal_ordinary, 1e-4));
}

TEST_CASE("Bed heaters emit configured zero and maximum temperature targets", "[GCode][GCodeExportWorkflows][BedTemperaturePipeline]")
{
    const auto &families = bed_temperature_families();
    const size_t family_index = GENERATE(size_t(0), size_t(1), size_t(2), size_t(3), size_t(4), size_t(5));
    const bool initial_zero = GENERATE(false, true);
    const BedTemperatureFamily &bed = families[family_index];
    // An initial zero needs no heating command; a later zero turns the heated bed off.
    // High Temp initial temperature has no declared minimum.
    const int expected_initial = initial_zero ? 0 : bed.maximum_initial;
    const int expected_ordinary = initial_zero ? bed.maximum_ordinary : 0;
    INFO("bed " << bed.name << ", initial " << expected_initial << ", ordinary " << expected_ordinary);

    DynamicPrintConfig config = base_config();
    for (const BedTemperatureFamily &family : families) {
        config.set_deserialize_strict(family.initial_key, "70");
        config.set_deserialize_strict(family.ordinary_key, "50");
    }
    config.set_deserialize_strict({
        {"curr_bed_type", bed.name},
        {"support_multi_bed_types", initial_zero},
        {"bed_temperature_formula", "by_first_filament"},
        {"machine_start_gcode", ""},
        {"nozzle_temperature_initial_layer", 208},
        {"nozzle_temperature", 202},
    });
    config.set_deserialize_strict(bed.initial_key, std::to_string(expected_initial));
    config.set_deserialize_strict(bed.ordinary_key, std::to_string(expected_ordinary));

    const auto exported = fresh_custom_mesh_export(config, make_cube(5., 5., 0.4));
    const auto commands = parse_bed_commands(exported.text, config);
    REQUIRE(commands.size() == (initial_zero ? 1 : 2));
    if (!initial_zero) {
        CHECK(commands.front().wait);
        CHECK(commands.front().layer == 0);
        CHECK_THAT(commands.front().temperature, Catch::Matchers::WithinAbs(expected_initial, 1e-4));
    }
    CHECK_FALSE(commands.back().wait);
    CHECK(commands.back().layer == 2);
    CHECK_THAT(commands.back().temperature, Catch::Matchers::WithinAbs(expected_ordinary, 1e-4));
}



TEST_CASE("Scheduled G-code follows print mode and preserves expansion semantics",
          "[GCode][GCodeExportWorkflows][ScheduledLayerGCode]")
{
    const bool sequential = GENERATE(false, true);
    DynamicPrintConfig config = base_config();
    config.set_deserialize_strict({
        {"print_sequence", sequential ? "by object" : "by layer"},
        {"use_relative_e_distances", true},
        {"nozzle_temperature_initial_layer", 211},
        {"nozzle_temperature", 205},
        {"machine_pause_gcode", "M117 PAUSE-[nozzle_temperature_initial_layer]\n"},
        {"template_custom_gcode", "M117 TEMPLATE-[nozzle_temperature]\n"},
        {"enable_prime_tower", false},
        {"enable_support", false},
    });
    INFO("sequential printing " << sequential);

    const auto export_events = [&](bool scheduled) {
        Print print;
        Model model;
        Test::init_print({make_cube(5., 5., 0.8)}, print, model, config);
        if (scheduled) {
            CustomGCode::Info events;
            events.mode = CustomGCode::SingleExtruder;
            events.gcodes = {
                {0.2, CustomGCode::PausePrint, 1, "", "M117 UNUSED-PAUSE-ITEM\n"},
                {0.4, CustomGCode::Template, 1, "", "M117 UNUSED-TEMPLATE-ITEM\n"},
                {0.8, CustomGCode::Custom, 1, "", "M117 RAW-[nozzle_temperature]\n"},
            };
            model.plates_custom_gcodes[model.curr_plate_index] = events;
            CustomGCode::Info other_plate;
            other_plate.mode = CustomGCode::SingleExtruder;
            other_plate.gcodes = {{0.6, CustomGCode::Custom, 1, "", "M117 OTHER-PLATE\n"}};
            model.plates_custom_gcodes[model.curr_plate_index + 1] = other_plate;
            // The public helper applied the model before these events existed.
            print.apply(model, config);
            REQUIRE(print.model().get_curr_plate_custom_gcodes() == events);
        }
        require_valid(print);
        return export_print(print);
    };

    const ExportedPrint baseline = export_events(false);
    const ExportedPrint scheduled = export_events(true);
    require_same_material(motion_evidence(baseline.text, config), motion_evidence(scheduled.text, config));
    REQUIRE(executable_command_count(baseline.text, config, "M117", "") == 0);

    struct Event {
        std::string command;
        size_t layer;
        size_t prior_layer_extrusions;
    };
    std::vector<Event> events;
    std::array<size_t, 5> layer_extrusions{};
    size_t layer = 0;
    GCodeReader reader;
    reader.apply_config(config);
    reader.parse_buffer(scheduled.text, [&](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        if (line.raw().rfind(";LAYER_CHANGE", 0) == 0)
            ++layer;
        REQUIRE(layer < layer_extrusions.size());
        if (line.cmd_is("M117"))
            events.push_back({line.raw(), layer, layer_extrusions[layer]});
        if (line.extruding(self) && line.dist_XY(self) > EPSILON)
            ++layer_extrusions[layer];
    });
    REQUIRE(layer == 4);
    for (size_t i = 1; i <= layer; ++i)
        REQUIRE(layer_extrusions[i] > 0);

    if (sequential) {
        CHECK(events.empty());
    } else {
        REQUIRE(events.size() == 3);
        const std::array<size_t, 3> expected_layers = {1, 2, 4};
        const std::array<const char *, 3> expected_commands = {
            "M117 PAUSE-211", "M117 TEMPLATE-205", "M117 RAW-[nozzle_temperature]",
        };
        for (size_t i = 0; i < events.size(); ++i) {
            CHECK_THAT(events[i].command, Catch::Matchers::ContainsSubstring(expected_commands[i]));
            CHECK(events[i].layer == expected_layers[i]);
            CHECK(events[i].prior_layer_extrusions == 0);
        }
    }
}


TEST_CASE("Bed temperature formula emits the configured first-layer command", "[GCode][GCodeExportWorkflows]")
{
    auto config = base_config();
    config.set_deserialize_strict({
        {"nozzle_temperature_initial_layer", 208},
        {"nozzle_temperature", 202},
        {"curr_bed_type", "High Temp Plate"},
        {"hot_plate_temp_initial_layer", 57},
        {"hot_plate_temp", 47},
    });
    const auto out = fresh_export(config);
    REQUIRE(temperature_commands(out.text, config, "M190", 57.f) + temperature_commands(out.text, config, "M140", 57.f) > 0);
    REQUIRE(temperature_commands(out.text, config, "M109", 208.f) + temperature_commands(out.text, config, "M104", 208.f) > 0);
    CHECK(temperature_commands(out.text, config, "M109", 202.f) + temperature_commands(out.text, config, "M104", 202.f) > 0);
}

TEST_CASE("Power-loss recovery modes emit the documented Marlin transition", "[GCode][GCodeExportWorkflows][PowerLossRecovery]")
{
    const std::array<const char *, 3> modes = {
        "printer_configuration", "enable", "disable",
    };
    std::optional<MotionEvidence> reference_motion;

    for (const char *mode : modes) {
        INFO("power-loss recovery mode " << mode);
        DynamicPrintConfig config = base_config();
        config.set_deserialize_strict({
            { "layer_height",                 0.2 },
            { "initial_layer_print_height",   0.2 },
            { "enable_power_loss_recovery",   mode },
            { "gcode_flavor",                 "marlin2" },
        });
        const ExportedPrint exported = fresh_custom_mesh_export(config, make_cube(10., 10., 0.4));
        const MotionEvidence motion = motion_evidence(exported.text, config);
        REQUIRE(motion.extrusion_moves > 0);
        REQUIRE(count_text(exported.text, ";LAYER_CHANGE") == 2);

        if (reference_motion.has_value())
            require_same_material(*reference_motion, motion);
        else
            reference_motion = motion;

        struct Transition {
            float state;
            size_t layer;
            size_t preceding_extrusions;
        };
        std::vector<Transition> transitions;
        size_t layer = 0;
        size_t extrusions = 0;
        GCodeReader reader;
        reader.apply_config(config);
        reader.parse_buffer(exported.text, [&](GCodeReader &self, const GCodeReader::GCodeLine &line) {
            if (line.raw().rfind(";LAYER_CHANGE", 0) == 0)
                ++layer;
            if (line.cmd_is("M413")) {
                float state = -1.f;
                REQUIRE(line.has_value('S', state));
                transitions.push_back({state, layer, extrusions});
            }
            if (line.extruding(self) && line.dist_XY(self) > EPSILON)
                ++extrusions;
        });
        REQUIRE(layer == 2);
        REQUIRE(extrusions == motion.extrusion_moves);
        if (std::string(mode) == "printer_configuration") {
            CHECK(transitions.empty());
        } else {
            const bool enabled = std::string(mode) == "enable";
            REQUIRE(transitions.size() == (enabled ? 2 : 1));
            CHECK_THAT(transitions.front().state, Catch::Matchers::WithinAbs(enabled ? 1.f : 0.f, 1e-6));
            CHECK(transitions.front().layer == 2);
            CHECK(transitions.front().preceding_extrusions > 0);
            CHECK(transitions.front().preceding_extrusions < extrusions);
            if (enabled) {
                CHECK_THAT(transitions.back().state, Catch::Matchers::WithinAbs(0.f, 1e-6));
                CHECK(transitions.back().preceding_extrusions == extrusions);
            }
        }
    }
}

TEST_CASE("Bed temperature formula selects the used filament set", "[GCode][GCodeExportWorkflows][BedTemperatureFormula]")
{
    struct BedCommand {
        double temperature = 0.0;
        size_t layer = 0;
        bool   wait = false;
    };

    const std::array<const char *, 2> formulas = {"by_first_filament", "by_highest_temp"};
    for (const char *formula : formulas) {
        INFO("bed temperature formula " << formula);
        DynamicPrintConfig config = Test::multifilament_config(2, {
            {"layer_height", 0.2},
            {"initial_layer_print_height", 0.2},
            {"wall_loops", 1},
            {"top_shell_layers", 1},
            {"bottom_shell_layers", 1},
            {"sparse_infill_density", "25%"},
            {"outer_wall_filament_id", 1},
            {"inner_wall_filament_id", 1},
            {"sparse_infill_filament_id", 2},
            {"internal_solid_filament_id", 2},
            {"top_surface_filament_id", 2},
            {"bottom_surface_filament_id", 2},
            {"hot_plate_temp_initial_layer", "61,73"},
            {"hot_plate_temp", "41,53"},
            {"curr_bed_type", "High Temp Plate"},
            {"bed_temperature_formula", formula},
            {"first_layer_sequence_choice", "Customize"},
            {"first_layer_print_sequence", "1,2"},
            {"other_layers_sequence_choice", "Customize"},
            {"other_layers_print_sequence_nums", 1},
            {"other_layers_print_sequence", "2,10,1,2"},
            {"gcode_flavor", "marlin2"},
            {"gcode_comments", true},
            {"machine_start_gcode", ""},
            {"machine_end_gcode", ""},
            {"layer_change_gcode", "G92 E0\n"},
            {"skirt_loops", 0},
            {"brim_type", "no_brim"},
            {"raft_layers", 0},
            {"print_sequence", "by layer"},
        });
        const ExportedPrint exported = fresh_custom_mesh_export(config, make_cube(5., 5., 2.));
        REQUIRE(exported.text.size() < 4 * 1024 * 1024);

        std::vector<BedCommand> bed_commands;
        std::array<double, 2> tool_path{};
        std::array<int, 3> first_tool_by_layer = {-1, -1, -1};
        size_t layer_markers = 0;
        int current_tool = 0;
        GCodeReader reader;
        reader.apply_config(config);
        reader.parse_buffer(exported.text, [&](GCodeReader &self, const GCodeReader::GCodeLine &line) {
            if (line.raw().rfind(";LAYER_CHANGE", 0) == 0)
                ++layer_markers;
            if (line.cmd_is("T0"))
                current_tool = 0;
            else if (line.cmd_is("T1"))
                current_tool = 1;

            const bool is_m140 = line.cmd_is("M140");
            const bool is_m190 = line.cmd_is("M190");
            if (is_m140 || is_m190) {
                float value = 0.f;
                REQUIRE(line.has_value('S', value));
                bed_commands.push_back({double(value), layer_markers, is_m190});
            }
            if (line.extruding(self) && line.dist_XY(self) > EPSILON) {
                tool_path[size_t(current_tool)] += line.dist_XY(self);
                if (layer_markers == 1 && first_tool_by_layer[1] < 0)
                    first_tool_by_layer[1] = current_tool;
                if (layer_markers == 2 && first_tool_by_layer[2] < 0)
                    first_tool_by_layer[2] = current_tool;
            }
        });

        REQUIRE(layer_markers > 2);
        REQUIRE(tool_path[0] > 1.0);
        REQUIRE(tool_path[1] > 1.0);
        REQUIRE(first_tool_by_layer[1] == 0);
        REQUIRE(first_tool_by_layer[2] == 0);

        const bool highest = std::string(formula) == "by_highest_temp";
        const double expected_first = highest ? 73.0 : 61.0;
        const double expected_ordinary = highest ? 53.0 : 41.0;
        REQUIRE(bed_commands.size() == 2);
        CHECK_THAT(bed_commands[0].temperature, Catch::Matchers::WithinAbs(expected_first, 1e-4));
        CHECK(bed_commands[0].layer == 0);
        CHECK(bed_commands[0].wait);
        CHECK_THAT(bed_commands[1].temperature, Catch::Matchers::WithinAbs(expected_ordinary, 1e-4));
        CHECK(bed_commands[1].layer == 2);
        CHECK_FALSE(bed_commands[1].wait);
    }
}

TEST_CASE("Klipper omits Marlin machine envelope while preserving motion", "[GCode][GCodeExportWorkflows]")
{
    auto marlin = base_config();
    marlin.set_deserialize_strict({{"emit_machine_limits_to_gcode", true}, {"gcode_flavor", "marlin2"}});
    auto klipper = marlin;
    klipper.set_deserialize_strict({{"gcode_flavor", "klipper"}});
    const auto a = fresh_export(marlin);
    const auto b = fresh_export(klipper);
    REQUIRE(count_text(a.text, "M201 ") > 0);
    CHECK(count_text(b.text, "M201 ") == 0);
    require_same_material(motion_evidence(a.text, marlin), motion_evidence(b.text, klipper));
}

TEST_CASE("RepRap machine limits use millimeters per minute for speed and jerk", "[GCode][GCodeExportWorkflows]")
{
    auto config = base_config();
    config.set_deserialize_strict({
        {"gcode_flavor", "reprapfirmware"},
        {"emit_machine_limits_to_gcode", true},
        {"machine_max_speed_x", "11,11"},
        {"machine_max_speed_y", "12,12"},
        {"machine_max_speed_z", "3,3"},
        {"machine_max_speed_e", "4,4"},
        {"machine_max_jerk_x", "1,1"},
        {"machine_max_jerk_y", "2,2"},
        {"machine_max_jerk_z", "0.5,0.5"},
        {"machine_max_jerk_e", "0.75,0.75"},
    });
    const auto out = fresh_export(config);
    REQUIRE(count_text(out.text, "M203 X660 Y720 Z180 E240") > 0);
    REQUIRE(count_text(out.text, "M566 X60.00 Y120.00 Z30.00 E45.00") > 0);
}

TEST_CASE("Skirt and outer brim add distinct boundary metadata", "[GCode][GCodeExportWorkflows]")
{
    auto skirt = base_config();
    skirt.set_deserialize_strict({{"skirt_loops", 2}, {"skirt_distance", 3.0}});
    auto brim = base_config();
    brim.set_deserialize_strict({{"brim_type", "outer_only"}, {"brim_width", 4.0}});
    const auto s = fresh_export(skirt);
    const auto b = fresh_export(brim);
    const auto plain = fresh_export(base_config());
    REQUIRE(count_text(s.text, ";TYPE:Skirt") > 0);
    REQUIRE(count_text(b.text, ";TYPE:Brim") > 0);
    CHECK(motion_evidence(s.text, skirt).xy_path > motion_evidence(plain.text, base_config()).xy_path);
}

TEST_CASE("Support-enabled overhang export preserves typed toolpaths", "[GCode][GCodeExportWorkflows]")
{
    auto config = base_config();
    config.set_deserialize_strict({
                {"enable_support", true},
        {"support_type", "normal(auto)"},
        {"support_on_build_plate_only", false},
        {"support_base_pattern", "rectilinear"},
    });
    const auto out = fresh_custom_mesh_export(config, support_fixture());
    REQUIRE_FALSE(out.text.empty());
    REQUIRE(count_text(out.text, ";TYPE:Support") > 0);
    auto disabled = config;
    disabled.set_deserialize_strict({{"enable_support", false}});
    const auto without_support = fresh_custom_mesh_export(disabled, support_fixture());
    CHECK(count_text(without_support.text, ";TYPE:Support") == 0);
    REQUIRE(motion_evidence(out.text, config).extrusion_moves > 0);
}

TEST_CASE("Wipe while retracting adds wipe travel without changing deposited material", "[GCode][GCodeExportWorkflows][GCodeWipeMotion]")
{
    auto off = base_config();
    off.set_deserialize_strict({
        {"wipe", false},
        {"wipe_distance", 2.0},
        {"retraction_length", 1.2},
        {"retraction_minimum_travel", 0.0},
        {"retraction_speed", 30.0},
        {"deretraction_speed", 30.0},
        {"retract_when_changing_layer", false},
        {"retract_before_wipe", "0%"},
        {"retract_after_wipe", "0%"},
        {"role_based_wipe_speed", false},
        {"wipe_speed", 100.0},
        {"wipe_on_loops", false},
        {"wipe_before_external_loop", false},
        {"wipe_inward", false},
        {"wipe_inward_distance", "0%"},
        {"use_firmware_retraction", false},
        {"z_hop", 0.0},
        {"enable_arc_fitting", false},
        {"resolution", 0.0},
        {"gcode_comments", true},
    });
    auto on = off;
    on.set_deserialize_strict({{"wipe", true}});
    const auto a = fresh_export(off);
    const auto b = fresh_export(on);
    const MotionEvidence off_material = motion_evidence(a.text, off);
    const MotionEvidence on_material = motion_evidence(b.text, on);
    REQUIRE(off_material.extrusion_moves > 0);
    require_same_material(off_material, on_material);

    const NegativeXYMotionEvidence off_wipe = negative_xy_motion_evidence(a.text, off);
    const NegativeXYMotionEvidence on_wipe = negative_xy_motion_evidence(b.text, on);
    REQUIRE(off_wipe.finite);
    REQUIRE(on_wipe.finite);
    CHECK(off_wipe.negative_e_xy_moves == 0);
    CHECK(on_wipe.negative_e_xy_moves > 0);
}

TEST_CASE("Firmware retraction changes emitted retraction commands", "[GCode][GCodeExportWorkflows]")
{
    auto regular = base_config();
    regular.set_deserialize_strict({{"use_firmware_retraction", false}, {"retraction_length", 1.1}});
    auto firmware = regular;
    firmware.set_deserialize_strict({{"use_firmware_retraction", true}, {"retraction_length", 0.0}, {"z_hop", 1.0}});
    const auto a = fresh_export(regular, true);
    const auto b = fresh_export(firmware, true);
    REQUIRE(count_text(a.text, "G1 E-") > 0);
    REQUIRE(count_text(b.text, "G10") > 0);
    CHECK(count_text(b.text, "G1 E-") < count_text(a.text, "G1 E-"));
    CHECK(motion_evidence(b.text, firmware).z_only_moves > 0);
}

TEST_CASE("Retraction travel threshold partitions short and long moves", "[GCode][GCodeExportWorkflows]")
{
    auto low = base_config();
    low.set_deserialize_strict({{"retraction_minimum_travel", 0.0}});
    auto high = low;
    high.set_deserialize_strict({{"retraction_minimum_travel", 1000.0}});
    const auto a = fresh_export(low, true);
    const auto b = fresh_export(high, true);
    REQUIRE(motion_evidence(a.text, low).extrusion_moves > 0);
    REQUIRE(count_text(a.text, "G1 E-") > count_text(b.text, "G1 E-"));
    CHECK(motion_evidence(a.text, low).filament > 0.0);
}

TEST_CASE("Timelapse custom G-code is emitted at layer boundaries", "[GCode][GCodeExportWorkflows]")
{
    auto config = base_config();
    config.set_deserialize_strict({{"time_lapse_gcode", "M117 SNAP-[layer_num]\n"}, {"timelapse_type", "0"}});
    const auto out = fresh_export(config);
    REQUIRE(executable_command_count(out.text, config, "M117", "SNAP-") > 0);
    CHECK(executable_command_count(out.text, config, "M117", "SNAP-") == count_text(out.text, ";LAYER_CHANGE"));
}

TEST_CASE("Spiral vase raises Z during deposited perimeter motion", "[GCode][GCodeExportWorkflows]")
{
    auto config = base_config();
    config.set_deserialize_strict({
        {"spiral_mode", true},
        {"wall_loops", 1},
        {"top_shell_layers", 0},
        {"bottom_shell_layers", 0},
        {"sparse_infill_density", "0%"},
    });
    const auto out = fresh_export(config);
    const auto ordinary = fresh_export(base_config());
    REQUIRE(motion_evidence(out.text, config).extrusion_moves > 0);
    CHECK(count_text(out.text, ";LAYER_CHANGE") >= 1);
    CHECK(rising_extrusions(out.text, config) > 0);
    CHECK(rising_extrusions(ordinary.text, base_config()) == 0);
}

struct SpiralLayerEvidence {
    size_t perimeter_extrusions = 0;
    size_t spiral_z_extrusions = 0;
    size_t flat_extrusions = 0;
    size_t post_spiral_flat_extrusions = 0;
    double filament_length = 0.0;
    double xy_path_length = 0.0;
    double spiral_filament_length = 0.0;
    double spiral_xy_path_length = 0.0;
    double flat_filament_length = 0.0;
    double post_spiral_flat_filament_length = 0.0;
    std::vector<Vec2d> spiral_xy_endpoints;
    bool saw_spiral = false;
};

struct SpiralSettingsEvidence {
    std::vector<SpiralLayerEvidence> layers;
    size_t perimeter_extrusions = 0;
    size_t spiral_z_extrusions = 0;
    size_t negative_z_extrusions = 0;
    bool finite = true;
    size_t parsed_lines = 0;
};

std::string_view trim_spiral_comment(std::string_view comment)
{
    while (!comment.empty() && std::isspace(static_cast<unsigned char>(comment.front())))
        comment.remove_prefix(1);
    while (!comment.empty() && std::isspace(static_cast<unsigned char>(comment.back())))
        comment.remove_suffix(1);
    return comment;
}

bool spiral_perimeter_role(std::string_view role)
{
    const std::string_view trimmed = trim_spiral_comment(role);
    return trimmed == "Outer wall" || trimmed == "Inner wall" ||
           trimmed == "External perimeter" || trimmed == "Internal perimeter";
}

SpiralSettingsEvidence parse_spiral_settings_output(const std::string &gcode,
                                                     const DynamicPrintConfig &config)
{
    constexpr size_t max_gcode_bytes = 4u * 1024u * 1024u;
    constexpr size_t max_gcode_lines = 100000u;
    REQUIRE(gcode.size() <= max_gcode_bytes);
    REQUIRE(static_cast<size_t>(std::count(gcode.begin(), gcode.end(), '\n')) <= max_gcode_lines);

    SpiralSettingsEvidence evidence;
    GCodeReader reader;
    reader.apply_config(config);
    int current_layer = -1;
    std::string active_role;
    reader.parse_buffer(gcode, [&evidence, &current_layer, &active_role](GCodeReader &self,
                                                                          const GCodeReader::GCodeLine &line) {
        ++evidence.parsed_lines;
        const std::string_view comment = trim_spiral_comment(line.comment());
        if (comment.rfind("LAYER_CHANGE", 0) == 0) {
            ++current_layer;
            evidence.layers.emplace_back();
            // GCode emits role tags only when the role changes.  Keep the
            // previous role across layer markers until a new TYPE/FEATURE
            // tag is encountered.
        }
        if (comment.rfind("TYPE:", 0) == 0)
            active_role.assign(trim_spiral_comment(comment.substr(5)));
        else if (comment.rfind("FEATURE:", 0) == 0)
            active_role.assign(trim_spiral_comment(comment.substr(8)));

        const auto finite = [](float value) { return std::isfinite(static_cast<double>(value)); };
        const bool finite_state = finite(self.x()) && finite(self.y()) && finite(self.z()) && finite(self.e()) &&
                                  (!line.has_x() || finite(line.x())) && (!line.has_y() || finite(line.y())) &&
                                  (!line.has_z() || finite(line.z())) && (!line.has_e() || finite(line.e()));
        if (!finite_state)
            evidence.finite = false;
        if (current_layer < 0 || current_layer >= static_cast<int>(evidence.layers.size()) ||
            !spiral_perimeter_role(active_role) || !finite_state || !line.extruding(self) ||
            line.dist_XY(self) <= EPSILON)
            return;

        SpiralLayerEvidence &layer = evidence.layers[static_cast<size_t>(current_layer)];
        ++layer.perimeter_extrusions;
        ++evidence.perimeter_extrusions;
        const double de = line.dist_E(self);
        const double dxy = line.dist_XY(self);
        const double dz = line.dist_Z(self);
        if (!std::isfinite(de) || !std::isfinite(dxy) || !std::isfinite(dz)) {
            evidence.finite = false;
            return;
        }
        if (dz < -EPSILON) {
            ++evidence.negative_z_extrusions;
            return;
        }
        const bool spiral_move = dz > EPSILON;
        layer.filament_length += de;
        layer.xy_path_length += dxy;
        if (spiral_move) {
            ++layer.spiral_z_extrusions;
            ++evidence.spiral_z_extrusions;
            layer.spiral_filament_length += de;
            layer.spiral_xy_path_length += dxy;
            layer.spiral_xy_endpoints.emplace_back(line.new_X(self), line.new_Y(self));
            layer.saw_spiral = true;
        } else {
            ++layer.flat_extrusions;
            layer.flat_filament_length += de;
            if (layer.saw_spiral) {
                ++layer.post_spiral_flat_extrusions;
                layer.post_spiral_flat_filament_length += de;
            }
        }
    });
    REQUIRE(evidence.parsed_lines <= max_gcode_lines);
    REQUIRE(evidence.finite);
    REQUIRE(evidence.negative_z_extrusions == 0);
    return evidence;
}

bool spiral_xy_geometry_equal(const SpiralSettingsEvidence &candidate,
                               const SpiralSettingsEvidence &baseline)
{
    if (candidate.layers.size() != baseline.layers.size())
        return false;
    for (size_t layer_idx = 0; layer_idx < baseline.layers.size(); ++layer_idx) {
        const auto &candidate_points = candidate.layers[layer_idx].spiral_xy_endpoints;
        const auto &baseline_points = baseline.layers[layer_idx].spiral_xy_endpoints;
        if (candidate_points.size() != baseline_points.size())
            return false;
        for (size_t point_idx = 0; point_idx < baseline_points.size(); ++point_idx)
            if (std::abs(candidate_points[point_idx].x() - baseline_points[point_idx].x()) > 1e-4 ||
                std::abs(candidate_points[point_idx].y() - baseline_points[point_idx].y()) > 1e-4)
                return false;
    }
    return true;
}

DynamicPrintConfig spiral_settings_config(bool smooth, const char *max_xy_smoothing,
                                          double starting_flow, double finishing_flow)
{
    DynamicPrintConfig config = base_config();
    config.set_deserialize_strict({
        { "wall_loops",                  1 },
        { "top_shell_layers",            0 },
        { "bottom_shell_layers",         2 },
        { "nozzle_diameter",             0.4 },
        { "top_shell_thickness",         0.0 },
        { "bottom_shell_thickness",      0.0 },
        { "sparse_infill_density",       "0%" },
        { "spiral_mode",                 true },
        { "spiral_mode_smooth",          smooth },
        { "spiral_mode_max_xy_smoothing", max_xy_smoothing },
        { "spiral_starting_flow_ratio",  starting_flow },
        { "spiral_finishing_flow_ratio", finishing_flow },
        { "use_relative_e_distances",    true },
    });
    return config;
}

SpiralSettingsEvidence export_spiral_settings(const DynamicPrintConfig &config)
{
    TriangleMesh shape = Test::mesh(Test::TestMesh::slopy_cube);
    shape.scale(Vec3f(1.0f, 1.0f, 0.2f));
    const ExportedPrint exported = fresh_custom_mesh_export(config, std::move(shape));
    return parse_spiral_settings_output(exported.text, config);
}

size_t first_spiral_settings_layer(const SpiralSettingsEvidence &evidence)
{
    for (size_t index = 0; index < evidence.layers.size(); ++index)
        if (evidence.layers[index].spiral_z_extrusions > 0)
            return index;
    return evidence.layers.size();
}

size_t last_spiral_settings_layer(const SpiralSettingsEvidence &evidence)
{
    for (size_t index = evidence.layers.size(); index > 0; --index)
        if (evidence.layers[index - 1].spiral_z_extrusions > 0)
            return index - 1;
    return evidence.layers.size();
}

const SpiralLayerEvidence *final_flat_transition(const SpiralSettingsEvidence &evidence,
                                                 size_t last_spiral_layer)
{
    if (last_spiral_layer >= evidence.layers.size())
        return nullptr;
    for (size_t index = evidence.layers.size(); index > last_spiral_layer + 1; --index) {
        const SpiralLayerEvidence &candidate = evidence.layers[index - 1];
        if (candidate.flat_extrusions > 0)
            return &candidate;
    }
    const SpiralLayerEvidence &last = evidence.layers[last_spiral_layer];
    return last.post_spiral_flat_extrusions > 0 ? &last : nullptr;
}

TEST_CASE("Spiral settings preserve smooth geometry and flow transitions", "[GCode][GCodeExportWorkflows][SpiralSettings]")
{
    // Seven bounded public exports. The smooth-off output is also the flow baseline.
    const DynamicPrintConfig smooth_off_config = spiral_settings_config(false, "0%", 0.0, 0.0);
    const DynamicPrintConfig smooth_zero_config = spiral_settings_config(true, "0%", 0.0, 0.0);
    const DynamicPrintConfig smooth_high_config = spiral_settings_config(true, "1000%", 0.0, 0.0);
    const DynamicPrintConfig flow_start_config = spiral_settings_config(false, "0%", 0.5, 0.0);
    const DynamicPrintConfig flow_start_max_config = spiral_settings_config(false, "0%", 1.0, 0.0);
    const DynamicPrintConfig flow_finish_config = spiral_settings_config(false, "0%", 0.0, 0.5);
    const DynamicPrintConfig flow_finish_max_config = spiral_settings_config(false, "0%", 0.0, 1.0);

    const SpiralSettingsEvidence smooth_off = export_spiral_settings(smooth_off_config);
    const SpiralSettingsEvidence smooth_zero = export_spiral_settings(smooth_zero_config);
    const SpiralSettingsEvidence smooth_high = export_spiral_settings(smooth_high_config);
    const SpiralSettingsEvidence flow_baseline = smooth_off;
    const SpiralSettingsEvidence flow_start = export_spiral_settings(flow_start_config);
    const SpiralSettingsEvidence flow_start_max = export_spiral_settings(flow_start_max_config);
    const SpiralSettingsEvidence flow_finish = export_spiral_settings(flow_finish_config);
    const SpiralSettingsEvidence flow_finish_max = export_spiral_settings(flow_finish_max_config);

    REQUIRE(smooth_off.layers.size() >= 3);
    REQUIRE(smooth_off.perimeter_extrusions > 0);
    REQUIRE(smooth_off.spiral_z_extrusions > 0);
    REQUIRE(smooth_high.spiral_z_extrusions > 0);

    // max_xy_smoothing=0 is an explicit no-op control. Effective smoothing is
    // judged geometrically; it may change path length and segment count.
    CHECK(smooth_zero.perimeter_extrusions == smooth_off.perimeter_extrusions);
    CHECK(smooth_zero.spiral_z_extrusions == smooth_off.spiral_z_extrusions);
    bool zero_geometry_equal = smooth_zero.layers.size() == smooth_off.layers.size() &&
                               smooth_zero.perimeter_extrusions == smooth_off.perimeter_extrusions &&
                               smooth_zero.spiral_z_extrusions == smooth_off.spiral_z_extrusions;
    if (zero_geometry_equal) {
        for (size_t index = 0; index < smooth_off.layers.size(); ++index) {
            zero_geometry_equal = zero_geometry_equal &&
                std::abs(smooth_zero.layers[index].xy_path_length - smooth_off.layers[index].xy_path_length) <= 1e-4 &&
                std::abs(smooth_zero.layers[index].spiral_filament_length - smooth_off.layers[index].spiral_filament_length) <= 1e-4;
            if (!zero_geometry_equal)
                break;
        }
    }
    zero_geometry_equal = zero_geometry_equal && spiral_xy_geometry_equal(smooth_zero, smooth_off);
    CHECK(zero_geometry_equal);
    const bool geometry_changed = smooth_high.layers.size() != smooth_off.layers.size() ||
                                  smooth_high.perimeter_extrusions != smooth_off.perimeter_extrusions ||
                                  !spiral_xy_geometry_equal(smooth_high, smooth_off);
    bool path_changed = false;
    for (size_t index = 0; index < std::min(smooth_high.layers.size(), smooth_off.layers.size()); ++index)
        path_changed = path_changed ||
            std::abs(smooth_high.layers[index].xy_path_length - smooth_off.layers[index].xy_path_length) > 1e-3;
    const bool smoothing_witness = geometry_changed || path_changed;
    CHECK(smoothing_witness);

    const size_t baseline_first_index = first_spiral_settings_layer(flow_baseline);
    const size_t start_first_index = first_spiral_settings_layer(flow_start);
    const size_t start_max_first_index = first_spiral_settings_layer(flow_start_max);
    const size_t baseline_last_index = last_spiral_settings_layer(flow_baseline);
    const size_t finish_last_index = last_spiral_settings_layer(flow_finish);
    const size_t finish_max_last_index = last_spiral_settings_layer(flow_finish_max);
    REQUIRE(baseline_first_index < flow_baseline.layers.size());
    REQUIRE(start_first_index < flow_start.layers.size());
    REQUIRE(start_max_first_index < flow_start_max.layers.size());
    REQUIRE(baseline_last_index < flow_baseline.layers.size());
    REQUIRE(finish_last_index < flow_finish.layers.size());
    REQUIRE(finish_max_last_index < flow_finish_max.layers.size());

    const SpiralLayerEvidence &base_first = flow_baseline.layers[baseline_first_index];
    const SpiralLayerEvidence &start_first = flow_start.layers[start_first_index];
    const SpiralLayerEvidence &start_max_first = flow_start_max.layers[start_max_first_index];
    const SpiralLayerEvidence &base_last = flow_baseline.layers[baseline_last_index];
    const SpiralLayerEvidence &finish_last = flow_finish.layers[finish_last_index];
    const SpiralLayerEvidence &finish_max_last = flow_finish_max.layers[finish_max_last_index];
    REQUIRE(base_first.perimeter_extrusions > 0);
    REQUIRE(base_last.perimeter_extrusions > 0);

    // The configured ratios are the endpoints of the per-move interpolation in
    // SpiralVase; retain the 0.5 samples and require more material at the 1.0 end.
    // Finishing flow changes only the appended flat transition loop; its rising
    // final loop remains unchanged and is checked independently.
    CHECK(finish_last.spiral_z_extrusions == base_last.spiral_z_extrusions);
    CHECK_THAT(finish_last.spiral_filament_length,
               Catch::Matchers::WithinAbs(base_last.spiral_filament_length, 1e-4));
    CHECK_THAT(finish_last.spiral_xy_path_length,
               Catch::Matchers::WithinAbs(base_last.spiral_xy_path_length, 1e-4));
    CHECK(finish_max_last.spiral_z_extrusions == base_last.spiral_z_extrusions);
    CHECK_THAT(finish_max_last.spiral_filament_length,
               Catch::Matchers::WithinAbs(base_last.spiral_filament_length, 1e-4));
    CHECK_THAT(finish_max_last.spiral_xy_path_length,
               Catch::Matchers::WithinAbs(base_last.spiral_xy_path_length, 1e-4));
    const SpiralLayerEvidence *base_transition = final_flat_transition(flow_baseline, baseline_last_index);
    const SpiralLayerEvidence *finish_transition = final_flat_transition(flow_finish, finish_last_index);
    const SpiralLayerEvidence *finish_max_transition =
        final_flat_transition(flow_finish_max, finish_max_last_index);
    REQUIRE(base_transition != nullptr);
    REQUIRE(finish_transition != nullptr);
    REQUIRE(finish_max_transition != nullptr);
    CHECK(start_first.filament_length > base_first.filament_length + 1e-4);
    CHECK(start_max_first.filament_length > start_first.filament_length + 1e-4);
    CHECK(finish_transition->flat_filament_length >
          base_transition->flat_filament_length + 1e-4);
    CHECK(finish_max_transition->flat_filament_length >
          finish_transition->flat_filament_length + 1e-4);
}

TEST_CASE("Sequential spiral vase preserves rising perimeter flow and by-layer geometry",
          "[GCode][GCodeExportWorkflows][SpiralSequential]")
{
    const DynamicPrintConfig by_layer_config = spiral_settings_config(true, "1000%", 0.5, 0.5);
    const SpiralSettingsEvidence by_layer = export_spiral_settings(by_layer_config);

    DynamicPrintConfig by_object_config = by_layer_config;
    by_object_config.set_deserialize_strict({{"print_sequence", "by object"}});
    const SpiralSettingsEvidence by_object = export_spiral_settings(by_object_config);

    DynamicPrintConfig no_smoothing_config = by_object_config;
    no_smoothing_config.set_deserialize_strict({{"spiral_mode_max_xy_smoothing", "0%"}});
    const SpiralSettingsEvidence no_smoothing = export_spiral_settings(no_smoothing_config);

    DynamicPrintConfig flow_off_config = by_object_config;
    flow_off_config.set_deserialize_strict({
        {"spiral_starting_flow_ratio", 0.0},
        {"spiral_finishing_flow_ratio", 0.0},
    });
    const SpiralSettingsEvidence flow_off = export_spiral_settings(flow_off_config);

    REQUIRE(by_layer.layers.size() >= 3);
    REQUIRE(by_object.layers.size() == by_layer.layers.size());
    REQUIRE(no_smoothing.layers.size() == by_object.layers.size());
    REQUIRE(no_smoothing.perimeter_extrusions > 0);
    REQUIRE(no_smoothing.spiral_z_extrusions > 0);
    REQUIRE(by_layer.spiral_z_extrusions > 0);
    REQUIRE(by_object.perimeter_extrusions > 0);
    REQUIRE(by_object.spiral_z_extrusions > 0);
    REQUIRE(flow_off.spiral_z_extrusions > 0);
    REQUIRE(flow_off.layers.size() == by_object.layers.size());
    for (size_t layer_idx = 0; layer_idx < by_layer.layers.size(); ++layer_idx) {
        const SpiralLayerEvidence &layer = by_layer.layers[layer_idx];
        const SpiralLayerEvidence &sequential = by_object.layers[layer_idx];
        CHECK(sequential.perimeter_extrusions == layer.perimeter_extrusions);
        CHECK(sequential.spiral_z_extrusions == layer.spiral_z_extrusions);
        CHECK_THAT(sequential.filament_length,
                   Catch::Matchers::WithinAbs(layer.filament_length, 1e-4));
        CHECK_THAT(sequential.xy_path_length,
                   Catch::Matchers::WithinAbs(layer.xy_path_length, 1e-4));
        CHECK_THAT(sequential.spiral_filament_length,
                   Catch::Matchers::WithinAbs(layer.spiral_filament_length, 1e-4));
        CHECK_THAT(sequential.spiral_xy_path_length,
                   Catch::Matchers::WithinAbs(layer.spiral_xy_path_length, 1e-4));
    }
    CHECK(spiral_xy_geometry_equal(by_object, by_layer));

    bool smoothing_changed = !spiral_xy_geometry_equal(by_object, no_smoothing);
    for (size_t layer_idx = 0; layer_idx < by_object.layers.size() && !smoothing_changed; ++layer_idx)
        smoothing_changed = std::abs(by_object.layers[layer_idx].spiral_xy_path_length -
                                     no_smoothing.layers[layer_idx].spiral_xy_path_length) > 1e-3;
    CHECK(smoothing_changed);

    const size_t flow_first_layer = first_spiral_settings_layer(by_object);
    const size_t no_flow_first_layer = first_spiral_settings_layer(flow_off);
    REQUIRE(flow_first_layer < by_object.layers.size());
    REQUIRE(no_flow_first_layer < flow_off.layers.size());
    REQUIRE(by_object.layers[flow_first_layer].spiral_filament_length > 0.0);
    REQUIRE(flow_off.layers[no_flow_first_layer].spiral_filament_length > 0.0);
    CHECK(by_object.layers[flow_first_layer].filament_length >
          flow_off.layers[no_flow_first_layer].filament_length + 1e-4);

    const size_t flow_last_layer = last_spiral_settings_layer(by_object);
    const size_t no_flow_last_layer = last_spiral_settings_layer(flow_off);
    REQUIRE(flow_last_layer < by_object.layers.size());
    REQUIRE(no_flow_last_layer < flow_off.layers.size());
    const SpiralLayerEvidence *flow_transition = final_flat_transition(by_object, flow_last_layer);
    const SpiralLayerEvidence *no_flow_transition = final_flat_transition(flow_off, no_flow_last_layer);
    REQUIRE(flow_transition != nullptr);
    REQUIRE(no_flow_transition != nullptr);
    CHECK(flow_transition->flat_filament_length > no_flow_transition->flat_filament_length + 1e-4);
}

TEST_CASE("Arc fitting converts eligible perimeter motion to arcs", "[GCode][GCodeExportWorkflows]")
{
    auto off = base_config();
    off.set_deserialize_strict({{"enable_arc_fitting", false}});
    auto on = off;
    on.set_deserialize_strict({{"enable_arc_fitting", true}});
    const auto a = fresh_custom_mesh_export(off, make_cylinder(10., 4., PI / 90.));
    const auto b = fresh_custom_mesh_export(on, make_cylinder(10., 4., PI / 90.));
    REQUIRE(motion_evidence(a.text, off).extrusion_moves > 0);
    REQUIRE(motion_evidence(b.text, on).extrusion_moves > 0);
    const size_t arcs_off = count_text(a.text, "G2 ") + count_text(a.text, "G3 ");
    const size_t arcs_on = count_text(b.text, "G2 ") + count_text(b.text, "G3 ");
    REQUIRE(arcs_on > 0);
    CHECK(arcs_on > arcs_off);
}

TEST_CASE("Layer-change code is ordered after the layer marker and before next extrusion", "[GCode][GCodeExportWorkflows]")
{
    auto config = base_config();
    config.set_deserialize_strict({{"layer_change_gcode", "G92 E0\nM117 ORDER-[layer_num]\n"}});
    const auto out = fresh_export(config);
    const size_t marker = out.text.find(";LAYER_CHANGE");
    const size_t hook = out.text.find("M117 ORDER-");
    REQUIRE(marker != std::string::npos);
    REQUIRE(hook != std::string::npos);
    CHECK(hook > marker);
    const auto motion = out.text.find("G1", hook);
    REQUIRE(motion != std::string::npos);
    CHECK(hook < motion);
}

TEST_CASE("Input shaping emission follows the configured firmware gate", "[GCode][GCodeExportWorkflows]")
{
    auto marlin = base_config();
    marlin.set_deserialize_strict({{"emit_machine_limits_to_gcode", true}, {"input_shaping_emit", true}, {"input_shaping_freq_x", 45.0}, {"input_shaping_freq_y", 50.0}});
    auto legacy = marlin;
    legacy.set_deserialize_strict({{"gcode_flavor", "marlin"}});
    const auto a = fresh_export(marlin);
    const auto b = fresh_export(legacy);
    REQUIRE(motion_evidence(a.text, marlin).extrusion_moves > 0);
    REQUIRE(executable_command_count(a.text, marlin, "M593", "") > 0);
    CHECK(executable_command_count(b.text, legacy, "M593", "") == 0);
}


namespace print_order_contracts {

struct XY {
    double x;
    double y;
};

constexpr size_t kInstanceCount = 7;
constexpr size_t kMaxOutputBytes = 4 * 1024 * 1024;

struct OrderRun {
    std::array<XY, kInstanceCount> centers{};
    std::vector<std::vector<size_t>> layers;
};

std::array<XY, kInstanceCount> fixture_centers()
{
    // Three deliberately separated rows keep every 10 mm instance comfortably
    // inside the default bed while exercising both ordering strategies.
    return {{
        {30., 171.}, {25., 20.}, {65., 109.}, {20., 109.},
        {140., 172.}, {100., 109.}, {180., 109.},
    }};
}

DynamicPrintConfig print_order_config(const char *serialized_mode)
{
    DynamicPrintConfig config = base_config();
    config.set_deserialize_strict({
        {"print_order", serialized_mode},
        {"print_sequence", "by layer"},
        {"gcode_label_objects", true},
        // Export assigns distinct copy IDs while defining cancellable objects.
        {"exclude_object", true},
        {"enable_prime_tower", false},
        {"wipe_tower_x", 0.0},
        {"wipe_tower_y", 0.0},
        {"skirt_loops", 0},
        {"brim_type", "no_brim"},
        {"raft_layers", 0},
    });
    return config;
}

std::vector<std::vector<size_t>> parse_layer_object_copies(const std::string &gcode)
{
    std::vector<std::vector<size_t>> layers;
    size_t current_layer = std::numeric_limits<size_t>::max();
    size_t line_start = 0;
    while (line_start <= gcode.size()) {
        const size_t line_end = gcode.find('\n', line_start);
        const size_t line_length = line_end == std::string::npos ? gcode.size() - line_start : line_end - line_start;
        const std::string_view line(gcode.data() + line_start, line_length);
        if (line.rfind(";LAYER_CHANGE", 0) == 0) {
            layers.emplace_back();
            current_layer = layers.size() - 1;
        } else if (current_layer != std::numeric_limits<size_t>::max()) {
            const size_t object_marker = line.find("; printing object ");
            const size_t copy_marker = line.find(" copy ", object_marker);
            if (object_marker != std::string_view::npos && copy_marker != std::string_view::npos) {
                size_t value = 0;
                bool have_digit = false;
                bool valid = true;
                for (size_t pos = copy_marker + 6; pos < line.size(); ++pos) {
                    const unsigned char ch = static_cast<unsigned char>(line[pos]);
                    if (ch < '0' || ch > '9')
                        break;
                    have_digit = true;
                    const size_t digit = size_t(ch - '0');
                    if (value > (std::numeric_limits<size_t>::max() - digit) / 10) {
                        valid = false;
                        break;
                    }
                    value = value * 10 + digit;
                }
                if (have_digit && valid)
                    layers[current_layer].push_back(value);
            }
        }
        if (line_end == std::string::npos)
            break;
        line_start = line_end + 1;
    }
    return layers;
}

OrderRun export_order_fixture(const char *serialized_mode)
{
    const auto centers = fixture_centers();
    DynamicPrintConfig config = print_order_config(serialized_mode);
    Model model;
    Print print;
    ModelObject *object = model.add_object();
    object->name = "print-order-instance.stl";
    object->add_volume(make_cube(10., 10., 0.8));
    for (size_t index = 0; index < centers.size(); ++index) {
        ModelInstance *instance = object->add_instance();
        instance->set_offset(Vec3d(centers[index].x, centers[index].y, 0.));
        instance->arrange_order = static_cast<int>(index + 1);
    }
    object->ensure_on_bed();
    print.auto_assign_extruders(object);
    print.apply(model, config);
    REQUIRE(print.config().opt_serialize("print_order") == serialized_mode);
    require_valid(print);

    OrderRun result;
    result.centers = centers;
    const std::string text = export_print(print).text;
    REQUIRE(text.size() < kMaxOutputBytes);
    result.layers = parse_layer_object_copies(text);
    return result;
}

void require_registered_print_order_domain()
{
    const ConfigOptionDef *definition = print_config_def.get("print_order");
    REQUIRE(definition != nullptr);
    const std::set<std::string> registered(definition->enum_values.begin(), definition->enum_values.end());
    const std::set<std::string> expected{"default", "as_obj_list", "best_of", "snake"};
    REQUIRE(registered == expected);
}

double cycle_cost(const std::vector<size_t> &order, const std::array<XY, kInstanceCount> &centers)
{
    double total = 0.;
    for (size_t index = 0; index < order.size(); ++index) {
        const size_t next = (index + 1) % order.size();
        const XY &from = centers[order[index]];
        const XY &to = centers[order[next]];
        total += std::hypot(from.x - to.x, from.y - to.y);
    }
    return total;
}

} // namespace print_order_contracts

TEST_CASE("Print order modes preserve complete per-layer instance visits",
          "[GCode][PrintOrder][GCodeExportWorkflows]")
{
    using namespace print_order_contracts;
    require_registered_print_order_domain();
    const std::array<const char *, 4> modes{{"default", "as_obj_list", "best_of", "snake"}};
    std::array<OrderRun, 4> runs;

    for (size_t mode_index = 0; mode_index < modes.size(); ++mode_index) {
        INFO("print_order=" << modes[mode_index]);
        runs[mode_index] = export_order_fixture(modes[mode_index]);
        REQUIRE(runs[mode_index].layers.size() >= 2);
        for (const std::vector<size_t> &layer_order : runs[mode_index].layers) {
            REQUIRE(layer_order.size() == kInstanceCount);
            std::vector<size_t> sorted = layer_order;
            for (size_t id : sorted)
                REQUIRE(id < kInstanceCount);
            std::sort(sorted.begin(), sorted.end());
            for (size_t expected = 0; expected < kInstanceCount; ++expected)
                CHECK(sorted[expected] == expected);

            if (mode_index == 1) {
                // AsObjectList is the explicit predictable-order contract.
                for (size_t expected = 0; expected < kInstanceCount; ++expected)
                    CHECK(layer_order[expected] == expected);
            }
        }
    }

    REQUIRE(runs[0].layers.size() == runs[2].layers.size());
    REQUIRE(runs[0].layers.size() == runs[3].layers.size());

    const double default_first_cost = cycle_cost(runs[0].layers.front(), runs[0].centers);
    const double snake_first_cost = cycle_cost(runs[3].layers.front(), runs[3].centers);
    INFO("default first-layer cycle cost: " << default_first_cost);
    INFO("snake first-layer cycle cost: " << snake_first_cost);
    for (size_t layer_index = 0; layer_index < runs[2].layers.size(); ++layer_index) {
        const double default_cost = cycle_cost(runs[0].layers[layer_index], runs[0].centers);
        const double snake_cost = cycle_cost(runs[3].layers[layer_index], runs[3].centers);
        const double best_cost = cycle_cost(runs[2].layers[layer_index], runs[2].centers);
        const double shortest_candidate = std::min(default_cost, snake_cost);
        // BestOf may equal Default; it must never exceed either public
        // candidate's independently measured cycle cost.
        CHECK(best_cost <= shortest_candidate + 1e-4);
    }
}

namespace input_shaping_followup {

constexpr size_t kMaxOutputBytes = 4 * 1024 * 1024;

using InputShapingTokens = std::vector<std::string>;

std::vector<InputShapingTokens> parse_m593_commands(const std::string &gcode)
{
    std::vector<InputShapingTokens> result;
    size_t line_start = 0;
    while (line_start <= gcode.size()) {
        const size_t line_end = gcode.find('\n', line_start);
        const size_t length = line_end == std::string::npos ? gcode.size() - line_start : line_end - line_start;
        const std::string line = gcode.substr(line_start, length);
        const size_t first = line.find_first_not_of(" \t\r");
        if (first != std::string::npos && line.compare(first, 4, "M593") == 0) {
            const size_t comment = line.find(';', first);
            const size_t command_end = comment == std::string::npos ? line.size() : comment;
            const size_t after_command = first + 4;
            const bool command_boundary = after_command == command_end;
            const bool command_has_separator = !command_boundary &&
                std::isspace(static_cast<unsigned char>(line[after_command]));
            if (command_boundary || command_has_separator) {
                std::istringstream stream(line.substr(first, command_end - first));
                InputShapingTokens tokens;
                std::string token;
                while (stream >> token)
                    tokens.push_back(token);
                result.push_back(std::move(tokens));
            }
        }
        if (line_end == std::string::npos)
            break;
        line_start = line_end + 1;
    }
    return result;
}

bool has_input_shaping_token(const InputShapingTokens &tokens, const std::string &token)
{
    return std::find(tokens.begin(), tokens.end(), token) != tokens.end();
}

bool has_input_shaping_parameter(const InputShapingTokens &tokens, char parameter)
{
    for (const std::string &token : tokens) {
        if (token.size() > 1 && token.front() == parameter)
            return true;
    }
    return false;
}

void require_bounded_material(const ExportedPrint &exported, const DynamicPrintConfig &config,
                              const MotionEvidence &reference)
{
    REQUIRE(exported.text.size() < kMaxOutputBytes);
    const MotionEvidence observed = motion_evidence(exported.text, config);
    REQUIRE(observed.extrusion_moves > 0);
    require_same_material(reference, observed);
}

void require_registered_input_shaper_domain()
{
    const ConfigOptionDef *definition = print_config_def.get("input_shaping_type");
    REQUIRE(definition != nullptr);
    const std::set<std::string> registered(definition->enum_values.begin(), definition->enum_values.end());
    const std::set<std::string> expected{
        "Default", "MZV", "ZV", "ZVD", "ZVDD", "ZVDDD", "EI", "EI2",
        "2HUMP_EI", "EI3", "3HUMP_EI", "DAA", "Disable",
    };
    REQUIRE(registered == expected);
    REQUIRE(definition->enum_values.size() == 13);
}

void configure_input_shaping(DynamicPrintConfig &config, const char *flavor, const char *type,
                             bool emit, double freq_x, double freq_y, double damp_x, double damp_y)
{
    config.set_deserialize_strict({
        {"gcode_flavor", flavor},
        {"emit_machine_limits_to_gcode", true},
        {"input_shaping_emit", emit},
        {"input_shaping_type", type},
        {"input_shaping_freq_x", freq_x},
        {"input_shaping_freq_y", freq_y},
        {"input_shaping_damp_x", damp_x},
        {"input_shaping_damp_y", damp_y},
    });
}

TEST_CASE("Registered input shaper types serialize through the RRF configuration route",
          "[GCode][InputShaping][GCodeExportWorkflows]")
{
    require_registered_input_shaper_domain();

    DynamicPrintConfig control_config = base_config();
    configure_input_shaping(control_config, "reprapfirmware", "Default", false, 0., 0., 0., 0.);
    const ExportedPrint control = fresh_custom_mesh_export(control_config, make_cube(10., 10., 0.4));
    REQUIRE(control.text.size() < kMaxOutputBytes);
    const MotionEvidence reference = motion_evidence(control.text, control_config);
    REQUIRE(reference.extrusion_moves > 0);
    REQUIRE(parse_m593_commands(control.text).empty());

    const ConfigOptionDef *definition = print_config_def.get("input_shaping_type");
    REQUIRE(definition != nullptr);
    const std::set<std::string> documented_rrf_types{
        "MZV", "ZVD", "ZVDD", "ZVDDD", "EI2", "EI3",
    };
    for (const std::string &type : definition->enum_values) {
        INFO("input_shaping_type=" << type);
        DynamicPrintConfig config = base_config();
        configure_input_shaping(config, "reprapfirmware", type.c_str(), true, 45., 71., 0.125, 0.275);
        REQUIRE(config.opt_serialize("input_shaping_type") == type);
        const ExportedPrint exported = fresh_custom_mesh_export(config, make_cube(10., 10., 0.4));
        require_bounded_material(exported, config, reference);

        const std::vector<InputShapingTokens> commands = parse_m593_commands(exported.text);
        REQUIRE(commands.size() == 1);
        const InputShapingTokens &command = commands.front();
        CHECK(has_input_shaping_token(command, "M593"));

        // RRF type acceptance is deliberately not asserted here.  This loop
        // records all registered serialization choices and shared X routing;
        // firmware support for every registry spelling is a separate question.
        if (type == "Disable") {
            CHECK_FALSE(has_input_shaping_token(command, "F45.00"));
            CHECK_FALSE(has_input_shaping_token(command, "S0.125"));
            CHECK_FALSE(has_input_shaping_token(command, "F71.00"));
            CHECK_FALSE(has_input_shaping_token(command, "S0.275"));
        } else {
            CHECK(has_input_shaping_token(command, "F45.00"));
            CHECK(has_input_shaping_token(command, "S0.125"));
            CHECK_FALSE(has_input_shaping_token(command, "F71.00"));
            CHECK_FALSE(has_input_shaping_token(command, "S0.275"));
        }
        if (documented_rrf_types.count(type) > 0)
            CHECK(has_input_shaping_token(command, "P\"" + type + "\""));
        if (type == "Default")
            CHECK_FALSE(has_input_shaping_token(command, "P\"Default\""));
    }
}

TEST_CASE("Marlin 2 input shaping routes axes and honors endpoints and gates",
          "[GCode][InputShaping][GCodeExportWorkflows]")
{
    DynamicPrintConfig control_config = base_config();
    configure_input_shaping(control_config, "marlin2", "ZV", false, 45., 50., 0.125, 0.275);
    const ExportedPrint control = fresh_custom_mesh_export(control_config, make_cube(10., 10., 0.4));
    REQUIRE(control.text.size() < kMaxOutputBytes);
    const MotionEvidence reference = motion_evidence(control.text, control_config);
    REQUIRE(reference.extrusion_moves > 0);
    REQUIRE(parse_m593_commands(control.text).empty());

    DynamicPrintConfig representative = base_config();
    configure_input_shaping(representative, "marlin2", "ZV", true, 45., 50., 0.125, 0.275);
    const ExportedPrint representative_export = fresh_custom_mesh_export(representative, make_cube(10., 10., 0.4));
    require_bounded_material(representative_export, representative, reference);
    const std::vector<InputShapingTokens> representative_commands = parse_m593_commands(representative_export.text);
    REQUIRE(representative_commands.size() == 2);
    CHECK(has_input_shaping_token(representative_commands[0], "M593"));
    CHECK(has_input_shaping_token(representative_commands[0], "X"));
    CHECK(has_input_shaping_token(representative_commands[0], "F45.00"));
    CHECK(has_input_shaping_token(representative_commands[0], "D0.125"));
    CHECK(has_input_shaping_token(representative_commands[1], "M593"));
    CHECK(has_input_shaping_token(representative_commands[1], "Y"));
    CHECK(has_input_shaping_token(representative_commands[1], "F50.00"));
    CHECK(has_input_shaping_token(representative_commands[1], "D0.275"));
    CHECK_FALSE(has_input_shaping_token(representative_commands[0], "P\"ZV\""));
    CHECK_FALSE(has_input_shaping_token(representative_commands[1], "P\"ZV\""));

    struct EndpointRow {
        double freq_x;
        double damp_x;
        double freq_y;
        double damp_y;
    };
    const std::array<EndpointRow, 2> endpoints{{
        {0., 0., 1000., 1.},
        {1000., 1., 0., 0.},
    }};
    for (const EndpointRow &row : endpoints) {
        INFO("endpoint X=" << row.freq_x << "/" << row.damp_x << " Y=" << row.freq_y << "/" << row.damp_y);
        DynamicPrintConfig config = base_config();
        configure_input_shaping(config, "marlin2", "ZV", true,
                                row.freq_x, row.freq_y, row.damp_x, row.damp_y);
        const ExportedPrint exported = fresh_custom_mesh_export(config, make_cube(10., 10., 0.4));
        require_bounded_material(exported, config, reference);
        const std::vector<InputShapingTokens> commands = parse_m593_commands(exported.text);
        REQUIRE(commands.size() == 2);
        REQUIRE(has_input_shaping_token(commands[0], "X"));
        REQUIRE(has_input_shaping_token(commands[1], "Y"));
        if (row.freq_x > 0.)
            CHECK(has_input_shaping_token(commands[0], "F1000.00"));
        else
            CHECK_FALSE(has_input_shaping_parameter(commands[0], 'F'));
        if (row.damp_x > 0.)
            CHECK(has_input_shaping_token(commands[0], "D1.000"));
        else
            CHECK_FALSE(has_input_shaping_parameter(commands[0], 'D'));
        if (row.freq_y > 0.)
            CHECK(has_input_shaping_token(commands[1], "F1000.00"));
        else
            CHECK_FALSE(has_input_shaping_parameter(commands[1], 'F'));
        if (row.damp_y > 0.)
            CHECK(has_input_shaping_token(commands[1], "D1.000"));
        else
            CHECK_FALSE(has_input_shaping_parameter(commands[1], 'D'));
    }

    DynamicPrintConfig disabled = base_config();
    configure_input_shaping(disabled, "marlin2", "Disable", true, 45., 50., 0.125, 0.275);
    const ExportedPrint disabled_export = fresh_custom_mesh_export(disabled, make_cube(10., 10., 0.4));
    require_bounded_material(disabled_export, disabled, reference);
    const std::vector<InputShapingTokens> disabled_commands = parse_m593_commands(disabled_export.text);
    REQUIRE(disabled_commands.size() == 1);
    CHECK(has_input_shaping_token(disabled_commands[0], "M593"));
    CHECK_FALSE(has_input_shaping_token(disabled_commands[0], "X"));
    CHECK_FALSE(has_input_shaping_token(disabled_commands[0], "Y"));
    CHECK_FALSE(has_input_shaping_token(disabled_commands[0], "Z"));
    CHECK_FALSE(has_input_shaping_token(disabled_commands[0], "A"));
    CHECK(has_input_shaping_token(disabled_commands[0], "F0.00"));
    CHECK(has_input_shaping_token(disabled_commands[0], "D0.000"));
}

} // namespace input_shaping_followup

namespace calibration_gcode_followup {

namespace {

constexpr size_t kMaxOutputBytes = 4 * 1024 * 1024;
constexpr double kDampingStart = 0.125;
constexpr double kFrequencyStartX = 40.0;
constexpr double kFrequencyEndX = 80.0;
constexpr double kFrequencyStartY = 55.0;
constexpr double kFrequencyEndY = 95.0;

struct CalibrationCommand {
    size_t      layer = 0;
    std::string line;
};

std::vector<CalibrationCommand> parse_calibration_commands(const std::string &gcode)
{
    std::vector<CalibrationCommand> commands;
    size_t current_layer = std::numeric_limits<size_t>::max();
    size_t line_start = 0;
    while (line_start <= gcode.size()) {
        const size_t line_end = gcode.find('\n', line_start);
        const size_t length = line_end == std::string::npos ? gcode.size() - line_start : line_end - line_start;
        const std::string line = gcode.substr(line_start, length);
        const size_t first = line.find_first_not_of(" \t\r");
        if (first != std::string::npos && line.compare(first, 13, ";LAYER_CHANGE") == 0) {
            current_layer = current_layer == std::numeric_limits<size_t>::max() ? 0 : current_layer + 1;
        } else if (current_layer != std::numeric_limits<size_t>::max() && first != std::string::npos) {
            const std::string command = line.substr(first);
            const bool is_marlin_or_rrf = command.compare(0, 4, "M593") == 0 &&
                (command.size() == 4 || std::isspace(static_cast<unsigned char>(command[4])));
            const bool is_klipper = command.compare(0, 16, "SET_INPUT_SHAPER") == 0 &&
                (command.size() == 16 || std::isspace(static_cast<unsigned char>(command[16])));
            if (is_marlin_or_rrf || is_klipper)
                commands.push_back({current_layer, command});
        }
        if (line_end == std::string::npos)
            break;
        line_start = line_end + 1;
    }
    return commands;
}

bool has_command_token(const std::string &line, const std::string &expected)
{
    std::istringstream stream(line);
    std::string token;
    while (stream >> token)
        if (token == expected)
            return true;
    return false;
}

std::string fixed_value(double value, int precision)
{
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(precision) << value;
    return stream.str();
}

double expected_ramp(size_t layer_index, size_t layer_count, double start, double end)
{
    // This is the public calibration contract: the first two zero-based layer
    // indices retain the start value, then the remaining layers interpolate
    // across the complete configured range.
    if (layer_index <= 1)
        return start;
    return start + (static_cast<double>(layer_index) / static_cast<double>(layer_count - 1)) * (end - start);
}

void require_shaper_command(const CalibrationCommand &command, const char *flavor,
                            char axis, double frequency, bool include_frequency,
                            double damping, bool include_damping, const char *type)
{
    const std::string line = command.line;
    const bool klipper = std::string(flavor) == "klipper";
    const bool reprap = std::string(flavor) == "reprapfirmware";
    const std::string prefix = klipper ? "SET_INPUT_SHAPER" : "M593";
    REQUIRE(line.rfind(prefix, 0) == 0);

    if (klipper) {
        if (type != nullptr && type[0] != '\0')
            CHECK(has_command_token(line, std::string("SHAPER_TYPE=") + type));
        const std::string axis_name = axis == 'A' ? "" : std::string(1, axis);
        if (include_frequency) {
            if (axis == 'A') {
                CHECK(has_command_token(line, "SHAPER_FREQ_X=" + fixed_value(frequency, 2)));
                CHECK(has_command_token(line, "SHAPER_FREQ_Y=" + fixed_value(frequency, 2)));
            } else {
                CHECK(has_command_token(line, "SHAPER_FREQ_" + axis_name + "=" + fixed_value(frequency, 2)));
            }
        }
        if (include_damping) {
            if (axis == 'A') {
                CHECK(has_command_token(line, "DAMPING_RATIO_X=" + fixed_value(damping, 3)));
                CHECK(has_command_token(line, "DAMPING_RATIO_Y=" + fixed_value(damping, 3)));
            } else {
                CHECK(has_command_token(line, "DAMPING_RATIO_" + axis_name + "=" + fixed_value(damping, 3)));
            }
        }
        return;
    }

    if (reprap) {
        // RRF does not encode an axis in M593.  Each calibration call is a
        // separate command, so the independently calculated F/S value is the
        // axis-specific witness.
        if (type != nullptr && type[0] != '\0')
            CHECK(has_command_token(line, std::string("P\"") + type + "\""));
    } else if (axis != 'A') {
        CHECK(has_command_token(line, std::string(1, axis)));
    }
    if (include_frequency)
        CHECK(has_command_token(line, "F" + fixed_value(frequency, 2)));
    if (include_damping) {
        const std::string damping_token = (reprap ? "S" : "D") + fixed_value(damping, 3);
        CHECK(has_command_token(line, damping_token));
    }
}

ExportedPrint export_calibration(const char *flavor, const Calib_Params &params,
                                 const DynamicPrintConfig *config_override = nullptr)
{
    DynamicPrintConfig config = config_override == nullptr ? base_config() : *config_override;
    config.set_deserialize_strict({
        {"gcode_flavor", flavor},
        {"input_shaping_emit", false},
        {"input_shaping_type", "Default"},
        {"input_shaping_freq_x", 0.0},
        {"input_shaping_freq_y", 0.0},
        {"input_shaping_damp_x", 0.0},
        {"input_shaping_damp_y", 0.0},
        {"top_shell_layers", 0},
        {"bottom_shell_layers", 0},
        {"sparse_infill_density", "0%"},
        {"enable_support", false},
        {"skirt_loops", 0},
        {"brim_type", "no_brim"},
        {"raft_layers", 0},
    });

    Print print;
    Model model;
    Test::init_print({make_cube(8.0, 8.0, 1.2)}, print, model, config);
    require_valid(print);
    print.set_calib_params(params);
    return export_print(print);
}

void require_six_layers(const ExportedPrint &exported, const DynamicPrintConfig &config)
{
    REQUIRE(count_text(exported.text, ";LAYER_CHANGE") == kCalibrationLayerCount);
    REQUIRE(exported.text.size() < kMaxOutputBytes);
    REQUIRE(motion_evidence(exported.text, config).extrusion_moves > 0);
}

} // namespace

TEST_CASE("Calibration input shaping frequency ramps emit independent XY commands and disabled control",
          "[GCode][Calibration][InputShaping][CalibrationInputShapingGCode][GCodeExportWorkflows]")
{
    const char *flavor = GENERATE("marlin2", "reprapfirmware", "klipper");
    const bool asymmetric = GENERATE(false, true);
    INFO("gcode_flavor=" << flavor << ", asymmetric=" << asymmetric);

    Calib_Params params;
    params.mode = CalibMode::Calib_Input_shaping_freq;
    params.start = kDampingStart;
    params.freqStartX = kFrequencyStartX;
    params.freqEndX = kFrequencyEndX;
    params.freqStartY = asymmetric ? kFrequencyStartY : kFrequencyStartX;
    params.freqEndY = asymmetric ? kFrequencyEndY : kFrequencyEndX;
    params.shaper_type = "ZV";

    const ExportedPrint exported = export_calibration(flavor, params);
    DynamicPrintConfig config = base_config();
    config.set_deserialize_strict({{"gcode_flavor", flavor}, {"input_shaping_emit", false}});
    require_six_layers(exported, config);
    const std::vector<CalibrationCommand> commands = parse_calibration_commands(exported.text);
    const size_t expected_commands = asymmetric ? 2 * kCalibrationLayerCount - 1 : kCalibrationLayerCount;
    REQUIRE(commands.size() == expected_commands);

    size_t cursor = 0;
    for (size_t layer = 0; layer < kCalibrationLayerCount; ++layer) {
        INFO("layer=" << layer);
        if (layer == 1) {
            REQUIRE(commands[cursor].layer == layer);
            require_shaper_command(commands[cursor++], flavor, 'A', 0.0, false,
                                    params.start, true, params.shaper_type.c_str());
        } else if (asymmetric) {
            REQUIRE(commands[cursor].layer == layer);
            require_shaper_command(commands[cursor++], flavor, 'X',
                                    expected_ramp(layer, kCalibrationLayerCount, params.freqStartX, params.freqEndX), true,
                                    0.0, false, "");
            REQUIRE(commands[cursor].layer == layer);
            require_shaper_command(commands[cursor++], flavor, 'Y',
                                    expected_ramp(layer, kCalibrationLayerCount, params.freqStartY, params.freqEndY), true,
                                    0.0, false, "");
        } else {
            REQUIRE(commands[cursor].layer == layer);
            require_shaper_command(commands[cursor++], flavor, 'A',
                                    expected_ramp(layer, kCalibrationLayerCount, params.freqStartX, params.freqEndX), true,
                                    0.0, false, "");
        }
    }
    REQUIRE(cursor == commands.size());

    if (std::string(flavor) == "marlin2" && !asymmetric) {
        Calib_Params disabled_params;
        disabled_params.mode = CalibMode::Calib_None;
        const ExportedPrint disabled = export_calibration("marlin2", disabled_params);
        DynamicPrintConfig disabled_config = base_config();
        disabled_config.set_deserialize_strict({{"gcode_flavor", "marlin2"}, {"input_shaping_emit", false}});
        require_six_layers(disabled, disabled_config);
        CHECK(parse_calibration_commands(disabled.text).empty());
    }
}

TEST_CASE("Calibration input shaping damping ramps preserve XY frequency endpoints",
          "[GCode][Calibration][InputShaping][CalibrationInputShapingGCode][GCodeExportWorkflows]")
{
    const char *flavor = GENERATE("marlin2", "reprapfirmware", "klipper");
    INFO("gcode_flavor=" << flavor);

    Calib_Params params;
    params.mode = CalibMode::Calib_Input_shaping_damp;
    params.start = 0.10;
    params.end = 0.40;
    params.freqStartX = 45.0;
    params.freqStartY = 65.0;
    params.shaper_type = "ZV";

    const ExportedPrint exported = export_calibration(flavor, params);
    DynamicPrintConfig config = base_config();
    config.set_deserialize_strict({{"gcode_flavor", flavor}, {"input_shaping_emit", false}});
    require_six_layers(exported, config);
    const std::vector<CalibrationCommand> commands = parse_calibration_commands(exported.text);
    REQUIRE(commands.size() == kCalibrationLayerCount + 1);

    size_t cursor = 0;
    for (size_t layer = 0; layer < kCalibrationLayerCount; ++layer) {
        INFO("layer=" << layer);
        REQUIRE(commands[cursor].layer == layer);
        if (layer == 1) {
            require_shaper_command(commands[cursor++], flavor, 'X', params.freqStartX, true,
                                   0.0, false, params.shaper_type.c_str());
            REQUIRE(commands[cursor].layer == layer);
            require_shaper_command(commands[cursor++], flavor, 'Y', params.freqStartY, true,
                                   0.0, false, params.shaper_type.c_str());
        } else {
            require_shaper_command(commands[cursor++], flavor, 'A', 0.0, false,
                                   expected_ramp(layer, kCalibrationLayerCount, params.start, params.end), true, "");
        }
    }
    REQUIRE(cursor == commands.size());
}

namespace {
struct CalibrationLayerCommand {
    size_t      layer = 0;
    std::string raw;
};

std::vector<CalibrationLayerCommand> parse_layer_commands(const std::string &gcode,
                                                          std::string_view prefix)
{
    std::vector<CalibrationLayerCommand> commands;
    size_t current_layer = std::numeric_limits<size_t>::max();
    size_t line_start = 0;
    while (line_start <= gcode.size()) {
        const size_t line_end = gcode.find('\n', line_start);
        const size_t length = line_end == std::string::npos ? gcode.size() - line_start : line_end - line_start;
        const std::string line = gcode.substr(line_start, length);
        const size_t first = line.find_first_not_of(" \t\r");
        if (first != std::string::npos && line.compare(first, 13, ";LAYER_CHANGE") == 0) {
            current_layer = current_layer == std::numeric_limits<size_t>::max() ? 0 : current_layer + 1;
        } else if (current_layer != std::numeric_limits<size_t>::max() && first != std::string::npos &&
                   line.compare(first, prefix.size(), prefix) == 0 &&
                   (first + prefix.size() == line.size() || std::isspace(static_cast<unsigned char>(line[first + prefix.size()])))) {
            commands.push_back({current_layer, line.substr(first)});
        }
        if (line_end == std::string::npos)
            break;
        line_start = line_end + 1;
    }
    return commands;
}

std::optional<double> command_word_value(const std::string &line, char word)
{
    std::istringstream stream(line);
    std::string token;
    while (stream >> token) {
        if (token == ";" || (!token.empty() && token.front() == ';'))
            break;
        if (token.size() < 2 || token.front() != word)
            continue;
        try {
            size_t parsed = 0;
            const double value = std::stod(token.substr(1), &parsed);
            if (parsed == token.size() - 1 && std::isfinite(value))
                return value;
        } catch (...) {
            return std::nullopt;
        }
    }
    return std::nullopt;
}


void require_outer_wall_speed_bands(const ExportedPrint &exported,
                                    const DynamicPrintConfig &config,
                                    const std::array<double, kCalibrationLayerCount> &expected_mm_s)
{
    const OuterWallFeedEvidence evidence = parse_outer_wall_feeds(exported.text, config);
    REQUIRE(evidence.finite);
    REQUIRE(evidence.feeds.size() == kCalibrationLayerCount);
    for (size_t layer = 1; layer < kCalibrationLayerCount; ++layer) {
        INFO("layer=" << layer);
        REQUIRE_FALSE(evidence.feeds[layer].empty());
        for (const double feed : evidence.feeds[layer])
            CHECK_THAT(feed, Catch::Matchers::WithinAbs(expected_mm_s[layer] * 60., 0.25));
    }
}

std::array<double, kCalibrationLayerCount> parse_effective_word_by_layer(const std::string &gcode,
                                                                            std::string_view prefix,
                                                                            char word)
{
    const std::vector<CalibrationLayerCommand> commands = parse_layer_commands(gcode, prefix);
    double current = std::numeric_limits<double>::quiet_NaN();
    bool have_current = false;
    bool saw_explicit_layer_zero = false;
    for (const CalibrationLayerCommand &command : commands)
        REQUIRE(command.layer < kCalibrationLayerCount);

    std::array<double, kCalibrationLayerCount> effective{};
    for (size_t layer = 0; layer < kCalibrationLayerCount; ++layer) {
        for (const CalibrationLayerCommand &command : commands) {
            if (command.layer != layer)
                continue;
            const std::optional<double> value = command_word_value(command.raw, word);
            REQUIRE(value.has_value());
            current = *value;
            have_current = true;
            if (layer == 0)
                saw_explicit_layer_zero = true;
        }
        // A missing later command is valid modal G-code; layer zero must have
        // an actual calibration command so this state is never fabricated.
        REQUIRE(have_current);
        effective[layer] = current;
    }
    REQUIRE(saw_explicit_layer_zero);
    return effective;
}

void require_calibration_temperature(const ExportedPrint &exported,
                                     const std::array<double, kCalibrationLayerCount> &expected)
{
    const std::array<double, kCalibrationLayerCount> effective =
        parse_effective_word_by_layer(exported.text, "M104", 'S');
    for (size_t layer = 0; layer < kCalibrationLayerCount; ++layer)
        CHECK_THAT(effective[layer], Catch::Matchers::WithinAbs(expected[layer], 0.01));
}

void require_calibration_junction_deviation(const ExportedPrint &exported,
                                            const std::array<double, kCalibrationLayerCount> &expected)
{
    const std::array<double, kCalibrationLayerCount> effective =
        parse_effective_word_by_layer(exported.text, "M205", 'J');
    for (size_t layer = 0; layer < kCalibrationLayerCount; ++layer)
        CHECK_THAT(effective[layer], Catch::Matchers::WithinAbs(expected[layer], 0.001));
}

void require_calibration_jerk(const ExportedPrint &exported,
                              const std::array<double, kCalibrationLayerCount> &expected)
{
    const std::array<double, kCalibrationLayerCount> effective_x =
        parse_effective_word_by_layer(exported.text, "M205", 'X');
    const std::array<double, kCalibrationLayerCount> effective_y =
        parse_effective_word_by_layer(exported.text, "M205", 'Y');
    for (size_t layer = 0; layer < kCalibrationLayerCount; ++layer) {
        INFO("layer=" << layer);
        CHECK_THAT(effective_x[layer], Catch::Matchers::WithinAbs(expected[layer], 0.01));
        CHECK_THAT(effective_y[layer], Catch::Matchers::WithinAbs(expected[layer], 0.01));
    }
}

DynamicPrintConfig calibration_tower_config()
{
    DynamicPrintConfig config = base_config();
    config.set_deserialize_strict({
        {"filament_max_volumetric_speed", "1000"},
        {"filament_adaptive_volumetric_speed", false},
        {"slow_down_layers", 0},
        {"small_perimeter_speed", 0.0},
        {"small_perimeter_threshold", 0.0},
        {"slow_down_for_layer_cooling", false},
        {"slowdown_for_curled_perimeters", false},
        {"enable_overhang_speed", false},
        {"enable_overhang_bridge_fan", false},
        {"resonance_avoidance", false},
        {"default_jerk", 0.0},
        {"initial_layer_jerk", 0.0},
        {"outer_wall_jerk", 0.0},
        {"inner_wall_jerk", 0.0},
        {"top_surface_jerk", 0.0},
        {"infill_jerk", 0.0},
        {"travel_jerk", 0.0},
        {"initial_layer_travel_jerk", 0.0},
        {"default_junction_deviation", 0.0},
        {"nozzle_temperature_initial_layer", 200},
        {"nozzle_temperature", 200},
        {"machine_max_jerk_x", "100,100"},
        {"machine_max_jerk_y", "100,100"},
    });
    return config;
}

} // namespace

TEST_CASE("Calibration temperature and speed towers emit bounded per-layer settings",
          "[GCode][Calibration][CalibrationTowerOutput][GCodeExportWorkflows]")
{
    DynamicPrintConfig neutral_config = calibration_tower_config();
    Calib_Params none;
    const ExportedPrint neutral = export_calibration("marlin2", none, &neutral_config);
    require_six_layers(neutral, neutral_config);
    const MotionEvidence reference = motion_evidence(neutral.text, neutral_config);
    REQUIRE(reference.xy_path > 0.0);

    Calib_Params temperature;
    temperature.mode = CalibMode::Calib_Temp_Tower;
    temperature.start = 200.;
    temperature.end = 225.;
    temperature.step = 5.;
    temperature.nozzle_based_resize = false;
    const ExportedPrint temperature_export = export_calibration("marlin2", temperature, &neutral_config);
    require_six_layers(temperature_export, neutral_config);
    require_same_material(reference, motion_evidence(temperature_export.text, neutral_config));
    require_calibration_temperature(temperature_export, {200., 200., 210., 215., 220., 225.});

    Calib_Params vfa;
    vfa.mode = CalibMode::Calib_VFA_Tower;
    vfa.start = 20.;
    vfa.end = 60.;
    vfa.step = 10.;
    vfa.nozzle_based_resize = false;
    const ExportedPrint vfa_export = export_calibration("marlin2", vfa, &neutral_config);
    require_six_layers(vfa_export, neutral_config);
    require_same_material(reference, motion_evidence(vfa_export.text, neutral_config));
    require_outer_wall_speed_bands(vfa_export, neutral_config, {20., 20., 40., 50., 60., 60.});

    Calib_Params volumetric;
    volumetric.mode = CalibMode::Calib_Vol_speed_Tower;
    volumetric.start = 20.;
    volumetric.end = 200.;
    volumetric.step = 10.;
    volumetric.nozzle_based_resize = false;
    const ExportedPrint volumetric_export = export_calibration("marlin2", volumetric, &neutral_config);
    require_six_layers(volumetric_export, neutral_config);
    require_same_material(reference, motion_evidence(volumetric_export.text, neutral_config));
    require_outer_wall_speed_bands(volumetric_export, neutral_config, {22., 24., 26., 28., 30., 32.});
}

TEST_CASE("Cornering calibration selects junction deviation and jerk fallback per layer",
          "[GCode][Calibration][CalibrationTowerOutput][GCodeExportWorkflows]")
{
    DynamicPrintConfig junction_config = calibration_tower_config();
    junction_config.set_deserialize_strict({{"machine_max_junction_deviation", "0.5,0.5"}});
    Calib_Params junction;
    junction.mode = CalibMode::Calib_Cornering;
    junction.start = 0.05;
    junction.end = 0.15;
    const ExportedPrint junction_export = export_calibration("marlin2", junction, &junction_config);
    require_six_layers(junction_export, junction_config);
    const MotionEvidence junction_motion = motion_evidence(junction_export.text, junction_config);
    REQUIRE(junction_motion.extrusion_moves > 0);
    require_calibration_junction_deviation(junction_export, {0.05, 0.05, 0.09, 0.11, 0.13, 0.15});

    DynamicPrintConfig jerk_config = calibration_tower_config();
    jerk_config.set_deserialize_strict({{"machine_max_junction_deviation", "0,0"}});
    Calib_Params jerk;
    jerk.mode = CalibMode::Calib_Cornering;
    jerk.start = 3.;
    jerk.end = 9.;
    const ExportedPrint jerk_export = export_calibration("marlin2", jerk, &jerk_config);
    require_six_layers(jerk_export, jerk_config);
    const MotionEvidence jerk_motion = motion_evidence(jerk_export.text, jerk_config);
    REQUIRE(jerk_motion.extrusion_moves > 0);
    require_calibration_jerk(jerk_export, {3., 3., 5.4, 6.6, 7.8, 9.});
    require_same_material(junction_motion, jerk_motion);
}


} // namespace calibration_gcode_followup


namespace small_area_flow_compensation_contracts {

namespace {

constexpr std::size_t kSmallAreaMaxOutputBytes = 4u * 1024u * 1024u;
constexpr std::size_t kSmallAreaMaxLines = 100000u;
constexpr double      kSmallAreaModelEnd = 100.0;
constexpr double      kSmallAreaQuantization = 2e-4;

struct SmallAreaSegment
{
    double      xy = 0.0;
    double      e  = 0.0;
    double      x  = 0.0;
    double      y  = 0.0;
    double      z  = 0.0;
    std::size_t layer = 0;
};

struct SmallAreaRoleEvidence
{
    std::vector<SmallAreaSegment> segments;
    double xy = 0.0;
    double e  = 0.0;
};

struct SmallAreaEvidence
{
    std::map<std::string, SmallAreaRoleEvidence> roles;
    std::size_t lines = 0;
    std::size_t layer_markers = 0;
    std::size_t moves = 0;
    double      xy = 0.0;
    double      e  = 0.0;
    bool        finite = true;
    bool        eligible_arc = false;
};

std::string_view trim_small_area_comment(std::string_view value)
{
    while (!value.empty() && std::isspace(static_cast<unsigned char>(value.front())))
        value.remove_prefix(1);
    while (!value.empty() && std::isspace(static_cast<unsigned char>(value.back())))
        value.remove_suffix(1);
    return value;
}

bool small_area_eligible_role(const std::string &role)
{
    return role == "Bottom surface" || role == "Top surface" ||
           role == "Internal solid infill" || role == "Solid infill";
}

bool small_area_is_layer_marker(std::string_view comment)
{
    comment = trim_small_area_comment(comment);
    return comment == "LAYER_CHANGE" || comment == "CHANGE_LAYER";
}

SmallAreaEvidence parse_small_area_motion(const std::string &text, const DynamicPrintConfig &config)
{
    REQUIRE(text.size() <= kSmallAreaMaxOutputBytes);
    REQUIRE(static_cast<std::size_t>(std::count(text.begin(), text.end(), '\n')) <= kSmallAreaMaxLines);

    SmallAreaEvidence evidence;
    GCodeReader reader;
    reader.apply_config(config);
    std::string active_role;
    std::size_t layer = 0;
    reader.parse_buffer(text, [&evidence, &active_role, &layer](GCodeReader &self,
                                                                  const GCodeReader::GCodeLine &line) {
        ++evidence.lines;
        const std::string_view comment = trim_small_area_comment(line.comment());
        if (small_area_is_layer_marker(comment)) {
            ++evidence.layer_markers;
            ++layer;
        }
        if (comment.rfind("TYPE:", 0) == 0)
            active_role.assign(trim_small_area_comment(comment.substr(5)));
        else if (comment.rfind("FEATURE:", 0) == 0)
            active_role.assign(trim_small_area_comment(comment.substr(8)));

        if (!(line.cmd_is("G0") || line.cmd_is("G1") || line.cmd_is("G2") || line.cmd_is("G3")))
            return;

        const double xy = line.dist_XY(self);
        const double e  = std::abs(line.dist_E(self));
        const double x  = line.new_X(self);
        const double y  = line.new_Y(self);
        const double z  = line.new_Z(self);
        if (!std::isfinite(xy) || !std::isfinite(e) || !std::isfinite(x) ||
            !std::isfinite(y) || !std::isfinite(z)) {
            evidence.finite = false;
            return;
        }
        if (!line.extruding(self) || xy <= EPSILON)
            return;

        const std::string role = active_role.empty() ? "<unknown>" : active_role;
        if (small_area_eligible_role(role) && (line.cmd_is("G2") || line.cmd_is("G3")))
            evidence.eligible_arc = true;
        SmallAreaRoleEvidence &role_evidence = evidence.roles[role];
        role_evidence.segments.push_back({xy, e, x, y, z, layer});
        role_evidence.xy += xy;
        role_evidence.e += e;
        ++evidence.moves;
        evidence.xy += xy;
        evidence.e += e;
    });

    REQUIRE(evidence.finite);
    REQUIRE_FALSE(evidence.eligible_arc);
    REQUIRE(evidence.lines <= kSmallAreaMaxLines);
    REQUIRE(evidence.layer_markers == 2);
    REQUIRE(evidence.moves > 0);
    return evidence;
}

void require_same_small_area_geometry(const SmallAreaEvidence &expected,
                                      const SmallAreaEvidence &observed)
{
    REQUIRE(expected.moves == observed.moves);
    CHECK_THAT(observed.xy, Catch::Matchers::WithinAbs(expected.xy, kSmallAreaQuantization));
    CHECK_THAT(observed.e, Catch::Matchers::WithinAbs(expected.e, kSmallAreaQuantization));
    REQUIRE(expected.roles.size() == observed.roles.size());
    for (const auto &entry : expected.roles) {
        const auto found = observed.roles.find(entry.first);
        REQUIRE(found != observed.roles.end());
        REQUIRE(entry.second.segments.size() == found->second.segments.size());
        CHECK_THAT(found->second.xy, Catch::Matchers::WithinAbs(entry.second.xy, kSmallAreaQuantization));
        CHECK_THAT(found->second.e, Catch::Matchers::WithinAbs(entry.second.e, kSmallAreaQuantization));
        for (std::size_t index = 0; index < entry.second.segments.size(); ++index) {
            const SmallAreaSegment &expected_segment = entry.second.segments[index];
            const SmallAreaSegment &observed_segment = found->second.segments[index];
            REQUIRE(expected_segment.layer == observed_segment.layer);
            CHECK_THAT(observed_segment.xy,
                       Catch::Matchers::WithinAbs(expected_segment.xy, kSmallAreaQuantization));
            CHECK_THAT(observed_segment.x,
                       Catch::Matchers::WithinAbs(expected_segment.x, kSmallAreaQuantization));
            CHECK_THAT(observed_segment.y,
                       Catch::Matchers::WithinAbs(expected_segment.y, kSmallAreaQuantization));
            CHECK_THAT(observed_segment.z,
                       Catch::Matchers::WithinAbs(expected_segment.z, kSmallAreaQuantization));
        }
    }
}

DynamicPrintConfig small_area_config(bool enabled, bool with_model)
{
    DynamicPrintConfig config = base_config();
    config.set_deserialize_strict({
        { "use_relative_e_distances", true },
        { "minimum_sparse_infill_area", 0.0 },
        { "enable_arc_fitting", false },
        { "top_surface_density", "100%" },
        { "bottom_surface_density", "100%" },
        // Rectilinear is in GCode::_needSAFC's supported first-layer pattern
        // list.  That makes the first-layer Bottom-surface and Outer-wall
        // paths both reach modify_flow; the role filter then protects the
        // latter while the bottom path supplies a positive eligible witness.
        { "bottom_surface_pattern", "rectilinear" },
        { "top_surface_pattern", "rectilinear" },
        { "internal_solid_infill_pattern", "rectilinear" },
        { "small_area_infill_flow_compensation", enabled },
    });
    if (with_model) {
        config.set_key_value("small_area_infill_flow_compensation_model",
                             new ConfigOptionStrings({"0,0.5", "100,1.0"}));
    } else {
        config.set_key_value("small_area_infill_flow_compensation_model",
                             new ConfigOptionStrings());
    }
    return config;
}

} // namespace

TEST_CASE("Small-area flow compensation changes only eligible solid export paths",
          "[GCode][SmallAreaFlowCompensation][GCodeExportWorkflows]")
{
    const TriangleMesh fixture = make_cube(10.0, 10.0, 0.4);

    const DynamicPrintConfig control_config = small_area_config(false, false);
    const DynamicPrintConfig disabled_with_model = small_area_config(false, true);
    const DynamicPrintConfig enabled_config = small_area_config(true, true);

    const ExportedPrint control = fresh_custom_mesh_export(control_config, fixture);
    const ExportedPrint disabled = fresh_custom_mesh_export(disabled_with_model, fixture);
    const ExportedPrint enabled = fresh_custom_mesh_export(enabled_config, fixture);

    const SmallAreaEvidence control_evidence = parse_small_area_motion(control.text, control_config);
    const SmallAreaEvidence disabled_evidence = parse_small_area_motion(disabled.text, disabled_with_model);
    const SmallAreaEvidence enabled_evidence = parse_small_area_motion(enabled.text, enabled_config);

    // The disabled model row is a public no-op control: it must retain every
    // role's segment count, layer, endpoint, XY length, and E output.
    require_same_small_area_geometry(control_evidence, disabled_evidence);
    REQUIRE(control_evidence.roles.size() == enabled_evidence.roles.size());

    const auto outer = control_evidence.roles.find("Outer wall");
    const auto bottom = control_evidence.roles.find("Bottom surface");
    const auto top = control_evidence.roles.find("Top surface");
    REQUIRE(outer != control_evidence.roles.end());
    REQUIRE(bottom != control_evidence.roles.end());
    REQUIRE(top != control_evidence.roles.end());
    REQUIRE_FALSE(outer->second.segments.empty());
    REQUIRE_FALSE(bottom->second.segments.empty());
    REQUIRE_FALSE(top->second.segments.empty());
    REQUIRE(control_evidence.layer_markers == 2);
    const std::size_t bottom_layer = bottom->second.segments.front().layer;
    REQUIRE(bottom_layer == 1);
    const bool has_second_layer_top = std::any_of(
        top->second.segments.begin(), top->second.segments.end(),
        [](const SmallAreaSegment &segment) { return segment.layer == 2; });
    REQUIRE(has_second_layer_top);
    const bool has_first_layer_outer = std::any_of(
        outer->second.segments.begin(), outer->second.segments.end(),
        [bottom_layer](const SmallAreaSegment &segment) { return segment.layer == bottom_layer; });
    REQUIRE(has_first_layer_outer);

    std::size_t eligible_segments = 0;
    for (const auto &entry : control_evidence.roles) {
        const auto enabled_role = enabled_evidence.roles.find(entry.first);
        REQUIRE(enabled_role != enabled_evidence.roles.end());
        REQUIRE(entry.second.segments.size() == enabled_role->second.segments.size());
        CHECK_THAT(enabled_role->second.xy,
                   Catch::Matchers::WithinAbs(entry.second.xy, kSmallAreaQuantization));
        for (std::size_t index = 0; index < entry.second.segments.size(); ++index) {
            const SmallAreaSegment &baseline = entry.second.segments[index];
            const SmallAreaSegment &variant = enabled_role->second.segments[index];
            REQUIRE(baseline.layer == variant.layer);
            CHECK_THAT(variant.xy, Catch::Matchers::WithinAbs(baseline.xy, kSmallAreaQuantization));
            CHECK_THAT(variant.x, Catch::Matchers::WithinAbs(baseline.x, kSmallAreaQuantization));
            CHECK_THAT(variant.y, Catch::Matchers::WithinAbs(baseline.y, kSmallAreaQuantization));
            CHECK_THAT(variant.z, Catch::Matchers::WithinAbs(baseline.z, kSmallAreaQuantization));

            if (!small_area_eligible_role(entry.first)) {
                CHECK_THAT(variant.e, Catch::Matchers::WithinAbs(baseline.e, kSmallAreaQuantization));
                continue;
            }

            ++eligible_segments;
            REQUIRE(baseline.xy > EPSILON);
            REQUIRE(baseline.xy < kSmallAreaModelEnd);
            const double expected_factor = 0.5 + 0.5 * baseline.xy / kSmallAreaModelEnd;
            const double expected_e = baseline.e * expected_factor;
            CHECK_THAT(variant.e, Catch::Matchers::WithinAbs(expected_e, kSmallAreaQuantization));
            CHECK(variant.e < baseline.e);
        }
    }
    REQUIRE(eligible_segments > 0);
}

} // namespace small_area_flow_compensation_contracts

TEST_CASE("Extrusion role hooks publish ordered role transitions and placeholders",
          "[GCode][ExtrusionRoleGCode][GCodePrintVariance]")
{
    struct RoleHookRecord {
        std::string kind;
        std::string role;
        std::string last_role;
        std::string type_role;
        int         layer = 0;
        double      z = 0.;
    };

    const auto parse_record = [](const std::string &line) -> std::optional<RoleHookRecord> {
        constexpr std::string_view prefix = "M117 RH_";
        if (line.rfind(prefix, 0) != 0)
            return std::nullopt;
        std::string body = line.substr(prefix.size());
        if (!body.empty() && body.back() == '\r')
            body.pop_back();
        std::vector<std::string> fields;
        size_t start = 0;
        while (true) {
            const size_t end = body.find('|', start);
            fields.emplace_back(body.substr(start, end == std::string::npos ? end : end - start));
            if (end == std::string::npos)
                break;
            start = end + 1;
        }
        if (fields.size() != 5)
            return std::nullopt;
        char *layer_end = nullptr;
        const long layer = std::strtol(fields[3].c_str(), &layer_end, 10);
        char *z_end = nullptr;
        const double z = std::strtod(fields[4].c_str(), &z_end);
        if (layer_end == fields[3].c_str() || *layer_end != '\0' ||
            z_end == fields[4].c_str() || *z_end != '\0')
            return std::nullopt;
        return RoleHookRecord{fields[0], fields[1], fields[2], {}, static_cast<int>(layer), z};
    };

    const auto parser_role_matches_type = [](std::string_view parser_role, std::string_view type_role) {
        // The hook placeholder uses GCode::extrusion_role_to_string_for_parser(),
        // while the later TYPE marker uses ExtrusionEntity::role_to_string().
        // Keep this translation table independent of the production helpers.
        if (parser_role == "Perimeter")
            return type_role == "Inner wall";
        if (parser_role == "ExternalPerimeter")
            return type_role == "Outer wall";
        if (parser_role == "OverhangPerimeter")
            return type_role == "Overhang wall";
        if (parser_role == "InternalInfill")
            return type_role == "Sparse infill";
        if (parser_role == "SolidInfill")
            return type_role == "Internal solid infill";
        if (parser_role == "TopSolidInfill")
            return type_role == "Top surface";
        if (parser_role == "BottomSurface")
            return type_role == "Bottom surface";
        if (parser_role == "BridgeInfill")
            return type_role == "Bridge" || type_role == "Internal Bridge";
        if (parser_role == "GapFill")
            return type_role == "Gap infill";
        if (parser_role == "Ironing")
            return type_role == "Ironing";
        if (parser_role == "Skirt")
            return type_role == "Skirt";
        if (parser_role == "Brim")
            return type_role == "Brim";
        if (parser_role == "SupportMaterial")
            return type_role == "Support";
        if (parser_role == "SupportMaterialInterface")
            return type_role == "Support interface";
        if (parser_role == "SupportTransition")
            return type_role == "Support transition";
        if (parser_role == "WipeTower")
            return type_role == "Prime tower";
        if (parser_role == "Custom")
            return type_role == "Custom";
        return false;
    };

    const auto collect_records = [&](const std::string &text) {
        std::vector<RoleHookRecord> records;
        size_t start = 0;
        while (start < text.size()) {
            const size_t end = text.find('\n', start);
            const std::string line = text.substr(start, end == std::string::npos ? end : end - start);
            if (line.rfind("M117 RH_", 0) == 0) {
                const auto record = parse_record(line);
                REQUIRE(record.has_value());
                if (record.has_value())
                    records.push_back(*record);
            } else if (line.rfind(";TYPE:", 0) == 0 && records.size() >= 3 &&
                       records[records.size() - 3].type_role.empty()) {
                const std::string type_role = line.substr(6);
                REQUIRE_FALSE(type_role.empty());
                for (size_t offset = 0; offset < 3; ++offset)
                    records[records.size() - 3 + offset].type_role = type_role;
            }
            if (end == std::string::npos)
                break;
            start = end + 1;
        }
        return records;
    };

    DynamicPrintConfig control = base_config();
    control.set_deserialize_strict({
        { "change_extrusion_role_gcode",          "" },
        { "filament_change_extrusion_role_gcode", "" },
        { "process_change_extrusion_role_gcode",  "" },
    });
    DynamicPrintConfig enabled = control;
    enabled.set_deserialize_strict({
        { "change_extrusion_role_gcode",          "M117 RH_MACHINE|[extrusion_role]|[last_extrusion_role]|[layer_num]|[layer_z]" },
        { "filament_change_extrusion_role_gcode", "M117 RH_FILAMENT|[extrusion_role]|[last_extrusion_role]|[layer_num]|[layer_z]" },
        { "process_change_extrusion_role_gcode",  "M117 RH_PROCESS|[extrusion_role]|[last_extrusion_role]|[layer_num]|[layer_z]" },
    });

    const auto export_role_fixture = [](const DynamicPrintConfig &config) {
        Print print;
        Model model;
        Test::init_print({make_cube(8., 8., 1.2)}, print, model, config);
        require_valid(print);
        return export_print(print);
    };

    const ExportedPrint disabled = export_role_fixture(control);
    const ExportedPrint exported = export_role_fixture(enabled);
    const MotionEvidence disabled_motion = motion_evidence(disabled.text, control);
    const MotionEvidence enabled_motion = motion_evidence(exported.text, enabled);
    require_same_material(disabled_motion, enabled_motion);
    REQUIRE(count_text(disabled.text, "RH_") == 0);
    REQUIRE(exported.text.size() < 1000000);

    const std::vector<RoleHookRecord> records = collect_records(exported.text);
    REQUIRE(records.size() >= 6);
    REQUIRE(records.size() % 3 == 0);
    const size_t layers = count_text(exported.text, ";LAYER_CHANGE");
    REQUIRE(layers == 6);
    const std::array<std::string_view, 3> expected_kinds{{"MACHINE", "FILAMENT", "PROCESS"}};
    std::set<std::pair<std::string, std::string>> transitions;
    for (size_t group = 0; group < records.size(); group += 3) {
        const RoleHookRecord &first = records[group];
        REQUIRE_FALSE(first.type_role.empty());
        REQUIRE(first.layer >= 1);
        REQUIRE(first.layer <= static_cast<int>(layers));
        REQUIRE(std::isfinite(first.z));
        CHECK_THAT(first.z, Catch::Matchers::WithinAbs(0.2 * first.layer, 1e-6));
        REQUIRE_FALSE(first.role.empty());
        REQUIRE_FALSE(first.last_role.empty());
        if (group == 0)
            CHECK(first.last_role == "Mixed");
        else
            CHECK(first.last_role == records[group - 3].role);
        CHECK(parser_role_matches_type(first.role, first.type_role));
        transitions.emplace(first.last_role, first.role);
        for (size_t offset = 0; offset < 3; ++offset) {
            const RoleHookRecord &record = records[group + offset];
            CHECK(record.kind == expected_kinds[offset]);
            CHECK(record.role == first.role);
            CHECK(record.last_role == first.last_role);
            CHECK(record.layer == first.layer);
            CHECK_THAT(record.z, Catch::Matchers::WithinAbs(first.z, 1e-6));
            CHECK(record.type_role == first.type_role);
        }
    }
    REQUIRE(transitions.size() >= 2);
}



struct ExhaustFanEvidence {
    std::vector<int> p2_pwm_during_deposition;
    std::vector<double> deposition_feeds;
    std::vector<int> p3_pwm;
    std::vector<size_t> p3_ordinals;
    std::optional<size_t> first_deposition_ordinal;
    std::optional<size_t> last_deposition_ordinal;
};

ExhaustFanEvidence inspect_exhaust_fan_commands(const std::string &text,
                                               const DynamicPrintConfig &config)
{
    ExhaustFanEvidence evidence;
    GCodeReader reader;
    reader.apply_config(config);
    size_t ordinal = 0;
    std::optional<int> current_p2_pwm;
    reader.parse_buffer(text, [&evidence, &ordinal, &current_p2_pwm](GCodeReader &self,
                                                                    const GCodeReader::GCodeLine &line) {
        const size_t event = ordinal++;
        if (line.cmd_is("M106")) {
            float fan = 0.0f;
            float pwm = 0.0f;
            if (line.has_value('P', fan) && std::isfinite(fan) &&
                (std::fabs(fan - 2.0f) <= 1e-6f || std::fabs(fan - 3.0f) <= 1e-6f)) {
                REQUIRE(line.has_value('S', pwm));
                REQUIRE(std::isfinite(pwm));
                REQUIRE(pwm >= 0.0f);
                REQUIRE(pwm <= 255.0f);
                REQUIRE(std::fabs(pwm - std::round(pwm)) <= 1e-4f);
                const int value = static_cast<int>(std::lround(pwm));
                if (std::fabs(fan - 2.0f) <= 1e-6f) {
                    current_p2_pwm = value;
                } else {
                    evidence.p3_pwm.push_back(value);
                    evidence.p3_ordinals.push_back(event);
                }
            }
        }

        if (!line.extruding(self) || line.dist_XY(self) <= EPSILON)
            return;
        if (!evidence.first_deposition_ordinal.has_value())
            evidence.first_deposition_ordinal = event;
        evidence.last_deposition_ordinal = event;
        evidence.p2_pwm_during_deposition.push_back(current_p2_pwm.value_or(-1));
        evidence.deposition_feeds.push_back(static_cast<double>(line.new_F(self)));
    });
    return evidence;
}

DynamicPrintConfig air_filtration_config(bool capability, bool master, bool during, bool completion)
{
    DynamicPrintConfig config = base_config();
    config.set_deserialize_strict({
        {"support_air_filtration", capability},
        {"activate_air_filtration", master ? "1" : "0"},
        {"activate_air_filtration_during_print", during ? "1" : "0"},
        {"activate_air_filtration_on_completion", completion ? "1" : "0"},
        {"during_print_exhaust_fan_speed", "35"},
        {"complete_print_exhaust_fan_speed", "65"},
        {"support_cooling_filter", false},
        {"cooling_filter_enabled", false},
        {"auxiliary_fan", false},
        {"additional_cooling_fan_speed", 0},
        {"enable_support", false},
        {"enable_prime_tower", false},
        {"enable_overhang_bridge_fan", false},
        {"overhang_fan_threshold", "95%"},
        {"overhang_fan_speed", 0},
        {"internal_bridge_fan_speed", -1},
        {"support_material_interface_fan_speed", -1},
        {"ironing_fan_speed", -1},
        {"fan_speedup_time", 0.0},
        {"fan_kickstart", 0.0},
        {"enable_arc_fitting", false},
        {"use_relative_e_distances", true},
        {"machine_start_gcode", ""},
        {"machine_end_gcode", ""},
        {"filament_start_gcode", ""},
        {"filament_end_gcode", ""},
    });
    return config;
}

void require_exhaust_fan_row(const ExhaustFanEvidence &commands,
                            const MotionEvidence &material,
                            const std::vector<int> &expected_pwm)
{
    REQUIRE(material.extrusion_moves > 0);
    REQUIRE(std::isfinite(material.xy_path));
    REQUIRE(std::isfinite(material.filament));
    REQUIRE(material.xy_path > 0.0);
    REQUIRE(material.filament > 0.0);
    REQUIRE(commands.first_deposition_ordinal.has_value());
    REQUIRE(commands.last_deposition_ordinal.has_value());
    REQUIRE(commands.p3_pwm == expected_pwm);
    REQUIRE(commands.p3_ordinals.size() == expected_pwm.size());
    for (size_t i = 0; i < expected_pwm.size(); ++i) {
        if (expected_pwm[i] == 89)
            REQUIRE(commands.p3_ordinals[i] < *commands.first_deposition_ordinal);
        if (expected_pwm[i] == 165)
            REQUIRE(commands.p3_ordinals[i] > *commands.last_deposition_ordinal);
    }
}

TEST_CASE("Air filtration capability gates directed exhaust fan phases", "[GCode][AirFiltration]")
{
    struct Row {
        const char *name;
        bool capability;
        bool master;
        bool during;
        bool completion;
        std::vector<int> expected_pwm;
    };
    const std::vector<Row> rows{
        {"all enabled", true, true, true, true, {89, 165}},
        {"capability disabled", false, true, true, true, {}},
        {"master activation disabled", true, false, true, true, {}},
        {"during phase disabled", true, true, false, true, {165}},
        {"completion phase disabled", true, true, true, false, {89}},
    };

    const DynamicPrintConfig reference_config = air_filtration_config(true, true, true, true);
    const ExportedPrint reference_output = fresh_custom_mesh_export(reference_config, make_cube(8.0, 8.0, 0.8));
    const ExhaustFanEvidence reference_commands = inspect_exhaust_fan_commands(reference_output.text, reference_config);
    const MotionEvidence reference_material = motion_evidence(reference_output.text, reference_config);
    require_exhaust_fan_row(reference_commands, reference_material, rows.front().expected_pwm);

    for (size_t i = 1; i < rows.size(); ++i) {
        CAPTURE(rows[i].name);
        const DynamicPrintConfig config = air_filtration_config(rows[i].capability, rows[i].master,
                                                                rows[i].during, rows[i].completion);
        const ExportedPrint output = fresh_custom_mesh_export(config, make_cube(8.0, 8.0, 0.8));
        const ExhaustFanEvidence commands = inspect_exhaust_fan_commands(output.text, config);
        const MotionEvidence material = motion_evidence(output.text, config);
        require_exhaust_fan_row(commands, material, rows[i].expected_pwm);
        CHECK_THAT(material.xy_path, Catch::Matchers::WithinRel(reference_material.xy_path, 1e-4));
        CHECK_THAT(material.filament, Catch::Matchers::WithinRel(reference_material.filament, 1e-4));
    }
}

TEST_CASE("Auxiliary and exhaust fan speed endpoints reach physical fan commands during deposition",
          "[GCode][CoolingFanBoundaries]")
{
    struct SpeedRow {
        const char *name;
        int auxiliary;
        int during;
        int completion;
    };
    const std::array<SpeedRow, 7> rows{{
        {"baseline", 40, 35, 65},
        {"auxiliary minimum", 0, 35, 65},
        {"auxiliary maximum", 100, 35, 65},
        {"during exhaust minimum", 40, 0, 65},
        {"during exhaust maximum", 40, 100, 65},
        {"completion exhaust minimum", 40, 35, 0},
        {"completion exhaust maximum", 40, 35, 100},
    }};

    const auto make_config = [](const SpeedRow &row) {
        DynamicPrintConfig config = air_filtration_config(true, true, true, true);
        config.set_deserialize_strict({
            {"auxiliary_fan", true},
            {"close_fan_the_first_x_layers", 0},
            {"additional_cooling_fan_speed", row.auxiliary},
            {"during_print_exhaust_fan_speed", std::to_string(row.during)},
            {"complete_print_exhaust_fan_speed", std::to_string(row.completion)},
            {"slow_down_for_layer_cooling", false},
        });
        return config;
    };

    const DynamicPrintConfig reference_config = make_config(rows.front());
    const ExportedPrint reference_output = fresh_custom_mesh_export(reference_config, make_cube(8.0, 8.0, 0.8));
    const ExhaustFanEvidence reference_commands = inspect_exhaust_fan_commands(reference_output.text, reference_config);
    const MotionEvidence reference_material = motion_evidence(reference_output.text, reference_config);
    const std::vector<int> reference_p3_pwm{35 * 255 / 100, 65 * 255 / 100};
    require_exhaust_fan_row(reference_commands, reference_material, reference_p3_pwm);
    REQUIRE(reference_commands.deposition_feeds.size() == reference_material.extrusion_moves);
    REQUIRE_FALSE(reference_commands.deposition_feeds.empty());
    for (double feed : reference_commands.deposition_feeds) {
        REQUIRE(std::isfinite(feed));
        REQUIRE(feed > 0.0);
    }

    for (size_t i = 0; i < rows.size(); ++i) {
        CAPTURE(rows[i].name, rows[i].auxiliary, rows[i].during, rows[i].completion);
        const DynamicPrintConfig config = make_config(rows[i]);
        const ExportedPrint output = i == 0 ? reference_output
                                            : fresh_custom_mesh_export(config, make_cube(8.0, 8.0, 0.8));
        const ExhaustFanEvidence commands = inspect_exhaust_fan_commands(output.text, config);
        const MotionEvidence material = motion_evidence(output.text, config);
        const int expected_p2_pwm = rows[i].auxiliary * 255 / 100;
        const std::vector<int> expected_p3_pwm{
            rows[i].during * 255 / 100,
            rows[i].completion * 255 / 100,
        };

        require_same_material(reference_material, material);
        require_exhaust_fan_row(commands, material, expected_p3_pwm);
        REQUIRE(commands.p3_ordinals.size() == 2);
        CHECK(commands.p3_ordinals.front() < *commands.first_deposition_ordinal);
        CHECK(commands.p3_ordinals.back() > *commands.last_deposition_ordinal);

        REQUIRE(commands.p2_pwm_during_deposition.size() == material.extrusion_moves);
        CHECK(std::all_of(commands.p2_pwm_during_deposition.begin(), commands.p2_pwm_during_deposition.end(),
                          [expected_p2_pwm](int pwm) { return pwm == expected_p2_pwm; }));
        REQUIRE(commands.deposition_feeds.size() == reference_commands.deposition_feeds.size());
        REQUIRE_FALSE(commands.deposition_feeds.empty());
        for (size_t move = 0; move < commands.deposition_feeds.size(); ++move)
            CHECK_THAT(commands.deposition_feeds[move],
                       Catch::Matchers::WithinAbs(reference_commands.deposition_feeds[move], 1e-3));
    }
}

} // namespace gcode_print_variance_contracts

namespace gcode_broad_contracts {
using namespace Slic3r;
using namespace Slic3r::Test;

namespace {

class TestGCode : public GCode
{
public:
    using GCode::change_layer;
    using GCode::cur_extruder_index;
    using GCode::mass_load_limited_machine_acceleration;
    using GCode::calc_max_volumetric_speed;
    using GCode::set_extruders;
    using GCode::set_last_pos;
    using GCode::to_lift_type;
};

DynamicPrintConfig base_config()
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        {"gcode_flavor", "marlin2"},
        {"use_relative_e_distances", true},
        {"layer_height", 0.2},
        {"initial_layer_print_height", 0.2},
        {"nozzle_diameter", 0.4},
        {"wall_loops", 1},
        {"top_shell_layers", 1},
        {"bottom_shell_layers", 1},
        {"top_shell_thickness", 0.0},
        {"bottom_shell_thickness", 0.0},
        {"sparse_infill_density", "20%"},
        {"line_width", 0.5},
        {"initial_layer_line_width", 0.5},
        {"outer_wall_line_width", 0.5},
        {"inner_wall_line_width", 0.5},
        {"sparse_infill_line_width", 0.5},
        {"internal_solid_infill_line_width", 0.5},
        {"top_surface_line_width", 0.5},
        {"skin_infill_line_width", 0.5},
        {"skirt_loops", 0},
        {"brim_type", "no_brim"},
        {"raft_layers", 0},
        {"machine_start_gcode", ""},
        {"machine_end_gcode", ""},
        {"file_start_gcode", ""},
        {"before_layer_change_gcode", ""},
        {"layer_change_gcode", "G92 E0\n"},
        {"printing_by_object_gcode", ""},
        {"gcode_label_objects", false},
        {"exclude_object", false},
        {"gcode_comments", true},
        {"slow_down_for_layer_cooling", false},
        {"fan_cooling_layer_time", 0.0},
        {"slow_down_layer_time", 0.0},
        {"retraction_length", 0.8},
        {"retraction_minimum_travel", 0.0},
        {"z_hop", 0.0},
    });
    return config;
}

void require_valid(const Print& print)
{
    const StringObjectException error = print.validate();
    INFO("print validation error: " << error.string << " [" << error.opt_key << "]");
    REQUIRE(error.string.empty());
}

struct Exported
{
    std::string text;
    std::unique_ptr<GCodeProcessorResult> result;
};

Exported export_cube(const DynamicPrintConfig& config, TestMesh mesh_id = TestMesh::A, bool bbl = false)
{
    Print print;
    Model model;
    init_print({mesh_id}, print, model, config);
    print.is_BBL_printer() = bbl;
    require_valid(print);
    print.set_status_silent();
    print.process();
    ScopedTemporaryFile temp(".gcode");
    Exported result;
    result.result = std::make_unique<GCodeProcessorResult>();
    print.export_gcode(temp.string(), result.result.get(), nullptr);
    std::ifstream input(temp.string(), std::ios::binary);
    result.text.assign(std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>());
    REQUIRE_FALSE(result.text.empty());
    return result;
}

struct Motion
{
    size_t extrusion_moves = 0;
    size_t z_only_moves    = 0;
    double xy_path         = 0.0;
    double filament        = 0.0;
    double max_feed        = 0.0;
    std::map<std::string, size_t> commands;
};

Motion motion(const std::string& text, const DynamicPrintConfig& config)
{
    Motion result;
    GCodeReader reader;
    reader.apply_config(config);
    reader.parse_buffer(text, [&result](GCodeReader& self, const GCodeReader::GCodeLine& line) {
        const std::string command(line.cmd());
        if (!command.empty())
            ++result.commands[command];
        const double xy = line.dist_XY(self);
        if (std::abs(line.dist_Z(self)) > EPSILON && xy <= EPSILON)
            ++result.z_only_moves;
        if (!line.extruding(self) || xy <= EPSILON)
            return;
        ++result.extrusion_moves;
        result.xy_path += xy;
        result.filament += std::abs(line.dist_E(self));
        result.max_feed = std::max(result.max_feed, static_cast<double>(line.new_F(self)));
    });
    return result;
}

size_t count_command(const std::string& text, const DynamicPrintConfig& config, const char* command)
{
    size_t count = 0;
    GCodeReader reader;
    reader.apply_config(config);
    reader.parse_buffer(text, [&count, command](GCodeReader&, const GCodeReader::GCodeLine& line) {
        if (line.cmd_is(command))
            ++count;
    });
    return count;
}

PrintConfig direct_print_config()
{
    DynamicPrintConfig dynamic = base_config();
    PrintConfig config;
    config.apply(dynamic, true);
    config.extruder_offset.values     = {Vec2d(1.25, -2.5)};
    config.filament_diameter.values   = {1.75};
    config.filament_flow_ratio.values = {1.0};
    return config;
}

void init_direct_gcode(TestGCode& gcode, bool relative_e = true)
{
    PrintConfig config                    = direct_print_config();
    config.use_relative_e_distances.value = relative_e;
    gcode.apply_print_config(config);
    gcode.set_extruders({0});
    gcode.writer().set_extruder(0);
}

size_t count_substring(const std::string& text, std::string_view needle)
{
    size_t count = 0;
    for (size_t pos = text.find(needle); pos != std::string::npos; pos = text.find(needle, pos + needle.size()))
        ++count;
    return count;
}

std::vector<std::string> command_lines(const std::string& text, const std::string& prefix)
{
    std::istringstream stream(text);
    std::vector<std::string> result;
    for (std::string line; std::getline(stream, line);)
        if (line.rfind(prefix, 0) == 0)
            result.push_back(line);
    return result;
}

std::vector<Vec3d> snapshot_positions(const std::string& text, const DynamicPrintConfig& config)
{
    std::vector<Vec3d> positions;
    GCodeReader reader;
    reader.apply_config(config);
    reader.parse_buffer(text, [&positions](GCodeReader& state, const GCodeReader::GCodeLine& line) {
        if (line.cmd_is("M117"))
            positions.emplace_back(state.x(), state.y(), state.z());
    });
    return positions;
}

} // namespace

TEST_CASE("GCode coordinate conversion round-trips the configured origin and nozzle offset", "[GCode][GCodeBroad]")
{
    TestGCode gcode;
    init_direct_gcode(gcode);
    gcode.set_origin(Vec2d(10.5, -4.25));
    const Point point      = Point::new_scale(12.345, 6.789);
    const Vec2d emitted    = gcode.point_to_gcode(point);
    const Point round_trip = gcode.gcode_to_point(emitted);
    CHECK_THAT(unscale<double>(round_trip.x()), Catch::Matchers::WithinAbs(12.345, 1e-6));
    CHECK_THAT(unscale<double>(round_trip.y()), Catch::Matchers::WithinAbs(6.789, 1e-6));
    CHECK_THAT(emitted.x(), Catch::Matchers::WithinAbs(21.595, 1e-6));
    CHECK_THAT(emitted.y(), Catch::Matchers::WithinAbs(5.039, 1e-6));
}

TEST_CASE("GCode quantized coordinates stay within formatter precision", "[GCode][GCodeBroad]")
{
    TestGCode gcode;
    init_direct_gcode(gcode);
    gcode.set_origin(Vec2d(-2.0, 3.0));
    const Point point     = Point::new_scale(4.123456, -8.765432);
    const Vec2d raw       = gcode.point_to_gcode(point);
    const Vec2d quantized = gcode.point_to_gcode_quantized(point);
    CHECK_THAT(quantized.x(), Catch::Matchers::WithinAbs(raw.x(), 0.001));
    CHECK_THAT(quantized.y(), Catch::Matchers::WithinAbs(raw.y(), 0.001));
    CHECK(std::isfinite(quantized.x()));
}

TEST_CASE("Changing GCode origin preserves the model-space last position delta", "[GCode][GCodeBroad]")
{
    TestGCode gcode;
    init_direct_gcode(gcode);
    gcode.set_origin(Vec2d(2.0, 3.0));
    const Point before = Point::new_scale(15.0, 16.0);
    gcode.set_last_pos(before);
    gcode.set_origin(Vec2d(7.0, -1.0));
    const Point after = gcode.last_pos();
    CHECK_THAT(unscale<double>(after.x() - before.x()), Catch::Matchers::WithinAbs(-5.0, 1e-6));
    CHECK_THAT(unscale<double>(after.y() - before.y()), Catch::Matchers::WithinAbs(4.0, 1e-6));
}

TEST_CASE("GCode layer interpolation handles initial, gradual, and stepped bands", "[GCode][GCodeBroad]")
{
    TestGCode gcode;
    init_direct_gcode(gcode);
    gcode.set_layer_count(5);
    CHECK_THAT(gcode.interpolate_value_across_layers(10.f, 20.f), Catch::Matchers::WithinAbs(10.f, 1e-6));
    gcode.change_layer(0.2);
    CHECK_THAT(gcode.interpolate_value_across_layers(10.f, 20.f), Catch::Matchers::WithinAbs(10.f, 1e-6));
    gcode.change_layer(0.4);
    CHECK_THAT(gcode.interpolate_value_across_layers(10.f, 20.f), Catch::Matchers::WithinAbs(10.f, 1e-6));
    CHECK_THAT(gcode.interpolate_value_across_layers(10.f, 20.f, 5.f), Catch::Matchers::WithinAbs(10.f, 1e-6));
    gcode.change_layer(0.6);
    CHECK_THAT(gcode.interpolate_value_across_layers(10.f, 20.f), Catch::Matchers::WithinAbs(15.f, 1e-6));
    CHECK_THAT(gcode.interpolate_value_across_layers(10.f, 20.f, 5.f), Catch::Matchers::WithinAbs(15.f, 1e-6));
    gcode.change_layer(0.8);
    CHECK_THAT(gcode.interpolate_value_across_layers(10.f, 20.f), Catch::Matchers::WithinAbs(17.5f, 1e-6));
    gcode.change_layer(1.0);
    CHECK_THAT(gcode.interpolate_value_across_layers(10.f, 20.f), Catch::Matchers::WithinAbs(20.f, 1e-6));
}

TEST_CASE("GCode extrusion-role parser names preserve every printable role", "[GCode][GCodeBroad]")
{
    TestGCode gcode;
    const std::vector<std::pair<ExtrusionRole, std::string>> roles = {
        {erExternalPerimeter, "ExternalPerimeter"},
        {erPerimeter, "Perimeter"},
        {erOverhangPerimeter, "OverhangPerimeter"},
        {erInternalInfill, "InternalInfill"},
        {erSolidInfill, "SolidInfill"},
        {erTopSolidInfill, "TopSolidInfill"},
        {erBottomSurface, "BottomSurface"},
        {erBridgeInfill, "BridgeInfill"},
        {erInternalBridgeInfill, "BridgeInfill"},
        {erGapFill, "GapFill"},
        {erIroning, "Ironing"},
        {erSkirt, "Skirt"},
        {erBrim, "Brim"},
        {erSupportMaterial, "SupportMaterial"},
        {erSupportMaterialInterface, "SupportMaterialInterface"},
        {erSupportTransition, "SupportTransition"},
        {erWipeTower, "WipeTower"},
    };
    for (const auto& [role, expected] : roles) {
        INFO("role " << expected);
        CHECK(gcode.extrusion_role_to_string_for_parser(role) == expected);
    }
    CHECK(gcode.extrusion_role_to_string_for_parser(erNone) == "Mixed");
    CHECK(gcode.extrusion_role_to_string_for_parser(erCustom) == "Mixed");
}

TEST_CASE("GCode lift policy maps every configured Z-hop mode", "[GCode][GCodeBroad]")
{
    TestGCode gcode;
    CHECK(gcode.to_lift_type(ZHopType::zhtNormal) == LiftType::NormalLift);
    CHECK(gcode.to_lift_type(ZHopType::zhtSlope) == LiftType::SlopeLift);
    CHECK(gcode.to_lift_type(ZHopType::zhtSpiral) == LiftType::SpiralLift);
    CHECK(gcode.to_lift_type(ZHopType::zhtAuto) == LiftType::NormalLift);
}

TEST_CASE("GCode mass load acceleration is capped by machine force and bed mass", "[GCode][GCodeBroad]")
{
    DynamicPrintConfig config = base_config();
    config.set_deserialize_strict({{"machine_max_acceleration_y", "1000"}, {"machine_max_force_Y", 0.001}, {"machine_bed_mass_Y", 10.0}});
    Print print;
    Model model;
    init_print({cube(10)}, print, model, config);
    TestGCode gcode;
    PrintStatistics stats;
    stats.total_weight = 100.0;
    double limit       = 0.0;
    double mass        = 0.0;
    gcode.mass_load_limited_machine_acceleration(stats, print, limit, mass);
    CHECK_THAT(mass, Catch::Matchers::WithinAbs(100.0, 1e-6));
    CHECK_THAT(limit, Catch::Matchers::WithinAbs(1000.0 / 110.0, 1e-6));
}

TEST_CASE("GCode mass load acceleration falls back to configured limit without force", "[GCode][GCodeBroad]")
{
    DynamicPrintConfig config = base_config();
    config.set_deserialize_strict({{"machine_max_acceleration_y", "777"}, {"machine_max_force_Y", 0.0}, {"machine_bed_mass_Y", 100.0}});
    Print print;
    Model model;
    init_print({cube(10)}, print, model, config);
    TestGCode gcode;
    PrintStatistics stats;
    stats.total_weight = 0.0;
    double limit       = 0.0;
    double mass        = 0.0;
    gcode.mass_load_limited_machine_acceleration(stats, print, limit, mass);
    CHECK_THAT(mass, Catch::Matchers::WithinAbs(0.0, 1e-6));
    CHECK_THAT(limit, Catch::Matchers::WithinAbs(777.0, 1e-6));
}

TEST_CASE("GCode zero Y force keeps the configured acceleration with printed mass", "[GCode][GCodeBroad]")
{
    struct Acceleration {
        double limit = 0.0;
        double printed_mass = 0.0;
    };
    const auto measure = [](double force) {
        DynamicPrintConfig config = base_config();
        config.set_deserialize_strict({
            {"machine_max_acceleration_y", "777"},
            {"machine_max_force_Y", force},
            {"machine_bed_mass_Y", 100.0},
        });
        Print print;
        Model model;
        init_print({cube(10)}, print, model, config);
        TestGCode gcode;
        PrintStatistics stats;
        stats.total_weight = 100.0;
        Acceleration observed;
        gcode.mass_load_limited_machine_acceleration(stats, print, observed.limit, observed.printed_mass);
        return observed;
    };

    // Both rows share the bed mass, configured limit, and positive printed mass.
    const Acceleration no_force = measure(0.0);
    const Acceleration with_force = measure(0.001);
    REQUIRE_THAT(no_force.printed_mass, Catch::Matchers::WithinAbs(100.0, 1e-6));
    REQUIRE_THAT(with_force.printed_mass, Catch::Matchers::WithinAbs(no_force.printed_mass, 1e-6));
    CHECK_THAT(no_force.limit, Catch::Matchers::WithinAbs(777.0, 1e-6));
    // 0.001 N gives 1000 g*mm/s^2; divide by 100 g bed + 100 g print.
    CHECK_THAT(with_force.limit, Catch::Matchers::WithinAbs(5.0, 1e-6));
}

TEST_CASE("GCode object metadata selects the supported firmware label protocol", "[GCode][GCodeBroad]")
{
    const std::vector<std::pair<std::string, std::string>> flavors = {
        {"marlin", "M486 S"}, {"marlin2", "M486 S"}, {"reprapfirmware", "M486 S"}, {"klipper", "EXCLUDE_OBJECT_DEFINE NAME="},
        {"repetier", ""},
    };
    for (const auto& [flavor, marker] : flavors) {
        INFO("flavor " << flavor);
        DynamicPrintConfig config = base_config();
        config.set_deserialize_strict({{"gcode_flavor", flavor}, {"exclude_object", true}});
        Print print;
        Model model;
        init_print({cube(10), cube(10)}, print, model, config);
        TestGCode gcode;
        PrintConfig full;
        full.apply(config, true);
        gcode.apply_print_config(full);
        gcode.set_extruders(print.extruders());
        const std::string labels = gcode.set_object_info(&print);
        if (marker.empty()) {
            CHECK(labels.empty());
        } else {
            CHECK(labels.find(marker) != std::string::npos);
        }
        CHECK(labels.find("object_id") == std::string::npos);
    }
}

TEST_CASE("GCode full config dump contains applied settings but omits device-only keys", "[GCode][GCodeBroad]")
{
    DynamicPrintConfig config = base_config();
    config.set_deserialize_strict({{"farthest_point_timelapse", true}, {"machine_start_gcode", "M117 START\n"}});
    Print print;
    Model model;
    init_print({cube(10)}, print, model, config);
    std::string dumped;
    GCode::append_full_config(print, dumped);
    REQUIRE_FALSE(dumped.empty());
    CHECK(dumped.find("machine_start_gcode") != std::string::npos);
    CHECK(dumped.find("farthest_point_timelapse") == std::string::npos);
    CHECK(dumped.find("compatible_printers") == std::string::npos);
}

TEST_CASE("GCode volumetric polynomial evaluates six coefficients and rejects malformed models", "[GCode][GCodeBroad]")
{
    TestGCode gcode;
    init_direct_gcode(gcode);
    CHECK_THAT(gcode.calc_max_volumetric_speed(0.2, 0.5, "1 2 3 4 5 6"), Catch::Matchers::WithinAbs(10.14, 1e-9));
    CHECK_THAT(gcode.calc_max_volumetric_speed(0.2, 0.5, "0 0 0 0 0 1"), Catch::Matchers::WithinAbs(1.0, 1e-9));
    CHECK(gcode.calc_max_volumetric_speed(0.2, 0.5, "1 2 3") >= std::numeric_limits<double>::max() / 2.0);
    CHECK(gcode.calc_max_volumetric_speed(0.2, 0.5, "0 0 0 0 0 0") >= std::numeric_limits<double>::max() / 2.0);
}

TEST_CASE("Adaptive volumetric speed caps otherwise unlimited sparse infill", "[GCode][GCodeBroad]")
{
    DynamicPrintConfig baseline = base_config();
    baseline.set_deserialize_strict({
        {"wall_loops", 0},
        {"top_shell_layers", 0},
        {"bottom_shell_layers", 0},
        {"sparse_infill_density", "25%"},
        {"sparse_infill_speed", 0.0},
        {"filament_max_volumetric_speed", 1000.0},
        {"filament_adaptive_volumetric_speed", false},
    });
    DynamicPrintConfig adaptive = baseline;
    adaptive.set_deserialize_strict({
        {"filament_adaptive_volumetric_speed", true},
        {"volumetric_speed_coefficients", "0 0 0 0 0 1"},
    });
    const Exported a = export_cube(baseline, TestMesh::A);
    const Exported b = export_cube(adaptive, TestMesh::A);
    const Motion ma  = motion(a.text, baseline);
    const Motion mb  = motion(b.text, adaptive);
    REQUIRE(ma.extrusion_moves > 0);
    REQUIRE(mb.extrusion_moves > 0);
    CHECK_THAT(mb.filament, Catch::Matchers::WithinRel(ma.filament, 1e-6));
    CHECK(mb.max_feed < ma.max_feed);
    CHECK(count_command(b.text, adaptive, "G1") > 0);
    CHECK(mb.xy_path > 0.0);
}

TEST_CASE("Farthest-point timelapse changes the emitted snapshot sequence on CoreXY", "[GCode][GCodeBroad]")
{
    DynamicPrintConfig off = base_config();
    off.set_deserialize_strict({
        {"printer_structure", "corexy"},
        {"timelapse_type", "0"},
        {"time_lapse_gcode", "M117 SNAPSHOT-[layer_num]\n"},
        {"farthest_point_timelapse", false},
    });
    DynamicPrintConfig on = off;
    on.set_deserialize_strict({{"farthest_point_timelapse", true}});
    const Exported a    = export_cube(off, TestMesh::A);
    const Exported b    = export_cube(on, TestMesh::A);
    const auto disabled = snapshot_positions(a.text, off);
    const auto enabled  = snapshot_positions(b.text, on);
    REQUIRE_FALSE(disabled.empty());
    REQUIRE_FALSE(enabled.empty());
    bool changed = disabled.size() != enabled.size();
    for (size_t i = 0; i < std::min(disabled.size(), enabled.size()); ++i)
        changed = changed || (disabled[i] - enabled[i]).norm() > 1e-5;
    CHECK(changed);
}

TEST_CASE("Farthest-point timelapse is gated off for I3 and smooth timelapse", "[GCode][GCodeBroad]")
{
    DynamicPrintConfig corexy = base_config();
    corexy.set_deserialize_strict({
        {"printer_structure", "corexy"},
        {"timelapse_type", "0"},
        {"time_lapse_gcode", "M117 SNAPSHOT-[layer_num]\n"},
        {"farthest_point_timelapse", false},
    });
    for (const auto& [key, value] : std::vector<std::pair<std::string, std::string>>{{"printer_structure", "i3"}, {"timelapse_type", "1"}}) {
        CAPTURE(key, value);
        DynamicPrintConfig off = corexy;
        off.set_deserialize_strict(key, value);
        DynamicPrintConfig on = off;
        on.set_deserialize_strict("farthest_point_timelapse", "1");
        const Exported a = export_cube(off), b = export_cube(on);
        const auto disabled = snapshot_positions(a.text, off), gated = snapshot_positions(b.text, on);
        REQUIRE_FALSE(disabled.empty());
        REQUIRE(gated.size() == disabled.size());
        for (size_t i = 0; i < disabled.size(); ++i)
            CHECK_THAT((gated[i] - disabled[i]).norm(), Catch::Matchers::WithinAbs(0.0, 1e-5));
    }
}

TEST_CASE("Belt brim leading and sideways widths increase belt apron motion", "[GCode][GCodeBroad]")
{
    DynamicPrintConfig base = base_config();
    base.set_deserialize_strict({
        {"belt_printer", true},
        {"belt_slice_rotation", "x"},
        {"belt_slice_rotation_angle", 45.0},
        {"brim_type", "outer_only"},
        {"brim_width", 1.0},
        {"leading_brim_length", 0.0},
        {"extra_brim_width", 0.0},
    });
    DynamicPrintConfig leading = base;
    leading.set_deserialize_strict({{"leading_brim_length", 5.0}});
    DynamicPrintConfig sideways = base;
    sideways.set_deserialize_strict({{"extra_brim_width", 5.0}});
    const Exported plain = export_cube(base, TestMesh::step);
    const Exported lead  = export_cube(leading, TestMesh::step);
    const Exported side  = export_cube(sideways, TestMesh::step);
    const Motion mp      = motion(plain.text, base);
    const Motion ml      = motion(lead.text, leading);
    const Motion ms      = motion(side.text, sideways);
    REQUIRE(mp.extrusion_moves > 0);
    REQUIRE(ml.extrusion_moves > 0);
    REQUIRE(ms.extrusion_moves > 0);
    CHECK(ml.xy_path > mp.xy_path);
    CHECK(ms.xy_path > mp.xy_path);
}

TEST_CASE("GCode belt configuration leaves the nonbelt exporter path unchanged", "[GCode][GCodeBroad]")
{
    DynamicPrintConfig normal    = base_config();
    DynamicPrintConfig belt_keys = normal;
    belt_keys.set_deserialize_strict({
        {"belt_printer", false},
        {"belt_printer_infinite_y", true},
        {"belt_slice_rotation", "y"},
        {"belt_slice_rotation_angle", 30.0},
        {"belt_frame_tilt_decouple", true},
        {"belt_frame_tilt_angle", 25.0},
        {"gcode_remap_x", "pos_y"},
        {"gcode_remap_y", "pos_z"},
        {"gcode_remap_z", "pos_x"},
        {"belt_support_floor_offset", -5.0},
    });
    const Exported a = export_cube(normal, TestMesh::A);
    const Exported b = export_cube(belt_keys, TestMesh::A);
    const Motion ma  = motion(a.text, normal);
    const Motion mb  = motion(b.text, belt_keys);
    REQUIRE(ma.extrusion_moves > 0);
    REQUIRE(mb.extrusion_moves > 0);
    CHECK(ma.extrusion_moves == mb.extrusion_moves);
    CHECK_THAT(ma.xy_path, Catch::Matchers::WithinRel(mb.xy_path, 1e-6));
    CHECK_THAT(ma.filament, Catch::Matchers::WithinRel(mb.filament, 1e-6));
}

TEST_CASE("GCode pressure advance settings add a firmware-specific command", "[GCode][GCodeBroad]")
{
    DynamicPrintConfig disabled = base_config();
    disabled.set_deserialize_strict({{"gcode_flavor", "klipper"}, {"enable_pressure_advance", false}, {"pressure_advance", 0.55}});
    DynamicPrintConfig enabled = disabled;
    enabled.set_deserialize_strict({{"enable_pressure_advance", true}, {"pressure_advance", 0.55}});
    const Exported a = export_cube(disabled);
    const Exported b = export_cube(enabled);
    CHECK(a.text.find("SET_PRESSURE_ADVANCE") == std::string::npos);
    const auto commands = command_lines(b.text, "SET_PRESSURE_ADVANCE ");
    REQUIRE_FALSE(commands.empty());
    for (const auto& line : commands) {
        const auto marker = line.find("ADVANCE=");
        REQUIRE(marker != std::string::npos);
        CHECK_THAT(std::stod(line.substr(marker + 8)), Catch::Matchers::WithinAbs(0.55, 1e-5));
    }
    CHECK(motion(a.text, disabled).extrusion_moves > 0);
    CHECK(motion(b.text, enabled).extrusion_moves > 0);
}

TEST_CASE("GCode firmware flavor changes command dialect without changing deposited path", "[GCode][GCodeBroad]")
{
    struct FlavorDialect {
        std::string_view flavor;
        std::string_view acceleration;
        std::string_view pressure_advance;
    };
    constexpr std::array<FlavorDialect, 5> flavors{{
        {"marlin",         "M204 S300",                    "M900 K0.55"},
        {"marlin2",        "M204 P300",                    "M900 K0.55"},
        {"klipper",        "SET_VELOCITY_LIMIT ACCEL=300", "SET_PRESSURE_ADVANCE ADVANCE=0.55"},
        {"reprapfirmware", "M204 P300",                    "M572 D0 S0.55"},
        {"repetier",       "M201 X300 Y300",               "M233 X0.55 Y0.55"},
    }};

    DynamicPrintConfig common = base_config();
    common.set_deserialize_strict({
        {"default_acceleration", 1200},
        {"initial_layer_acceleration", 300},
        {"travel_acceleration", 4000},
        {"default_jerk", 10},
        {"travel_jerk", 12},
        {"enable_pressure_advance", true},
        {"pressure_advance", 0.55},
    });

    std::optional<Motion> reference_motion;
    for (const FlavorDialect& dialect : flavors) {
        INFO("flavor " << dialect.flavor);
        DynamicPrintConfig config = common;
        config.set_deserialize_strict("gcode_flavor", std::string(dialect.flavor));
        if (dialect.flavor == "reprapfirmware" || dialect.flavor == "repetier")
            config.set_deserialize_strict("layer_change_gcode", "");

        Print print;
        Model model;
        TriangleMesh mesh = Test::mesh(TestMesh::A);
        const double original_height = mesh.bounding_box().size().z();
        REQUIRE(original_height > 0.0);
        mesh.scale(Vec3f(1.f, 1.f, static_cast<float>(1.2 / original_height)));
        init_print({std::move(mesh)}, print, model, config);
        require_valid(print);
        const std::string text = gcode(print);
        REQUIRE_FALSE(text.empty());
        CHECK(count_substring(text, ";LAYER_CHANGE") >= 6);

        const Motion actual = motion(text, config);
        REQUIRE(actual.extrusion_moves > 0);
        CHECK(text.find("G1") != std::string::npos);
        CHECK(text.find(dialect.acceleration) != std::string::npos);
        CHECK(text.find(dialect.pressure_advance) != std::string::npos);

        if (reference_motion.has_value()) {
            CHECK(actual.extrusion_moves == reference_motion->extrusion_moves);
            CHECK_THAT(actual.xy_path, Catch::Matchers::WithinRel(reference_motion->xy_path, 1e-6));
            CHECK_THAT(actual.filament, Catch::Matchers::WithinRel(reference_motion->filament, 1e-6));
        } else {
            reference_motion = actual;
        }
    }
}

TEST_CASE("GCode machine load placeholders expose the current acceleration limit", "[GCode][GCodeBroad]")
{
    DynamicPrintConfig config = base_config();
    config.set_deserialize_strict({
        {"filament_density", 1.24},
        {"machine_max_acceleration_y", "1000"},
        {"machine_max_force_Y", 0.001},
        {"machine_bed_mass_Y", 10.0},
        {"layer_change_gcode", "G92 E0\nM117 LIM-[curr_y_acceleration_limit]-M-[curr_accumulated_mass]\n"},
    });
    const Exported out       = export_cube(config, TestMesh::A);
    const size_t limit_lines = count_substring(out.text, "M117 LIM-");
    REQUIRE(limit_lines > 1);
    CHECK(count_substring(out.text, "LIM-1000") < limit_lines);
    CHECK(out.text.find("M-") != std::string::npos);
}

TEST_CASE("GCode fan and slowdown branches preserve material while changing feeds", "[GCode][GCodeBroad]")
{
    DynamicPrintConfig fast = base_config();
    fast.set_deserialize_strict({
        {"slow_down_for_layer_cooling", false},
        {"fan_cooling_layer_time", 0.0},
        {"slow_down_layer_time", 0.0},
        {"fan_min_speed", 100},
        {"fan_max_speed", 100},
    });
    DynamicPrintConfig slow = fast;
    slow.set_deserialize_strict({
        {"slow_down_for_layer_cooling", true},
        {"fan_cooling_layer_time", 1001.0},
        {"slow_down_layer_time", 1000.0},
        {"slow_down_min_speed", 5.0},
        {"slow_down_layers", 3},
    });
    const Exported a = export_cube(fast, TestMesh::A);
    const Exported b = export_cube(slow, TestMesh::A);
    const Motion ma  = motion(a.text, fast);
    const Motion mb  = motion(b.text, slow);
    REQUIRE(ma.extrusion_moves > 0);
    REQUIRE(mb.extrusion_moves > 0);
    CHECK_THAT(mb.xy_path, Catch::Matchers::WithinRel(ma.xy_path, 1e-6));
    CHECK_THAT(mb.filament, Catch::Matchers::WithinRel(ma.filament, 1e-6));
    CHECK(mb.max_feed < ma.max_feed);
    CHECK(a.text != b.text);
}

TEST_CASE("GCode acceleration and jerk settings emit firmware commands", "[GCode][GCodeBroad]")
{
    DynamicPrintConfig config = base_config();
    config.set_deserialize_strict({
        {"default_acceleration", 1200},
        {"default_jerk", 8},
        {"outer_wall_acceleration", 333},
        {"outer_wall_jerk", 9},
        {"gcode_flavor", "marlin2"},
    });
    const Exported out = export_cube(config);
    REQUIRE(out.text.find("M204") != std::string::npos);
    REQUIRE(out.text.find("M205") != std::string::npos);
    CHECK(motion(out.text, config).extrusion_moves > 0);
}

TEST_CASE("Fresh GCode exports enable object labels consistently", "[GCode][GCodeBroad]")
{
    DynamicPrintConfig config = base_config();
    config.set_deserialize_strict({{"exclude_object", true}, {"gcode_flavor", "klipper"}});
    const Exported first  = export_cube(config, TestMesh::A);
    const Exported second = export_cube(config, TestMesh::A);
    REQUIRE(count_substring(first.text, "EXCLUDE_OBJECT_DEFINE") > 0);
    REQUIRE(count_substring(second.text, "EXCLUDE_OBJECT_DEFINE") > 0);
    CHECK(count_substring(first.text, "EXCLUDE_OBJECT_DEFINE") == count_substring(second.text, "EXCLUDE_OBJECT_DEFINE"));
    REQUIRE(first.result != nullptr);
    REQUIRE(second.result != nullptr);
    CHECK(first.result->label_object_enabled);
    CHECK(second.result->label_object_enabled);
}

TEST_CASE("GCode processor result reports output filament statistics for a real export", "[GCode][GCodeBroad]")
{
    DynamicPrintConfig config = base_config();
    config.set_deserialize_strict({{"filament_diameter", 1.75}, {"filament_density", 1.24}, {"filament_cost", 100.0}});
    const Exported out = export_cube(config);
    REQUIRE(out.result != nullptr);
    REQUIRE_FALSE(out.result->print_statistics.total_volumes_per_extruder.empty());
    double total_volume = 0.0;
    for (const auto& [extruder, volume] : out.result->print_statistics.total_volumes_per_extruder) {
        (void) extruder;
        CHECK(volume > 0.0);
        total_volume += volume;
    }
    CHECK(total_volume > 0.0);
    CHECK(out.result->print_statistics.modes[static_cast<size_t>(PrintEstimatedStatistics::ETimeMode::Normal)].time > 0.0f);
}

TEST_CASE("Clumping detection exports layer metadata and restores deposition height", "[GCode][WrappingDetectionGCode]")
{
    const std::string hook =
        "M117 WRAP_META {layer_num} {layer_z} {max_layer_z} "
        "{most_used_physical_extruder_id} {curr_physical_extruder_id}\n"
        "G1 Z{layer_z + 0.7} F600\n";
    struct Mode { bool enabled; bool has_hook; };
    const std::array<Mode, 3> modes{{{false, true}, {true, true}, {true, false}}};
    double baseline_filament = 0.;
    double baseline_xy = 0.;
    for (size_t mode_index = 0; mode_index < modes.size(); ++mode_index) {
        const Mode mode = modes[mode_index];
        INFO("enabled=" << mode.enabled << " has_hook=" << mode.has_hook);
        DynamicPrintConfig config = base_config();
        config.set_deserialize_strict({
            {"layer_height", 0.2}, {"initial_layer_print_height", 0.2},
            {"wall_loops", 1}, {"top_shell_layers", 0}, {"bottom_shell_layers", 0},
            {"sparse_infill_density", "0%"}, {"enable_support", false},
            {"print_sequence", "by layer"}, {"enable_prime_tower", false},
            {"enable_wrapping_detection", mode.enabled},
            {"wrapping_detection_gcode", mode.has_hook ? hook : std::string{}},
            {"physical_extruder_map", "0"},
        });
        Print print;
        Model model;
        init_print({make_cube(8., 8., 1.2)}, print, model, config);
        require_valid(print);
        REQUIRE_FALSE(print.has_wipe_tower());
        const std::string output = gcode(print);
        REQUIRE(print.objects().size() == 1);
        REQUIRE(print.objects().front()->layer_count() == 6);

        size_t hook_count = 0;
        std::istringstream lines(output);
        std::string line;
        while (std::getline(lines, line)) {
            if (line.rfind("M117 WRAP_META ", 0) != 0)
                continue;
            std::istringstream fields(line.substr(std::string("M117 WRAP_META ").size()));
            int layer_number = -1, most_used = -1, current = -1;
            double layer_z = -1., max_z = -1.;
            REQUIRE(static_cast<bool>(fields >> layer_number >> layer_z >> max_z >> most_used >> current));
            ++hook_count;
            CHECK(layer_number == static_cast<int>(hook_count) - 1);
            CHECK_THAT(layer_z, Catch::Matchers::WithinAbs(0.2 * hook_count, 1e-5));
            CHECK_THAT(max_z, Catch::Matchers::WithinAbs(0.2 * hook_count, 1e-5));
            CHECK(most_used == 0);
            CHECK(current == 0);
        }
        const size_t expected_hooks = mode.enabled && mode.has_hook ? 6 : 0;
        CHECK(hook_count == expected_hooks);

        GCodeReader reader;
        reader.apply_config(print.config());
        std::set<int> deposition_layers;
        double filament = 0., xy = 0.;
        reader.parse_buffer(output, [&](GCodeReader &self, const GCodeReader::GCodeLine &move) {
            if (!move.extruding(self) || move.dist_XY(self) <= 1e-6)
                return;
            const double z = move.new_Z(self);
            REQUIRE(std::isfinite(z));
            const int layer = static_cast<int>(std::llround(z / 0.2));
            REQUIRE(layer >= 1);
            REQUIRE(layer <= 6);
            CHECK_THAT(z, Catch::Matchers::WithinAbs(0.2 * layer, 1e-4));
            deposition_layers.insert(layer);
            filament += move.dist_E(self);
            xy += move.dist_XY(self);
        });
        const std::set<int> expected_layers{1, 2, 3, 4, 5, 6};
        CHECK(deposition_layers == expected_layers);
        REQUIRE(filament > 0.);
        REQUIRE(xy > 0.);
        if (mode_index == 0) {
            baseline_filament = filament;
            baseline_xy = xy;
        } else {
            CHECK_THAT(filament, Catch::Matchers::WithinRel(baseline_filament, 1e-5));
            CHECK_THAT(xy, Catch::Matchers::WithinRel(baseline_xy, 1e-6));
        }
    }
}


TEST_CASE("GCode placeholder layer count remains finite for a small real export", "[GCode][GCodeBroad]")
{
    DynamicPrintConfig config = base_config();
    config.set_deserialize_strict({{"before_layer_change_gcode", "M117 L-[layer_num]-[layer_z]-[max_layer_z]\n"}});
    const Exported out = export_cube(config, TestMesh::A);
    REQUIRE(count_substring(out.text, "M117 L-") > 1);
    size_t marker = out.text.find("M117 L-");
    while (marker != std::string::npos) {
        const size_t end       = out.text.find('\n', marker);
        const std::string line = out.text.substr(marker, end == std::string::npos ? std::string::npos : end - marker);
        CHECK(line.find("nan") == std::string::npos);
        CHECK(line.find("inf") == std::string::npos);
        marker = out.text.find("M117 L-", end == std::string::npos ? out.text.size() : end + 1);
    }
}

TEST_CASE("GCode relative and absolute extrusion modes preserve exported material", "[GCode][GCodeBroad]")
{
    DynamicPrintConfig relative = base_config();
    DynamicPrintConfig absolute = relative;
    absolute.set_deserialize_strict({{"use_relative_e_distances", false}, {"layer_change_gcode", ""}});
    const Exported r = export_cube(relative);
    const Exported a = export_cube(absolute);
    const Motion mr  = motion(r.text, relative);
    const Motion ma  = motion(a.text, absolute);
    REQUIRE(mr.extrusion_moves > 0);
    REQUIRE(ma.extrusion_moves > 0);
    CHECK_THAT(ma.xy_path, Catch::Matchers::WithinRel(mr.xy_path, 1e-6));
    CHECK_THAT(ma.filament, Catch::Matchers::WithinRel(mr.filament, 1e-5));
    CHECK(r.text.find("M83") != std::string::npos);
    CHECK(a.text.find("M82") != std::string::npos);
}

namespace {

std::vector<int> parse_public_fan_pwm(const std::string &text, const DynamicPrintConfig &config)
{
    std::vector<int> fan_pwm;
    GCodeReader reader;
    reader.apply_config(config);
    reader.parse_buffer(text, [&fan_pwm](GCodeReader &, const GCodeReader::GCodeLine &line) {
        if (!line.cmd_is("M106"))
            return;
        if (line.has('P')) {
            float channel = 0.0f;
            REQUIRE(line.has_value('P', channel));
            REQUIRE(std::isfinite(channel));
            CHECK_THAT(channel, Catch::Matchers::WithinAbs(1.0f, 1e-6f));
        }
        float pwm = 0.0f;
        REQUIRE(line.has_value('S', pwm));
        REQUIRE(std::isfinite(pwm));
        REQUIRE(pwm >= 0.0f);
        REQUIRE(pwm <= 255.0f);
        CHECK_THAT(pwm, Catch::Matchers::WithinAbs(std::round(pwm), 1e-6f));
        fan_pwm.emplace_back(static_cast<int>(std::lround(pwm)));
    });
    return fan_pwm;
}

std::string export_public_fan_fixture(const DynamicPrintConfig &config)
{
    Model model;
    Print print;
    std::vector<TriangleMesh> meshes;
    meshes.emplace_back(make_cube(10.0, 10.0, 0.6));
    TriangleMesh second = make_cube(10.0, 10.0, 0.6);
    second.translate(30.0, 0.0, 0.0);
    meshes.emplace_back(std::move(second));
    init_print(std::move(meshes), print, model, config, nullptr, false);
    require_valid(print);
    const std::string output = Test::gcode(print);
    const size_t layer_markers = count_substring(output, ";LAYER_CHANGE");
    REQUIRE(layer_markers >= 2);
    REQUIRE(layer_markers <= 6);
    return output;
}

} // namespace

TEST_CASE("Fan kickstart preserves extrusion across print sequences", "[GCode][GCodeFanPipeline]")
{
    const char *sequence = GENERATE("by layer", "by object");
    DynamicPrintConfig zero = base_config();
    zero.set_deserialize_strict({
        {"print_sequence", sequence},
        {"printable_area", "0x0,200x0,200x200,0x200"},
        {"nozzle_height", 2.5},
        {"extruder_clearance_radius", 40.0},
        {"belt_printer", false},
        {"spiral_mode", false},
        {"cooling_filter_enabled", false},
        {"support_cooling_filter", false},
        {"support_air_filtration", false},
        {"auxiliary_fan", false},
        {"additional_cooling_fan_speed", 0},
        {"enable_support", false},
        {"enable_prime_tower", false},
        {"enable_overhang_bridge_fan", false},
        {"overhang_fan_threshold", "95%"},
        {"overhang_fan_speed", 0},
        {"internal_bridge_fan_speed", -1},
        {"support_material_interface_fan_speed", -1},
        {"ironing_fan_speed", -1},
        {"fan_min_speed", 30},
        {"fan_max_speed", 60},
        {"fan_cooling_layer_time", 1000.0},
        {"slow_down_for_layer_cooling", false},
        {"slow_down_layer_time", 0.0},
        {"slow_down_min_speed", 1.0},
        {"close_fan_the_first_x_layers", 0},
        {"full_fan_speed_layer", 0},
        {"initial_layer_fan_speed", 0},
        {"fan_speedup_time", 0.0},
        {"fan_speedup_overhangs", false},
        {"fan_kickstart", 0.0},
        {"part_cooling_fan_min_pwm", 0},
    });
    DynamicPrintConfig kickstart = zero;
    kickstart.set_deserialize_strict({{"fan_kickstart", 0.5}});

    const std::string zero_gcode = export_public_fan_fixture(zero);
    const std::string kick_gcode = export_public_fan_fixture(kickstart);
    const std::vector<int> zero_fan = parse_public_fan_pwm(zero_gcode, zero);
    const std::vector<int> kick_fan = parse_public_fan_pwm(kick_gcode, kickstart);
    const Motion zero_motion = motion(zero_gcode, zero);
    const Motion kick_motion = motion(kick_gcode, kickstart);
    REQUIRE(zero_motion.extrusion_moves > 0);
    REQUIRE(kick_motion.extrusion_moves > 0);
    REQUIRE(std::isfinite(zero_motion.filament));
    REQUIRE(std::isfinite(kick_motion.filament));
    REQUIRE(std::isfinite(zero_motion.xy_path));
    REQUIRE(std::isfinite(kick_motion.xy_path));
    CHECK_THAT(kick_motion.filament, Catch::Matchers::WithinRel(zero_motion.filament, 1e-4));
    CHECK_THAT(kick_motion.xy_path, Catch::Matchers::WithinRel(zero_motion.xy_path, 1e-4));

    // The configured 30–60% target maps to PWM 76–153, allowing integer rounding.
    // FanMover starts at zero and suppresses redundant fan-off commands.
    const auto is_target = [](int pwm) { return pwm >= 76 && pwm <= 153; };
    for (const std::vector<int> *fan : {&zero_fan, &kick_fan}) {
        const auto target = std::find_if(fan->begin(), fan->end(), is_target);
        REQUIRE(target != fan->end());
    }
    CHECK(std::all_of(zero_fan.begin(), zero_fan.end(), [&](int pwm) {
        return pwm == 0 || is_target(pwm);
    }));
    CHECK(std::all_of(kick_fan.begin(), kick_fan.end(), [&](int pwm) {
        return pwm == 0 || pwm == 255 || is_target(pwm);
    }));
    const auto pulse = std::find(kick_fan.begin(), kick_fan.end(), 255);
    REQUIRE(pulse != kick_fan.end());
    CHECK(std::find_if(kick_fan.begin(), kick_fan.end(), [](int pwm) { return pwm > 0; }) == pulse);
    CHECK(std::find_if(pulse + 1, kick_fan.end(), is_target) != kick_fan.end());
}


namespace {

Exported export_prime_tower(const DynamicPrintConfig& config)
{
    Print print;
    Model model;
    init_print({TestMesh::A}, print, model, config);
    // The first apply sees only one used filament and normalizes a prime tower away;
    // reapply after regions have settled to exercise the real two-filament tower path.
    print.apply(model, config);
    require_valid(print);
    print.set_status_silent();
    print.process();
    ScopedTemporaryFile temp(".gcode");
    Exported result;
    result.result = std::make_unique<GCodeProcessorResult>();
    print.export_gcode(temp.string(), result.result.get(), nullptr);
    std::ifstream input(temp.string(), std::ios::binary);
    result.text.assign(std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>());
    REQUIRE_FALSE(result.text.empty());
    return result;
}

DynamicPrintConfig belt_contract_config()
{
    DynamicPrintConfig config = base_config();
    config.set_deserialize_strict({
        {"belt_printer", true},
        {"belt_slice_rotation", "x"},
        {"belt_slice_rotation_angle", 45.0},
        {"gcode_remap_x", "rev_x"},
        {"gcode_remap_y", "pos_z"},
        {"gcode_remap_z", "pos_y"},
        {"brim_type", "outer_only"},
        {"brim_width", 4.0},
        {"brim_object_gap", 0.0},
        {"leading_brim_length", 2.0},
        {"extra_brim_width", 1.0},
    });
    return config;
}

DynamicPrintConfig tower_contract_config(const char* type)
{
    return multifilament_config(2, {
                                       {"sparse_infill_filament_id", 1},
                                       {"internal_solid_filament_id", 1},
                                       {"top_surface_filament_id", 1},
                                       {"bottom_surface_filament_id", 1},
                                       {"outer_wall_filament_id", 2},
                                       {"inner_wall_filament_id", 2},
                                       {"enable_prime_tower", true},
                                       {"prime_tower_width", 35.0},
                                       {"wipe_tower_x", 50.0},
                                       {"wipe_tower_y", 50.0},
                                       {"wipe_tower_type", type},
                                       {"layer_height", 0.3},
                                       {"skirt_loops", 0},
                                       {"brim_type", "no_brim"},
                                       {"use_relative_e_distances", true},
                                       {"layer_change_gcode", "G92 E0\n"},
                                       {"flush_volumes_matrix", "0,70,80,0"},
                                   });
}

std::vector<Vec3d> remap_extrusion_endpoints(const std::string &text, const DynamicPrintConfig &config)
{
    std::vector<Vec3d> points;
    GCodeReader reader;
    reader.apply_config(config);
    reader.parse_buffer(text, [&points](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        if (!line.extruding(self))
            return;
        points.emplace_back(line.new_X(self), line.new_Y(self), line.new_Z(self));
    });
    return points;
}

std::string export_remap_fixture(Print &print, Model &model, const DynamicPrintConfig &config)
{
    print.apply(model, config);
    require_valid(print);
    return Test::gcode(print);
}

struct ExpectedRemap
{
    int  source_axis;
    int  sign;
    bool reverse;
};

const ExpectedRemap &expected_remap(const int value)
{
    static constexpr std::array<ExpectedRemap, 9> values{{
        {0, 1, false}, {1, 1, false}, {2, 1, false},
        {0, -1, false}, {1, -1, false}, {2, -1, false},
        {0, 1, true}, {1, 1, true}, {2, 1, true},
    }};
    return values.at(static_cast<size_t>(value));
}

double remap_component(const Vec3d &point, const int remap, const Vec3d &build_max)
{
    const std::array<double, 3> source{point.x(), point.y(), point.z()};
    const ExpectedRemap &spec = expected_remap(remap);
    const double value = static_cast<double>(spec.sign) * source[spec.source_axis];
    return spec.reverse ? build_max[spec.source_axis] - source[spec.source_axis] : value;
}

Vec3d expected_remap_point(const Vec3d &point, const std::array<int, 3> &mapping,
                           const Vec3d &build_max)
{
    return { remap_component(point, mapping[0], build_max),
             remap_component(point, mapping[1], build_max),
             remap_component(point, mapping[2], build_max) };
}

Vec3d frame_forward(const Vec3d &point, const double angle_deg)
{
    const double angle = angle_deg * PI / 180.0;
    const double sin_a = std::sin(angle);
    return { point.x(), point.y() / std::abs(sin_a),
             point.z() + point.y() * std::cos(angle) / sin_a };
}

Vec3d frame_inverse(const Vec3d &point, const double angle_deg)
{
    const double angle = angle_deg * PI / 180.0;
    const double sin_a = std::sin(angle);
    const double logical_y = point.y() * std::abs(sin_a);
    return { point.x(), logical_y,
             point.z() - logical_y * std::cos(angle) / sin_a };
}

DynamicPrintConfig remap_public_config()
{
    DynamicPrintConfig config = base_config();
    config.set_deserialize_strict({
        {"belt_printer", true},
        {"belt_slice_rotation", "none"},
        {"belt_slice_rotation_angle", 0.0},
        {"belt_frame_tilt_decouple", false},
        {"belt_frame_tilt_angle", 45.0},
        {"printable_area", "0x0,240x0,240x180,0x180"},
        {"printable_height", 310.0},
        {"layer_height", 0.2},
        {"initial_layer_print_height", 0.2},
        {"sparse_infill_density", "0%"},
        {"wall_loops", 1},
        {"enable_arc_fitting", false},
        {"gcode_remap_x", "pos_x"},
        {"gcode_remap_y", "pos_y"},
        {"gcode_remap_z", "pos_z"},
    });
    return config;
}

void init_remap_fixture(Print &print, Model &model, const DynamicPrintConfig &config)
{
    TriangleMesh body = make_cube(18.0, 13.0, 1.2);
    body.translate(37.0, 53.0, 0.0);
    std::vector<TriangleMesh> meshes;
    meshes.emplace_back(std::move(body));
    init_print(std::move(meshes), print, model, config, nullptr, false);
}

} // namespace

TEST_CASE("GCode belt export emits brim geometry and layer markers", "[GCode][GCodeBroad]")
{
    const DynamicPrintConfig config = belt_contract_config();
    const Exported out              = export_cube(config, TestMesh::A);
    const Motion m                  = motion(out.text, config);
    REQUIRE(m.extrusion_moves > 0);
    CHECK(count_substring(out.text, ";TYPE:Brim") > 0);
    CHECK(count_substring(out.text, ";LAYER_CHANGE") > 2);
    CHECK(m.xy_path > 100.0);
}

TEST_CASE("GCode belt apron width settings increase brim material path", "[GCode][GCodeBroad]")
{
    DynamicPrintConfig narrow = belt_contract_config();
    narrow.set_deserialize_strict({{"leading_brim_length", 0.0}, {"extra_brim_width", 0.0}});
    DynamicPrintConfig wide = narrow;
    wide.set_deserialize_strict({{"leading_brim_length", 8.0}, {"extra_brim_width", 5.0}});
    const Exported a = export_cube(narrow, TestMesh::A);
    const Exported b = export_cube(wide, TestMesh::A);
    const Motion ma  = motion(a.text, narrow);
    const Motion mb  = motion(b.text, wide);
    REQUIRE(ma.extrusion_moves > 0);
    REQUIRE(mb.extrusion_moves > 0);
    CHECK(mb.xy_path > ma.xy_path);
    CHECK(mb.filament > ma.filament);
}

TEST_CASE("GCode prime tower records two-filament transitions", "[GCode][GCodeBroad]")
{
    const DynamicPrintConfig config = tower_contract_config("type2");
    const Exported out              = export_prime_tower(config);
    const Motion m                  = motion(out.text, config);
    REQUIRE(m.extrusion_moves > 0);
    REQUIRE(count_substring(out.text, "WIPE_TOWER_START") > 0);
    CHECK(count_command(out.text, config, "T1") > 0);
    CHECK(count_command(out.text, config, "T0") > 0);
    CHECK(count_substring(out.text, "WIPE_TOWER_END") > 0);
    CHECK(m.filament > 0.0);
}

TEST_CASE("GCode prime tower implementation changes purge path", "[GCode][GCodeBroad]")
{
    const Exported rib       = export_prime_tower(tower_contract_config("type2"));
    const Exported rectangle = export_prime_tower(tower_contract_config("type1"));
    const Motion mr          = motion(rib.text, tower_contract_config("type2"));
    const Motion mc          = motion(rectangle.text, tower_contract_config("type1"));
    REQUIRE(mr.extrusion_moves > 0);
    REQUIRE(mc.extrusion_moves > 0);
    CHECK(count_substring(rib.text, "WIPE_TOWER_START") > 0);
    CHECK(count_substring(rectangle.text, "WIPE_TOWER_START") > 0);
    CHECK(mr.xy_path > 0.0);
    CHECK(mc.xy_path > 0.0);
    CHECK(std::abs(mr.xy_path - mc.xy_path) > 1e-6);
}

TEST_CASE("GCode sequential objects emit a distinct object transition", "[GCode][GCodeBroad]")
{
    DynamicPrintConfig config = base_config();
    config.set_deserialize_strict({
        {"print_sequence", "by object"},
        {"printing_by_object_gcode", "M117 BETWEEN-[current_object_idx]\n"},
        {"gcode_label_objects", true},
        {"z_hop", 0.0},
    });
    Print print;
    Model model;
    std::vector<TriangleMesh> meshes;
    TriangleMesh first = cube(20.0);
    first.translate(80., 80., 0.);
    TriangleMesh second = cube(20.0);
    second.translate(140., 80., 0.);
    meshes.emplace_back(std::move(first));
    meshes.emplace_back(std::move(second));
    init_print(std::move(meshes), print, model, config, nullptr, false);
    require_valid(print);
    print.set_status_silent();
    print.process();
    ScopedTemporaryFile temp(".gcode");
    Exported out;
    out.result = std::make_unique<GCodeProcessorResult>();
    print.export_gcode(temp.string(), out.result.get(), nullptr);
    std::ifstream input(temp.string(), std::ios::binary);
    out.text.assign(std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>());
    REQUIRE_FALSE(out.text.empty());
    const Motion m = motion(out.text, config);
    REQUIRE(m.extrusion_moves > 0);
    CHECK(count_substring(out.text, "M117 BETWEEN-") > 0);
    CHECK(count_substring(out.text, "; stop printing object ") > 0);
    CHECK(m.xy_path > 100.0);
}

TEST_CASE("GCode direct extruder index is stable before Print binding", "[GCode][GCodeBroad]")
{
    TestGCode gcode;
    init_direct_gcode(gcode);
    CHECK(gcode.cur_extruder_index() == 0);
    gcode.set_extruders({0, 1});
    gcode.writer().set_extruder(1);
    CHECK(gcode.cur_extruder_index() == 0);
}

TEST_CASE("GCode belt path remains finite for a sloped test mesh", "[GCode][GCodeBroad]")
{
    const DynamicPrintConfig config = belt_contract_config();
    const Exported out              = export_cube(config, TestMesh::step);
    const Motion m                  = motion(out.text, config);
    REQUIRE(m.extrusion_moves > 0);
    CHECK(std::isfinite(m.xy_path));
    CHECK(std::isfinite(m.filament));
    CHECK(m.z_only_moves > 0);
}

TEST_CASE("Public G-code remap maps every registered enum value", "[GCode][GCodeRemap]")
{
    const std::array<const char *, 9> values{{
        "pos_x", "pos_y", "pos_z", "neg_x", "neg_y", "neg_z", "rev_x", "rev_y", "rev_z",
    }};
    const std::array<const char *, 3> output_keys{{"gcode_remap_x", "gcode_remap_y", "gcode_remap_z"}};

    const DynamicPrintConfig base = remap_public_config();
    const std::array<const char *, 3> identity_values{{"pos_x", "pos_y", "pos_z"}};
    for (size_t output_axis = 0; output_axis < output_keys.size(); ++output_axis) {
        const ConfigOption *option = base.option(output_keys[output_axis]);
        REQUIRE(option != nullptr);
        CHECK(option->serialize() == identity_values[output_axis]);
    }
    Print print;
    Model model;
    init_remap_fixture(print, model, base);
    const std::string identity_export = export_remap_fixture(print, model, base);
    const std::vector<Vec3d> identity_points = remap_extrusion_endpoints(identity_export, base);
    REQUIRE(identity_points.size() > 20);

    const ConfigOptionPoints *bed_option = base.option<ConfigOptionPoints>("printable_area");
    const ConfigOptionFloat *height_option = base.option<ConfigOptionFloat>("printable_height");
    REQUIRE(bed_option != nullptr);
    REQUIRE(height_option != nullptr);
    const BoundingBoxf bed(bed_option->values);
    const Vec3d build_max(bed.max.x(), bed.max.y(), height_option->value);
    for (size_t output_axis = 0; output_axis < output_keys.size(); ++output_axis) {
        for (size_t value_index = 0; value_index < values.size(); ++value_index) {
            DynamicPrintConfig config = base;
            config.set_deserialize_strict(output_keys[output_axis], values[value_index]);
            const ConfigOption *selected = config.option(output_keys[output_axis]);
            REQUIRE(selected != nullptr);
            CHECK(selected->serialize() == values[value_index]);
            INFO("output axis " << output_keys[output_axis] << " value " << values[value_index]);
            const std::string exported = export_remap_fixture(print, model, config);
            const std::vector<Vec3d> actual = remap_extrusion_endpoints(exported, config);
            REQUIRE(actual.size() == identity_points.size());

            std::array<int, 3> mapping{{0, 1, 2}};
            mapping[output_axis] = static_cast<int>(value_index);
            for (size_t point_index = 0; point_index < identity_points.size(); ++point_index) {
                const Vec3d expected = expected_remap_point(identity_points[point_index], mapping, build_max);
                REQUIRE_THAT(actual[point_index].x(), Catch::Matchers::WithinAbs(expected.x(), 0.002));
                REQUIRE_THAT(actual[point_index].y(), Catch::Matchers::WithinAbs(expected.y(), 0.002));
                REQUIRE_THAT(actual[point_index].z(), Catch::Matchers::WithinAbs(expected.z(), 0.002));
            }
        }
    }
}

TEST_CASE("Public belt frame decoupling transforms a remapped export", "[GCode][GCodeRemap][GCodeFrame]")
{
    // Decoupling is a separate post-remap frame stage. Reuse one sliced print so this
    // pair changes only the export-time machine-frame angle, not the sliced geometry.
    const DynamicPrintConfig base = remap_public_config();
    const DynamicPrintConfig coupled = [&] {
        DynamicPrintConfig config = base;
        config.set_deserialize_strict({
            {"belt_slice_rotation", "x"},
            {"belt_slice_rotation_angle", 30.0},
            {"gcode_remap_x", "pos_x"},
            {"gcode_remap_y", "pos_z"},
            {"gcode_remap_z", "pos_y"},
            {"belt_frame_tilt_decouple", false},
            {"belt_frame_tilt_angle", 45.0},
        });
        return config;
    }();
    Print frame_print;
    Model frame_model;
    init_remap_fixture(frame_print, frame_model, coupled);
    const std::vector<Vec3d> coupled_points = remap_extrusion_endpoints(
        export_remap_fixture(frame_print, frame_model, coupled), coupled);
    DynamicPrintConfig decoupled = coupled;
    decoupled.set_deserialize_strict({{"belt_frame_tilt_decouple", true}});
    const std::vector<Vec3d> decoupled_points = remap_extrusion_endpoints(
        export_remap_fixture(frame_print, frame_model, decoupled), decoupled);
    REQUIRE(coupled_points.size() > 20);
    REQUIRE(decoupled_points.size() == coupled_points.size());

    bool changed = false;
    for (size_t point_index = 0; point_index < coupled_points.size(); ++point_index) {
        const Vec3d expected = frame_forward(frame_inverse(coupled_points[point_index], 30.0), 45.0);
        REQUIRE_THAT(decoupled_points[point_index].x(), Catch::Matchers::WithinAbs(expected.x(), 0.002));
        REQUIRE_THAT(decoupled_points[point_index].y(), Catch::Matchers::WithinAbs(expected.y(), 0.002));
        REQUIRE_THAT(decoupled_points[point_index].z(), Catch::Matchers::WithinAbs(expected.z(), 0.002));
        if ((decoupled_points[point_index] - coupled_points[point_index]).norm() > 0.01)
            changed = true;
    }
    REQUIRE(changed);
}

struct SupportTimelapseEvidence
{
    bool finite = true;
    size_t support_extrusions = 0;
    size_t object_extrusions = 0;
    size_t snapshots = 0;
    size_t inline_snapshots = 0;
    size_t inline_after_support = 0;
    std::vector<double> inline_layer_z;
};

std::string_view trim_support_timelapse_comment(std::string_view comment)
{
    while (!comment.empty() && std::isspace(static_cast<unsigned char>(comment.front())))
        comment.remove_prefix(1);
    while (!comment.empty() && std::isspace(static_cast<unsigned char>(comment.back())))
        comment.remove_suffix(1);
    return comment;
}

SupportTimelapseEvidence parse_support_timelapse_output(const std::string &text,
                                                        const DynamicPrintConfig &config)
{
    SupportTimelapseEvidence evidence;
    GCodeReader reader;
    reader.apply_config(config);
    std::string active_role;
    size_t support_moves_in_layer = 0;
    reader.parse_buffer(text, [&evidence, &active_role, &support_moves_in_layer](GCodeReader &self,
                                                                                  const GCodeReader::GCodeLine &line) {
        const std::string_view comment = trim_support_timelapse_comment(line.comment());
        if (comment.rfind("LAYER_CHANGE", 0) == 0 || comment.rfind("CHANGE_LAYER", 0) == 0)
            support_moves_in_layer = 0;
        if (comment.rfind("TYPE:", 0) == 0)
            active_role = std::string(trim_support_timelapse_comment(comment.substr(5)));
        else if (comment.rfind("FEATURE:", 0) == 0)
            active_role = std::string(trim_support_timelapse_comment(comment.substr(8)));

        const bool motion = line.cmd_is("G0") || line.cmd_is("G1") || line.cmd_is("G2") || line.cmd_is("G3");
        if (motion && (!std::isfinite(static_cast<double>(self.x())) ||
                       !std::isfinite(static_cast<double>(self.y())) ||
                       !std::isfinite(static_cast<double>(self.z())) ||
                       !std::isfinite(static_cast<double>(self.e())) ||
                       (line.has_x() && !std::isfinite(static_cast<double>(line.x()))) ||
                       (line.has_y() && !std::isfinite(static_cast<double>(line.y()))) ||
                       (line.has_z() && !std::isfinite(static_cast<double>(line.z()))) ||
                       (line.has_e() && !std::isfinite(static_cast<double>(line.e()))))) {
            evidence.finite = false;
            return;
        }

        const bool support_role = active_role == "Support" ||
                                  active_role == "Support interface" ||
                                  active_role == "Support transition";
        if (motion && line.extruding(self) && line.dist_XY(self) > EPSILON) {
            if (support_role) {
                ++evidence.support_extrusions;
                ++support_moves_in_layer;
            } else {
                ++evidence.object_extrusions;
            }
        }

        if (!line.cmd_is("M117"))
            return;
        const std::string raw(line.raw());
        if (raw.find("SUPPORT-SNAPSHOT-") == std::string::npos)
            return;
        ++evidence.snapshots;
        const bool inline_photo = raw.find("-INLINE-1-Z-") != std::string::npos;
        if (!inline_photo)
            return;
        ++evidence.inline_snapshots;
        if (support_moves_in_layer > 0)
            ++evidence.inline_after_support;

        const size_t z_marker = raw.find("-Z-");
        if (z_marker == std::string::npos) {
            evidence.finite = false;
            return;
        }
        const char *start = raw.c_str() + z_marker + 3;
        char *end = nullptr;
        const double z = std::strtod(start, &end);
        if (end == start || !std::isfinite(z))
            evidence.finite = false;
        else
            evidence.inline_layer_z.push_back(z);
    });
    return evidence;
}

TEST_CASE("Farthest-point timelapse snapshots follow support deposition when no object perimeter exists",
          "[GCode][GCodeBroad][FarthestSupport]")
{
    DynamicPrintConfig enabled = base_config();
    enabled.set_deserialize_strict({
        {"wall_loops", 0},
        {"top_shell_layers", 0},
        {"bottom_shell_layers", 0},
        {"top_shell_thickness", 0.0},
        {"bottom_shell_thickness", 0.0},
        {"sparse_infill_density", "0%"},
        {"enable_support", true},
        {"support_type", "normal(auto)"},
        {"support_on_build_plate_only", false},
        {"support_base_pattern", "rectilinear"},
        {"support_filament", 1},
        {"support_interface_filament", 1},
        {"support_threshold_angle", 30},
        {"printer_structure", "corexy"},
        {"timelapse_type", "0"},
        {"farthest_point_timelapse", true},
        {"time_lapse_gcode", "M117 SUPPORT-SNAPSHOT-[layer_num]-INLINE-{timelapse_inline_photo ? 1 : 0}-Z-[layer_z]\n"},
    });
    DynamicPrintConfig control = enabled;
    control.set_deserialize_strict({
        {"farthest_point_timelapse", false},
        {"time_lapse_gcode", "M117 SUPPORT-SNAPSHOT-[layer_num]-INLINE-{timelapse_inline_photo ? 1 : 0}-Z-[layer_z]\n"},
    });

    const auto enabled_output = gcode_print_variance_contracts::fresh_custom_mesh_export(
        enabled, gcode_print_variance_contracts::support_fixture(), true);
    const auto control_output = gcode_print_variance_contracts::fresh_custom_mesh_export(
        control, gcode_print_variance_contracts::support_fixture(), true);
    const SupportTimelapseEvidence enabled_evidence = parse_support_timelapse_output(enabled_output.text, enabled);
    const SupportTimelapseEvidence control_evidence = parse_support_timelapse_output(control_output.text, control);
    const auto enabled_material = gcode_print_variance_contracts::motion_evidence(enabled_output.text, enabled);
    const auto control_material = gcode_print_variance_contracts::motion_evidence(control_output.text, control);

    REQUIRE(enabled_evidence.finite);
    REQUIRE(control_evidence.finite);
    REQUIRE(enabled_evidence.support_extrusions > 0);
    REQUIRE(control_evidence.support_extrusions > 0);
    CHECK(enabled_evidence.object_extrusions == 0);
    CHECK(control_evidence.object_extrusions == 0);
    REQUIRE(enabled_evidence.snapshots > 0);
    REQUIRE(control_evidence.snapshots > 0);
    REQUIRE(enabled_evidence.inline_snapshots > 0);
    REQUIRE(enabled_evidence.inline_after_support > 0);
    REQUIRE_FALSE(enabled_evidence.inline_layer_z.empty());
    for (double layer_z : enabled_evidence.inline_layer_z) {
        INFO("support snapshot layer z=" << layer_z);
        CHECK(std::isfinite(layer_z));
        CHECK(layer_z > 0.0);
    }
    gcode_print_variance_contracts::require_same_material(control_material, enabled_material);
    CHECK(control_evidence.inline_snapshots == 0);
}

namespace {

std::string process_adaptive_pa_bridge_case(const int bridge_role, const float bridge_override)
{
    TestGCode gcode;
    PrintConfig config = direct_print_config();
    config.gcode_flavor.value = gcfMarlinFirmware; // Serialized as "marlin2".
    config.gcode_comments.value = true;
    config.enable_pressure_advance.values = { true };
    config.adaptive_pressure_advance.values = { true };
    config.adaptive_pressure_advance_model.values = { "0.10,5,1000\n0.30,15,1000" };
    config.adaptive_pressure_advance_bridges.values = { bridge_override };
    gcode.apply_print_config(config);
    gcode.set_extruders({ 0 });
    gcode.writer().set_extruder(0);

    AdaptivePAProcessor processor(gcode);
    std::string layer = "; PA_CHANGE:T0 MM3MM:0.1 ACCEL:1000 BR:" +
                        std::to_string(bridge_role) + " RC:0 OV:0\n";
    layer += "G1 F6000\nG1 X10 Y0 E1\n";
    return processor.process_layer(std::move(layer));
}

} // namespace

TEST_CASE("Adaptive PA bridge override honors zero and maximum without changing motion",
          "[GCode][AdaptivePABridgeOverride]")
{
    // GCodeConfig stores this as an ordinary per-extruder ConfigOptionFloats field.
    const ConfigOptionDef *definition = print_config_def.get("adaptive_pressure_advance_bridges");
    REQUIRE(definition != nullptr);
    CHECK(definition->type == coFloats);
    CHECK_THAT(definition->max, Catch::Matchers::WithinAbs(2.0, 1e-12));
    CHECK(definition->is_value_valid(2.0));
    CHECK_FALSE(definition->is_value_valid(2.01));

    const PrintConfig defaults = direct_print_config();
    REQUIRE(defaults.adaptive_pressure_advance_bridges.values.size() == 1);
    CHECK_THAT(defaults.adaptive_pressure_advance_bridges.get_at(0),
               Catch::Matchers::WithinAbs(0.0, 1e-12));

    struct Case
    {
        int bridge_role;
        float bridge_override;
        double expected_pa;
        bool expect_override_comment;
    };
    const std::array<Case, 4> cases{{
        { 0, 0.f, 0.2, false },
        { 0, 2.f, 0.2, false },
        { 1, 0.f, 0.2, false },
        { 1, 2.f, 2.0, true },
    }};

    std::vector<std::string> reference_motion;
    for (size_t i = 0; i < cases.size(); ++i) {
        const Case &test_case = cases[i];
        CAPTURE(test_case.bridge_role, test_case.bridge_override);
        const std::string output = process_adaptive_pa_bridge_case(
            test_case.bridge_role, test_case.bridge_override);

        const std::vector<std::string> motion = command_lines(output, "G1 ");
        REQUIRE(motion.size() == 2);
        CHECK(motion[0] == "G1 F6000");
        CHECK(motion[1] == "G1 X10 Y0 E1");
        if (i == 0)
            reference_motion = motion;
        else
            CHECK(motion == reference_motion);

        const std::vector<std::string> pa_commands = command_lines(output, "M900 K");
        REQUIRE(pa_commands.size() == 1);
        constexpr std::string_view pa_prefix = "M900 K";
        const char *value_begin = pa_commands.front().c_str() + pa_prefix.size();
        char *value_end = nullptr;
        const double observed_pa = std::strtod(value_begin, &value_end);
        REQUIRE(value_end != value_begin);
        const bool value_end_is_boundary = *value_end == '\0' || *value_end == ';' ||
                                           std::isspace(static_cast<unsigned char>(*value_end));
        REQUIRE(value_end_is_boundary);
        CHECK_THAT(observed_pa, Catch::Matchers::WithinAbs(test_case.expected_pa, 1e-6));

        const bool has_override_comment =
            output.find("APA Model Override (bridge)") != std::string::npos;
        CHECK(has_override_comment == test_case.expect_override_comment);
    }
}

namespace {

std::string process_adaptive_pa_model_case(const std::vector<std::string> &models, const size_t config_index)
{
    TestGCode gcode;
    PrintConfig config = direct_print_config();
    config.gcode_flavor.value = gcfMarlinFirmware; // Serialized as "marlin2".
    config.gcode_comments.value = true;
    config.enable_pressure_advance.values = { true, true };
    config.adaptive_pressure_advance.values = { true, true };
    config.adaptive_pressure_advance_model.values = models;
    config.pressure_advance.values = { 0.42f, 0.42f };
    config.adaptive_pressure_advance_bridges.values = { 0.f, 0.f };
    config.extruder_offset.values = { Vec2d(1.25, -2.5), Vec2d(1.25, -2.5) };
    config.filament_diameter.values = { 1.75f, 1.75f };
    config.filament_flow_ratio.values = { 1.f, 1.f };
    gcode.apply_print_config(config);
    gcode.set_extruders({ 0, 1 });
    gcode.writer().set_extruder(static_cast<unsigned int>(config_index));

    AdaptivePAProcessor processor(gcode);
    std::string layer = "; PA_CHANGE:T" + std::to_string(config_index) + " MM3MM:0.1 ACCEL:1500 BR:0 RC:0 OV:0\n";
    layer += "G1 F6000\nG1 X10 Y0 E1\n";
    return processor.process_layer(std::move(layer));
}

double adaptive_pa_command_value(const std::string &output)
{
    const std::vector<std::string> pa_commands = command_lines(output, "M900 K");
    REQUIRE(pa_commands.size() == 1);
    constexpr std::string_view prefix = "M900 K";
    const char *value_begin = pa_commands.front().c_str() + prefix.size();
    char *value_end = nullptr;
    const double value = std::strtod(value_begin, &value_end);
    REQUIRE(value_end != value_begin);
    REQUIRE(std::isfinite(value));
    const bool value_end_is_boundary = *value_end == '\0' || *value_end == ';' ||
                                       std::isspace(static_cast<unsigned char>(*value_end));
    REQUIRE(value_end_is_boundary);
    return value;
}

void check_adaptive_pa_fallback(const std::string &output, const std::string &expected_diagnostic_token)
{
    CHECK(count_substring(output, expected_diagnostic_token) == 1);
    CHECK_THAT(adaptive_pa_command_value(output), Catch::Matchers::WithinAbs(0.42, 1e-6));
    const std::vector<std::string> motion = command_lines(output, "G1 ");
    REQUIRE(motion.size() == 2);
    CHECK(motion[0] == "G1 F6000");
    CHECK(motion[1] == "G1 X10 Y0 E1");
}

} // namespace

TEST_CASE("Adaptive PA selects each filament's nonlinear model without changing motion",
          "[GCode][AdaptivePAModelVariance]")
{
    const std::vector<std::string> models{
        "0.1,10,1000\n0.1,20,1000\n0.2,10,2000\n0.2,20,2000\n0.5,10,3000\n0.5,20,3000",
        "0.2,10,1000\n0.2,20,1000\n0.4,10,2000\n0.4,20,2000\n0.8,10,3000\n0.8,20,3000",
    };
    // At t=0.5 the cubic Hermite basis is (.5, .125, .5, -.125). With 1000-unit
    // spacing, endpoint slopes are the secants and the interior slope is their
    // weighted harmonic mean: .00015 for model 0 and .0002666667 for model 1.
    // The unrounded midpoint results are .14375 and .2916667; interpolation rounds to 3 decimals.
    const std::array<double, 2> expected_pa{ 0.144, 0.292 };
    std::vector<std::string> reference_motion;

    for (size_t config_index = 0; config_index < models.size(); ++config_index) {
        CAPTURE(config_index);
        const std::string output = process_adaptive_pa_model_case(models, config_index);
        const std::vector<std::string> motion = command_lines(output, "G1 ");
        REQUIRE(motion.size() == 2);
        CHECK(motion[0] == "G1 F6000");
        CHECK(motion[1] == "G1 X10 Y0 E1");
        if (config_index == 0)
            reference_motion = motion;
        else
            CHECK(motion == reference_motion);
        CHECK_THAT(adaptive_pa_command_value(output), Catch::Matchers::WithinAbs(expected_pa[config_index], 1e-6));
    }
}

TEST_CASE("Adaptive PA falls back when a filament model cannot be parsed",
          "[GCode][AdaptivePAMalformedFallback]")
{
    const std::string output = process_adaptive_pa_model_case({ "not-a-number,10,1000" }, 0);
    check_adaptive_pa_fallback(output, "Interpolator setup failed");
}

TEST_CASE("Adaptive PA falls back when a model has no interpolatable curve",
          "[GCode][AdaptivePANoCurveFallback]")
{
    const std::string output = process_adaptive_pa_model_case({ "0.1,10,1000" }, 0);
    check_adaptive_pa_fallback(output, "Interpolation failed");
}

} // namespace gcode_broad_contracts

namespace belt_apron_contracts {
using namespace Slic3r::Test;

namespace {

DynamicPrintConfig belt_apron_config(const double leading = 50.0,
                                     const double width = 4.0,
                                     const double lateral = 0.0,
                                     const char *brim_type = "outer_only")
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        {"belt_printer", true},
        {"belt_slice_rotation", "x"},
        {"belt_slice_rotation_angle", 45.0},
        {"gcode_remap_x", "rev_x"},
        {"gcode_remap_y", "pos_z"},
        {"gcode_remap_z", "pos_y"},
        {"layer_height", 0.2},
        {"initial_layer_print_height", 0.2},
        {"skirt_loops", 0},
        {"top_shell_layers", 0},
        {"bottom_shell_layers", 1},
        {"brim_type", brim_type},
        {"brim_width", width},
        {"leading_brim_length", leading},
        {"extra_brim_width", lateral},
        {"brim_object_gap", 0},
        {"use_relative_e_distances", false},
        {"machine_start_gcode", "T[initial_tool]\n"},
    });
    return config;
}

struct ExportedBelt {
    std::string gcode;
    size_t prologue_bands = 0;
    double first_object_z = 0.0;
};

ExportedBelt export_belt(DynamicPrintConfig config,
                         std::vector<TriangleMesh> meshes = {cube(20)},
                         const std::vector<std::vector<ConfigBase::SetDeserializeItem>> *overrides = nullptr,
                         const bool arrange = true,
                         const size_t instances = 1)
{
    Print print;
    Model model;
    init_print(std::move(meshes), print, model, config, overrides, arrange, instances);
    const auto validation = print.validate();
    INFO("validation=" << validation.string);
    REQUIRE(validation.string.empty());
    REQUIRE_FALSE(print.objects().empty());
    const std::string output = gcode(print);
    REQUIRE_FALSE(output.empty());
    const PrintObject &object = *print.objects().front();
    const double first_object_z = object.layers().empty() ? std::numeric_limits<double>::max()
                                                          : object.layers().front()->print_z;
    return {output, object.belt_brim_prologue().size(), first_object_z};
}

struct BeltMotion {
    std::vector<double> brim_feedrates;
    std::vector<size_t> apron_extrusions_by_band;
    std::vector<double> apron_band_z_tags;
    std::vector<double> apron_z_tags;
    std::vector<double> apron_height_tags;
    std::set<int> brim_tools;
    size_t brim_extrusions = 0;
    size_t all_extrusions = 0;
    size_t layer_changes_before_perimeter = 0;
    double brim_xy_length = 0.0;
    bool finite = true;
};

bool is_wall_type_marker(const std::string &raw)
{
    std::string lower = raw;
    std::transform(lower.begin(), lower.end(), lower.begin(), [](const unsigned char c) { return char(std::tolower(c)); });
    return lower.find(";type:") != std::string::npos &&
           (lower.find("perimeter") != std::string::npos ||
            lower.find("outer wall") != std::string::npos ||
            lower.find("inner wall") != std::string::npos);
}

std::set<int> fan_percents_before_object_z(const std::string &gcode, const double first_object_z)
{
    std::set<int> result;
    double current_z = std::numeric_limits<double>::max();
    bool seen_z = false;
    std::istringstream lines(gcode);
    std::string raw;
    while (std::getline(lines, raw)) {
        if (raw.rfind(";Z:", 0) == 0) {
            seen_z = true;
            try {
                current_z = std::stod(raw.substr(3));
            } catch (...) {
                current_z = std::numeric_limits<double>::max();
            }
        }
        if (is_wall_type_marker(raw))
            break;
        const auto pos = raw.find("M106 S");
        if ((seen_z && current_z >= first_object_z - 1e-4) || pos == std::string::npos || (pos > 0 && !std::isspace(static_cast<unsigned char>(raw[pos - 1]))))
            continue;
        const char *begin = raw.c_str() + pos + 6;
        char *end = nullptr;
        const long pwm = std::strtol(begin, &end, 10);
        if (end == begin || pwm <= 0)
            continue;
        result.insert(int(std::lround(double(pwm) * 100.0 / 255.0)));
    }
    return result;
}

BeltMotion inspect_belt(const std::string &gcode, const DynamicPrintConfig &config)
{
    BeltMotion out;
    bool before_perimeter = true;
    int current_tool = 0;
    std::istringstream lines(gcode);
    std::string raw;
    while (std::getline(lines, raw)) {
        if (raw.rfind(";Z:", 0) == 0 && before_perimeter) {
            try {
                out.apron_z_tags.push_back(std::stod(raw.substr(3)));
            } catch (...) {
                out.finite = false;
            }
        }
        if (raw.rfind(";HEIGHT:", 0) == 0 && before_perimeter) {
            try {
                out.apron_height_tags.push_back(std::stod(raw.substr(8)));
            } catch (...) {
                out.finite = false;
            }
        }
        if (is_wall_type_marker(raw))
            before_perimeter = false;

    }

    GCodeReader reader;
    reader.apply_config(config);
    bool reader_before_wall = true;
    size_t reader_band = size_t(-1);
    reader.parse_buffer(gcode, [&out, &current_tool, &reader_before_wall, &reader_band](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        const std::string raw(line.raw());
        if (reader_before_wall && raw.find(";LAYER_CHANGE") != std::string::npos) {
            ++out.layer_changes_before_perimeter;
            out.apron_extrusions_by_band.push_back(0);
            out.apron_band_z_tags.push_back(std::numeric_limits<double>::quiet_NaN());
            reader_band = out.apron_extrusions_by_band.size() - 1;
        }
        if (reader_before_wall && raw.rfind(";Z:", 0) == 0 && reader_band != size_t(-1)) {
            try {
                out.apron_band_z_tags[reader_band] = std::stod(raw.substr(3));
            } catch (...) {
                out.finite = false;
            }
        }
        if (reader_before_wall && is_wall_type_marker(raw))
            reader_before_wall = false;
        const std::string cmd(line.cmd());
        if (cmd.size() >= 2 && cmd[0] == 'T' && std::isdigit(static_cast<unsigned char>(cmd[1]))) {
            current_tool = std::stoi(cmd.substr(1));
            return;
        }
        if (!line.extruding(self))
            return;
        ++out.all_extrusions;
        out.finite = out.finite && std::isfinite(double(self.x())) && std::isfinite(double(self.y()))
                   && std::isfinite(double(self.z())) && std::isfinite(double(self.e()))
                   && std::isfinite(double(self.f()));
        const std::string comment(line.comment());
        if (comment.find("Brim") == std::string::npos && comment.find("brim") == std::string::npos)
            return;
        ++out.brim_extrusions;
        if (reader_before_wall && reader_band != size_t(-1))
            ++out.apron_extrusions_by_band[reader_band];
        out.brim_tools.insert(current_tool);
        out.brim_feedrates.push_back(double(self.f()));
        out.brim_xy_length += double(line.dist_XY(self));
    });
    return out;
}

std::vector<int> hook_layer_numbers_before_object_z(const std::string &gcode,
                                                     const std::string &prefix,
                                                     const double first_object_z)
{
    std::vector<int> result;
    double current_z = std::numeric_limits<double>::max();
    std::istringstream lines(gcode);
    for (std::string raw; std::getline(lines, raw);) {
        if (raw.rfind(";Z:", 0) == 0) {
            try {
                current_z = std::stod(raw.substr(3));
            } catch (...) {
                current_z = std::numeric_limits<double>::max();
            }
        }
        if (current_z >= first_object_z - 1e-4)
            continue;
        const std::string marker = "M117 " + prefix;
        if (raw.rfind(marker, 0) != 0)
            continue;
        const char *begin = raw.c_str() + marker.size();
        char *end = nullptr;
        const long layer_num = std::strtol(begin, &end, 10);
        if (end != begin)
            result.push_back(int(layer_num));
    }
    return result;
}

bool has_token_before_perimeter(const std::string &gcode, const std::string &token)
{
    std::istringstream lines(gcode);
    std::string raw;
    while (std::getline(lines, raw)) {
        if (is_wall_type_marker(raw))
            return false;
        if (raw.find(token) != std::string::npos)
            return true;
    }
    return false;
}

} // namespace

TEST_CASE("belt apron emits a real prologue before object perimeter", "[GCode][GCodeBeltApron]")
{
    const DynamicPrintConfig config = belt_apron_config();
    const ExportedBelt out = export_belt(config);
    const BeltMotion motion = inspect_belt(out.gcode, config);
    REQUIRE(out.prologue_bands > 0);
    REQUIRE(motion.brim_extrusions > 0);
    std::vector<size_t> prologue_band_extrusions;
    for (size_t band = 0; band < motion.apron_extrusions_by_band.size(); ++band)
        if (std::isfinite(motion.apron_band_z_tags[band]) && motion.apron_band_z_tags[band] < out.first_object_z - 1e-4)
            prologue_band_extrusions.push_back(motion.apron_extrusions_by_band[band]);
    REQUIRE(prologue_band_extrusions.size() == out.prologue_bands);
    for (const size_t band_extrusions : prologue_band_extrusions)
        CHECK(band_extrusions > 0);
    REQUIRE(motion.layer_changes_before_perimeter >= out.prologue_bands + 1);
    const size_t emitted_apron_markers = size_t(std::count_if(
        motion.apron_z_tags.begin(), motion.apron_z_tags.end(),
        [&out](const double z) { return z < out.first_object_z - 1e-4; }));
    CHECK(emitted_apron_markers == out.prologue_bands);
    REQUIRE(!motion.apron_height_tags.empty());
    CHECK(std::all_of(motion.apron_height_tags.begin(), motion.apron_height_tags.end(), [](const double h) {
        return std::isfinite(h) && h > 0.0;
    }));
    CHECK(std::all_of(motion.apron_height_tags.begin(), motion.apron_height_tags.end(), [](const double h) {
        return h <= 0.2001;
    }));
    CHECK(motion.brim_xy_length > 0.0);
}

TEST_CASE("belt apron layer markers are ordered and each band has positive motion", "[GCode][GCodeBeltApron]")
{
    const DynamicPrintConfig config = belt_apron_config(50.0);
    const ExportedBelt out = export_belt(config);
    const BeltMotion motion = inspect_belt(out.gcode, config);
    REQUIRE(motion.apron_z_tags.size() > 3);
    REQUIRE(motion.brim_extrusions > out.prologue_bands);
    CHECK(motion.finite);
    for (size_t i = 1; i < motion.apron_z_tags.size(); ++i)
        CHECK(motion.apron_z_tags[i] > motion.apron_z_tags[i - 1]);
}

TEST_CASE("belt apron uses first layer speed for every apron extrusion", "[GCode][GCodeBeltApron]")
{
    DynamicPrintConfig config = belt_apron_config();
    config.set_deserialize_strict({{"initial_layer_speed", "8"}, {"outer_wall_speed", "80"}});
    const ExportedBelt out = export_belt(config);
    const BeltMotion motion = inspect_belt(out.gcode, config);
    REQUIRE(motion.brim_extrusions > 20);
    REQUIRE(motion.finite);
    CHECK(std::all_of(motion.brim_feedrates.begin(), motion.brim_feedrates.end(), [](const double f) {
        return f > 0.0 && f <= 8.0 * 60.0 + 1e-3;
    }));
    CHECK(std::any_of(motion.brim_feedrates.begin(), motion.brim_feedrates.end(), [](const double f) {
        return std::abs(f - 8.0 * 60.0) < 1e-3;
    }));
}

TEST_CASE("belt apron first layer fan override appears before the object", "[GCode][GCodeBeltApron]")
{
    DynamicPrintConfig config = belt_apron_config();
    config.set_deserialize_strict({{"slow_down_for_layer_cooling", true}, {"close_fan_the_first_x_layers", 0}, {"fan_max_speed", 100}, {"fan_min_speed", 100},
                                   {"full_fan_speed_layer", 0}, {"slow_down_layer_time", 0}, {"fan_cooling_layer_time", 0},
                                   {"initial_layer_fan_speed", "37"}});
    const ExportedBelt out = export_belt(config);
    REQUIRE(out.prologue_bands > 0);
    const std::set<int> fan_percents = fan_percents_before_object_z(out.gcode, out.first_object_z);
    REQUIRE(!fan_percents.empty());
    CHECK(fan_percents == std::set<int>{37});
}

TEST_CASE("leading belt apron length adds bands and path", "[GCode][GCodeBeltApron]")
{
    const ExportedBelt short_apron = export_belt(belt_apron_config(30.0));
    const ExportedBelt long_apron = export_belt(belt_apron_config(50.0));
    const BeltMotion short_motion = inspect_belt(short_apron.gcode, belt_apron_config(30.0));
    const BeltMotion long_motion = inspect_belt(long_apron.gcode, belt_apron_config(50.0));
    REQUIRE(long_apron.prologue_bands > 0);
    CHECK(long_apron.prologue_bands > short_apron.prologue_bands);
    CHECK(long_motion.brim_xy_length > short_motion.brim_xy_length);
}

TEST_CASE("belt apron remains finite with lateral widening", "[GCode][GCodeBeltApron]")
{
    const DynamicPrintConfig narrow_config = belt_apron_config(50.0, 4.0, 0.0);
    const DynamicPrintConfig wide_config = belt_apron_config(50.0, 4.0, 6.0);
    const ExportedBelt narrow = export_belt(narrow_config);
    const ExportedBelt wide = export_belt(wide_config);
    const BeltMotion narrow_motion = inspect_belt(narrow.gcode, narrow_config);
    const BeltMotion wide_motion = inspect_belt(wide.gcode, wide_config);
    REQUIRE(narrow_motion.brim_extrusions > 0);
    REQUIRE(wide_motion.brim_extrusions > 0);
    CHECK(narrow_motion.finite);
    CHECK(wide_motion.finite);
    CHECK(wide_motion.brim_xy_length > narrow_motion.brim_xy_length);
}

TEST_CASE("disabled belt brim has no apron or brim motion", "[GCode][GCodeBeltApron]")
{
    const DynamicPrintConfig config = belt_apron_config(0.0, 0.0, 0.0);
    const ExportedBelt out = export_belt(config);
    const BeltMotion motion = inspect_belt(out.gcode, config);
    CHECK(out.prologue_bands == 0);
    CHECK(motion.brim_extrusions == 0);
    CHECK_FALSE(has_token_before_perimeter(out.gcode, ";TYPE:Brim"));
}

TEST_CASE("inner-only belt brim does not create a leading apron", "[GCode][GCodeBeltApron]")
{
    const DynamicPrintConfig config = belt_apron_config(50.0, 4.0, 0.0, "inner_only");
    const ExportedBelt out = export_belt(config, {mesh(TestMesh::cube_with_hole)});
    const BeltMotion motion = inspect_belt(out.gcode, config);
    CHECK(out.prologue_bands == 0);
    CHECK(motion.finite);
}

TEST_CASE("belt apron uses the configured wall filament", "[GCode][GCodeBeltApron]")
{
    DynamicPrintConfig config = multifilament_config(2);
    config.set_deserialize_strict({
        {"belt_printer", true}, {"belt_slice_rotation", "x"}, {"belt_slice_rotation_angle", 45.0},
        {"gcode_remap_x", "rev_x"}, {"gcode_remap_y", "pos_z"}, {"gcode_remap_z", "pos_y"},
        {"layer_height", 0.2}, {"initial_layer_print_height", 0.2}, {"skirt_loops", 0},
        {"top_shell_layers", 0}, {"bottom_shell_layers", 1}, {"brim_type", "outer_only"},
        {"brim_width", 4.0}, {"leading_brim_length", 50.0}, {"brim_object_gap", 0}, {"use_relative_e_distances", false},
        {"outer_wall_filament_id", 2}, {"inner_wall_filament_id", 2},
        {"machine_start_gcode", "T[initial_tool]\n"},
    });
    const std::vector<std::vector<ConfigBase::SetDeserializeItem>> overrides{{{"extruder", 2}}};
    const ExportedBelt out = export_belt(config, {cube(20)}, &overrides);
    const BeltMotion motion = inspect_belt(out.gcode, config);
    REQUIRE(out.prologue_bands > 0);
    CHECK(motion.brim_tools == std::set<int>{1});
}

TEST_CASE("belt apron is emitted once per object instance", "[GCode][GCodeBeltApron]")
{
    const DynamicPrintConfig config = belt_apron_config();
    const ExportedBelt one = export_belt(config, {cube(20)}, nullptr, true, 1);
    const ExportedBelt two = export_belt(config, {cube(20)}, nullptr, true, 2);
    const BeltMotion one_motion = inspect_belt(one.gcode, config);
    const BeltMotion two_motion = inspect_belt(two.gcode, config);
    REQUIRE(one_motion.brim_extrusions > 0);
    CHECK(two_motion.brim_extrusions >= 2 * one_motion.brim_extrusions);
    CHECK(two_motion.brim_xy_length >= 2.0 * one_motion.brim_xy_length - 1e-3);
}

TEST_CASE("belt apron runs before and after layer change hooks", "[GCode][GCodeBeltApron]")
{
    DynamicPrintConfig config = belt_apron_config();
    config.set_deserialize_strict({
        {"before_layer_change_gcode", "M117 APRON_BEFORE_[layer_num]\n"},
        {"layer_change_gcode", "M117 APRON_CHANGE_[layer_num]\n"},
    });
    const ExportedBelt out = export_belt(config);
    REQUIRE(out.prologue_bands > 0);
    const std::vector<int> before_layers = hook_layer_numbers_before_object_z(out.gcode, "APRON_BEFORE_", out.first_object_z);
    const std::vector<int> change_layers = hook_layer_numbers_before_object_z(out.gcode, "APRON_CHANGE_", out.first_object_z);
    REQUIRE(before_layers.size() == out.prologue_bands);
    REQUIRE(change_layers.size() == out.prologue_bands);

}

} // namespace belt_apron_contracts

namespace adaptive_pa_validation_contracts {
using Slic3r::AdaptivePAProcessor;
using Catch::Matchers::ContainsSubstring;

TEST_CASE("Adaptive PA accepts empty models and blank physical lines", "[GCode][AdaptivePAValidation]")
{
    const std::string input = GENERATE("", "\n", " \t\r\n\n \t");
    CHECK(AdaptivePAProcessor::validate_adaptive_pa_model(input).empty());
}

TEST_CASE("Adaptive PA accepts complete decimal triples and edge whitespace", "[GCode][AdaptivePAValidation]")
{
    const std::string input = GENERATE("0,1,2", ".1,10.,1000", "1.999,2,2.001", " \t0.04,12.5,3000\t\r\n");
    CHECK(AdaptivePAProcessor::validate_adaptive_pa_model(input).empty());
}

TEST_CASE("Adaptive PA accepts multiple independently valid model rows", "[GCode][AdaptivePAValidation]")
{
    CHECK(AdaptivePAProcessor::validate_adaptive_pa_model("0.04,10,1000\n\n0.05,20,2000\n0.06,30,3000\n").empty());
}

TEST_CASE("Adaptive PA rejects an incorrect number of comma separated fields", "[GCode][AdaptivePAValidation]")
{
    const std::string input = GENERATE("0.1", "0.1,10", "0.1,10,1000,2000", "0.1,10,1000,");
    CHECK_THAT(AdaptivePAProcessor::validate_adaptive_pa_model(input), ContainsSubstring("exactly 3"));
}

TEST_CASE("Adaptive PA rejects missing numeric values in every column", "[GCode][AdaptivePAValidation]")
{
    const std::string input = GENERATE(",10,1000", "0.1,,1000", "0.1,10,", ",,");
    CHECK_FALSE(AdaptivePAProcessor::validate_adaptive_pa_model(input).empty());
}

TEST_CASE("Adaptive PA rejects partial decimal parses in every column", "[GCode][AdaptivePAValidation]")
{
    const std::string input = GENERATE("0.1.2,10,1000", "0.1,10.20.30,1000", "0.1,10,1000.2.3");
    CHECK_THAT(AdaptivePAProcessor::validate_adaptive_pa_model(input), ContainsSubstring("invalid numeric"));
}

TEST_CASE("Adaptive PA rejects decimal punctuation without digits", "[GCode][AdaptivePAValidation]")
{
    const std::string input = GENERATE(".,10,1000", "0.1,.,1000", "0.1,10,.", "..,10,1000");
    CHECK_THAT(AdaptivePAProcessor::validate_adaptive_pa_model(input), ContainsSubstring("invalid numeric"));
}

TEST_CASE("Adaptive PA rejects signs exponents and embedded whitespace", "[GCode][AdaptivePAValidation]")
{
    const std::string input = GENERATE("-0.1,10,1000", "+0.1,10,1000", "0.1,1e1,1000", "0.1,10,10 00", "0.1, 10,1000", "0.1,10,NaN", "0.1,10,inf");
    CHECK_THAT(AdaptivePAProcessor::validate_adaptive_pa_model(input), ContainsSubstring("only numbers"));
}

TEST_CASE("Adaptive PA rejects out of range decimal magnitudes", "[GCode][AdaptivePAValidation]")
{
    const std::string enormous(400, '9');
    const int column = GENERATE(0, 1, 2);
    std::string input = column == 0 ? enormous + ",10,1000" : column == 1 ? "0.1," + enormous + ",1000" : "0.1,10," + enormous;
    CHECK_THAT(AdaptivePAProcessor::validate_adaptive_pa_model(input), ContainsSubstring("invalid numeric"));
}

TEST_CASE("Adaptive PA enforces its pressure advance upper boundary", "[GCode][AdaptivePAValidation]")
{
    const std::string input = GENERATE("2,10,1000", "2.001,10,1000");
    CHECK_THAT(AdaptivePAProcessor::validate_adaptive_pa_model(input), ContainsSubstring("PA value must be less than 2"));
}

TEST_CASE("Adaptive PA requires flow and acceleration to increase strictly", "[GCode][AdaptivePAValidation]")
{
    const std::string input = GENERATE("0.1,0.1,1000", "0.1,0.09,1000", "0.1,10,10", "0.1,10,9.9");
    CHECK_THAT(AdaptivePAProcessor::validate_adaptive_pa_model(input), ContainsSubstring("must be greater than"));
}

TEST_CASE("Adaptive PA errors identify the first invalid physical line", "[GCode][AdaptivePAValidation]")
{
    const std::string result = AdaptivePAProcessor::validate_adaptive_pa_model("\n0.1,10,1000\n\t\r\n0.2,20,20\n2,10,1000");
    CHECK_THAT(result, ContainsSubstring("Line 4:"));
    CHECK_THAT(result, ContainsSubstring("acceleration"));
}
} // namespace adaptive_pa_validation_contracts

namespace gcode_travel_point_reachable_contracts {

namespace {

constexpr std::size_t kMaxOutputBytes = 2u * 1024u * 1024u;

struct RoutePoints
{
    double p1_x = 0.;
    double p1_y = 0.;
    double p2_x = 0.;
    double p2_y = 0.;
    double p3_x = 0.;
    double p3_y = 0.;
};

struct SkipFlushFixture
{
    bool enabled = false;
    bool bbl_printer = true;
    bool exclude_object = true;
    size_t first_label_id = 0;
    size_t second_label_id = 0;
    std::vector<size_t> sorted_label_ids;
    std::vector<unsigned int> used_extruders;
};

DynamicPrintConfig route_config(double end_x, double end_y, bool manual_filament_change = false)
{
    // This is the validated fff_print helper: it sizes registered filament
    // vectors and the filament-by-filament flush matrix before export.  The
    // route fixture then sizes the physical mapping vectors it indexes.
    DynamicPrintConfig config = Test::multifilament_config(2);
    config.set_key_value("gcode_flavor", new ConfigOptionEnum<GCodeFlavor>(gcfMarlinFirmware));
    // The route fixture models two physical nozzles.  Keep this setting
    // explicit, but verify the effective object assignment below: the
    // production writer decides whether a tool change is possible from the
    // effective zero-based filament IDs, not this flag alone.
    config.set_key_value("single_extruder_multi_material", new ConfigOptionBool(false));
    config.set_key_value("manual_filament_change", new ConfigOptionBool(manual_filament_change));
    config.set_key_value("gcode_comments", new ConfigOptionBool(true));
    config.set_key_value("emit_machine_limits_to_gcode", new ConfigOptionBool(false));
    config.set_key_value("machine_start_gcode", new ConfigOptionString(""));
    config.set_key_value("machine_end_gcode", new ConfigOptionString(""));
    config.set_key_value("layer_height", new ConfigOptionFloat(0.2));
    config.set_key_value("initial_layer_print_height", new ConfigOptionFloat(0.2));
    config.set_key_value("initial_layer_line_width", new ConfigOptionFloatOrPercent(0., false));
    config.set_key_value("z_hop", new ConfigOptionFloats({0., 0.}));
    // By-layer avoids sequential-print clearance validation while still
    // emitting a real tool change between the two same-layer objects.
    config.set_key_value("print_sequence", new ConfigOptionEnum<PrintSequence>(PrintSequence::ByLayer));
    config.set_deserialize_strict({
        {"seam_position", "back"},
        {"use_relative_e_distances", 1},
        {"layer_change_gcode", "G92 E0\n"},
        {"skirt_loops", 0},
        {"brim_type", "no_brim"},
        {"raft_layers", 0}
    });

    // The marker is emitted by the change-filament context.  It makes all six
    // dynamic values observable without relying on the trailing raw-config dump.
    config.set_key_value("change_filament_gcode", new ConfigOptionString(
        ";TP p1={travel_point_1_x},{travel_point_1_y} "
        "p2={travel_point_2_x},{travel_point_2_y} "
        "p3={travel_point_3_x},{travel_point_3_y}\n"));
    config.set_key_value("start_end_points", new ConfigOptionPoints{
        Vec2d(30., -3.), Vec2d(end_x, end_y)
    });
    // get_path_of_change_filament uses point [2] as the cutter-area corner and
    // adds two millimetres to its Y coordinate.  All four points are supplied
    // so the production cardinality gate is passed.
    config.set_key_value("bed_exclude_area", new ConfigOptionPoints{
        Vec2d(90., 90.), Vec2d(108., 90.), Vec2d(108., 118.), Vec2d(90., 118.)
    });

    // Two physical extruders, one filament per extruder.  The validated helper
    // sizes registered filament vectors; these route-sensitive physical maps
    // and temperature/flush vectors are explicitly sized here.
    config.set_key_value("nozzle_diameter", new ConfigOptionFloats({0.4, 0.4}));
    config.set_key_value("printer_extruder_id", new ConfigOptionInts({1, 2}));
    config.set_key_value("printer_extruder_variant", new ConfigOptionStrings({
        "Direct Drive Standard", "Direct Drive Standard"
    }));
    config.set_key_value("filament_diameter", new ConfigOptionFloats({1.75, 1.75}));
    config.set_key_value("filament_colour", new ConfigOptionStrings({"#FF0000", "#00FF00"}));
    config.set_key_value("default_filament_colour", new ConfigOptionStrings({"#FF0000", "#00FF00"}));
    config.set_key_value("filament_type", new ConfigOptionStrings({"PLA", "PLA"}));
    config.set_key_value("filament_soluble", new ConfigOptionBools({false, false}));
    config.set_key_value("filament_is_support", new ConfigOptionBools({false, false}));
    config.option<ConfigOptionEnum<FilamentMapMode>>("filament_map_mode", true)->value = fmmManual;
    config.set_key_value("filament_map", new ConfigOptionInts({1, 2}));
    config.set_key_value("nozzle_temperature", new ConfigOptionInts({210, 210}));
    config.set_key_value("nozzle_temperature_range_low", new ConfigOptionInts({190, 190}));
    config.set_key_value("nozzle_temperature_range_high", new ConfigOptionInts({240, 240}));
    config.set_key_value("hotend_cooling_rate", new ConfigOptionFloatsNullable({2., 2.}));
    config.set_key_value("hotend_heating_rate", new ConfigOptionFloatsNullable({2., 2.}));
    config.set_key_value("flush_multiplier", new ConfigOptionFloats({1.}));
    config.set_key_value("flush_volumes_matrix", new ConfigOptionFloats({0., 140., 140., 0.}));
    config.set_key_value("wipe_tower_x", new ConfigOptionFloats({50.}));
    config.set_key_value("wipe_tower_y", new ConfigOptionFloats({50.}));
    return config;
}

std::string export_with_object_rows(double object_y, double end_x, double end_y,
                                   GCodeProcessorResult *result = nullptr,
                                   bool manual_filament_change = false,
                                   SkipFlushFixture *skip_flush = nullptr)
{
    DynamicPrintConfig config = route_config(end_x, end_y, manual_filament_change);
    if (skip_flush != nullptr) {
        config.set_key_value("support_object_skip_flush", new ConfigOptionBool(skip_flush->enabled));
        config.set_key_value("exclude_object", new ConfigOptionBool(skip_flush->exclude_object));
        config.set_key_value("enable_prime_tower", new ConfigOptionBool(false));
        // The tool-change path emits the outgoing tool end macro, and non-SEMM print shutdown
        // emits each configured end macro once more. This one-layer 0-to-1 fixture
        // therefore contains exactly two END_TOOL_0 lines.
        config.set_deserialize_strict({
            {"toolchange_ordering", "cyclic"},
            {"toolchange_cyclic_order", "1,2"},
            {"toolchange_cyclic_first_layer", true},
        });
        config.set_key_value("filament_end_gcode", new ConfigOptionStrings({
            "M117 END_TOOL_{filament_extruder_id}\n",
            "M117 END_TOOL_{filament_extruder_id}\n"
        }));
    }
    Model model;

    ModelObject *first = model.add_object();
    const double object_height = skip_flush != nullptr ? 0.2 : 2.;
    first->add_volume(make_cube(20., 20., object_height));
    ModelInstance *first_instance = first->add_instance();
    first_instance->set_offset(Vec3d(0., object_y, 0.));

    ModelObject *second = model.add_object();
    second->add_volume(make_cube(20., 20., object_height));
    ModelInstance *second_instance = second->add_instance();
    second_instance->set_offset(Vec3d(26., object_y, 0.));
    // `extruder` is the canonical 1-based object setting.  PrintApply
    // normalizes it into the region feature filament IDs used by
    // Print::extruders(); keep the assignment at the object level so the
    // contract exercises that production path rather than bypassing it.
    second->config.set_key_value("extruder", new ConfigOptionInt(2));

    Print print;
    print.is_BBL_printer() = skip_flush != nullptr ? skip_flush->bbl_printer : true;
    for (ModelObject *object : model.objects) {
        object->ensure_on_bed();
        print.auto_assign_extruders(object);
    }
    const PrintBase::ApplyStatus status = print.apply(model, config);
    REQUIRE(status != PrintBase::APPLY_STATUS_UNCHANGED);
    const auto validation = print.validate();
    INFO("Print validation error: " << validation.string << " [" << validation.opt_key << "]");
    REQUIRE(validation.string.empty());

    // GCodeWriter::multiple_extruders is derived from the effective IDs and
    // is independent of single_extruder_multi_material.  Prove the fixture
    // reached both physical extruders before process/export; otherwise a
    // missing marker would only report a downstream symptom.
    const std::vector<unsigned int> used_extruders = print.extruders();
    if (skip_flush != nullptr) {
        skip_flush->first_label_id = first_instance->get_labeled_id();
        skip_flush->second_label_id = second_instance->get_labeled_id();
        skip_flush->sorted_label_ids = {skip_flush->first_label_id, skip_flush->second_label_id};
        std::sort(skip_flush->sorted_label_ids.begin(), skip_flush->sorted_label_ids.end());
        skip_flush->used_extruders = used_extruders;
    }
    std::ostringstream used_extruder_info;
    used_extruder_info << "print.extruders count=" << used_extruders.size() << " ids=";
    constexpr std::size_t kMaxReportedExtruders = 8;
    const std::size_t reported_count = std::min(kMaxReportedExtruders, used_extruders.size());
    for (std::size_t index = 0; index < reported_count; ++index) {
        if (index > 0)
            used_extruder_info << ',';
        used_extruder_info << used_extruders[index];
    }
    if (reported_count < used_extruders.size())
        used_extruder_info << ",...";
    INFO(used_extruder_info.str());
    const bool uses_first_extruder = std::find(used_extruders.begin(), used_extruders.end(), 0u) != used_extruders.end();
    const bool uses_second_extruder = std::find(used_extruders.begin(), used_extruders.end(), 1u) != used_extruders.end();
    REQUIRE(uses_first_extruder);
    REQUIRE(uses_second_extruder);

    print.set_status_silent();
    print.process();
    const std::string output = Test::gcode(print, result);
    REQUIRE_FALSE(output.empty());
    REQUIRE(output.size() < kMaxOutputBytes);
    return output;
}

// Keep a missing-toolchange failure actionable without copying the full G-code
// into Catch output.  Only the route marker and bounded tool tokens are useful
// for this contract; everything else is unrelated slicing noise.
std::string bounded_custom_token_excerpt(const std::string &gcode)
{
    constexpr std::size_t kMaxExcerptBytes = 4096;
    constexpr std::size_t kMaxExcerptLines = 32;
    std::string excerpt;
    std::size_t cursor = 0;
    std::size_t lines = 0;
    while (cursor < gcode.size() && lines < kMaxExcerptLines && excerpt.size() < kMaxExcerptBytes) {
        const std::size_t end = gcode.find('\n', cursor);
        const std::size_t line_end = end == std::string::npos ? gcode.size() : end;
        const std::string_view line(gcode.data() + cursor, line_end - cursor);
        const bool interesting = line.rfind(";TP ", 0) == 0 || line.rfind("T0", 0) == 0
                              || line.rfind("T1", 0) == 0 || line.find("M1020") != std::string_view::npos;
        if (interesting) {
            const std::size_t remaining = kMaxExcerptBytes - excerpt.size();
            const bool truncated = line.size() > remaining;
            const std::size_t body_size = truncated && remaining >= 3 ? remaining - 3 : std::min(line.size(), remaining);
            excerpt.append(line.data(), body_size);
            if (truncated && remaining >= 3)
                excerpt += "...";
            if (excerpt.size() < kMaxExcerptBytes)
                excerpt.push_back('\n');
            ++lines;
        }
        cursor = end == std::string::npos ? gcode.size() : end + 1;
    }
    if (excerpt.empty())
        return "<no TP/tool tokens>";
    return excerpt;
}

std::string bounded_filament_sequence(const GCodeProcessorResult &result)
{
    constexpr std::size_t kMaxSequenceItems = 16;
    std::ostringstream sequence;
    sequence << "filament_changes=" << result.filament_change_sequence.size() << " sequence=";
    const std::size_t count = std::min(kMaxSequenceItems, result.filament_change_sequence.size());
    for (std::size_t i = 0; i < count; ++i) {
        if (i > 0)
            sequence << ',';
        sequence << result.filament_change_sequence[i];
    }
    if (count < result.filament_change_sequence.size())
        sequence << ",...";
    return sequence.str();
}

bool finite_route_points(const RoutePoints &points)
{
    return std::isfinite(points.p1_x) && std::isfinite(points.p1_y)
        && std::isfinite(points.p2_x) && std::isfinite(points.p2_y)
        && std::isfinite(points.p3_x) && std::isfinite(points.p3_y);
}

std::optional<RoutePoints> route_marker(const std::string &gcode)
{
    // The configuration dump can contain the literal template `;TP ` before
    // the actual emitted G-code.  Only inspect a marker at the beginning of a
    // complete line, then require all six finite values before accepting it.
    std::size_t line_start = 0;
    while (line_start <= gcode.size()) {
        const std::size_t line_end = gcode.find('\n', line_start);
        const std::size_t length = line_end == std::string::npos
            ? gcode.size() - line_start : line_end - line_start;
        const std::string line = gcode.substr(line_start, length);
        if (line.rfind(";TP ", 0) == 0) {
            RoutePoints points;
            const int parsed = std::sscanf(line.c_str(), ";TP p1=%lf,%lf p2=%lf,%lf p3=%lf,%lf",
                                           &points.p1_x, &points.p1_y, &points.p2_x, &points.p2_y,
                                           &points.p3_x, &points.p3_y);
            if (parsed == 6 && finite_route_points(points))
                return points;
        }
        if (line_end == std::string::npos)
            break;
        line_start = line_end + 1;
    }
    return std::nullopt;
}

std::size_t complete_route_marker_count(const std::string &gcode)
{
    std::istringstream lines(gcode);
    std::string line;
    std::size_t count = 0;
    while (std::getline(lines, line)) {
        if (route_marker(line).has_value())
            ++count;
    }
    return count;
}

std::optional<std::string> previous_line_before_exact_line(const std::string &gcode,
                                                             const std::string &target,
                                                             size_t &target_count)
{
    target_count = 0;
    std::optional<std::string> previous_line;
    std::istringstream lines(gcode);
    std::string line;
    std::string previous;
    while (std::getline(lines, line)) {
        if (!line.empty() && line.back() == '\r')
            line.pop_back();
        if (line == target) {
            ++target_count;
            if (target_count == 1)
                previous_line = previous;
        }
        previous = line;
    }
    return previous_line;
}

std::optional<std::vector<size_t>> label_ids_from_m624_line(
    const std::string &line, const std::vector<size_t> &sorted_label_ids)
{
    if (line.rfind("M624 ", 0) != 0 || sorted_label_ids.size() >= 64)
        return std::nullopt;

    const std::string payload = line.substr(5);
    if (payload.size() != boost::beast::detail::base64::encoded_size(8)
        || payload.back() != '=' || payload.find('=') != payload.size() - 1)
        return std::nullopt;

    // Beast's decoder requires decoded_size(payload.size()) output capacity,
    // which is nine bytes for this 12-character padded encoding, although the
    // final '=' means only eight bytes are written.
    std::array<unsigned char, 9> bytes{};
    const auto [decoded_bytes, consumed_chars] =
        boost::beast::detail::base64::decode(bytes.data(), payload.data(), payload.size());
    if (decoded_bytes != 8 || consumed_chars != payload.size() - 1)
        return std::nullopt;

    // GCode::_encode_label_ids_to_base64 serializes uint64_t byte i as
    // (mask >> (8 * i)) & 0xff, so reconstruct the mask in little-endian order.
    std::uint64_t mask = 0;
    for (size_t index = 0; index < 8; ++index)
        mask |= static_cast<std::uint64_t>(bytes[index]) << (8 * index);

    const std::uint64_t valid_bits = (std::uint64_t{1} << sorted_label_ids.size()) - 1;
    if ((mask & ~valid_bits) != 0)
        return std::nullopt;

    std::vector<size_t> selected_ids;
    for (size_t index = 0; index < sorted_label_ids.size(); ++index)
        if ((mask & (std::uint64_t{1} << index)) != 0)
            selected_ids.push_back(sorted_label_ids[index]);
    return selected_ids;
}

void require_finite(const RoutePoints &points)
{
    const bool finite = std::isfinite(points.p1_x);
    REQUIRE(finite);
    const bool finite_y = std::isfinite(points.p1_y);
    REQUIRE(finite_y);
    const bool finite_p2 = std::isfinite(points.p2_x);
    REQUIRE(finite_p2);
    const bool finite_p2_y = std::isfinite(points.p2_y);
    REQUIRE(finite_p2_y);
    const bool finite_p3 = std::isfinite(points.p3_x);
    REQUIRE(finite_p3);
    const bool finite_p3_y = std::isfinite(points.p3_y);
    REQUIRE(finite_p3_y);
}

} // namespace

TEST_CASE("The skip-flush command names the next object before the outgoing filament end macro",
          "[GCode][GCodeExportWorkflows][SettingsVariance][SupportObjectSkipFlush]")
{
    const auto [scenario, enabled, bbl_printer, exclude_object] =
        GENERATE(table<std::string, bool, bool, bool>({
            {"enabled BBL", true, true, true},
            {"disabled BBL", false, true, true},
            {"exclude-object gate disabled", true, false, false},
        }));
    INFO("scenario=" << scenario);

    SkipFlushFixture fixture;
    fixture.enabled = enabled;
    fixture.bbl_printer = bbl_printer;
    fixture.exclude_object = exclude_object;
    const std::string output = export_with_object_rows(10., 30., 180., nullptr, false, &fixture);

    REQUIRE(fixture.used_extruders.size() == 2);
    CHECK(fixture.used_extruders[0] == 0u);
    CHECK(fixture.used_extruders[1] == 1u);
    REQUIRE(fixture.sorted_label_ids.size() == 2);
    REQUIRE(fixture.first_label_id != fixture.second_label_id);

    size_t end_macro_count = 0;
    const auto previous_line = previous_line_before_exact_line(output, "M117 END_TOOL_0", end_macro_count);
    // GCode emits END_TOOL_0 once during the 0-to-1 change and again at print shutdown.
    REQUIRE(end_macro_count == 2);
    REQUIRE(previous_line.has_value());
    INFO("previous_line=" << *previous_line);

    const bool injection_expected = enabled && (bbl_printer || exclude_object);
    const bool has_pre_macro_mask = previous_line->rfind("M624 ", 0) == 0;
    if (injection_expected) {
        REQUIRE(has_pre_macro_mask);
        const auto selected_label_ids = label_ids_from_m624_line(*previous_line, fixture.sorted_label_ids);
        REQUIRE(selected_label_ids.has_value());
        REQUIRE(selected_label_ids->size() == 1);
        CHECK(selected_label_ids->front() == fixture.second_label_id);
        CHECK(selected_label_ids->front() != fixture.first_label_id);
    } else {
        CHECK_FALSE(has_pre_macro_mask);
    }
}

TEST_CASE("Manual filament change mode selects the exported BBL tool-change contract",
          "[GCode][GCodeManualFilamentChange][GCodeExportWorkflows]")
{
    const std::string automatic = export_with_object_rows(10., 30., 180., nullptr, false);
    const std::string manual = export_with_object_rows(10., 30., 180., nullptr, true);

    // In automatic mode the first tool change runs the configured custom block and BBL command.
    REQUIRE(route_marker(automatic).has_value());
    CHECK(automatic.find("M1020 S1") != std::string::npos);
    // Manual mode emits the tagged manual T command and skips that first custom block.
    const std::string manual_tag = GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Manual_Tool_Change);
    CHECK(manual.find(manual_tag) != std::string::npos);
    CHECK(manual.find("M1020 S1") == std::string::npos);
    const std::size_t automatic_markers = complete_route_marker_count(automatic);
    const std::size_t manual_markers = complete_route_marker_count(manual);
    REQUIRE(automatic_markers > 1);
    CHECK(manual_markers == automatic_markers - 1);
    CHECK(manual_markers > 0);
}

TEST_CASE("Filament change placeholders expose a bounded obstacle route",
          "[GCodeTravelPointReachable][GCodeExportWorkflows]")
{
    // The 20x20x2 meshes occupy x=0..20 and x=26..46; get_path_of_change_filament
    // expands each interval by 2 mm, leaving a left candidate at 24 below the
    // configured start x=30 and a right candidate at 48.  With end_x=30, the
    // right candidate is farther (18 mm) than the left candidate (6 mm). With both
    // objects below the cutter area, the production left-travel guard rejects
    // the left candidate and emits the right route.
    GCodeProcessorResult processor_result;
    const std::string blocked_output = export_with_object_rows(10., 30., 180., &processor_result);
    INFO(bounded_filament_sequence(processor_result));
    INFO("bounded custom-token excerpt:\n" << bounded_custom_token_excerpt(blocked_output));
    const auto blocked = route_marker(blocked_output);
    REQUIRE(blocked.has_value());
    require_finite(*blocked);
    CHECK_THAT(blocked->p1_x, Catch::Matchers::WithinAbs(blocked->p2_x, 1e-6));
    CHECK_THAT(blocked->p2_x, Catch::Matchers::WithinAbs(blocked->p3_x, 1e-6));
    CHECK_THAT(blocked->p1_y, Catch::Matchers::WithinAbs(0., 1e-6));
    CHECK_THAT(blocked->p2_y, Catch::Matchers::WithinAbs(0., 1e-6));
    CHECK_THAT(blocked->p1_x, Catch::Matchers::WithinAbs(48., 1e-6));
    CHECK_THAT(blocked->p3_y, Catch::Matchers::WithinAbs(180., 1e-6));
}

TEST_CASE("Filament change route keeps the left candidate when the cutter is clear",
          "[GCodeTravelPointReachable][GCodeExportWorkflows]")
{
    // Move both small objects above cutter_area_y=120.  The same interval
    // geometry now permits the left route.  Its three points must be the
    // production turn at (start_x, cutter_y), the left candidate, and the
    // configured end Y; this proves route shape and all six placeholder values.
    GCodeProcessorResult processor_result;
    const std::string clear_output = export_with_object_rows(150., 30., 180., &processor_result);
    INFO(bounded_filament_sequence(processor_result));
    INFO("bounded custom-token excerpt:\n" << bounded_custom_token_excerpt(clear_output));
    const auto clear = route_marker(clear_output);
    REQUIRE(clear.has_value());
    require_finite(*clear);
    CHECK_THAT(clear->p1_x, Catch::Matchers::WithinAbs(30., 1e-6));
    CHECK_THAT(clear->p1_y, Catch::Matchers::WithinAbs(120., 1e-6));
    CHECK_THAT(clear->p2_x, Catch::Matchers::WithinAbs(24., 1e-6));
    CHECK_THAT(clear->p2_y, Catch::Matchers::WithinAbs(120., 1e-6));
    CHECK_THAT(clear->p3_x, Catch::Matchers::WithinAbs(24., 1e-6));
    CHECK_THAT(clear->p3_y, Catch::Matchers::WithinAbs(180., 1e-6));
}

TEST_CASE("Filament change route responds to endpoint configuration",
          "[GCodeTravelPointReachable][GCodeExportWorkflows]")
{
    // This case is intentionally independent of the interval constants above:
    // a valid route with no left-travel restriction must carry a changed end Y
    // through placeholder publication.  The helper is kept tiny and bounded.
    GCodeProcessorResult processor_result;
    const std::string output = export_with_object_rows(150., 30., 160., &processor_result);
    REQUIRE(output.size() < kMaxOutputBytes);
    INFO(bounded_filament_sequence(processor_result));
    INFO("bounded custom-token excerpt:\n" << bounded_custom_token_excerpt(output));
    const auto route = route_marker(output);
    REQUIRE(route.has_value());
    require_finite(*route);
    CHECK_THAT(route->p3_y, Catch::Matchers::WithinAbs(160., 1e-6));
}

} // namespace gcode_travel_point_reachable_contracts

namespace transform_gcode_contracts {

struct TransformMotion
{
    Vec2f position;
    std::string raw;
    bool has_e = false;
    bool has_f = false;
    bool has_z = false;
    bool has_i = false;
    bool has_j = false;
    float e = 0.f;
    float f = 0.f;
    float z = 0.f;
    float i = 0.f;
    float j = 0.f;
};

std::vector<TransformMotion> parse_transformed_g1(const std::string &gcode)
{
    std::vector<TransformMotion> result;
    GCodeReader reader;
    reader.parse_buffer(gcode, [&result](GCodeReader &state, const GCodeReader::GCodeLine &line) {
        if (!line.cmd_is("G1"))
            return;
        TransformMotion move;
        move.position = Vec2f(line.new_X(state), line.new_Y(state));
        move.raw = line.raw();
        move.has_e = line.has_e();
        move.has_f = line.has_f();
        move.has_z = line.has_z();
        move.has_i = line.has_i();
        move.has_j = line.has_j();
        if (move.has_e)
            move.e = line.e();
        if (move.has_f)
            move.f = line.f();
        if (move.has_z)
            move.z = line.z();
        if (move.has_i)
            move.i = line.i();
        if (move.has_j)
            move.j = line.j();
        result.push_back(std::move(move));
    });
    return result;
}

TEST_CASE("G1 transformation rotates and translates coordinates while preserving motion words",
          "[GCode][TransformGCode]")
{
    const std::string comment = "; preserve this comment X99 Y99\n";
    const std::string input = comment
        + "G1 X1.000 Y2.000 E1.500 F1200 Z0.200 I0.500 J-0.250\n"
        + "G1 X3.000 Y4.000 E2.500 F900 Z0.300 I-0.500 J0.250\n";
    const float quarter_turn = float(std::acos(-1.0) / 2.0);
    const std::string output = transform_gcode(input, Vec2f(1.f, 2.f), Vec2f(10.f, 20.f), quarter_turn);
    const auto motions = parse_transformed_g1(output);

    REQUIRE(motions.size() == 2);
    CHECK_THAT(motions[0].position.x(), Catch::Matchers::WithinAbs(8.f, 1e-4f));
    CHECK_THAT(motions[0].position.y(), Catch::Matchers::WithinAbs(21.f, 1e-4f));
    CHECK_THAT(motions[1].position.x(), Catch::Matchers::WithinAbs(6.f, 1e-4f));
    CHECK_THAT(motions[1].position.y(), Catch::Matchers::WithinAbs(23.f, 1e-4f));
    CHECK(output.find(comment) != std::string::npos);
    REQUIRE(motions[0].has_e);
    REQUIRE(motions[0].has_f);
    REQUIRE(motions[0].has_z);
    REQUIRE(motions[0].has_i);
    REQUIRE(motions[0].has_j);
    CHECK_THAT(motions[0].e, Catch::Matchers::WithinAbs(1.5f, 1e-4f));
    CHECK_THAT(motions[0].f, Catch::Matchers::WithinAbs(1200.f, 1e-4f));
    CHECK_THAT(motions[0].z, Catch::Matchers::WithinAbs(0.2f, 1e-4f));
    CHECK_THAT(motions[0].i, Catch::Matchers::WithinAbs(0.5f, 1e-4f));
    CHECK_THAT(motions[0].j, Catch::Matchers::WithinAbs(-0.25f, 1e-4f));
    CHECK(output.find("E1.500") != std::string::npos);
    CHECK(output.find("F1200") != std::string::npos);
    CHECK(output.find("Z0.200") != std::string::npos);
    CHECK(output.find("I0.500") != std::string::npos);
    CHECK(output.find("J-0.250") != std::string::npos);
}

TEST_CASE("G1 transformation carries modal coordinates across partial XY words",
          "[GCode][TransformGCode]")
{
    const std::string input = "G1 X1.000 E1.000 F600 Z0.200\nG1 Y3.000 E2.000\n";
    const std::string output = transform_gcode(input, Vec2f(1.f, 2.f), Vec2f(10.f, -4.f), 0.f);
    const auto motions = parse_transformed_g1(output);

    REQUIRE(motions.size() == 2);
    CHECK_THAT(motions[0].position.x(), Catch::Matchers::WithinAbs(11.f, 1e-4f));
    CHECK_THAT(motions[0].position.y(), Catch::Matchers::WithinAbs(-2.f, 1e-4f));
    CHECK_THAT(motions[1].position.x(), Catch::Matchers::WithinAbs(11.f, 1e-4f));
    CHECK_THAT(motions[1].position.y(), Catch::Matchers::WithinAbs(-1.f, 1e-4f));
    CHECK(motions[1].raw.find(" X") == std::string::npos);
    REQUIRE(motions[0].has_e);
    REQUIRE(motions[1].has_e);
    CHECK_THAT(motions[0].e, Catch::Matchers::WithinAbs(1.f, 1e-4f));
    CHECK_THAT(motions[1].e, Catch::Matchers::WithinAbs(2.f, 1e-4f));
}

TEST_CASE("G1 transformation leaves comments and arc commands with IJ words untouched",
          "[GCode][TransformGCode]")
{
    const std::string arc = "G2 X4.000 Y5.000 I1.000 J0.000 E2.000 F700 ; arc stays local";
    const std::string input = "; G1 X99 Y99\n" + arc + "\nG1 X4.000 Y5.000 E3.000\n";
    const std::string output = transform_gcode(input, Vec2f(4.f, 5.f), Vec2f(10.f, -2.f), 0.f);
    const auto motions = parse_transformed_g1(output);

    REQUIRE(motions.size() == 1);
    CHECK_THAT(motions.front().position.x(), Catch::Matchers::WithinAbs(14.f, 1e-4f));
    CHECK_THAT(motions.front().position.y(), Catch::Matchers::WithinAbs(3.f, 1e-4f));
    CHECK(output.find("; G1 X99 Y99\n") != std::string::npos);
    CHECK(output.find(arc) != std::string::npos);
}

TEST_CASE("G1 never-skip markers force repeated transformed coordinates and are removed",
          "[GCode][TransformGCode]")
{
    const std::string input = std::string("G1 X1.000 Y2.000 E1.000\n"
        "G1 X1.000 Y2.000 E2.000 ") + WipeTower::never_skip_tag() + "\n";
    const std::string output = transform_gcode(input, Vec2f(1.f, 2.f), Vec2f(10.f, 20.f), 0.f);
    const auto motions = parse_transformed_g1(output);

    REQUIRE(motions.size() == 2);
    CHECK_THAT(motions[0].position.x(), Catch::Matchers::WithinAbs(11.f, 1e-4f));
    CHECK_THAT(motions[0].position.y(), Catch::Matchers::WithinAbs(22.f, 1e-4f));
    CHECK_THAT(motions[1].position.x(), Catch::Matchers::WithinAbs(11.f, 1e-4f));
    CHECK_THAT(motions[1].position.y(), Catch::Matchers::WithinAbs(22.f, 1e-4f));
    CHECK(motions[1].raw.find(" X11.000") != std::string::npos);
    CHECK(motions[1].raw.find(" Y22.000") != std::string::npos);
    CHECK(output.find(WipeTower::never_skip_tag()) == std::string::npos);
}

TEST_CASE("G1 transformation preserves the position of repeated untagged moves",
          "[GCode][TransformGCode]")
{
    const std::string input = "G1 X1.000 Y2.000 E1.000\n"
                              "G1 X1.000 Y2.000 E2.000\n"
                              "G1 E3.000\n";
    const std::string output = transform_gcode(input, Vec2f(1.f, 2.f), Vec2f(10.f, 20.f), 0.f);
    const auto motions = parse_transformed_g1(output);

    REQUIRE(motions.size() == 3);
    for (std::size_t index = 0; index < motions.size(); ++index) {
        CAPTURE(index);
        CHECK_THAT(motions[index].position.x(), Catch::Matchers::WithinAbs(11.f, 1e-4f));
        CHECK_THAT(motions[index].position.y(), Catch::Matchers::WithinAbs(22.f, 1e-4f));
        REQUIRE(motions[index].has_e);
        CHECK_THAT(motions[index].e, Catch::Matchers::WithinAbs(float(index + 1), 1e-4f));
    }
}

} // namespace transform_gcode_contracts
namespace gcode_resonance_contracts {

using namespace Slic3r;
using namespace Slic3r::Test;

namespace {

struct OuterWallFeeds {
    std::array<std::vector<double>, 6> by_layer;
    size_t layer_changes = 0;
    bool finite = true;
};

std::string export_resonance_cube(const DynamicPrintConfig &config)
{
    Model model;
    Print print;
    init_print({make_cube(12.0, 12.0, 1.2)}, print, model, config);
    const auto validation = print.validate();
    INFO("Print validation error: " << validation.string << " [" << validation.opt_key << "]");
    REQUIRE(validation.string.empty());
    const std::string output = gcode(print);
    REQUIRE_FALSE(output.empty());
    return output;
}

OuterWallFeeds parse_later_outer_wall_feeds(const std::string &text, const DynamicPrintConfig &config)
{
    OuterWallFeeds result;
    GCodeReader reader;
    reader.apply_config(config);
    int layer = -1;
    bool outer_wall = false;
    reader.parse_buffer(text, [&result, &layer, &outer_wall](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        const std::string_view comment = line.comment();
        if (comment.find("LAYER_CHANGE") != std::string_view::npos) {
            ++layer;
            ++result.layer_changes;
            // TYPE markers are emitted only when the role changes. Preserve the
            // active role across a layer marker so a continuing outer wall is
            // attributed to the next layer as well.
        }
        if (comment.rfind("TYPE:", 0) == 0)
            outer_wall = comment == "TYPE:Outer wall";
        if (layer < 1 || layer >= 6 || !outer_wall || !line.extruding(self) || line.dist_XY(self) <= EPSILON)
            return;
        const double feed = double(line.new_F(self));
        if (!std::isfinite(feed) || feed <= 0.)
            result.finite = false;
        else
            result.by_layer[static_cast<size_t>(layer)].push_back(feed);
    });
    return result;
}

void check_resonance_row(double requested, double expected, bool enabled)
{
    DynamicPrintConfig config = gcode_broad_contracts::base_config();
    config.set_deserialize_strict({
        {"wall_loops", 1},
        {"top_shell_layers", 0},
        {"bottom_shell_layers", 0},
        {"sparse_infill_density", "0%"},
        {"skirt_loops", 0},
        {"brim_type", "no_brim"},
        {"raft_layers", 0},
        {"enable_support", false},
        {"outer_wall_speed", requested},
        {"initial_layer_speed", 5.0},
        {"initial_layer_infill_speed", 5.0},
        {"slow_down_layers", 0},
        {"slow_down_for_layer_cooling", false},
        {"slow_down_layer_time", 0.0},
        {"slow_down_min_speed", 1.0},
        {"fan_cooling_layer_time", 0.0},
        {"small_perimeter_speed", "0"},
        {"small_perimeter_threshold", 0.0},
        {"small_support_perimeter_speed", "0"},
        {"small_support_perimeter_threshold", 0.0},
        {"slowdown_for_curled_perimeters", false},
        {"enable_overhang_speed", false},
        {"filament_max_volumetric_speed", "1000"},
        {"filament_adaptive_volumetric_speed", false},
        {"resonance_avoidance", enabled},
        {"min_resonance_avoidance_speed", 30.0},
        {"max_resonance_avoidance_speed", 60.0},
    });

    const std::string output = export_resonance_cube(config);
    const OuterWallFeeds feeds = parse_later_outer_wall_feeds(output, config);
    REQUIRE(feeds.layer_changes == 6);
    REQUIRE(feeds.finite);
    for (size_t layer = 1; layer < feeds.by_layer.size(); ++layer) {
        INFO("layer=" << layer << " requested_mm_per_s=" << requested);
        REQUIRE_FALSE(feeds.by_layer[layer].empty());
        for (const double feed : feeds.by_layer[layer]) {
            INFO("feed_mm_per_s=" << feed / 60.0);
            CHECK_THAT(feed, Catch::Matchers::WithinAbs(expected * 60.0, 0.25));
        }
    }
}

} // namespace

TEST_CASE("Resonance avoidance clamps lower and upper outer-wall speeds",
          "[GCode][ResonanceAvoidanceGCode]")
{
    const auto [requested, expected] = GENERATE(table<double, double>({
        {40., 30.}, // lower half clamps to the configured minimum.
        {45., 60.}, // exact midpoint takes the strict upper branch.
        {50., 60.}, // upper half selects the configured maximum.
    }));
    check_resonance_row(requested, expected, true);
}

TEST_CASE("Resonance avoidance preserves the exact upper bound and bypasses over-band speed",
          "[GCode][ResonanceAvoidanceGCode]")
{
    const auto [requested, expected] = GENERATE(table<double, double>({
        {60., 60.}, // speed < max is false at the exact upper bound.
        {80., 80.}, // the source speed above max disables avoidance for this loop.
    }));
    check_resonance_row(requested, expected, true);
}

TEST_CASE("Resonance avoidance is a public feature gate for outer-wall speed",
          "[GCode][ResonanceAvoidanceGCode]")
{
    check_resonance_row(40., 40., false);
}

} // namespace gcode_resonance_contracts
namespace chamber_temperature_pipeline_contracts {
using namespace Slic3r;
using namespace Slic3r::Test;

namespace {

struct ChamberCommand
{
    std::string code;
    double      target = 0.0;
    size_t      extrusions_before = 0;
};

struct ChamberEvidence
{
    std::vector<ChamberCommand> commands;
    std::array<size_t, 3>       tool_extrusions{};
    size_t                      extrusions = 0;
    size_t                      tool_one_changes = 0;
};

ChamberEvidence parse_chamber_commands(const std::string &text, const DynamicPrintConfig &config)
{
    ChamberEvidence evidence;
    GCodeReader reader;
    reader.apply_config(config);
    int current_tool = 0;
    reader.parse_buffer(text, [&evidence, &current_tool](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        if (line.cmd_is("M191") || line.cmd_is("M141")) {
            float target = -1.f;
            REQUIRE(line.has_value('S', target));
            REQUIRE(std::isfinite(target));
            evidence.commands.push_back({line.cmd_is("M191") ? "M191" : "M141", target, evidence.extrusions});
        }
        if (line.cmd_is("T0"))
            current_tool = 0;
        else if (line.cmd_is("T1")) {
            current_tool = 1;
            ++evidence.tool_one_changes;
        } else if (line.cmd_is("T2"))
            current_tool = 2;
        if (line.extruding(self) && line.dist_XY(self) > EPSILON) {
            ++evidence.extrusions;
            REQUIRE(current_tool >= 0);
            REQUIRE(current_tool < int(evidence.tool_extrusions.size()));
            ++evidence.tool_extrusions[size_t(current_tool)];
        }
    });
    return evidence;
}

void require_chamber_material_invariant(const std::string &observed, const DynamicPrintConfig &observed_config,
                                        const std::string &control, const DynamicPrintConfig &control_config)
{
    const auto observed_motion = gcode_print_variance_contracts::motion_evidence(observed, observed_config);
    const auto control_motion = gcode_print_variance_contracts::motion_evidence(control, control_config);
    REQUIRE(observed_motion.extrusion_moves > 0);
    gcode_print_variance_contracts::require_same_material(observed_motion, control_motion);
}

} // namespace

TEST_CASE("Chamber temperature activation emits guarded start and shutdown targets",
          "[GCode][GCodeExportWorkflows][ChamberTemperaturePipeline]")
{
    const auto [active, target] = GENERATE(table<bool, int>({
        {false, 0}, {false, 65}, {false, 1500},
        {true, 0},  {true, 65},  {true, 1500},
    }));
    INFO("active=" << active << ", target=" << target);

    DynamicPrintConfig config = gcode_print_variance_contracts::base_config();
    config.set_deserialize_strict({
        {"activate_chamber_temp_control", active ? "1" : "0"},
        {"chamber_temperature", std::to_string(target)},
    });
    const auto exported = gcode_print_variance_contracts::fresh_custom_mesh_export(config, make_cube(5., 5., 0.8));

    DynamicPrintConfig control_config = config;
    control_config.set_deserialize_strict({
        {"activate_chamber_temp_control", "0"},
        {"chamber_temperature", "0"},
    });
    const auto control = gcode_print_variance_contracts::fresh_custom_mesh_export(control_config, make_cube(5., 5., 0.8));
    require_chamber_material_invariant(exported.text, config, control.text, control_config);

    const ChamberEvidence evidence = parse_chamber_commands(exported.text, config);
    const bool should_emit = active && target > 0;
    if (!should_emit) {
        CHECK(evidence.commands.empty());
        return;
    }

    REQUIRE(evidence.commands.size() == 2);
    CHECK(evidence.commands[0].code == "M191");
    CHECK_THAT(evidence.commands[0].target, Catch::Matchers::WithinAbs(target, 1e-4));
    CHECK(evidence.commands[0].extrusions_before == 0);
    CHECK(evidence.commands[1].code == "M141");
    CHECK_THAT(evidence.commands[1].target, Catch::Matchers::WithinAbs(0., 1e-4));
    CHECK(evidence.commands[1].extrusions_before == evidence.extrusions);
}

TEST_CASE("Chamber targets follow used-filament activation, maximum temperature, and custom-start ownership",
          "[GCode][GCodeExportWorkflows][ChamberTemperaturePipeline]")
{
    const auto [activation, temperatures, custom_start, expected_automatic] = GENERATE(table<std::string, std::string, std::string, bool>({
        {"1,0,0", "65,80,99", "", true},
        {"0,1,0", "65,80,99", "", true},
        {"0,0,1", "65,80,99", "", false},
        {"1,0,0", "65,80,99", "M141 S44\n", false},
        {"1,0,0", "65,80,99", "M191 S44\n", false},
    }));
    INFO("activation=" << activation << ", temperatures=" << temperatures << ", custom_start=" << custom_start);

    DynamicPrintConfig config = Test::multifilament_config(3, {
        {"layer_height", 0.2},
        {"initial_layer_print_height", 0.2},
        {"wall_loops", 1},
        {"top_shell_layers", 1},
        {"bottom_shell_layers", 1},
        {"sparse_infill_density", "20%"},
        {"outer_wall_filament_id", 1},
        {"inner_wall_filament_id", 1},
        {"sparse_infill_filament_id", 2},
        {"internal_solid_filament_id", 2},
        {"top_surface_filament_id", 2},
        {"bottom_surface_filament_id", 2},
        {"first_layer_sequence_choice", "Customize"},
        {"first_layer_print_sequence", "1,2"},
        {"other_layers_sequence_choice", "Customize"},
        {"other_layers_print_sequence_nums", 1},
        {"other_layers_print_sequence", "2,10,1,2"},
        {"activate_chamber_temp_control", activation},
        {"chamber_temperature", temperatures},
        {"machine_start_gcode", custom_start},
        {"machine_end_gcode", ""},
        {"gcode_flavor", "marlin2"},
        {"use_relative_e_distances", true},
        {"layer_change_gcode", "G92 E0\n"},
        {"print_sequence", "by layer"},
        {"single_extruder_multi_material", false},
        {"enable_prime_tower", false},
        {"purge_in_prime_tower", false},
        {"skirt_loops", 0},
        {"brim_type", "no_brim"},
        {"raft_layers", 0},
    });

    const auto exported = gcode_print_variance_contracts::fresh_custom_mesh_export(config, make_cube(5., 5., 2.));
    const auto evidence = parse_chamber_commands(exported.text, config);
    REQUIRE(evidence.extrusions > 0);
    REQUIRE(evidence.tool_one_changes > 0);
    REQUIRE(evidence.tool_extrusions[0] > 0);
    REQUIRE(evidence.tool_extrusions[1] > 0);
    CHECK(evidence.tool_extrusions[2] == 0);

    const bool used_activation = activation.rfind("1,", 0) == 0 || activation.rfind("0,1", 0) == 0;
    if (!used_activation) {
        CHECK(evidence.commands.empty());
    } else {
        REQUIRE(evidence.commands.size() == 2);
        CHECK(evidence.commands[1].code == "M141");
        CHECK_THAT(evidence.commands[1].target, Catch::Matchers::WithinAbs(0., 1e-4));
        CHECK(evidence.commands[1].extrusions_before == evidence.extrusions);
        if (expected_automatic) {
            CHECK(evidence.commands[0].code == "M191");
            CHECK_THAT(evidence.commands[0].target, Catch::Matchers::WithinAbs(80., 1e-4));
        } else {
            CHECK(evidence.commands[0].code == (custom_start.rfind("M141", 0) == 0 ? "M141" : "M191"));
            CHECK_THAT(evidence.commands[0].target, Catch::Matchers::WithinAbs(44., 1e-4));
        }
        CHECK(evidence.commands[0].extrusions_before == 0);
    }

    DynamicPrintConfig control_config = config;
    control_config.set_deserialize_strict({
        {"activate_chamber_temp_control", "0,0,0"},
        {"machine_start_gcode", ""},
    });
    const auto control = gcode_print_variance_contracts::fresh_custom_mesh_export(control_config, make_cube(5., 5., 2.));
    require_chamber_material_invariant(exported.text, config, control.text, control_config);
}

TEST_CASE("Chamber minimal temperature supplies the custom start wait with automatic control disabled",
          "[GCode][GCodeExportWorkflows][ChamberTemperaturePipeline]")
{
    const int minimal_temperature = GENERATE(0, 50, 1500);
    CAPTURE(minimal_temperature);
    DynamicPrintConfig config = gcode_print_variance_contracts::base_config();
    config.set_deserialize_strict({
        {"activate_chamber_temp_control", "0"},
        {"chamber_temperature", "1500"},
        {"chamber_minimal_temperature", std::to_string(minimal_temperature)},
        {"machine_start_gcode", "M191 S[chamber_minimal_temperature]\n"},
    });
    const auto exported = gcode_print_variance_contracts::fresh_custom_mesh_export(config, make_cube(5., 5., 0.8));

    DynamicPrintConfig control_config = config;
    control_config.set_deserialize_strict({{"machine_start_gcode", ""}});
    const auto control = gcode_print_variance_contracts::fresh_custom_mesh_export(control_config, make_cube(5., 5., 0.8));
    require_chamber_material_invariant(exported.text, config, control.text, control_config);

    const ChamberEvidence evidence = parse_chamber_commands(exported.text, config);
    REQUIRE(evidence.extrusions > 0);
    REQUIRE(evidence.commands.size() == 1);
    CHECK(evidence.commands.front().code == "M191");
    CHECK_THAT(evidence.commands.front().target, Catch::Matchers::WithinAbs(minimal_temperature, 1e-4));
    CHECK(evidence.commands.front().extrusions_before == 0);
    CHECK(parse_chamber_commands(control.text, control_config).commands.empty());
}

} // namespace chamber_temperature_pipeline_contracts

TEST_CASE("Filament z-hop type overrides preserve material while selecting public lift geometry",
          "[GCode][ZHopType][FilamentRetractionOverrides][Pipeline]")
{
    struct Row {
        const char *serialized;
        const char *name;
        ZHopType    effective_type;
    };
    const std::array<Row, 5> rows{{
        {"nil",         "nil fallback", ZHopType::zhtNormal},
        {"Auto Lift",   "auto",         ZHopType::zhtAuto},
        {"Normal Lift", "normal",       ZHopType::zhtNormal},
        {"Slope Lift",  "slope",        ZHopType::zhtSlope},
        {"Spiral Lift", "spiral",       ZHopType::zhtSpiral},
    }};

    DynamicPrintConfig baseline_config = gcode_print_variance_contracts::zhop_type_config(ZHopType::zhtNormal);
    baseline_config.set_deserialize_strict({
        {"filament_map", "1"},
        {"filament_z_hop_types", "nil"},
    });
    const auto baseline_output = gcode_print_variance_contracts::fresh_export(baseline_config, true);
    const auto baseline_material = gcode_print_variance_contracts::motion_evidence(baseline_output.text, baseline_config);
    const auto baseline_motion = gcode_print_variance_contracts::zhop_motion_evidence(baseline_output.text, baseline_config);
    REQUIRE(baseline_material.extrusion_moves > 0);
    REQUIRE(baseline_motion.retractions > 0);
    REQUIRE(baseline_motion.unretractions > 0);
    REQUIRE(baseline_motion.same_layer_excursions > 0);
    REQUIRE(baseline_motion.same_layer_z_only_lift_moves > 0);
    CHECK(baseline_motion.same_layer_diagonal_lift_moves == 0);

    for (const Row &row : rows) {
        INFO("filament z-hop type " << row.name);
        DynamicPrintConfig config = gcode_print_variance_contracts::zhop_type_config(ZHopType::zhtNormal);
        config.set_deserialize_strict({
            {"filament_map", "1"},
            {"filament_z_hop_types", row.serialized},
        });
        const auto *override_option = config.option<ConfigOptionEnumsGenericNullable>("filament_z_hop_types", true);
        REQUIRE(override_option != nullptr);
        REQUIRE(override_option->values.size() == 1);
        if (std::string(row.serialized) == "nil")
            REQUIRE(override_option->is_nil());
        else
            REQUIRE_FALSE(override_option->is_nil());

        const auto output = gcode_print_variance_contracts::fresh_export(config, true);
        const auto material = gcode_print_variance_contracts::motion_evidence(output.text, config);
        const auto motion = gcode_print_variance_contracts::zhop_motion_evidence(output.text, config);
        REQUIRE(material.extrusion_moves > 0);
        REQUIRE(motion.retractions > 0);
        REQUIRE(motion.unretractions > 0);
        REQUIRE(motion.same_layer_excursions > 0);
        gcode_print_variance_contracts::require_same_material(baseline_material, material);

        if (row.effective_type == ZHopType::zhtNormal) {
            REQUIRE(motion.same_layer_z_only_lift_moves > 0);
            CHECK(motion.same_layer_diagonal_lift_moves == 0);
        } else if (row.effective_type == ZHopType::zhtSlope) {
            REQUIRE(motion.same_layer_diagonal_lift_moves > 0);
            CHECK(motion.same_layer_maximum_diagonal_run == 1);
        } else if (row.effective_type == ZHopType::zhtSpiral) {
            REQUIRE(motion.same_layer_maximum_diagonal_run >= 4);
        } else {
            // A straight-walled cube has no overhang crossing, so Auto selects the
            // ordinary slope route. The overhang-specific Auto route is covered by
            // the dedicated support_fixture case above.
            REQUIRE(motion.same_layer_diagonal_lift_moves > 0);
            CHECK(motion.same_layer_maximum_diagonal_run == 1);
            CHECK(motion.same_layer_overhang_spiral_excursions == 0);
        }
    }
}

namespace gcode_print_variance_contracts {
namespace calibration_gcode_followup {

namespace {

struct PublicPALineSegment {
    double pressure_advance = std::numeric_limits<double>::quiet_NaN();
    double z = std::numeric_limits<double>::quiet_NaN();
    double xy = 0.0;
    double e = 0.0;
};

struct PublicPALineEvidence {
    std::vector<double> pressure_advances;
    std::vector<PublicPALineSegment> outer_wall_segments;
    std::vector<double> bottom_surface_zs;
    std::vector<double> top_surface_zs;
    size_t              positive_xy_moves = 0;
    double              positive_xy_path = 0.0;
    double              bottom_surface_xy_path = 0.0;
    double              top_surface_xy_path = 0.0;
    bool                finite = true;
};

PublicPALineEvidence parse_public_pa_line_export(const std::string &gcode,
                                                  const DynamicPrintConfig &config)
{
    PublicPALineEvidence evidence;
    GCodeReader reader;
    reader.apply_config(config);
    std::string active_role;
    double active_pressure_advance = std::numeric_limits<double>::quiet_NaN();
    reader.parse_buffer(gcode, [&evidence, &active_role, &active_pressure_advance](GCodeReader &self,
                                                                                    const GCodeReader::GCodeLine &line) {
        const auto trim_tag = [](std::string_view value) {
            while (!value.empty() && std::isspace(static_cast<unsigned char>(value.front())))
                value.remove_prefix(1);
            while (!value.empty() && std::isspace(static_cast<unsigned char>(value.back())))
                value.remove_suffix(1);
            return value;
        };
        const std::string_view comment = trim_tag(line.comment());
        if (comment.rfind("TYPE:", 0) == 0)
            active_role = std::string(trim_tag(comment.substr(5)));
        else if (comment.rfind("FEATURE:", 0) == 0)
            active_role = std::string(trim_tag(comment.substr(8)));

        if (line.cmd_is("M900")) {
            float pa = 0.f;
            if (!line.has_value('K', pa) || !std::isfinite(pa)) {
                evidence.finite = false;
                return;
            }
            active_pressure_advance = pa;
            evidence.pressure_advances.push_back(pa);
            return;
        }

        if (!(line.cmd_is("G0") || line.cmd_is("G1") || line.cmd_is("G2") || line.cmd_is("G3")) ||
            !line.extruding(self))
            return;
        const double xy = line.dist_XY(self);
        const double de = line.dist_E(self);
        const double z = line.new_Z(self);
        if (!std::isfinite(xy) || !std::isfinite(de) || !std::isfinite(z)) {
            evidence.finite = false;
            return;
        }
        if (xy <= EPSILON || de <= EPSILON)
            return;

        ++evidence.positive_xy_moves;
        evidence.positive_xy_path += xy;
        if (active_role == "Outer wall") {
            if (!std::isfinite(active_pressure_advance)) {
                evidence.finite = false;
                return;
            }
            evidence.outer_wall_segments.push_back({active_pressure_advance, z, xy, de});
        } else if (active_role == "Bottom surface") {
            evidence.bottom_surface_xy_path += xy;
            evidence.bottom_surface_zs.push_back(z);
        } else if (active_role == "Top surface") {
            evidence.top_surface_xy_path += xy;
            evidence.top_surface_zs.push_back(z);
        }
    });
    return evidence;
}

bool contains_pa_sequence(const std::vector<double> &values,
                          const std::array<double, 6> &expected)
{
    if (values.size() < expected.size())
        return false;
    for (size_t start = 0; start + expected.size() <= values.size(); ++start) {
        bool match = true;
        for (size_t i = 0; i < expected.size(); ++i)
            match = match && std::abs(values[start + i] - expected[i]) <= 1e-4;
        if (match)
            return true;
    }
    return false;
}

void require_public_pa_values(const PublicPALineEvidence &evidence)
{
    REQUIRE(evidence.finite);
    REQUIRE(evidence.positive_xy_moves > 0);
    REQUIRE(evidence.positive_xy_path > 0.0);
    REQUIRE_FALSE(evidence.pressure_advances.empty());
    const std::array<double, 6> expected{{0.0, 0.02, 0.0, 0.03, 0.04, 0.0}};
    REQUIRE(contains_pa_sequence(evidence.pressure_advances, expected));
    for (const double pa : evidence.pressure_advances) {
        REQUIRE(std::isfinite(pa));
        CHECK(pa >= -1e-6);
        CHECK(pa <= 0.0401);
    }
}

void require_outer_wall_contract(const PublicPALineEvidence &evidence)
{
    REQUIRE(evidence.outer_wall_segments.size() == 3);
    const std::array<double, 3> expected_pa{{0.02, 0.03, 0.04}};
    for (size_t i = 0; i < expected_pa.size(); ++i) {
        const PublicPALineSegment &segment = evidence.outer_wall_segments[i];
        CHECK_THAT(segment.pressure_advance, Catch::Matchers::WithinAbs(expected_pa[i], 1e-4));
        CHECK_THAT(segment.z, Catch::Matchers::WithinAbs(0.2, 1e-4));
        CHECK_THAT(segment.xy, Catch::Matchers::WithinAbs(40.0, 1e-3));
        REQUIRE(std::isfinite(segment.e));
        REQUIRE(segment.e > 0.0);
    }
}

} // namespace

TEST_CASE("Public PA line export emits calibrated lines and optional number layers",
          "[GCode][Calibration][CalibrationPALine][GCodeExportWorkflows]")
{
    Calib_Params without_numbers_params;
    without_numbers_params.mode = CalibMode::Calib_PA_Line;
    without_numbers_params.start = 0.02;
    without_numbers_params.end = 0.04;
    without_numbers_params.step = 0.01;
    without_numbers_params.print_numbers = false;

    Calib_Params with_numbers_params = without_numbers_params;
    with_numbers_params.print_numbers = true;

    DynamicPrintConfig config = base_config();
    config.set_deserialize_strict({
        {"gcode_flavor", "marlin2"},
        {"printable_area", "0x0,200x0,200x200,0x200"},
        {"nozzle_diameter", "0.4"},
        {"initial_layer_print_height", "0.2"},
        {"z_offset", 0.0},
        {"belt_printer", false},
        {"enable_pressure_advance", false},
        {"adaptive_pressure_advance", false},
        {"adaptive_pressure_advance_overhangs", false},
        {"input_shaping_emit", false},
    });
    const ExportedPrint without_numbers = export_calibration("marlin2", without_numbers_params, &config);
    const ExportedPrint with_numbers = export_calibration("marlin2", with_numbers_params, &config);
    const PublicPALineEvidence plain = parse_public_pa_line_export(without_numbers.text, config);
    const PublicPALineEvidence annotated = parse_public_pa_line_export(with_numbers.text, config);
    require_public_pa_values(plain);
    require_public_pa_values(annotated);
    require_outer_wall_contract(plain);
    require_outer_wall_contract(annotated);

    CHECK_THAT(plain.bottom_surface_xy_path, Catch::Matchers::WithinAbs(0.0, 1e-6));
    CHECK_THAT(plain.top_surface_xy_path, Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE(annotated.bottom_surface_xy_path > 0.0);
    REQUIRE(annotated.top_surface_xy_path > 0.0);
    REQUIRE_FALSE(annotated.bottom_surface_zs.empty());
    REQUIRE_FALSE(annotated.top_surface_zs.empty());
    for (const double z : annotated.bottom_surface_zs)
        CHECK_THAT(z, Catch::Matchers::WithinAbs(0.2, 1e-4));
    for (const double z : annotated.top_surface_zs)
        CHECK_THAT(z, Catch::Matchers::WithinAbs(0.4, 1e-4));

    for (size_t i = 0; i < plain.outer_wall_segments.size(); ++i) {
        const PublicPALineSegment &plain_segment = plain.outer_wall_segments[i];
        const PublicPALineSegment &annotated_segment = annotated.outer_wall_segments[i];
        CHECK_THAT(annotated_segment.pressure_advance,
                   Catch::Matchers::WithinAbs(plain_segment.pressure_advance, 1e-4));
        CHECK_THAT(annotated_segment.z, Catch::Matchers::WithinAbs(plain_segment.z, 1e-4));
        CHECK_THAT(annotated_segment.xy, Catch::Matchers::WithinAbs(plain_segment.xy, 1e-3));
        CHECK_THAT(annotated_segment.e, Catch::Matchers::WithinAbs(plain_segment.e, 1e-5));
    }
    CHECK(annotated.positive_xy_path > plain.positive_xy_path);
    CHECK(annotated.positive_xy_moves > plain.positive_xy_moves);
}

} // namespace calibration_gcode_followup
} // namespace gcode_print_variance_contracts

namespace gcode_print_variance_contracts {
namespace timelapse_all_layers_contracts {

namespace {

ExportedPrint export_all_layer_fixture(const DynamicPrintConfig &config)
{
    Print print;
    Model model;
    // Set the public BBL marker before init_print applies the public configuration.
    print.is_BBL_printer() = true;
    TriangleMesh short_object = make_cube(10., 10., 0.4);
    TriangleMesh tall_object = make_cube(10., 10., 2.0);
    tall_object.translate(65., 0., 0.);

    Test::init_print(std::vector<TriangleMesh>{std::move(short_object), std::move(tall_object)},
                     print, model, config, nullptr, false);

    require_valid(print);

    const auto &objects = print.objects();
    REQUIRE(objects.size() == 2);
    REQUIRE(objects[0] != nullptr);
    REQUIRE(objects[1] != nullptr);
    REQUIRE(objects[0]->instances().size() == 1);
    REQUIRE(objects[1]->instances().size() == 1);
    const BoundingBoxf3 first_box = objects[0]->instances().front().get_bounding_box();
    const BoundingBoxf3 second_box = objects[1]->instances().front().get_bounding_box();
    CHECK_THAT(first_box.min.x(), Catch::Matchers::WithinAbs(0.0, 1e-6));
    CHECK_THAT(first_box.min.y(), Catch::Matchers::WithinAbs(0.0, 1e-6));
    CHECK_THAT(first_box.max.x(), Catch::Matchers::WithinAbs(10.0, 1e-6));
    CHECK_THAT(first_box.max.y(), Catch::Matchers::WithinAbs(10.0, 1e-6));
    CHECK_THAT(first_box.max.z(), Catch::Matchers::WithinAbs(0.4, 1e-6));
    CHECK_THAT(second_box.min.x(), Catch::Matchers::WithinAbs(65.0, 1e-6));
    CHECK_THAT(second_box.min.y(), Catch::Matchers::WithinAbs(0.0, 1e-6));
    CHECK_THAT(second_box.max.x(), Catch::Matchers::WithinAbs(75.0, 1e-6));
    CHECK_THAT(second_box.max.y(), Catch::Matchers::WithinAbs(10.0, 1e-6));
    CHECK_THAT(second_box.max.z(), Catch::Matchers::WithinAbs(2.0, 1e-6));

    return export_print(print);
}

std::vector<Vec3d> all_layer_snapshot_positions(const std::string &text,
                                                const DynamicPrintConfig &config)
{
    std::vector<Vec3d> positions;
    GCodeReader reader;
    reader.apply_config(config);
    reader.parse_buffer(text, [&positions](GCodeReader &state, const GCodeReader::GCodeLine &line) {
        if (!line.cmd_is("M117") || line.raw().find("ALL-LAYER") == std::string::npos)
            return;
        positions.emplace_back(state.x(), state.y(), state.z());
    });
    return positions;
}

bool outside_xy_box(const Vec3d &position, const BoundingBoxf3 &box, double clearance)
{
    const bool outside_x = position.x() < box.min.x() - clearance || position.x() > box.max.x() + clearance;
    const bool outside_y = position.y() < box.min.y() - clearance || position.y() > box.max.y() + clearance;
    return outside_x || outside_y;
}

DynamicPrintConfig all_layer_config()
{
    DynamicPrintConfig config = base_config();
    // One nozzle uses the explicitly configured bed without per-extruder restrictions.
    config.set_key_value("extruder_printable_area", new ConfigOptionPointsGroups{});
    config.set_deserialize_strict({
        {"nozzle_diameter", "0.4"},
        {"printable_area", "0x0,220x0,220x220,0x220"},
        {"printer_structure", "corexy"},
        {"timelapse_type", "1"},
        {"time_lapse_gcode", "G0 X{timelapse_pos_x} Y{timelapse_pos_y}\nM117 ALL-LAYER-[layer_num]-SAFE-{has_timelapse_safe_pos ? 1 : 0}\n"},
        {"gcode_comments", true},
        {"enable_prime_tower", false},
        {"enable_support", false},
        {"extruder_clearance_radius", 20.0},
        {"extruder_clearance_height_to_rod", 2.0},
        {"wall_loops", 1},
        {"top_shell_layers", 0},
        {"bottom_shell_layers", 0},
        {"sparse_infill_density", "15%"},
        {"skirt_loops", 0},
        {"brim_type", "no_brim"},
    });
    return config;
}

} // namespace

TEST_CASE("Smooth timelapse keeps one bed-safe position across all printed layers",
          "[GCode][GCodeExportWorkflows][TimelapseAllLayers]")
{
    const DynamicPrintConfig smooth = all_layer_config();
    const auto output = export_all_layer_fixture(smooth);
    const auto positions = all_layer_snapshot_positions(output.text, smooth);

    const ConfigOptionFloat *clearance_radius = smooth.option<ConfigOptionFloat>("extruder_clearance_radius");
    const ConfigOptionFloat *clearance_height = smooth.option<ConfigOptionFloat>("extruder_clearance_height_to_rod");
    REQUIRE(clearance_radius != nullptr);
    REQUIRE(clearance_height != nullptr);
    CHECK_THAT(clearance_radius->value, Catch::Matchers::WithinAbs(20.0, 1e-6));
    CHECK_THAT(clearance_height->value, Catch::Matchers::WithinAbs(2.0, 1e-6));

    REQUIRE(positions.size() >= 6);
    CHECK(count_text(output.text, "SAFE-1") == positions.size());
    CHECK(count_text(output.text, "SAFE-0") == 0);

    const Vec3d first = positions.front();
    const bool non_default = std::abs(first.x()) > 0.5 || std::abs(first.y()) > 0.5;
    CHECK(non_default);
    // pick_pos_for_all_layer expands each non-ByObject projection by radius / 2.
    // The generated placeholder stores integer coordinates; subtract 1 mm as a
    // conservative truncation allowance, rather than claiming extra clearance.
    const double object_clearance = clearance_radius->value / 2.0 - 1.0;
    REQUIRE(object_clearance > 0.0);
    const BoundingBoxf3 first_box(Vec3d(0., 0., 0.), Vec3d(10., 10., 0.4));
    const BoundingBoxf3 second_box(Vec3d(65., 0., 0.), Vec3d(75., 10., 2.0));
    for (const Vec3d &position : positions) {
        INFO("snapshot position=" << position.x() << "," << position.y() << "," << position.z());
        CHECK(std::isfinite(position.x()));
        CHECK(std::isfinite(position.y()));
        CHECK(std::isfinite(position.z()));
        CHECK(position.x() >= 0.0);
        CHECK(position.x() <= 220.0);
        CHECK(position.y() >= 0.0);
        CHECK(position.y() <= 220.0);
        CHECK_THAT(position.x(), Catch::Matchers::WithinAbs(first.x(), 1e-6));
        CHECK_THAT(position.y(), Catch::Matchers::WithinAbs(first.y(), 1e-6));
        CHECK(outside_xy_box(position, first_box, object_clearance));
        CHECK(outside_xy_box(position, second_box, object_clearance));
    }
}

} // namespace timelapse_all_layers_contracts
} // namespace gcode_print_variance_contracts


namespace print_extents_contracts {

ExtrusionPath planned_path(float width, std::initializer_list<Point3> points)
{
    Points3 scaled_points;
    scaled_points.reserve(points.size());
    for (const Point3 &point : points)
        scaled_points.push_back(point);
    ExtrusionPath path(erPerimeter, 1.0, width, 0.2);
    path.polyline = Polyline3(std::move(scaled_points));
    return path;
}

void check_extents(const BoundingBoxf &bbox, const bool expected_defined,
                   const double min_x = 0., const double min_y = 0.,
                   const double max_x = 0., const double max_y = 0.)
{
    CHECK(bbox.defined == expected_defined);
    if (!expected_defined)
        return;
    CHECK_THAT(bbox.min.x(), Catch::Matchers::WithinAbs(min_x, 1e-5));
    CHECK_THAT(bbox.min.y(), Catch::Matchers::WithinAbs(min_y, 1e-5));
    CHECK_THAT(bbox.max.x(), Catch::Matchers::WithinAbs(max_x, 1e-5));
    CHECK_THAT(bbox.max.y(), Catch::Matchers::WithinAbs(max_y, 1e-5));
}

} // namespace print_extents_contracts

TEST_CASE("Print object extrusion extents honor layers, instances, widths, and nested fills",
          "[GCode][PrintExtents]")
{
    using namespace print_extents_contracts;

    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        {"layer_height", 0.2},
        {"initial_layer_print_height", 0.2},
        {"skirt_loops", 0},
        {"brim_type", "no_brim"},
        {"raft_layers", 0},
        {"enable_support", false},
    });

    Print print;
    Model model;
    Test::init_print(std::vector<TriangleMesh>{Test::cube(1.)}, print, model, config, nullptr, false, 2);
    PrintObject *object = print.get_object(0);
    REQUIRE(object != nullptr);
    REQUIRE(object->instances().size() == 2);
    REQUIRE(object->num_printing_regions() == 1);
    object->instances()[0].shift = Point::new_scale(20., 30.);
    object->instances()[1].shift = Point::new_scale(60., 80.);

    Layer *lower = object->add_layer(0, 0.2, 0.2, 0.1);
    LayerRegion *lower_region = lower->add_region(&object->printing_region(0));
    ExtrusionMultiPath lower_perimeter;
    lower_perimeter.paths.emplace_back(planned_path(0.4f, {
        Point3::new_scale(-2., -1., 0.2), Point3::new_scale(3., -1., 0.2),
    }));
    lower_perimeter.paths.emplace_back(planned_path(1.0f, {
        Point3::new_scale(3., -1., 0.2), Point3::new_scale(3., 4., 0.2),
    }));
    lower_region->perimeters.append(std::move(lower_perimeter));

    ExtrusionMultiPath lower_fill;
    lower_fill.paths.emplace_back(planned_path(0.6f, {
        Point3::new_scale(-4., 2., 0.2), Point3::new_scale(-1., 2., 0.2),
    }));
    ExtrusionEntityCollection nested_fill;
    nested_fill.append(std::move(lower_fill));
    lower_region->fills.append(std::move(nested_fill));

    Layer *upper = object->add_layer(1, 0.2, 0.4, 0.3);
    LayerRegion *upper_region = upper->add_region(&object->printing_region(0));
    ExtrusionMultiPath upper_perimeter;
    upper_perimeter.paths.emplace_back(planned_path(0.8f, {
        Point3::new_scale(0., -6., 0.4), Point3::new_scale(9., -6., 0.4),
    }));
    upper_region->perimeters.append(std::move(upper_perimeter));

    REQUIRE(lower_region->perimeters.size() == 1);
    REQUIRE(lower_region->fills.size() == 1);
    REQUIRE(upper_region->perimeters.size() == 1);

    check_extents(get_print_object_extrusions_extents(*object, 0.199), false);
    check_extents(get_print_object_extrusions_extents(*object, 0.2), true, 15.7, 28.5, 63.5, 84.5);
    check_extents(get_print_object_extrusions_extents(*object, 0.399), true, 15.7, 28.5, 63.5, 84.5);
    check_extents(get_print_object_extrusions_extents(*object, 0.4), true, 15.7, 23.6, 69.4, 84.5);
}


TEST_CASE("First-layer scan marker appears only when enabled on BBL printers",
          "[GCode][GCodeExportWorkflows][FirstLayerScan]")
{
    struct ScanEvidence {
        size_t markers = 0;
        size_t depositions_before = 0;
        size_t depositions_after = 0;
        double max_deposition_z_before = -std::numeric_limits<double>::infinity();
        std::optional<double> first_deposition_z_after;
        bool   marker_words_valid = true;
    };
    const auto inspect = [](const std::string &text, const DynamicPrintConfig &config) {
        ScanEvidence evidence;
        GCodeReader reader;
        reader.apply_config(config);
        reader.parse_buffer(text, [&evidence](GCodeReader &self, const GCodeReader::GCodeLine &line) {
            if (line.cmd_is("M976")) {
                float scan = 0.f;
                float mode = 0.f;
                ++evidence.markers;
                evidence.marker_words_valid = line.has_value('S', scan) && line.has_value('P', mode) &&
                                              std::fabs(scan - 1.f) <= 1e-6f &&
                                              std::fabs(mode - 1.f) <= 1e-6f;
            }
            if (line.extruding(self) && line.dist_XY(self) > EPSILON) {
                const double z = line.new_Z(self);
                REQUIRE(std::isfinite(z));
                if (evidence.markers == 0) {
                    ++evidence.depositions_before;
                    evidence.max_deposition_z_before = std::max(evidence.max_deposition_z_before, z);
                } else {
                    ++evidence.depositions_after;
                    if (!evidence.first_deposition_z_after.has_value())
                        evidence.first_deposition_z_after = z;
                }
            }
        });
        return evidence;
    };
    const auto export_scan_case = [](bool enabled, bool bbl) {
        DynamicPrintConfig config = gcode_print_variance_contracts::base_config();
        config.set_deserialize_strict({
            {"scan_first_layer", enabled},
            {"layer_height", 0.2},
            {"initial_layer_print_height", 0.2},
            {"z_offset", 0.0},
        });
        auto output = gcode_print_variance_contracts::fresh_custom_mesh_export(
            config, make_cube(5., 5., 0.8), bbl);
        return std::make_pair(std::move(config), std::move(output));
    };

    const auto [enabled_config, enabled] = export_scan_case(true, true);
    const auto [disabled_config, disabled] = export_scan_case(false, true);
    const auto [unsupported_config, unsupported] = export_scan_case(true, false);
    const auto enabled_motion = gcode_print_variance_contracts::motion_evidence(enabled.text, enabled_config);
    const auto disabled_motion = gcode_print_variance_contracts::motion_evidence(disabled.text, disabled_config);
    gcode_print_variance_contracts::require_same_material(enabled_motion, disabled_motion);

    const ScanEvidence enabled_scan = inspect(enabled.text, enabled_config);
    const ScanEvidence disabled_scan = inspect(disabled.text, disabled_config);
    const ScanEvidence unsupported_scan = inspect(unsupported.text, unsupported_config);
    REQUIRE(enabled_scan.markers == 1);
    CHECK(enabled_scan.marker_words_valid);
    CHECK(enabled_scan.depositions_before > 0);
    CHECK(enabled_scan.depositions_after > 0);
    CHECK_THAT(enabled_scan.max_deposition_z_before, Catch::Matchers::WithinAbs(0.2, 1e-5));
    REQUIRE(enabled_scan.first_deposition_z_after.has_value());
    CHECK_THAT(*enabled_scan.first_deposition_z_after, Catch::Matchers::WithinAbs(0.4, 1e-5));
    CHECK(disabled_scan.markers == 0);
    CHECK(disabled_scan.depositions_before > 0);
    CHECK(disabled_scan.depositions_after == 0);
    CHECK(unsupported_scan.markers == 0);
    CHECK(gcode_print_variance_contracts::motion_evidence(unsupported.text, unsupported_config).extrusion_moves > 0);
}
