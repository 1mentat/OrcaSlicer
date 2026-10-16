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
#include <cmath>
#include <cstddef>
#include <fstream>
#include <initializer_list>
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

#include "test_helpers.hpp"

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

struct ExportedPrint {
    std::string text;
    bool        label_object_enabled = false;
};

struct MotionEvidence {
    size_t extrusion_moves = 0;
    size_t z_only_moves = 0;
    double xy_path = 0.0;
    double filament = 0.0;
};

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

void require_valid(const Print &print)
{
    const auto error = print.validate();
    INFO("Print validation error: " << error.string << " [" << error.opt_key << "]");
    REQUIRE(error.string.empty());
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

ExportedPrint export_print(Print &print)
{
    ExportedPrint result;
    GCodeProcessorResult processor_result;
    print.set_status_silent();
    print.process();
    ScopedTemporaryFile temp(".gcode");
    print.export_gcode(temp.string(), &processor_result, nullptr);
    std::ifstream input(temp.string(), std::ios::binary);
    result.text.assign((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
    result.label_object_enabled = processor_result.label_object_enabled;
    REQUIRE_FALSE(result.text.empty());
    return result;
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

size_t count_text(const std::string &text, const std::string &needle)
{
    size_t count = 0;
    for (size_t pos = text.find(needle); pos != std::string::npos; pos = text.find(needle, pos + needle.size()))
        ++count;
    return count;
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

MotionEvidence motion_evidence(const std::string &text, const DynamicPrintConfig &config)
{
    MotionEvidence evidence;
    GCodeReader reader;
    reader.apply_config(config);
    reader.parse_buffer(text, [&evidence](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        if (!(line.cmd_is("G0") || line.cmd_is("G1") || line.cmd_is("G2") || line.cmd_is("G3")))
            return;

        const double xy = line.dist_XY(self);
        if (std::abs(line.dist_Z(self)) > EPSILON && xy <= EPSILON)
            ++evidence.z_only_moves;
        if (!line.extruding(self) || xy <= EPSILON)
            return;
        ++evidence.extrusion_moves;
        evidence.xy_path += xy;
        evidence.filament += std::abs(line.dist_E(self));
    });
    return evidence;
}

void require_same_material(const MotionEvidence &lhs, const MotionEvidence &rhs)
{
    REQUIRE(lhs.extrusion_moves > 0);
    REQUIRE(lhs.extrusion_moves == rhs.extrusion_moves);
    CHECK_THAT(rhs.xy_path, Catch::Matchers::WithinRel(lhs.xy_path, 1e-6));
    CHECK_THAT(rhs.filament, Catch::Matchers::WithinRel(lhs.filament, 1e-6));
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


ExportedPrint fresh_custom_mesh_export(const DynamicPrintConfig &config, TriangleMesh mesh)
{
    Print print;
    Model model;
    Test::init_print({std::move(mesh)}, print, model, config);
    require_valid(print);
    return export_print(print);
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

TEST_CASE("Wipe while retracting adds wipe travel without changing deposited material", "[GCode][GCodeExportWorkflows]")
{
    auto off = base_config();
    off.set_deserialize_strict({{"wipe", false}, {"retraction_length", 1.2}});
    auto on = off;
    on.set_deserialize_strict({{"wipe", true}, {"wipe_distance", 2.0}});
    const auto a = fresh_export(off);
    const auto b = fresh_export(on);
    REQUIRE(motion_evidence(a.text, off).extrusion_moves > 0);
    require_same_material(motion_evidence(a.text, off), motion_evidence(b.text, on));
    CHECK(count_text(b.text, "; wipe") > count_text(a.text, "; wipe"));
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


} // namespace gcode_print_variance_contracts
