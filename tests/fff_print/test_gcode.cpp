#include <catch2/matchers/catch_matchers_string.hpp>
#include "libslic3r/GCode/AdaptivePAProcessor.hpp"
#include <cstdlib>
#include <cstdio>
#include <cctype>
#include <sstream>
#include <memory>
#include <limits>
#include "libslic3r/GCode.hpp"
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
    // Five bounded public exports. The smooth-off output is also the flow baseline.
    const DynamicPrintConfig smooth_off_config = spiral_settings_config(false, "0%", 0.0, 0.0);
    const DynamicPrintConfig smooth_zero_config = spiral_settings_config(true, "0%", 0.0, 0.0);
    const DynamicPrintConfig smooth_high_config = spiral_settings_config(true, "1000%", 0.0, 0.0);
    const DynamicPrintConfig flow_start_config = spiral_settings_config(false, "0%", 0.5, 0.0);
    const DynamicPrintConfig flow_finish_config = spiral_settings_config(false, "0%", 0.0, 0.5);

    const SpiralSettingsEvidence smooth_off = export_spiral_settings(smooth_off_config);
    const SpiralSettingsEvidence smooth_zero = export_spiral_settings(smooth_zero_config);
    const SpiralSettingsEvidence smooth_high = export_spiral_settings(smooth_high_config);
    const SpiralSettingsEvidence flow_baseline = smooth_off;
    const SpiralSettingsEvidence flow_start = export_spiral_settings(flow_start_config);
    const SpiralSettingsEvidence flow_finish = export_spiral_settings(flow_finish_config);

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
    const size_t baseline_last_index = last_spiral_settings_layer(flow_baseline);
    const size_t finish_last_index = last_spiral_settings_layer(flow_finish);
    REQUIRE(baseline_first_index < flow_baseline.layers.size());
    REQUIRE(start_first_index < flow_start.layers.size());
    REQUIRE(baseline_last_index < flow_baseline.layers.size());
    REQUIRE(finish_last_index < flow_finish.layers.size());

    const SpiralLayerEvidence &base_first = flow_baseline.layers[baseline_first_index];
    const SpiralLayerEvidence &start_first = flow_start.layers[start_first_index];
    const SpiralLayerEvidence &base_last = flow_baseline.layers[baseline_last_index];
    const SpiralLayerEvidence &finish_last = flow_finish.layers[finish_last_index];
    REQUIRE(base_first.perimeter_extrusions > 0);
    REQUIRE(base_last.perimeter_extrusions > 0);

    // Finishing flow changes only the appended flat transition loop; its rising
    // final loop remains unchanged and is checked independently.
    CHECK(finish_last.spiral_z_extrusions == base_last.spiral_z_extrusions);
    CHECK_THAT(finish_last.spiral_filament_length,
               Catch::Matchers::WithinAbs(base_last.spiral_filament_length, 1e-4));
    CHECK_THAT(finish_last.spiral_xy_path_length,
               Catch::Matchers::WithinAbs(base_last.spiral_xy_path_length, 1e-4));
    const SpiralLayerEvidence *base_transition = final_flat_transition(flow_baseline, baseline_last_index);
    const SpiralLayerEvidence *finish_transition = final_flat_transition(flow_finish, finish_last_index);
    REQUIRE(base_transition != nullptr);
    REQUIRE(finish_transition != nullptr);
    CHECK(start_first.filament_length > base_first.filament_length + 1e-4);
    CHECK(finish_transition->flat_filament_length >
          base_transition->flat_filament_length + 1e-4);
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

namespace settings_speed_accel_contracts {

using namespace Slic3r;
using namespace Slic3r::Test;

namespace {

DynamicPrintConfig motion_config(GCodeFlavor flavor = gcfMarlinFirmware)
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        {"gcode_flavor", flavor == gcfKlipper ? "klipper" : "marlin2"},
        {"layer_height", 0.2},
        {"initial_layer_print_height", 0.2},
        {"nozzle_diameter", 0.4},
        {"wall_loops", 2},
        {"top_shell_layers", 2},
        {"bottom_shell_layers", 2},
        {"top_shell_thickness", 0},
        {"bottom_shell_thickness", 0},
        {"sparse_infill_density", "20%"},
        {"gcode_comments", true},
        {"machine_start_gcode", ""},
        {"machine_end_gcode", ""},
        {"layer_change_gcode", "G92 E0\n"},
        {"skirt_loops", 0},
        {"brim_type", "no_brim"},
        {"raft_layers", 0},
        {"slow_down_for_layer_cooling", false},
        {"slow_down_layer_time", 0},
        {"slow_down_min_speed", 1},
        {"fan_cooling_layer_time", 0},
        {"filament_max_volumetric_speed", "200"},
        {"filament_adaptive_volumetric_speed", false},
        {"machine_max_acceleration_extruding", "20000"},
        {"machine_max_acceleration_retracting", "20000"},
        {"machine_max_acceleration_travel", "20000"},
        {"machine_max_jerk_x", "20000"},
        {"machine_max_jerk_y", "20000"},
        {"machine_max_jerk_z", "20000"},
        {"machine_max_jerk_e", "20000"},
        {"default_acceleration", 1200},
        {"initial_layer_acceleration", 300},
        {"travel_acceleration", 4000},
        {"default_jerk", 10},
        {"travel_jerk", 12},
        {"travel_speed", 80},
        {"initial_layer_speed", 30},
        {"initial_layer_infill_speed", 50},
        {"initial_layer_travel_speed", "50%"},
        {"initial_layer_travel_acceleration", "50%"},
        {"initial_layer_travel_jerk", "50%"},
        {"z_hop", 0.4},
        {"travel_speed_z", 9},
        {"bridge_speed", 21},
        {"internal_bridge_speed", "150%"},
        {"inner_wall_acceleration", 777},
        {"top_surface_acceleration", 888},
        {"accel_to_decel_enable", true},
        {"accel_to_decel_factor", 50},
    });
    return config;
}

TriangleMesh internal_bridge_fixture()
{
    TriangleMesh mesh  = make_cube(30, 24, 3);
    TriangleMesh tower = make_cube(14, 10, 1);
    tower.translate(8, 7, 3);
    mesh.merge(tower);
    return mesh;
}

std::string export_cube(const DynamicPrintConfig& config)
{
    Model model;
    Print print;
    TriangleMesh shape = cube(12.0);
    shape.scale(Vec3f(1.f, 1.f, 0.55f));
    init_print({std::move(shape)}, print, model, config);
    const auto validation = print.validate();
    INFO(validation.string);
    REQUIRE(validation.string.empty());
    const std::string output = gcode(print);
    REQUIRE_FALSE(output.empty());
    return output;
}

struct ParsedMotion
{
    std::map<std::string, std::vector<double>> first_layer_feeds;
    std::map<std::string, std::map<int, std::vector<int>>> role_accelerations;
    std::map<std::string, std::map<int, std::vector<double>>> role_jerks;
    std::vector<double> first_layer_xy_travel_feeds;
    std::vector<double> first_layer_z_travel_feeds;
    std::vector<int> first_layer_xy_travel_accelerations;
    std::vector<double> first_layer_xy_travel_jerks;
    std::vector<int> travel_accelerations;
    std::vector<double> travel_jerks;
    std::vector<double> bridge_feeds;
    size_t extrusions = 0;
    size_t travels    = 0;
};

struct GCodeCommentGuard
{
    const bool saved = GCodeWriter::full_gcode_comment;
    ~GCodeCommentGuard() { GCodeWriter::full_gcode_comment = saved; }
};

static bool first_layer(int layer_index) { return layer_index == 0; }

ParsedMotion parse_motion(const std::string& output, const DynamicPrintConfig& config)
{
    ParsedMotion parsed;
    GCodeReader reader;
    reader.apply_config(config);
    const double layer_height = config.opt_float("layer_height");
    std::string role;
    int layer_index         = -1;
    int print_acceleration  = 0;
    int travel_acceleration = 0;
    double jerk             = 0;
    reader.parse_buffer(output, [&parsed, &role, &layer_index, &print_acceleration, &travel_acceleration, &jerk,
                                 layer_height](GCodeReader& self, const GCodeReader::GCodeLine& line) {
        const std::string_view comment = line.comment();
        if (comment.find("LAYER_CHANGE") != std::string_view::npos) {
            ++layer_index;
            role.clear();
        }
        if (comment.find("FEATURE:") != std::string_view::npos || comment.find("TYPE:") != std::string_view::npos) {
            if (comment.find("Outer wall") != std::string_view::npos)
                role = "Outer wall";
            else if (comment.find("Inner wall") != std::string_view::npos)
                role = "Inner wall";
            else if (comment.find("Top surface") != std::string_view::npos)
                role = "Top surface";
            else if (comment.find("Bottom surface") != std::string_view::npos)
                role = "Bottom surface";
            else if (comment.find("Internal solid infill") != std::string_view::npos)
                role = "Internal solid infill";
            else if (comment.find("Sparse infill") != std::string_view::npos)
                role = "Sparse infill";
            else if (comment.find("Bridge") != std::string_view::npos)
                role = comment.find("Internal bridge") != std::string_view::npos ? "Internal bridge" : "Bridge";
            else
                role.clear();
        }

        float value = 0;
        if (line.cmd_is("M204")) {
            if (line.has_value('S', value) || line.has_value('P', value))
                print_acceleration = int(std::lround(value));
            if (line.has_value('T', value))
                travel_acceleration = int(std::lround(value));
        } else if (line.cmd_is("M205") && line.has_value('X', value)) {
            jerk = value;
        }

        if (!(line.cmd_is("G0") || line.cmd_is("G1") || line.cmd_is("G2") || line.cmd_is("G3")))
            return;
        const bool first     = first_layer(layer_index);
        const double xy      = line.dist_XY(self);
        const double dz      = std::abs(line.dist_Z(self));
        const bool extrusion = line.extruding(self) && xy > EPSILON;
        const bool travel    = !extrusion && (xy > EPSILON || dz > EPSILON);
        if (extrusion) {
            ++parsed.extrusions;
            if (first && !role.empty())
                parsed.first_layer_feeds[role].push_back(line.new_F(self));
            if (role == "Bridge")
                parsed.bridge_feeds.push_back(line.new_F(self));
            if (layer_index >= 0 && !role.empty()) {
                parsed.role_accelerations[role][layer_index].push_back(print_acceleration);
                parsed.role_jerks[role][layer_index].push_back(jerk);
            }
        } else if (travel) {
            ++parsed.travels;
            if (first && xy > EPSILON && dz <= EPSILON) {
                parsed.first_layer_xy_travel_feeds.push_back(line.new_F(self));
                parsed.first_layer_xy_travel_accelerations.push_back(travel_acceleration);
                parsed.first_layer_xy_travel_jerks.push_back(jerk);
            }
            if (first && xy <= EPSILON && dz > EPSILON && line.new_Z(self) > layer_height + EPSILON)
                parsed.first_layer_z_travel_feeds.push_back(line.new_F(self));
            if (travel_acceleration > 0)
                parsed.travel_accelerations.push_back(travel_acceleration);
            if (jerk > 0)
                parsed.travel_jerks.push_back(jerk);
        }
    });
    return parsed;
}

void require_all_first_role_feeds(const ParsedMotion& motion, const std::string& role, double expected)
{
    const auto found = motion.first_layer_feeds.find(role);
    REQUIRE(found != motion.first_layer_feeds.end());
    REQUIRE_FALSE(found->second.empty());
    for (double feed : found->second)
        CHECK_THAT(feed, Catch::Matchers::WithinAbs(expected, 1e-3));
}

void require_all_role_accelerations(const ParsedMotion& motion, const std::string& role, int expected)
{
    const auto found = motion.role_accelerations.find(role);
    REQUIRE(found != motion.role_accelerations.end());
    size_t samples = 0;
    for (const auto& [layer, values] : found->second) {
        REQUIRE(layer >= 0);
        REQUIRE_FALSE(values.empty());
        for (int value : values) {
            CHECK(value == expected);
            ++samples;
        }
    }
    REQUIRE(samples > 0);
}

void require_first_role_accelerations(const ParsedMotion& motion, const std::string& role, int expected)
{
    const auto found = motion.role_accelerations.find(role);
    REQUIRE(found != motion.role_accelerations.end());
    const auto layer = found->second.find(0);
    REQUIRE(layer != found->second.end());
    REQUIRE_FALSE(layer->second.empty());
    for (int value : layer->second)
        CHECK(value == expected);
}

void require_all_role_jerks(const ParsedMotion& motion, const std::string& role, double expected)
{
    const auto found = motion.role_jerks.find(role);
    REQUIRE(found != motion.role_jerks.end());
    size_t samples = 0;
    for (const auto& [layer, values] : found->second) {
        REQUIRE(layer >= 0);
        REQUIRE_FALSE(values.empty());
        for (double value : values) {
            CHECK_THAT(value, Catch::Matchers::WithinAbs(expected, 1e-3));
            ++samples;
        }
    }
    REQUIRE(samples > 0);
}

} // namespace

TEST_CASE("Initial layer print and solid-infill speeds reach their role feedrates", "[GCode][SettingsSpeedAccelVariance]")
{
    DynamicPrintConfig config = motion_config();
    config.set_deserialize_strict({
        {"initial_layer_speed", 27},
        {"initial_layer_infill_speed", 43},
    });
    const ParsedMotion motion = parse_motion(export_cube(config), config);
    REQUIRE(motion.extrusions > 0);
    require_all_first_role_feeds(motion, "Outer wall", 27 * 60);
    require_all_first_role_feeds(motion, "Bottom surface", 43 * 60);
}

TEST_CASE("First-layer sparse infill keeps its separate infill speed", "[GCode][SettingsSpeedAccelVariance]")
{
    DynamicPrintConfig config = motion_config();
    config.set_deserialize_strict({
        {"bottom_shell_layers", 0},
        {"top_shell_layers", 0},
        {"initial_layer_speed", 27},
        {"initial_layer_infill_speed", 43},
        {"sparse_infill_density", "35%"},
    });
    const ParsedMotion motion = parse_motion(export_cube(config), config);
    require_all_first_role_feeds(motion, "Outer wall", 27 * 60);
    require_all_first_role_feeds(motion, "Sparse infill", 43 * 60);
}

TEST_CASE("First layer percentage travel settings resolve and emit finite travel commands", "[GCode][SettingsSpeedAccelVariance]")
{
    DynamicPrintConfig config = motion_config();
    config.set_deserialize_strict({
        {"travel_speed", 80},
        {"initial_layer_travel_speed", "50%"},
        {"travel_acceleration", 4000},
        {"initial_layer_travel_acceleration", "25%"},
        {"travel_jerk", 12},
        {"initial_layer_travel_jerk", "50%"},
    });
    const ParsedMotion motion = parse_motion(export_cube(config), config);
    REQUIRE_FALSE(motion.first_layer_xy_travel_feeds.empty());
    CHECK_THAT(config.get_abs_value_at("initial_layer_travel_acceleration", 0), Catch::Matchers::WithinAbs(1000., 1e-6));
    CHECK_THAT(config.get_abs_value_at("initial_layer_travel_jerk", 0), Catch::Matchers::WithinAbs(6., 1e-6));
    REQUIRE_FALSE(motion.first_layer_xy_travel_accelerations.empty());
    REQUIRE_FALSE(motion.first_layer_xy_travel_jerks.empty());
    CHECK(std::all_of(motion.first_layer_xy_travel_feeds.begin(), motion.first_layer_xy_travel_feeds.end(),
                      [](double f) { return std::abs(f - 2400) < 1e-3; }));
    CHECK(std::all_of(motion.first_layer_xy_travel_accelerations.begin(), motion.first_layer_xy_travel_accelerations.end(),
                      [](int value) { return value == 1000; }));
    CHECK(std::all_of(motion.first_layer_xy_travel_jerks.begin(), motion.first_layer_xy_travel_jerks.end(),
                      [](double value) { return std::abs(value - 6) < 1e-3; }));
    CHECK(std::all_of(motion.travel_accelerations.begin(), motion.travel_accelerations.end(), [](int value) { return value > 0; }));
    CHECK(std::all_of(motion.travel_jerks.begin(), motion.travel_jerks.end(),
                      [](double value) { return std::isfinite(value) && value > 0; }));
}

TEST_CASE("Travel acceleration and jerk values emit the configured Marlin motion commands", "[GCode][SettingsSpeedAccelVariance]")
{
    DynamicPrintConfig config = motion_config();
    config.set_deserialize_strict({{"travel_acceleration", 1234}, {"travel_jerk", 6}});
    PrintConfig print_config;
    print_config.apply(config, true);

    GCodeCommentGuard comments_guard;
    GCodeWriter::full_gcode_comment = false;
    GCodeWriter writer;
    writer.apply_print_config(print_config);
    const std::string print_acceleration = writer.set_print_acceleration(
        static_cast<unsigned>(std::lround(config.opt_float_nullable("default_acceleration", 0))));
    const std::string print_jerk   = writer.set_jerk_xy(config.opt_float_nullable("default_jerk", 0));
    const std::string acceleration = writer.set_travel_acceleration(
        static_cast<unsigned>(std::lround(config.opt_float_nullable("travel_acceleration", 0))));
    const std::string jerk = writer.set_jerk_xy(config.opt_float_nullable("travel_jerk", 0));

    CHECK(acceleration == "M204 T1234\n");
    CHECK(jerk == "M205 X6 Y6\n");
    CHECK(print_acceleration == "M204 P1200\n");
    CHECK(print_jerk == "M205 X10 Y10\n");
}

TEST_CASE("Initial layer acceleration applies to wall and bottom surface motion", "[GCode][SettingsSpeedAccelVariance]")
{
    DynamicPrintConfig config = motion_config();
    config.set_deserialize_strict({
        {"initial_layer_acceleration", 333},
        {"default_acceleration", 1200},
        {"inner_wall_acceleration", 0},
        {"top_surface_acceleration", 0},
    });
    const ParsedMotion motion = parse_motion(export_cube(config), config);
    require_first_role_accelerations(motion, "Outer wall", 333);
    require_first_role_accelerations(motion, "Bottom surface", 333);
}

TEST_CASE("Inner-wall and top-surface acceleration select their own roles", "[GCode][SettingsSpeedAccelVariance]")
{
    DynamicPrintConfig config = motion_config();
    config.set_deserialize_strict({
        {"initial_layer_acceleration", 0},
        {"initial_layer_jerk", 0},
        {"default_acceleration", 1200},
        {"inner_wall_acceleration", 777},
        {"top_surface_acceleration", 888},
    });
    const ParsedMotion motion = parse_motion(export_cube(config), config);
    require_all_role_accelerations(motion, "Inner wall", 777);
    require_all_role_accelerations(motion, "Top surface", 888);
}

TEST_CASE("Role acceleration and jerk settings select independent parsed commands", "[GCode][SettingsSpeedAccelVariance]")
{
    DynamicPrintConfig config = motion_config();
    config.set_deserialize_strict({
        {"initial_layer_acceleration", 0},
        {"initial_layer_jerk", 0},
        {"default_acceleration", 1200},
        {"outer_wall_acceleration", 611},
        {"inner_wall_acceleration", 777},
        {"top_surface_acceleration", 888},
        {"sparse_infill_acceleration", 555},
        {"internal_solid_infill_acceleration", 666},
        {"default_jerk", 19},
        {"outer_wall_jerk", 23},
        {"inner_wall_jerk", 29},
        {"top_surface_jerk", 31},
        {"infill_jerk", 37},
    });
    const ParsedMotion motion = parse_motion(export_cube(config), config);
    require_all_role_accelerations(motion, "Outer wall", 611);
    require_all_role_accelerations(motion, "Inner wall", 777);
    require_all_role_accelerations(motion, "Top surface", 888);
    require_all_role_accelerations(motion, "Sparse infill", 555);
    require_all_role_accelerations(motion, "Internal solid infill", 666);
    require_all_role_jerks(motion, "Outer wall", 23);
    require_all_role_jerks(motion, "Inner wall", 29);
    require_all_role_jerks(motion, "Top surface", 31);
    require_all_role_jerks(motion, "Sparse infill", 37);
    require_all_role_jerks(motion, "Internal solid infill", 37);
}

TEST_CASE("Travel speed Z is used for a vertical hop and zero falls back to travel speed", "[GCode][SettingsSpeedAccelVariance]")
{
    DynamicPrintConfig explicit_z = motion_config();
    explicit_z.set_deserialize_strict({{"travel_speed", 80}, {"travel_speed_z", 9}, {"z_hop", 0.4}});
    const ParsedMotion explicit_motion = parse_motion(export_cube(explicit_z), explicit_z);
    REQUIRE_FALSE(explicit_motion.first_layer_z_travel_feeds.empty());
    CHECK(std::all_of(explicit_motion.first_layer_z_travel_feeds.begin(), explicit_motion.first_layer_z_travel_feeds.end(),
                      [](double f) { return std::abs(f - 540) < 1e-3; }));

    DynamicPrintConfig fallback = explicit_z;
    fallback.set_deserialize_strict({{"travel_speed_z", "0"}, {"initial_layer_travel_speed", "100%"}});
    const ParsedMotion fallback_motion = parse_motion(export_cube(fallback), fallback);
    REQUIRE_FALSE(fallback_motion.first_layer_z_travel_feeds.empty());
    CHECK(std::all_of(fallback_motion.first_layer_z_travel_feeds.begin(), fallback_motion.first_layer_z_travel_feeds.end(),
                      [](double f) { return std::abs(f - 4800) < 1e-3; }));
}

TEST_CASE("Bridge and internal bridge speed percentages reach bridge role moves", "[GCode][SettingsSpeedAccelVariance]")
{
    DynamicPrintConfig config = motion_config();
    config.set_deserialize_strict({
        {"bridge_speed", 21},
        {"internal_bridge_speed", "150%"},
        {"bridge_acceleration", 432},
        {"initial_layer_acceleration", 0},
        {"sparse_infill_density", "15%"},
        {"top_shell_layers", 3},
        {"bottom_shell_layers", 2},
        {"thick_internal_bridges", false},
    });
    const std::string output  = slice({TestMesh::bridge}, config);
    const ParsedMotion motion = parse_motion(output, config);
    REQUIRE(motion.extrusions > 0);
    REQUIRE_FALSE(motion.bridge_feeds.empty());
    CHECK(std::any_of(motion.bridge_feeds.begin(), motion.bridge_feeds.end(), [](double f) { return std::abs(f - 21 * 60) < 1e-3; }));
    require_all_role_accelerations(motion, "Bridge", 432);

    DynamicPrintConfig changed_external = config;
    changed_external.set_deserialize_strict("bridge_speed", "37");
    const ParsedMotion changed_motion = parse_motion(slice({TestMesh::bridge}, changed_external), changed_external);
    REQUIRE_FALSE(changed_motion.bridge_feeds.empty());
    CHECK(std::any_of(changed_motion.bridge_feeds.begin(), changed_motion.bridge_feeds.end(),
                      [](double f) { return std::abs(f - 37 * 60) < 1e-3; }));

    Model internal_model;
    Print internal_print;
    init_print(std::vector<TriangleMesh>{internal_bridge_fixture()}, internal_print, internal_model, config, nullptr, false);
    REQUIRE(internal_print.validate().string.empty());
    internal_print.process();
    size_t internal_paths = 0;
    for (const Layer* layer : internal_print.objects().front()->layers())
        for (const LayerRegion* region : layer->regions())
            for_each_extrusion_path(region->fills, [&internal_paths](const ExtrusionPath& path) {
                if (path.role() == erInternalBridgeInfill)
                    ++internal_paths;
            });
    REQUIRE(internal_paths > 0);
    const ParsedMotion internal_motion = parse_motion(gcode(internal_print), config);
    REQUIRE_FALSE(internal_motion.bridge_feeds.empty());
    CHECK(std::any_of(internal_motion.bridge_feeds.begin(), internal_motion.bridge_feeds.end(),
                      [](double f) { return std::abs(f - 31.5 * 60) < 1e-3; }));
}

TEST_CASE("Klipper accel-to-decel enable controls the emitted percentage command", "[GCode][SettingsSpeedAccelVariance]")
{
    GCodeCommentGuard comments_guard;
    GCodeWriter::full_gcode_comment = false;
    GCodeWriter enabled;
    enabled.config.gcode_flavor.value          = gcfKlipper;
    enabled.config.accel_to_decel_enable.value = true;
    enabled.config.accel_to_decel_factor.value = 50;
    enabled.set_extruders({0});
    enabled.set_extruder(0);
    GCodeWriter enabled_print;
    enabled_print.config.gcode_flavor.value          = gcfKlipper;
    enabled_print.config.accel_to_decel_enable.value = true;
    enabled_print.config.accel_to_decel_factor.value = 50;
    enabled_print.set_extruders({0});
    enabled_print.set_extruder(0);
    CHECK(enabled_print.set_print_acceleration(1600) == "SET_VELOCITY_LIMIT ACCEL=1600 ACCEL_TO_DECEL=800\n");
    CHECK(enabled.set_accel_and_jerk(1600, 0) == "SET_VELOCITY_LIMIT ACCEL=1600 ACCEL_TO_DECEL=800\n");

    GCodeWriter disabled;
    disabled.config.gcode_flavor.value          = gcfKlipper;
    disabled.config.accel_to_decel_enable.value = false;
    disabled.config.accel_to_decel_factor.value = 50;
    disabled.set_extruders({0});
    disabled.set_extruder(0);
    GCodeWriter disabled_print;
    disabled_print.config.gcode_flavor.value          = gcfKlipper;
    disabled_print.config.accel_to_decel_enable.value = false;
    disabled_print.config.accel_to_decel_factor.value = 50;
    disabled_print.set_extruders({0});
    disabled_print.set_extruder(0);
    CHECK(disabled_print.set_print_acceleration(1600) == "SET_VELOCITY_LIMIT ACCEL=1600\n");
    CHECK(disabled.set_accel_and_jerk(1600, 0) == "SET_VELOCITY_LIMIT ACCEL=1600\n");
}

TEST_CASE("Configured acceleration and jerk boundaries remain finite in emitted motion", "[GCode][SettingsSpeedAccelVariance]")
{
    DynamicPrintConfig config = motion_config();
    config.set_deserialize_strict({
        {"default_acceleration", 1},
        {"initial_layer_acceleration", 1},
        {"travel_acceleration", 1},
        {"default_jerk", 1},
        {"travel_jerk", 1},
        {"inner_wall_acceleration", 1},
        {"top_surface_acceleration", 1},
    });
    const ParsedMotion motion = parse_motion(export_cube(config), config);
    REQUIRE(motion.extrusions > 0);
    for (const auto& [role, layers] : motion.role_accelerations) {
        INFO(role);
        for (const auto& [layer, values] : layers)
            for (int value : values)
                CHECK(value >= 1);
    }
    for (double value : motion.travel_jerks)
        CHECK(std::isfinite(value));
}

} // namespace settings_speed_accel_contracts

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
    const std::vector<std::string> flavors = {"marlin", "marlin2", "klipper", "reprapfirmware", "repetier"};
    const Exported reference               = export_cube(base_config());
    const Motion expected                  = motion(reference.text, base_config());
    REQUIRE(expected.extrusion_moves > 0);
    for (const std::string& flavor : flavors) {
        INFO("flavor " << flavor);
        DynamicPrintConfig config = base_config();
        config.set_deserialize_strict("gcode_flavor", flavor);
        if (flavor == "reprapfirmware" || flavor == "repetier")
            config.set_deserialize_strict("layer_change_gcode", "");
        const Exported out  = export_cube(config);
        const Motion actual = motion(out.text, config);
        REQUIRE(actual.extrusion_moves > 0);
        CHECK_THAT(actual.xy_path, Catch::Matchers::WithinRel(expected.xy_path, 1e-6));
        CHECK_THAT(actual.filament, Catch::Matchers::WithinRel(expected.filament, 1e-6));
        CHECK(out.text.find("G1") != std::string::npos);
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
        {"cooling", true},
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
    config.set_deserialize_strict({{"cooling", true}, {"close_fan_the_first_x_layers", 0}, {"fan_max_speed", 100}, {"fan_min_speed", 100},
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

DynamicPrintConfig route_config(double end_x, double end_y)
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
    config.set_key_value("manual_filament_change", new ConfigOptionBool(false));
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

std::string export_with_object_rows(double object_y, double end_x, double end_y, GCodeProcessorResult *result = nullptr)
{
    DynamicPrintConfig config = route_config(end_x, end_y);
    Model model;

    ModelObject *first = model.add_object();
    first->add_volume(make_cube(20., 20., 2.));
    ModelInstance *first_instance = first->add_instance();
    first_instance->set_offset(Vec3d(0., object_y, 0.));

    ModelObject *second = model.add_object();
    second->add_volume(make_cube(20., 20., 2.));
    ModelInstance *second_instance = second->add_instance();
    second_instance->set_offset(Vec3d(26., object_y, 0.));
    // `extruder` is the canonical 1-based object setting.  PrintApply
    // normalizes it into the region feature filament IDs used by
    // Print::extruders(); keep the assignment at the object level so the
    // contract exercises that production path rather than bypassing it.
    second->config.set_key_value("extruder", new ConfigOptionInt(2));

    Print print;
    print.is_BBL_printer() = true;
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
