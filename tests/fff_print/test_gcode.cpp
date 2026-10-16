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
