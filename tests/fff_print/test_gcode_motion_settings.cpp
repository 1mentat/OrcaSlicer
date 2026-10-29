#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <initializer_list>
#include <map>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <catch2/catch_all.hpp>
#include <catch2/catch_message.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/catch_test_macros.hpp>

#include "libslic3r/Config.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/GCodeReader.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/GCodeWriter.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Preset.hpp"
#include "libslic3r/PresetBundle.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/libslic3r.h"

#include "test_gcode_contract_helpers.hpp"
#include "test_helpers.hpp"
#include "test_utils.hpp"

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
    std::vector<double> internal_bridge_feeds;
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
            else if (comment.find("Internal Bridge") != std::string_view::npos ||
                     comment.find("Internal bridge") != std::string_view::npos)
                role = "Internal bridge";
            else if (comment.find("Bridge") != std::string_view::npos)
                role = "Bridge";
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
            else if (role == "Internal bridge")
                parsed.internal_bridge_feeds.push_back(line.new_F(self));
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

void require_all_role_accelerations(const ParsedMotion& motion, const std::string& role, int expected,
                                    int minimum_layer = 0)
{
    const auto found = motion.role_accelerations.find(role);
    REQUIRE(found != motion.role_accelerations.end());
    size_t samples = 0;
    for (const auto& [layer, values] : found->second) {
        if (layer < minimum_layer)
            continue;
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

DynamicPrintConfig support_motion_config(double base_speed, double interface_speed)
{
    DynamicPrintConfig config = motion_config();
    config.set_deserialize_strict({
        {"enable_support", true},
        {"support_on_build_plate_only", false},
        {"support_interface_top_layers", 3},
        {"support_interface_bottom_layers", 0},
        {"support_speed", base_speed},
        {"support_interface_speed", interface_speed},
        {"initial_layer_speed", 30.0},
        {"initial_layer_infill_speed", 50.0},
        {"slow_down_layers", 1},
        {"small_support_perimeter_threshold", 0.0},
        {"slow_down_for_layer_cooling", false},
        {"slow_down_layer_time", 0.0},
        {"slow_down_min_speed", 1.0},
        {"fan_cooling_layer_time", 0.0},
        {"filament_max_volumetric_speed", "200"},
        {"filament_adaptive_volumetric_speed", false},
        {"machine_max_speed_x", 300.0},
        {"machine_max_speed_y", 300.0},
        {"machine_max_speed_z", 30.0},
        {"machine_max_speed_e", 120.0},
    });
    return config;
}

struct SupportFeedEvidence
{
    struct Sample
    {
        double z;
        double xy;
        double feed;
    };

    std::vector<Sample> base;
    std::vector<Sample> interface;
    double first_layer_z;
};

SupportFeedEvidence parse_support_feeds(const std::string &output, const DynamicPrintConfig &config,
                                        double first_layer_z)
{
    SupportFeedEvidence result{{}, {}, first_layer_z};
    GCodeReader reader;
    reader.apply_config(config);
    std::string role;
    reader.parse_buffer(output, [&result, &role](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        const std::string_view comment = line.comment();
        const size_t feature = comment.find("FEATURE:");
        const size_t type = comment.find("TYPE:");
        if (feature != std::string_view::npos || type != std::string_view::npos) {
            std::string label = feature != std::string_view::npos
                                    ? std::string(comment.substr(feature + 8))
                                    : (type != std::string_view::npos
                                           ? std::string(comment.substr(type + 5))
                                           : std::string{});
            const size_t first = label.find_first_not_of(" \t");
            if (first == std::string::npos)
                label.clear();
            else
                label.erase(0, first);
            const size_t last = label.find_last_not_of(" \t\r\n");
            if (last != std::string::npos)
                label.erase(last + 1);
            std::transform(label.begin(), label.end(), label.begin(), [](unsigned char ch) {
                return ch >= 'A' && ch <= 'Z' ? static_cast<char>(ch - 'A' + 'a') : static_cast<char>(ch);
            });
            if (label == "support interface" || label == "support material interface")
                role = "interface";
            else if (label == "support" || label == "support material")
                role = "base";
            else
                role.clear();
        }
        if (!line.extruding(self))
            return;
        const double xy = line.dist_XY(self);
        if (xy <= EPSILON)
            return;
        const SupportFeedEvidence::Sample sample{line.new_Z(self), xy, line.new_F(self)};
        if (role == "interface")
            result.interface.push_back(sample);
        else if (role == "base")
            result.base.push_back(sample);
    });
    return result;
}

enum class SupportObjectOverride { None, Nil, Explicit };

SupportFeedEvidence export_support_with_object_override(const DynamicPrintConfig &config,
                                                       SupportObjectOverride override_state)
{
    Model model;
    Print print;
    init_print({mesh(TestMesh::overhang)}, print, model, config);
    REQUIRE_FALSE(model.objects.empty());
    ModelObject *model_object = model.objects.front();
    REQUIRE(model_object != nullptr);

    if (override_state == SupportObjectOverride::Nil) {
        model_object->config.set_key_value(
            "support_speed", new ConfigOptionFloatsNullable({ConfigOptionFloatsNullable::nil_value()}));
        model_object->config.set_key_value(
            "support_interface_speed", new ConfigOptionFloatsNullable({ConfigOptionFloatsNullable::nil_value()}));
        const ConfigOption *base_source = model_object->config.option("support_speed");
        const ConfigOption *interface_source = model_object->config.option("support_interface_speed");
        REQUIRE(base_source != nullptr);
        REQUIRE(interface_source != nullptr);
        REQUIRE(base_source->is_nil());
        REQUIRE(interface_source->is_nil());
    } else if (override_state == SupportObjectOverride::Explicit) {
        model_object->config.set_key_value("support_speed", new ConfigOptionFloatsNullable({26.0}));
        model_object->config.set_key_value("support_interface_speed", new ConfigOptionFloatsNullable({38.0}));
        const ConfigOption *base_source = model_object->config.option("support_speed");
        const ConfigOption *interface_source = model_object->config.option("support_interface_speed");
        REQUIRE(base_source != nullptr);
        REQUIRE(interface_source != nullptr);
        REQUIRE_FALSE(base_source->is_nil());
        REQUIRE_FALSE(interface_source->is_nil());
    }

    // Print::apply maps nullable ModelObject overrides over the selected print variant.
    // Nil slots are skipped there; inspect the resolved static object config after the merge.
    (void)print.apply(model, config);
    REQUIRE_FALSE(print.objects().empty());
    const PrintObjectConfig &resolved = print.objects().front()->config();
    REQUIRE_FALSE(resolved.support_speed.is_nil(0));
    REQUIRE_FALSE(resolved.support_interface_speed.is_nil(0));
    const double resolved_base = resolved.support_speed.get_at(0);
    const double resolved_interface = resolved.support_interface_speed.get_at(0);
    const double expected_base = override_state == SupportObjectOverride::Explicit
                                     ? 26.0
                                     : config.opt_float_nullable("support_speed", 0);
    const double expected_interface = override_state == SupportObjectOverride::Explicit
                                          ? 38.0
                                          : config.opt_float_nullable("support_interface_speed", 0);
    CHECK_THAT(resolved_base, Catch::Matchers::WithinAbs(expected_base, 1e-9));
    CHECK_THAT(resolved_interface, Catch::Matchers::WithinAbs(expected_interface, 1e-9));

    const auto validation = print.validate();
    INFO(validation.string);
    REQUIRE(validation.string.empty());
    const std::string output = gcode(print);
    REQUIRE_FALSE(output.empty());
    REQUIRE_FALSE(print.objects().empty());
    const PrintObject *print_object = print.objects().front();
    REQUIRE(print_object->get_layer(0) != nullptr);
    return parse_support_feeds(output, config, print_object->get_layer(0)->print_z);
}

void require_support_role_feeds(const std::vector<SupportFeedEvidence::Sample> &feeds,
                                double expected_role_feed, double first_layer_z,
                                double expected_first_layer_feed)
{
    REQUIRE_FALSE(feeds.empty());
    bool has_non_first_layer_feed = false;
    for (const auto &sample : feeds) {
        CHECK(std::isfinite(sample.feed));
        CHECK(sample.xy > EPSILON);
        if (std::abs(sample.z - first_layer_z) <= EPSILON) {
            CHECK_THAT(sample.feed, Catch::Matchers::WithinAbs(expected_first_layer_feed, 1e-3));
        } else {
            CHECK(sample.z > first_layer_z + EPSILON);
            CHECK_THAT(sample.feed, Catch::Matchers::WithinAbs(expected_role_feed, 1e-3));
            has_non_first_layer_feed = true;
        }
    }
    REQUIRE(has_non_first_layer_feed);
}

void require_same_support_feeds(const std::vector<SupportFeedEvidence::Sample> &expected,
                                const std::vector<SupportFeedEvidence::Sample> &actual)
{
    REQUIRE_FALSE(expected.empty());
    REQUIRE(actual.size() == expected.size());
    for (size_t i = 0; i < expected.size(); ++i)
    {
        CHECK_THAT(actual[i].z, Catch::Matchers::WithinAbs(expected[i].z, EPSILON));
        CHECK_THAT(actual[i].xy, Catch::Matchers::WithinAbs(expected[i].xy, EPSILON));
        CHECK_THAT(actual[i].feed, Catch::Matchers::WithinAbs(expected[i].feed, 1e-3));
    }
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

TEST_CASE("Small perimeter speed resolves absolute and percent feeds and honors threshold disabling",
          "[GCode][SmallPerimeterSpeed]")
{
    struct SpeedRow {
        const char *name;
        const char *small_speed;
        double      threshold;
        double      expected_mm_s;
        enum class ObjectThresholdOverride { None, Nil, Zero } object_threshold_override;
    };
    using ObjectThresholdOverride = SpeedRow::ObjectThresholdOverride;
    const std::array<SpeedRow, 6> rows{{
        {"absolute", "15", 8.0, 15.0, ObjectThresholdOverride::None},
        {"percent", "35%", 8.0, 17.5, ObjectThresholdOverride::None},
        // A 12 mm square has an outer loop near 46 mm: 7 mm maps to 44 mm and 8 mm to 50 mm.
        {"below-loop-boundary", "15", 7.0, 50.0, ObjectThresholdOverride::None},
        {"threshold-disabled", "15", 0.0, 50.0, ObjectThresholdOverride::None},
        {"nil-object-threshold-inherits-parent", "15", 8.0, 15.0, ObjectThresholdOverride::Nil},
        {"zero-object-threshold-disables-small-speed", "15", 8.0, 50.0, ObjectThresholdOverride::Zero},
    }};

    DynamicPrintConfig shared = motion_config();
    shared.set_deserialize_strict({
        {"wall_loops", 1},
        {"top_shell_layers", 0},
        {"bottom_shell_layers", 0},
        {"sparse_infill_density", "0%"},
        {"outer_wall_speed", 50.0},
        {"initial_layer_speed", 50.0},
        {"initial_layer_infill_speed", 50.0},
        {"outer_wall_line_width", 0.5},
        {"initial_layer_line_width", 0.5},
        {"small_perimeter_speed", "15"},
        {"small_perimeter_threshold", 0.0},
        {"slow_down_for_layer_cooling", false},
        {"slow_down_layers", 0},
        {"slowdown_for_curled_perimeters", false},
        {"enable_overhang_speed", false},
        {"enable_overhang_bridge_fan", false},
        {"resonance_avoidance", false},
        {"slow_down_layer_time", 0.0},
        {"slow_down_min_speed", 1.0},
        {"fan_cooling_layer_time", 1000.0},
        {"filament_max_volumetric_speed", "1000"},
        {"filament_adaptive_volumetric_speed", false},
        {"enable_arc_fitting", false},
        {"layer_change_gcode", "G92 E0\n"},
        {"retraction_length", 0.8},
        {"retraction_minimum_travel", 0.0},
        {"z_hop", 0.0},
    });

    struct MaterialEvidence {
        size_t extrusion_moves = 0;
        double xy_path = 0.0;
        double filament = 0.0;
        bool   finite = true;
    };
    const auto read_material = [](const std::string &text, const DynamicPrintConfig &config) {
        MaterialEvidence result;
        GCodeReader reader;
        reader.apply_config(config);
        reader.parse_buffer(text, [&result](GCodeReader &self, const GCodeReader::GCodeLine &line) {
            if (!(line.cmd_is("G0") || line.cmd_is("G1") || line.cmd_is("G2") || line.cmd_is("G3")) ||
                !line.extruding(self))
                return;
            const double xy = line.dist_XY(self);
            const double dE = line.dist_E(self);
            if (!std::isfinite(xy) || !std::isfinite(dE)) {
                result.finite = false;
                return;
            }
            if (xy > EPSILON) {
                ++result.extrusion_moves;
                result.xy_path += xy;
                result.filament += std::abs(dE);
            }
        });
        return result;
    };

    const auto export_six_layer_cube = [](const DynamicPrintConfig &config,
                                          ObjectThresholdOverride object_threshold_override) {
        Model model;
        Print print;
        init_print({make_cube(12.0, 12.0, 1.2)}, print, model, config);
        if (object_threshold_override != ObjectThresholdOverride::None) {
            REQUIRE_FALSE(model.objects.empty());
            ModelObject *object = model.objects.front();
            REQUIRE(object != nullptr);
            if (object_threshold_override == ObjectThresholdOverride::Nil) {
                object->config.set_key_value("small_perimeter_threshold",
                    new ConfigOptionFloatsNullable({ConfigOptionFloatsNullable::nil_value()}));
                const ConfigOption *option = object->config.option("small_perimeter_threshold");
                REQUIRE(option != nullptr);
                REQUIRE(option->is_nil());
            } else {
                object->config.set_key_value(
                    "small_perimeter_threshold", new ConfigOptionFloatsNullable({0.0}));
                const ConfigOption *option = object->config.option("small_perimeter_threshold");
                REQUIRE(option != nullptr);
                REQUIRE_FALSE(option->is_nil());
            }
            (void)print.apply(model, config);
        }
        const auto validation = print.validate();
        INFO(validation.string);
        REQUIRE(validation.string.empty());
        const std::string output = gcode(print);
        REQUIRE_FALSE(output.empty());
        size_t layer_changes = 0;
        for (size_t pos = output.find(";LAYER_CHANGE"); pos != std::string::npos;
             pos = output.find(";LAYER_CHANGE", pos + 1))
            ++layer_changes;
        REQUIRE(layer_changes == 6);
        return output;
    };

    std::optional<MaterialEvidence> reference_material;
    std::optional<std::vector<double>> positive_parent_threshold_feeds;
    for (size_t row_index = 0; row_index < rows.size(); ++row_index) {
        const SpeedRow &row = rows[row_index];
        CAPTURE(row.name);
        DynamicPrintConfig config = shared;
        config.set_deserialize_strict({
            {"small_perimeter_speed", std::string(row.small_speed)},
            {"small_perimeter_threshold", row.threshold},
        });
        const std::string output = export_six_layer_cube(config, row.object_threshold_override);
        REQUIRE_FALSE(output.empty());

        const MaterialEvidence material = read_material(output, config);
        REQUIRE(material.finite);
        REQUIRE(material.extrusion_moves > 0);
        REQUIRE(material.xy_path > 0.0);
        REQUIRE(material.filament > 0.0);
        if (!reference_material.has_value()) {
            reference_material = material;
        } else {
            CHECK(material.extrusion_moves == reference_material->extrusion_moves);
            CHECK_THAT(material.xy_path, Catch::Matchers::WithinRel(reference_material->xy_path, 1e-6));
            CHECK_THAT(material.filament, Catch::Matchers::WithinRel(reference_material->filament, 1e-6));
        }

        const auto feeds = gcode_print_variance_contracts::calibration_gcode_followup::parse_outer_wall_feeds(output, config);
        REQUIRE(feeds.finite);
        REQUIRE(feeds.feeds.size() > 1);
        size_t samples = 0;
        bool all_feeds_match_expected = true;
        std::vector<double> observed_feeds;
        for (size_t layer = 1; layer < feeds.feeds.size(); ++layer) {
            INFO("row=" << row.name << " layer=" << layer);
            REQUIRE_FALSE(feeds.feeds[layer].empty());
            for (const double feed : feeds.feeds[layer]) {
                all_feeds_match_expected = all_feeds_match_expected &&
                    std::abs(feed - row.expected_mm_s * 60.0) <= 0.25;
                observed_feeds.push_back(feed);
                ++samples;
            }
        }
        REQUIRE(samples > 0);
        if (row_index == 0) {
            positive_parent_threshold_feeds = observed_feeds;
        } else if (row.object_threshold_override == ObjectThresholdOverride::Nil) {
            bool same_as_positive_parent = positive_parent_threshold_feeds.has_value() &&
                observed_feeds.size() == positive_parent_threshold_feeds->size();
            if (same_as_positive_parent) {
                for (size_t i = 0; i < observed_feeds.size(); ++i)
                    same_as_positive_parent = same_as_positive_parent &&
                        std::abs(observed_feeds[i] - (*positive_parent_threshold_feeds)[i]) <= 0.25;
            }
            all_feeds_match_expected = all_feeds_match_expected && same_as_positive_parent;
        }
        CHECK(all_feeds_match_expected);
    }
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
    REQUIRE_FALSE(internal_motion.internal_bridge_feeds.empty());
    CHECK(std::all_of(internal_motion.internal_bridge_feeds.begin(), internal_motion.internal_bridge_feeds.end(),
                      [](double f) { return std::abs(f - 31.5 * 60) < 1e-3; }));
}

struct KlipperAccelerationCommandEvidence
{
    std::optional<double> acceleration;
    std::optional<double> accel_to_decel;
};

std::vector<KlipperAccelerationCommandEvidence> parse_klipper_acceleration_commands(const std::string &output)
{
    std::vector<KlipperAccelerationCommandEvidence> commands;
    std::istringstream lines(output);
    std::string raw_line;
    while (std::getline(lines, raw_line)) {
        const size_t comment = raw_line.find(';');
        std::istringstream fields(raw_line.substr(0, comment));
        std::string command_name;
        fields >> command_name;
        if (command_name != "SET_VELOCITY_LIMIT")
            continue;

        KlipperAccelerationCommandEvidence command;
        std::string token;
        while (fields >> token) {
            const auto value_after = [&token](const char *key) -> std::optional<double> {
                const size_t key_size = std::string_view(key).size();
                if (token.compare(0, key_size, key) != 0)
                    return std::nullopt;
                const std::string value = token.substr(key_size);
                if (value.empty())
                    return std::nullopt;
                size_t parsed = 0;
                const double number = std::stod(value, &parsed);
                if (parsed != value.size() || !std::isfinite(number))
                    return std::nullopt;
                return number;
            };
            if (const auto acceleration = value_after("ACCEL="))
                command.acceleration = *acceleration;
            else if (const auto accel_to_decel = value_after("ACCEL_TO_DECEL="))
                command.accel_to_decel = *accel_to_decel;
        }
        if (command.acceleration.has_value())
            commands.emplace_back(command);
    }
    return commands;
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

    for (const auto &[factor, expected_accel_to_decel] : std::array<std::pair<int, int>, 2>{{{1, 16}, {100, 1600}}}) {
        CAPTURE(factor);
        GCodeWriter endpoint;
        endpoint.config.gcode_flavor.value = gcfKlipper;
        endpoint.config.accel_to_decel_enable.value = true;
        endpoint.config.accel_to_decel_factor.value = factor;
        endpoint.set_extruders({0});
        endpoint.set_extruder(0);
        CHECK(endpoint.set_accel_and_jerk(1600, 0) ==
              "SET_VELOCITY_LIMIT ACCEL=1600 ACCEL_TO_DECEL=" + std::to_string(expected_accel_to_decel) + "\n");
    }
}

TEST_CASE("Klipper accel-to-decel settings shape emitted acceleration commands without changing deposited motion",
          "[GCode][SettingsSpeedAccelVariance]")
{
    struct Row { bool enabled; int factor; };
    const std::array<Row, 6> rows{{
        {false, 1}, {false, 37}, {false, 100},
        {true, 1},  {true, 37},  {true, 100},
    }};
    std::optional<gcode_print_variance_contracts::MotionEvidence> reference_material;

    for (const Row &row : rows) {
        CAPTURE(row.enabled, row.factor);
        DynamicPrintConfig config = motion_config(gcfKlipper);
        config.set_deserialize_strict({
            {"default_acceleration", 1200},
            {"initial_layer_acceleration", 300},
            {"outer_wall_acceleration", 611},
            {"inner_wall_acceleration", 777},
            {"top_surface_acceleration", 888},
            {"travel_acceleration", 4000},
            {"machine_max_acceleration_extruding", "20000,20000"},
            {"machine_max_acceleration_x", "20000,20000"},
            {"machine_max_acceleration_y", "20000,20000"},
            {"accel_to_decel_enable", row.enabled},
            {"accel_to_decel_factor", row.factor},
        });

        // A 0.8 mm model at 0.2 mm layer height exercises four layers: first-layer
        // setup, ordinary-layer reset, and later role-specific acceleration commands.
        const gcode_print_variance_contracts::ExportedPrint exported = gcode_print_variance_contracts::fresh_custom_mesh_export(config, make_cube(8., 8., 0.8));
        REQUIRE(gcode_print_variance_contracts::count_text(exported.text, ";LAYER_CHANGE") == 4);
        const gcode_print_variance_contracts::MotionEvidence material = gcode_print_variance_contracts::motion_evidence(exported.text, config);
        REQUIRE(material.extrusion_moves > 0);
        CHECK(std::isfinite(material.xy_path));
        CHECK(material.xy_path > 0.0);
        CHECK(std::isfinite(material.filament));
        CHECK(material.filament > 0.0);
        if (!reference_material.has_value())
            reference_material = material;
        else
            gcode_print_variance_contracts::require_same_material(*reference_material, material);

        const auto commands = parse_klipper_acceleration_commands(exported.text);
        REQUIRE_FALSE(commands.empty());
        std::set<int> emitted_accelerations;
        for (const KlipperAccelerationCommandEvidence &command : commands) {
            REQUIRE(command.acceleration.has_value());
            CHECK(std::isfinite(*command.acceleration));
            CHECK(*command.acceleration > 0.0);
            emitted_accelerations.insert(static_cast<int>(std::lround(*command.acceleration)));
            if (row.enabled) {
                CHECK(command.accel_to_decel.has_value());
                if (command.accel_to_decel.has_value())
                    CHECK_THAT(*command.accel_to_decel,
                               Catch::Matchers::WithinAbs(*command.acceleration * row.factor / 100.0, 0.02));
            } else {
                CHECK_FALSE(command.accel_to_decel.has_value());
            }
        }

        // Initial and ordinary layer setup use set_print_acceleration; later
        // role overrides travel through the Klipper set_accel_and_jerk path.
        CHECK(emitted_accelerations.count(300) == 1);
        CHECK(emitted_accelerations.count(1200) == 1);
        CHECK(emitted_accelerations.count(611) == 1);
        CHECK(emitted_accelerations.count(777) == 1);
        CHECK(emitted_accelerations.count(888) == 1);
    }
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

TEST_CASE("Registered minimum speeds reach first-layer and bridge extrusion roles", "[GCode][SettingsSpeedAccelVariance]")
{
    struct FirstLayerMinimumRow {
        const char *target;
        double wall_speed;
        double infill_speed;
        const char *travel_speed;
        double expected_wall_feed;
        double expected_infill_feed;
        double expected_travel_feed;
    };
    const std::array<FirstLayerMinimumRow, 3> rows{{
        {"initial_layer_speed", 1, 43, "100%", 60, 2580, 4800},
        {"initial_layer_infill_speed", 27, 1, "100%", 1620, 60, 4800},
        {"initial_layer_travel_speed", 27, 43, "1", 1620, 2580, 60},
    }};
    for (const FirstLayerMinimumRow &row : rows) {
        CAPTURE(row.target);
        DynamicPrintConfig first_layer = motion_config();
        first_layer.set_deserialize_strict({
            {"initial_layer_speed", row.wall_speed},
            {"initial_layer_infill_speed", row.infill_speed},
            {"initial_layer_travel_speed", row.travel_speed},
            {"bottom_shell_layers", 0},
            {"top_shell_layers", 0},
            {"sparse_infill_density", "35%"},
        });
        const ParsedMotion motion = parse_motion(export_cube(first_layer), first_layer);
        REQUIRE(motion.extrusions > 0);
        require_all_first_role_feeds(motion, "Outer wall", row.expected_wall_feed);
        require_all_first_role_feeds(motion, "Sparse infill", row.expected_infill_feed);
        REQUIRE_FALSE(motion.first_layer_xy_travel_feeds.empty());
        for (const double feed : motion.first_layer_xy_travel_feeds)
            CHECK_THAT(feed, Catch::Matchers::WithinAbs(row.expected_travel_feed, 1e-3));
    }

    DynamicPrintConfig bridge = motion_config();
    bridge.set_deserialize_strict({
        {"bridge_speed", 1},
        {"internal_bridge_speed", "1"},
        {"bridge_acceleration", 432},
        {"initial_layer_acceleration", 0},
        {"sparse_infill_density", "15%"},
        {"top_shell_layers", 3},
        {"bottom_shell_layers", 2},
        {"thick_internal_bridges", false},
    });

    const ParsedMotion external_bridge = parse_motion(slice({TestMesh::bridge}, bridge), bridge);
    REQUIRE_FALSE(external_bridge.bridge_feeds.empty());
    CHECK(std::all_of(external_bridge.bridge_feeds.begin(), external_bridge.bridge_feeds.end(),
                      [](double feed) { return std::abs(feed - 60) < 1e-3; }));

    DynamicPrintConfig internal_bridge_config = bridge;
    internal_bridge_config.set_deserialize_strict({
        {"bridge_speed", 21},
        {"internal_bridge_speed", "1"},
    });
    Model internal_model;
    Print internal_print;
    init_print(std::vector<TriangleMesh>{internal_bridge_fixture()}, internal_print, internal_model,
               internal_bridge_config, nullptr, false);
    REQUIRE(internal_print.validate().string.empty());
    internal_print.process();
    size_t internal_paths = 0;
    for (const Layer *layer : internal_print.objects().front()->layers())
        for (const LayerRegion *region : layer->regions())
            for_each_extrusion_path(region->fills, [&internal_paths](const ExtrusionPath &path) {
                if (path.role() == erInternalBridgeInfill)
                    ++internal_paths;
            });
    REQUIRE(internal_paths > 0);
    const ParsedMotion internal_bridge = parse_motion(gcode(internal_print), internal_bridge_config);
    REQUIRE_FALSE(internal_bridge.internal_bridge_feeds.empty());
    CHECK(std::all_of(internal_bridge.internal_bridge_feeds.begin(), internal_bridge.internal_bridge_feeds.end(),
                      [](double feed) { return std::abs(feed - 60) < 1e-3; }));
}

TEST_CASE("Zero acceleration and jerk bounds preserve role fallback on emitted motion", "[GCode][SettingsSpeedAccelVariance]")
{
    DynamicPrintConfig config = motion_config();
    config.set_deserialize_strict({
        {"default_acceleration", 1200},
        {"emit_machine_limits_to_gcode", false},
        // Establish an emitted jerk state so the zero travel override is observable as a no-op.
        {"machine_start_gcode", "M205 X10 Y10\n"},
        {"retraction_minimum_travel", 0},
        {"outer_wall_acceleration", 611},
        {"initial_layer_acceleration", 0},
        {"inner_wall_acceleration", 0},
        {"top_surface_acceleration", 0},
        {"travel_jerk", 12},
        {"initial_layer_travel_jerk", "0"},
        {"default_jerk", 10},
        {"initial_layer_jerk", 0},
        {"outer_wall_jerk", 0},
        {"inner_wall_jerk", 0},
        {"top_surface_jerk", 0},
        {"infill_jerk", 0},
    });

    DynamicPrintConfig no_travel_acceleration = config;
    no_travel_acceleration.set_deserialize_strict({
        {"travel_acceleration", 0},
        {"initial_layer_travel_acceleration", "0"},
    });
    const std::string zero_travel_output = export_cube(no_travel_acceleration);
    const ParsedMotion zero_travel = parse_motion(zero_travel_output, no_travel_acceleration);
    CHECK(zero_travel_output.find("M205 X10 Y10") != std::string::npos);
    REQUIRE_FALSE(zero_travel.first_layer_xy_travel_feeds.empty());
    REQUIRE_FALSE(zero_travel.first_layer_xy_travel_accelerations.empty());
    CHECK(std::all_of(zero_travel.first_layer_xy_travel_accelerations.begin(),
                      zero_travel.first_layer_xy_travel_accelerations.end(), [](int value) { return value == 0; }));
    CHECK(zero_travel_output.find("M204 T") == std::string::npos);

    DynamicPrintConfig base_travel_acceleration = config;
    base_travel_acceleration.set_deserialize_strict({
        {"travel_acceleration", 4000},
        {"initial_layer_travel_acceleration", "0"},
    });
    const ParsedMotion base_travel = parse_motion(export_cube(base_travel_acceleration), base_travel_acceleration);
    REQUIRE_FALSE(base_travel.first_layer_xy_travel_accelerations.empty());
    CHECK(std::all_of(base_travel.first_layer_xy_travel_accelerations.begin(),
                      base_travel.first_layer_xy_travel_accelerations.end(), [](int value) { return value == 0; }));
    REQUIRE_FALSE(base_travel.travel_accelerations.empty());
    CHECK(std::all_of(base_travel.travel_accelerations.begin(), base_travel.travel_accelerations.end(),
                      [](int value) { return value == 4000; }));
    REQUIRE_FALSE(base_travel.first_layer_xy_travel_jerks.empty());
    CHECK(std::all_of(base_travel.first_layer_xy_travel_jerks.begin(), base_travel.first_layer_xy_travel_jerks.end(),
                      [](double value) { return std::abs(value - 10) < 1e-3; }));
    CHECK(std::any_of(base_travel.travel_jerks.begin(), base_travel.travel_jerks.end(),
                      [](double value) { return std::abs(value - 12) < 1e-3; }));

    require_first_role_accelerations(base_travel, "Outer wall", 611);
    require_first_role_accelerations(base_travel, "Bottom surface", 1200);
    require_all_role_accelerations(base_travel, "Inner wall", 1200);
    require_all_role_accelerations(base_travel, "Top surface", 1200);
}

TEST_CASE("Nullable object acceleration overrides inherit parent values on exported motion",
          "[GCode][SettingsSpeedAccelVariance]")
{
    DynamicPrintConfig config = motion_config();
    config.set_deserialize_strict({
        {"default_acceleration", 1200},
        {"travel_acceleration", 4000},
        {"outer_wall_acceleration", 611},
        {"initial_layer_acceleration", 333},
        {"inner_wall_acceleration", 777},
        {"top_surface_acceleration", 888},
        {"initial_layer_travel_acceleration", "0"},
        {"retraction_minimum_travel", 0.0},
        {"emit_machine_limits_to_gcode", false},
    });
    const std::array<std::pair<const char *, double>, 4> overrides{{
        {"initial_layer_acceleration", 444},
        {"inner_wall_acceleration", 701},
        {"top_surface_acceleration", 801},
        {"travel_acceleration", 2500},
    }};
    std::optional<gcode_print_variance_contracts::MotionEvidence> reference_material;
    const auto export_object_state = [&config, &overrides, &reference_material](int state) {
        Model model;
        Print print;
        TriangleMesh shape = cube(12.0);
        shape.scale(Vec3f(1.f, 1.f, 0.55f));
        init_print({std::move(shape)}, print, model, config);
        REQUIRE_FALSE(model.objects.empty());
        ModelObject *object = model.objects.front();
        REQUIRE(object != nullptr);

        if (state != 0) {
            for (const auto &[key, explicit_value] : overrides) {
                if (state == 1) {
                    object->config.set_key_value(key,
                        new ConfigOptionFloatsNullable({ConfigOptionFloatsNullable::nil_value()}));
                    const ConfigOption *option = object->config.option(key);
                    REQUIRE(option != nullptr);
                    REQUIRE(option->is_nil());
                } else {
                    object->config.set_key_value(key, new ConfigOptionFloatsNullable({explicit_value}));
                    const ConfigOption *option = object->config.option(key);
                    REQUIRE(option != nullptr);
                    REQUIRE_FALSE(option->is_nil());
                }
            }
        }

        (void)print.apply(model, config);
        const auto validation = print.validate();
        INFO(validation.string);
        REQUIRE(validation.string.empty());
        const std::string output = gcode(print);
        REQUIRE_FALSE(output.empty());
        const auto material = gcode_print_variance_contracts::motion_evidence(output, config);
        if (!reference_material.has_value())
            reference_material = material;
        else
            gcode_print_variance_contracts::require_same_material(*reference_material, material);
        return parse_motion(output, config);
    };

    const ParsedMotion inherited = export_object_state(0);
    const ParsedMotion nil_overrides = export_object_state(1);
    const ParsedMotion explicit_overrides = export_object_state(2);

    REQUIRE(inherited.extrusions > 0);
    REQUIRE(nil_overrides.extrusions == inherited.extrusions);
    REQUIRE(nil_overrides.travels == inherited.travels);
    REQUIRE(nil_overrides.first_layer_feeds.size() == inherited.first_layer_feeds.size());
    for (const auto &[role, inherited_feeds] : inherited.first_layer_feeds) {
        CAPTURE(role);
        const auto found = nil_overrides.first_layer_feeds.find(role);
        REQUIRE(found != nil_overrides.first_layer_feeds.end());
        REQUIRE(found->second.size() == inherited_feeds.size());
        for (size_t i = 0; i < inherited_feeds.size(); ++i)
            CHECK_THAT(found->second[i], Catch::Matchers::WithinAbs(inherited_feeds[i], 1e-3));
    }
    require_first_role_accelerations(inherited, "Outer wall", 333);
    require_first_role_accelerations(inherited, "Inner wall", 333);
    require_all_role_accelerations(inherited, "Inner wall", 777, 1);
    require_all_role_accelerations(inherited, "Top surface", 888);
    CHECK(nil_overrides.role_accelerations == inherited.role_accelerations);
    require_first_role_accelerations(nil_overrides, "Outer wall", 333);
    require_first_role_accelerations(nil_overrides, "Bottom surface", 333);
    require_first_role_accelerations(nil_overrides, "Inner wall", 333);
    require_all_role_accelerations(nil_overrides, "Inner wall", 777, 1);
    require_all_role_accelerations(nil_overrides, "Top surface", 888);
    require_first_role_accelerations(explicit_overrides, "Outer wall", 444);
    require_first_role_accelerations(explicit_overrides, "Inner wall", 444);
    require_all_role_accelerations(explicit_overrides, "Inner wall", 701, 1);
    require_all_role_accelerations(explicit_overrides, "Top surface", 801);

    REQUIRE_FALSE(inherited.travel_accelerations.empty());
    CHECK(std::all_of(inherited.travel_accelerations.begin(), inherited.travel_accelerations.end(),
                      [](int value) { return value == 4000; }));
    CHECK(nil_overrides.travel_accelerations == inherited.travel_accelerations);
    REQUIRE_FALSE(explicit_overrides.travel_accelerations.empty());
    CHECK(std::all_of(explicit_overrides.travel_accelerations.begin(), explicit_overrides.travel_accelerations.end(),
                      [](int value) { return value == 2500; }));
}


TEST_CASE("Support speed rejects zero and emits the valid minimum on both support roles",
          "[GCode][SettingsSpeedAccelVariance][SupportSpeedMinimum]")
{
    // Runtime zero selects a volumetric fallback in GCode.cpp, but these registered
    // options have min=1. Keep zero as a validation boundary and slice only valid input.
    DynamicPrintConfig zero = support_motion_config(0.0, 0.0);
    const auto zero_errors = zero.validate();
    CHECK(zero_errors.find("support_speed") != zero_errors.end());
    CHECK(zero_errors.find("support_interface_speed") != zero_errors.end());

    DynamicPrintConfig minimum = support_motion_config(1.0, 1.0);
    REQUIRE(minimum.validate().empty());
    const SupportFeedEvidence evidence =
        export_support_with_object_override(minimum, SupportObjectOverride::None);
    require_support_role_feeds(evidence.base, 60.0, evidence.first_layer_z, 3000.0);
    require_support_role_feeds(evidence.interface, 60.0, evidence.first_layer_z, 3000.0);
}

TEST_CASE("Nil object support speed overrides inherit parent rates on exported support",
          "[GCode][SettingsSpeedAccelVariance][SupportSpeedNil]")
{
    DynamicPrintConfig parent = support_motion_config(18.0, 31.0);
    REQUIRE(parent.validate().empty());

    const SupportFeedEvidence parent_output =
        export_support_with_object_override(parent, SupportObjectOverride::None);
    const SupportFeedEvidence nil_output =
        export_support_with_object_override(parent, SupportObjectOverride::Nil);
    const SupportFeedEvidence explicit_output =
        export_support_with_object_override(parent, SupportObjectOverride::Explicit);

    // The parent output is the independent fallback witness for both nil object slots.
    require_support_role_feeds(parent_output.base, 18.0 * 60.0, parent_output.first_layer_z, 3000.0);
    require_support_role_feeds(parent_output.interface, 31.0 * 60.0, parent_output.first_layer_z, 3000.0);
    require_support_role_feeds(nil_output.base, 18.0 * 60.0, nil_output.first_layer_z, 3000.0);
    require_support_role_feeds(nil_output.interface, 31.0 * 60.0, nil_output.first_layer_z, 3000.0);
    require_same_support_feeds(parent_output.base, nil_output.base);
    require_same_support_feeds(parent_output.interface, nil_output.interface);

    // Explicit object values are a separate witness that each selected role reads its own key.
    require_support_role_feeds(explicit_output.base, 26.0 * 60.0, explicit_output.first_layer_z, 3000.0);
    require_support_role_feeds(explicit_output.interface, 38.0 * 60.0, explicit_output.first_layer_z, 3000.0);
}

struct NullableToolMotion
{
    std::vector<double> first_layer_xy_feeds;
    std::vector<int>    first_layer_xy_accelerations;
    std::vector<double> first_layer_xy_jerks;
    std::vector<double> first_layer_z_feeds;
    size_t              extrusion_moves = 0;
    double              xy_path = 0.0;
    double              filament = 0.0;
};

struct NullableToolMotionEvidence
{
    std::array<NullableToolMotion, 2> tools;
    std::array<size_t, 2>             tool_selects{};
};

NullableToolMotionEvidence parse_nullable_tool_motion(const std::string &output,
                                                       const DynamicPrintConfig &config)
{
    NullableToolMotionEvidence evidence;
    GCodeReader reader;
    reader.apply_config(config);
    int current_tool = 0;
    int layer_index = -1;
    int travel_acceleration = 0;
    double travel_jerk = 0.0;
    const double layer_height = config.opt_float("layer_height");
    reader.parse_buffer(output, [&](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        if (line.comment().find("LAYER_CHANGE") != std::string_view::npos)
            ++layer_index;
        if (line.cmd_is("T0")) {
            current_tool = 0;
            ++evidence.tool_selects[0];
        } else if (line.cmd_is("T1")) {
            current_tool = 1;
            ++evidence.tool_selects[1];
        }

        float value = 0.f;
        if (line.cmd_is("M204") && line.has_value('T', value))
            travel_acceleration = int(std::lround(value));
        else if (line.cmd_is("M205") && line.has_value('X', value))
            travel_jerk = value;

        if (!(line.cmd_is("G0") || line.cmd_is("G1") || line.cmd_is("G2") || line.cmd_is("G3")))
            return;
        REQUIRE(current_tool >= 0);
        REQUIRE(current_tool < int(evidence.tools.size()));
        NullableToolMotion &tool = evidence.tools[size_t(current_tool)];
        const double xy = line.dist_XY(self);
        const double dz = std::abs(line.dist_Z(self));
        if (line.extruding(self) && xy > EPSILON) {
            ++tool.extrusion_moves;
            tool.xy_path += xy;
            tool.filament += std::abs(line.dist_E(self));
        } else if (layer_index == 0 && xy > EPSILON && dz <= EPSILON) {
            tool.first_layer_xy_feeds.push_back(line.new_F(self));
            tool.first_layer_xy_accelerations.push_back(travel_acceleration);
            tool.first_layer_xy_jerks.push_back(travel_jerk);
        } else if (layer_index == 0 && xy <= EPSILON && dz > EPSILON &&
                   line.new_Z(self) > layer_height + EPSILON) {
            tool.first_layer_z_feeds.push_back(line.new_F(self));
        }
    });
    return evidence;
}

void require_nullable_tool_motion(const NullableToolMotion &motion, double xy_feed,
                                 int xy_acceleration, double xy_jerk, double z_feed)
{
    REQUIRE(motion.extrusion_moves > 0);
    REQUIRE(std::isfinite(motion.xy_path));
    REQUIRE(motion.xy_path > 0.0);
    REQUIRE(std::isfinite(motion.filament));
    REQUIRE(motion.filament > 0.0);

    REQUIRE_FALSE(motion.first_layer_xy_feeds.empty());
    for (double value : motion.first_layer_xy_feeds) {
        CHECK(std::isfinite(value));
        CHECK_THAT(value, Catch::Matchers::WithinAbs(xy_feed, 1e-3));
    }
    REQUIRE_FALSE(motion.first_layer_xy_accelerations.empty());
    for (int value : motion.first_layer_xy_accelerations)
        CHECK(value == xy_acceleration);
    REQUIRE_FALSE(motion.first_layer_xy_jerks.empty());
    for (double value : motion.first_layer_xy_jerks) {
        CHECK(std::isfinite(value));
        CHECK_THAT(value, Catch::Matchers::WithinAbs(xy_jerk, 1e-3));
    }
    REQUIRE_FALSE(motion.first_layer_z_feeds.empty());
    for (double value : motion.first_layer_z_feeds) {
        CHECK(std::isfinite(value));
        CHECK_THAT(value, Catch::Matchers::WithinAbs(z_feed, 1e-3));
    }
}

void set_nullable_parent_motion(DynamicPrintConfig &config)
{
    config.option<ConfigOptionInts>("print_extruder_id", true)->values = {2, 1};
    config.option<ConfigOptionStrings>("print_extruder_variant", true)->values = {
        "Direct Drive High Flow", "Direct Drive Standard"};
    config.option<ConfigOptionFloatsNullable>("travel_speed", true)->values = {100., 80.};
    config.option<ConfigOptionFloatsNullable>("travel_acceleration", true)->values = {6000., 4000.};
    config.option<ConfigOptionFloatsNullable>("travel_jerk", true)->values = {20., 12.};
    config.option<ConfigOptionFloatsNullable>("travel_speed_z", true)->values = {8., 9.};
    config.option<ConfigOptionFloatsNullable>("default_acceleration", true)->values = {1200., 1200.};
    config.option<ConfigOptionFloatsNullable>("default_jerk", true)->values = {10., 10.};
    config.option<ConfigOptionFloatsOrPercentsNullable>("initial_layer_travel_speed", true)->values = {
        FloatOrPercent(75., true), FloatOrPercent(50., true)};
    config.option<ConfigOptionFloatsOrPercentsNullable>("initial_layer_travel_acceleration", true)->values = {
        FloatOrPercent(20., true), FloatOrPercent(25., true)};
    config.option<ConfigOptionFloatsOrPercentsNullable>("initial_layer_travel_jerk", true)->values = {
        FloatOrPercent(60., true), FloatOrPercent(50., true)};
    config.option<ConfigOptionString>(BBL_JSON_KEY_INHERITS, true)->value.clear();
}

void write_nullable_child_preset(const boost::filesystem::path &file)
{
    boost::filesystem::create_directories(file.parent_path());
    std::ofstream out(file.string());
    out << R"({"type":"process","name":"Nullable Child","from":"User","version":"1.0.0","inherits":"Nullable Parent",)"
        << R"("print_extruder_id":["2","1"],"print_extruder_variant":["Direct Drive High Flow","Direct Drive Standard"],)"
        << R"("initial_layer_travel_speed":["nil","25%"],)"
        << R"("initial_layer_travel_acceleration":["nil","30%"],)"
        << R"("initial_layer_travel_jerk":["nil","25%"],)"
        << R"("travel_speed_z":["nil","6"]})";
    REQUIRE(out.good());
}

DynamicPrintConfig nullable_full_config(Preset &printer, Preset &process,
                                        const DynamicPrintConfig &project,
                                        const std::vector<Preset> &filament_presets)
{
    std::vector<Preset> filaments = filament_presets;
    return PresetBundle::construct_full_config(
        printer, process, project, filaments, true,
        std::vector<int>{1, 2}, std::vector<int>{int(nvtStandard), int(nvtHighFlow)});
}

std::string export_nullable_two_tool_motion(const DynamicPrintConfig &input_config)
{
    DynamicPrintConfig config = input_config;
    config.set_key_value("gcode_flavor", new ConfigOptionEnum<GCodeFlavor>(gcfMarlinFirmware));
    config.set_key_value("single_extruder_multi_material", new ConfigOptionBool(false));
    config.set_key_value("enable_prime_tower", new ConfigOptionBool(false));
    config.set_key_value("z_hop", new ConfigOptionFloats({0.4, 0.4}));
    config.set_key_value("z_hop_types",
                         new ConfigOptionEnumsGeneric{ZHopType::zhtNormal, ZHopType::zhtNormal});
    config.set_key_value("retraction_length", new ConfigOptionFloats({0.8, 0.8}));
    config.set_key_value("retraction_minimum_travel", new ConfigOptionFloats({0., 0.}));
    config.set_key_value("print_sequence", new ConfigOptionEnum<PrintSequence>(PrintSequence::ByLayer));
    config.set_key_value("skirt_loops", new ConfigOptionInt(0));
    config.set_key_value("brim_type", new ConfigOptionEnum<BrimType>(btNoBrim));
    config.set_key_value("raft_layers", new ConfigOptionInt(0));
    config.set_key_value("machine_start_gcode", new ConfigOptionString("T[initial_tool]\n"));
    config.set_key_value("machine_end_gcode", new ConfigOptionString(""));
    config.set_key_value("layer_change_gcode", new ConfigOptionString("G92 E0\n"));
    config.set_key_value("gcode_comments", new ConfigOptionBool(true));

    Print print;
    Model model;
    for (int tool = 0; tool < 2; ++tool) {
        ModelObject *object = model.add_object();
        REQUIRE(object != nullptr);
        object->add_volume(cube(12.));
        ModelInstance *instance = object->add_instance();
        REQUIRE(instance != nullptr);
        instance->set_offset(Vec3d(double(tool) * 50., 0., 0.));
        object->ensure_on_bed();
        object->config.set_key_value("extruder", new ConfigOptionInt(tool + 1));
    }

    print.apply(model, config);
    const auto validation = print.validate();
    INFO("Print validation error: " << validation.string << " [" << validation.opt_key << "]");
    REQUIRE(validation.string.empty());
    const std::string output = Test::gcode(print);
    REQUIRE_FALSE(output.empty());
    return output;
}

TEST_CASE("Nullable process travel variants resolve through the parent preset and reach each selected tool",
          "[GCode][SettingsSpeedAccelVariance][NullableTravelSettings]")
{
    namespace fs = boost::filesystem;
    ScopedTemporaryDir temp("nullable-process-travel-resolver");
    PresetBundle bundle;
    const fs::path process_dir = temp.path() / PRESET_PRINT_NAME;
    const fs::path parent_file = process_dir / "base" / "Nullable Parent.json";
    const fs::path child_file = process_dir / "Nullable Child.json";

    DynamicPrintConfig parent_raw = bundle.prints.default_preset().config;
    set_nullable_parent_motion(parent_raw);
    fs::create_directories(parent_file.parent_path());
    parent_raw.save_to_json(parent_file.string(), "Nullable Parent", "User", "1.0.0");
    write_nullable_child_preset(child_file);

    PresetsConfigSubstitutions substitutions;
    bundle.prints.load_presets(temp.path().string(), PRESET_PRINT_NAME, substitutions,
                               ForwardCompatibilitySubstitutionRule::EnableSilent);
    REQUIRE(bundle.prints.error_count() == 0);
    Preset *parent = bundle.prints.find_preset("Nullable Parent");
    Preset *child = bundle.prints.find_preset("Nullable Child");
    REQUIRE(parent != nullptr);
    REQUIRE(child != nullptr);
    REQUIRE(child->inherits() == "Nullable Parent");

    const auto *child_speed = child->config.option<ConfigOptionFloatsOrPercentsNullable>("initial_layer_travel_speed");
    const auto *child_acceleration = child->config.option<ConfigOptionFloatsOrPercentsNullable>("initial_layer_travel_acceleration");
    const auto *child_jerk = child->config.option<ConfigOptionFloatsOrPercentsNullable>("initial_layer_travel_jerk");
    const auto *child_z_speed = child->config.option<ConfigOptionFloatsNullable>("travel_speed_z");
    const auto *base_speed = child->config.option<ConfigOptionFloatsNullable>("travel_speed");
    const auto *base_acceleration = child->config.option<ConfigOptionFloatsNullable>("travel_acceleration");
    const auto *base_jerk = child->config.option<ConfigOptionFloatsNullable>("travel_jerk");
    REQUIRE(child_speed != nullptr);
    REQUIRE(child_acceleration != nullptr);
    REQUIRE(child_jerk != nullptr);
    REQUIRE(child_z_speed != nullptr);
    REQUIRE(base_speed != nullptr);
    REQUIRE(base_acceleration != nullptr);
    REQUIRE(base_jerk != nullptr);
    REQUIRE(child_speed->values.size() == 2);
    REQUIRE(child_acceleration->values.size() == 2);
    REQUIRE(child_jerk->values.size() == 2);
    REQUIRE(child_z_speed->values.size() == 2);
    REQUIRE(base_speed->values.size() == 2);
    REQUIRE(base_acceleration->values.size() == 2);
    REQUIRE(base_jerk->values.size() == 2);
    for (size_t slot = 0; slot < 2; ++slot) {
        REQUIRE_FALSE(child_speed->is_nil(slot));
        REQUIRE_FALSE(child_acceleration->is_nil(slot));
        REQUIRE_FALSE(child_jerk->is_nil(slot));
        REQUIRE_FALSE(child_z_speed->is_nil(slot));
        REQUIRE_FALSE(base_speed->is_nil(slot));
        REQUIRE_FALSE(base_acceleration->is_nil(slot));
        REQUIRE_FALSE(base_jerk->is_nil(slot));
    }
    CHECK_THAT(child_speed->get_at(0).get_abs_value(base_speed->get_at(0)), Catch::Matchers::WithinAbs(75., 1e-9));
    CHECK_THAT(child_speed->get_at(1).get_abs_value(base_speed->get_at(1)), Catch::Matchers::WithinAbs(20., 1e-9));
    CHECK_THAT(child_acceleration->get_at(0).get_abs_value(base_acceleration->get_at(0)), Catch::Matchers::WithinAbs(1200., 1e-9));
    CHECK_THAT(child_acceleration->get_at(1).get_abs_value(base_acceleration->get_at(1)), Catch::Matchers::WithinAbs(1200., 1e-9));
    CHECK_THAT(child_jerk->get_at(0).get_abs_value(base_jerk->get_at(0)), Catch::Matchers::WithinAbs(12., 1e-9));
    CHECK_THAT(child_jerk->get_at(1).get_abs_value(base_jerk->get_at(1)), Catch::Matchers::WithinAbs(3., 1e-9));
    CHECK_THAT(child_z_speed->get_at(0), Catch::Matchers::WithinAbs(8., 1e-9));
    CHECK_THAT(child_z_speed->get_at(1), Catch::Matchers::WithinAbs(6., 1e-9));

    Preset printer(Preset::TYPE_PRINTER, "Nullable Test Printer");
    printer.config = bundle.printers.default_preset().config;
    printer.config.option<ConfigOptionFloats>("nozzle_diameter", true)->values = {0.4, 0.4};
    printer.config.option<ConfigOptionFloats>("extruder_printable_height", true)->values = {0., 0.};
    printer.config.option<ConfigOptionEnumsGeneric>("extruder_type", true)->values = {etDirectDrive, etDirectDrive};
    printer.config.option<ConfigOptionEnumsGeneric>("nozzle_volume_type", true)->values = {nvtStandard, nvtHighFlow};
    printer.config.option<ConfigOptionStrings>("extruder_variant_list", true)->values = {
        "Direct Drive Standard", "Direct Drive High Flow"};
    printer.config.option<ConfigOptionInts>("printer_extruder_id", true)->values = {1, 2};
    printer.config.option<ConfigOptionStrings>("printer_extruder_variant", true)->values = {
        "Direct Drive Standard", "Direct Drive High Flow"};
    printer.config.option<ConfigOptionFloats>("machine_max_acceleration_travel", true)->values = {20000., 20000.};
    printer.config.option<ConfigOptionFloats>("machine_max_acceleration_extruding", true)->values = {20000., 20000.};
    printer.config.option<ConfigOptionFloats>("machine_max_acceleration_retracting", true)->values = {20000., 20000.};
    printer.config.option<ConfigOptionFloats>("machine_max_jerk_x", true)->values = {20000., 20000.};
    printer.config.option<ConfigOptionFloats>("machine_max_jerk_y", true)->values = {20000., 20000.};
    printer.config.option<ConfigOptionFloats>("machine_max_speed_x", true)->values = {300., 300.};
    printer.config.option<ConfigOptionFloats>("machine_max_speed_y", true)->values = {300., 300.};
    printer.config.option<ConfigOptionFloats>("machine_max_speed_z", true)->values = {30., 30.};
    printer.config.set_key_value("gcode_flavor", new ConfigOptionEnum<GCodeFlavor>(gcfMarlinFirmware));

    std::vector<Preset> filaments;
    for (int tool = 0; tool < 2; ++tool) {
        Preset filament(Preset::TYPE_FILAMENT, "Nullable Test PLA " + std::to_string(tool + 1));
        filament.config = bundle.filaments.default_preset().config;
        filament.config.option<ConfigOptionFloats>("filament_diameter", true)->values = {1.75};
        filament.config.option<ConfigOptionStrings>("filament_extruder_variant", true)->values = {
            "Direct Drive Standard", "Direct Drive High Flow"};
        filament.config.option<ConfigOptionStrings>("filament_type", true)->values = {"PLA"};
        filament.config.option<ConfigOptionStrings>("filament_colour", true)->values = {tool == 0 ? "#FF0000" : "#00FF00"};
        filament.filament_id = "nullable-test-pla-" + std::to_string(tool + 1);
        filaments.emplace_back(std::move(filament));
    }

    DynamicPrintConfig project = bundle.project_config;
    project.option<ConfigOptionFloats>("flush_volumes_matrix", true)->values = {0., 140., 140., 0.};
    project.option<ConfigOptionStrings>("filament_colour", true)->values = {"#FF0000", "#00FF00"};
    project.option<ConfigOptionStrings>("filament_colour_type", true)->values = {"1", "1"};
    project.option<ConfigOptionStrings>("filament_multi_colour", true)->values = {"#FF0000", "#00FF00"};
    project.option<ConfigOptionEnumsGeneric>("nozzle_volume_type", true)->values = {nvtStandard, nvtHighFlow};
    project.option<ConfigOptionEnum<FilamentMapMode>>("filament_map_mode", true)->value = fmmManual;
    project.option<ConfigOptionInts>("filament_map", true)->values = {1, 2};
    project.option<ConfigOptionInts>("filament_volume_map", true)->values = {int(nvtStandard), int(nvtHighFlow)};

    DynamicPrintConfig parent_full = nullable_full_config(printer, *parent, project, filaments);
    DynamicPrintConfig child_full = nullable_full_config(printer, *child, project, filaments);
    const std::string parent_output = export_nullable_two_tool_motion(parent_full);
    const std::string child_output = export_nullable_two_tool_motion(child_full);
    const NullableToolMotionEvidence parent_motion = parse_nullable_tool_motion(parent_output, parent_full);
    const NullableToolMotionEvidence child_motion = parse_nullable_tool_motion(child_output, child_full);
    const auto parent_material = gcode_print_variance_contracts::motion_evidence(parent_output, parent_full);
    const auto child_material = gcode_print_variance_contracts::motion_evidence(child_output, child_full);
    REQUIRE(parent_material.extrusion_moves > 0);
    gcode_print_variance_contracts::require_same_material(parent_material, child_material);

    REQUIRE(parent_motion.tool_selects[0] > 0);
    REQUIRE(parent_motion.tool_selects[1] > 0);
    REQUIRE(child_motion.tool_selects[0] > 0);
    REQUIRE(child_motion.tool_selects[1] > 0);
    require_nullable_tool_motion(parent_motion.tools[0], 2400., 1000, 6., 540.);
    require_nullable_tool_motion(parent_motion.tools[1], 4500., 1200, 12., 480.);
    require_nullable_tool_motion(child_motion.tools[0], 1200., 1200, 3., 360.);
    require_nullable_tool_motion(child_motion.tools[1], 4500., 1200, 12., 480.);

    for (size_t tool = 0; tool < parent_motion.tools.size(); ++tool) {
        CHECK(parent_motion.tools[tool].extrusion_moves == child_motion.tools[tool].extrusion_moves);
        CHECK_THAT(child_motion.tools[tool].xy_path,
                   Catch::Matchers::WithinRel(parent_motion.tools[tool].xy_path, 1e-6));
        CHECK_THAT(child_motion.tools[tool].filament,
                   Catch::Matchers::WithinRel(parent_motion.tools[tool].filament, 1e-6));
    }
}

} // namespace settings_speed_accel_contracts
