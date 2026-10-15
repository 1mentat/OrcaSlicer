#include <catch2/catch_all.hpp>

#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/GCodeReader.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintConfig.hpp"

#include "test_helpers.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <initializer_list>
#include <string>
#include <string_view>
#include <vector>

using namespace Slic3r;
using namespace Slic3r::Test;

namespace {

// This is a schema inventory plus selected executable dimensions. Settings which
// only affect firmware startup text are intentionally covered by the config tests.
constexpr std::array<std::string_view, 31> boolean_keys = {{
    "precise_outer_wall",
    "only_one_wall_top",
    "only_one_wall_first_layer",
    "extra_perimeters_on_overhangs",
    "overhang_reverse",
    "overhang_reverse_internal_only",
    "reduce_crossing_wall",
    "relative_bridge_angle",
    "bridge_no_support",
    "thick_bridges",
    "thick_internal_bridges",
    "is_infill_first",
    "fuzzy_skin_first_layer",
    "enable_arc_fitting",
    "make_overhang_printable",
    "detect_overhang_wall",
    "unsupported_wall_last",
    "spiral_mode",
    "spiral_mode_smooth",
    "enable_support",
    "support_on_build_plate_only",
    "support_critical_regions_only",
    "support_remove_small_overhang",
    "support_ironing",
    "detect_thin_wall",
    "hole_to_polyhole",
    "hole_to_polyhole_twisted",
    "infill_combination",
    "align_infill_direction_to_model",
    "brim_use_efc_outline",
    "brim_ears_outer_only",
}};

DynamicPrintConfig boolean_config()
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        { "layer_height",               0.2 },
        { "initial_layer_print_height",  0.2 },
        { "wall_loops",                 2 },
        { "top_shell_layers",           2 },
        { "bottom_shell_layers",        2 },
        { "top_shell_thickness",        0.0 },
        { "bottom_shell_thickness",     0.0 },
        { "sparse_infill_density",      "15%" },
        { "top_surface_density",        "100%" },
        { "bottom_surface_density",     "100%" },
        { "skirt_loops",                0 },
        { "brim_type",                  "no_brim" },
        { "gcode_comments",             true },
        { "layer_change_gcode",         "G92 E0\n" },
    });
    return config;
}

TriangleMesh two_tier_fixture()
{
    TriangleMesh lower = make_cube(30, 30, 3);
    TriangleMesh column = make_cube(8, 8, 15);
    TriangleMesh upper = make_cube(50, 50, 3);
    column.translate(11.f, 11.f, 2.f);
    upper.translate(-10.f, -10.f, 16.f);
    lower.merge(column);
    lower.merge(upper);
    return lower;
}

struct BooleanEvidence {
    size_t layers = 0;
    size_t perimeter_paths = 0;
    size_t first_layer_perimeter_paths = 0;
    size_t top_layer_perimeter_paths = 0;
    size_t thin_fill_paths = 0;
    size_t support_points = 0;
    size_t support_ironing_paths = 0;
    size_t ironing_paths = 0;
    size_t bridge_paths = 0;
    size_t brim_items = 0;
    double brim_checksum = 0.0;
    double perimeter_length = 0.0;
    double overhang_direction = 0.0;
    double bridge_height = 0.0;
    size_t hole_vertices = 0;
    double hole_checksum = 0.0;
    size_t arc_moves = 0;
    size_t extruding_z_moves = 0;
    std::string gcode;
};

void inspect_paths(const ExtrusionEntityCollection &collection, BooleanEvidence &evidence)
{
    for_each_extrusion_path(collection, [&evidence](const ExtrusionPath &path) {
        const ExtrusionRole role = path.role();
        if (is_perimeter(role)) {
            ++evidence.perimeter_paths;
            evidence.perimeter_length += path.length();
            if (path.overhang_degree > EPSILON) {
                const Point3 &first = path.first_point3();
                const Point3 &last = path.last_point3();
                evidence.overhang_direction += static_cast<double>(first.x()) * last.y() -
                                               static_cast<double>(first.y()) * last.x();
            }
        }
        if (role == erIroning)
            ++evidence.ironing_paths;
        if (role == erBridgeInfill || role == erInternalBridgeInfill) {
            ++evidence.bridge_paths;
            evidence.bridge_height = std::max(evidence.bridge_height, static_cast<double>(path.height));
        }
    });
}

size_t count_command(const std::string &gcode, const char *command)
{
    size_t count = 0;
    GCodeReader reader;
    reader.parse_buffer(gcode, [&count, command](GCodeReader &, const GCodeReader::GCodeLine &line) {
        if (line.cmd_is(command))
            ++count;
    });
    return count;
}

size_t count_extruding_z_moves(const std::string &gcode, const GCodeConfig &config)
{
    size_t count = 0;
    GCodeReader reader;
    reader.apply_config(config);
    reader.parse_buffer(gcode, [&count](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        if (line.extruding(self) && std::abs(line.dist_Z(self)) > EPSILON)
            ++count;
    });
    return count;
}

BooleanEvidence process_fixture(DynamicPrintConfig config, TriangleMesh shape = cube(20.0))
{
    BooleanEvidence evidence;
    // Keep a compact, physically valid fixture: 20 mm XY and roughly 4 mm Z.
    shape.scale(Vec3f(1.0f, 1.0f, 0.2f));
    Print print;
    Model model;
    init_print({ shape }, print, model, config);
    REQUIRE(print.validate().string.empty());
    REQUIRE_NOTHROW(print.process());
    REQUIRE_FALSE(print.objects().empty());

    const PrintObject &object = *print.objects().front();
    evidence.layers = object.layers().size();
    for (size_t layer_idx = 0; layer_idx < object.layers().size(); ++layer_idx) {
        const Layer *layer = object.layers()[layer_idx];
        for (const LayerRegion *region : layer->regions()) {
            evidence.thin_fill_paths += region->thin_fills.items_count();
            if (layer_idx == 0)
                evidence.first_layer_perimeter_paths += region->perimeters.items_count();
            if (layer_idx + 1 == object.layers().size())
                evidence.top_layer_perimeter_paths += region->perimeters.items_count();
            inspect_paths(region->perimeters, evidence);
            inspect_paths(region->fills, evidence);
            for (const Surface &surface : region->slices.surfaces) {
                for (const Polygon &hole : surface.expolygon.holes) {
                    evidence.hole_vertices += hole.points.size();
                    if (!hole.points.empty())
                        evidence.hole_checksum += static_cast<double>(hole.points.front().x()) * 0.37 +
                                                  static_cast<double>(hole.points.front().y()) * 0.61;
                }
            }
        }
    }
    for (const SupportLayer *layer : object.support_layers()) {
        Points points;
        layer->support_fills.collect_points(points);
        evidence.support_points += points.size();
        for_each_extrusion_path(layer->support_fills, [&evidence](const ExtrusionPath &path) {
            if (path.role() == erIroning)
                ++evidence.support_ironing_paths;
        });
    }
    for (const auto &entry : print.get_brimMap()) {
        evidence.brim_items += entry.second.items_count();
        for_each_extrusion_path(entry.second, [&evidence](const ExtrusionPath &path) {
            const Point3 &point = path.first_point3();
            evidence.brim_checksum += static_cast<double>(point.x()) * 0.37 +
                                      static_cast<double>(point.y()) * 0.61;
        });
    }

    REQUIRE_NOTHROW(evidence.gcode = gcode(print));
    evidence.arc_moves = count_command(evidence.gcode, "G2") + count_command(evidence.gcode, "G3");
    evidence.extruding_z_moves = count_extruding_z_moves(evidence.gcode, print.config());
    REQUIRE_FALSE(evidence.gcode.empty());
    REQUIRE(evidence.layers > 0);
    REQUIRE(std::isfinite(max_z(evidence.gcode)));
    return evidence;
}

DynamicPrintConfig with_bool(const char *key, bool value)
{
    DynamicPrintConfig config = boolean_config();
    config.set(key, value);
    REQUIRE(config.option(key)->serialize() == (value ? "1" : "0"));
    return config;
}

} // namespace

TEST_CASE("Registered boolean settings accept both canonical values", "[SettingsBooleanVariance]")
{
    for (const std::string_view key : boolean_keys) {
        CAPTURE(key);
        DynamicPrintConfig config = boolean_config();
        REQUIRE_NOTHROW(config.set_deserialize_strict(key.data(), "0"));
        CHECK(config.option(key.data())->serialize() == "0");
        REQUIRE_NOTHROW(config.set_deserialize_strict(key.data(), "1"));
        CHECK(config.option(key.data())->serialize() == "1");
    }
}

TEST_CASE("Wall and overhang booleans alter their geometric or ordering contract", "[SettingsBooleanVariance]")
{
    DynamicPrintConfig wall_base = boolean_config();
    wall_base.set_deserialize_strict({ { "wall_loops", 3 } });
    const BooleanEvidence normal_top = process_fixture(wall_base);

    DynamicPrintConfig one_top = wall_base;
    one_top.set_deserialize_strict({ { "only_one_wall_top", true } });
    const BooleanEvidence one_top_evidence = process_fixture(one_top);
    CHECK(one_top_evidence.top_layer_perimeter_paths < normal_top.top_layer_perimeter_paths);

    DynamicPrintConfig normal_first = boolean_config();
    normal_first.set_deserialize_strict({ { "wall_loops", 3 } });
    const BooleanEvidence regular_first = process_fixture(normal_first);
    DynamicPrintConfig one_first = normal_first;
    one_first.set_deserialize_strict({ { "only_one_wall_first_layer", true } });
    const BooleanEvidence one_first_evidence = process_fixture(one_first);
    CHECK(one_first_evidence.first_layer_perimeter_paths < regular_first.first_layer_perimeter_paths);

    DynamicPrintConfig overhang_base = boolean_config();
    overhang_base.set_deserialize_strict({
        { "detect_overhang_wall", true },
        { "wall_loops", 3 },
    });
    const BooleanEvidence regular_overhang = process_fixture(overhang_base, mesh(TestMesh::overhang));
    DynamicPrintConfig extra = overhang_base;
    extra.set("extra_perimeters_on_overhangs", true);
    const BooleanEvidence extra_overhang = process_fixture(extra, mesh(TestMesh::overhang));
    CHECK(extra_overhang.perimeter_paths > regular_overhang.perimeter_paths);

    DynamicPrintConfig reverse = overhang_base;
    reverse.set("overhang_reverse", true);
    const BooleanEvidence reversed = process_fixture(reverse, mesh(TestMesh::overhang));
    CHECK(std::abs(reversed.overhang_direction - regular_overhang.overhang_direction) > 0.01);

    const BooleanEvidence walls_first = process_fixture(with_bool("is_infill_first", false));
    const BooleanEvidence infill_first = process_fixture(with_bool("is_infill_first", true));
    const std::vector<std::string> wall_then_fill = role_sequence(walls_first.gcode, { "perimeter", "infill" });
    const std::vector<std::string> fill_then_wall = role_sequence(infill_first.gcode, { "perimeter", "infill" });
    REQUIRE_FALSE(wall_then_fill.empty());
    REQUIRE_FALSE(fill_then_wall.empty());
    CHECK(wall_then_fill.front() == "perimeter");
    CHECK(fill_then_wall.front() == "infill");
}

TEST_CASE("Support and bridge booleans preserve support and bridge invariants", "[SettingsBooleanVariance]")
{
    DynamicPrintConfig unsupported = boolean_config();
    unsupported.set_deserialize_strict({
        { "enable_support", false },
        { "support_type", "normal(auto)" },
    });
    const BooleanEvidence no_support = process_fixture(unsupported, mesh(TestMesh::overhang));

    DynamicPrintConfig supported = unsupported;
    supported.set_deserialize_strict({
        { "enable_support", true },
        { "support_threshold_angle", 15 },
        { "support_remove_small_overhang", false },
    });
    const BooleanEvidence support = process_fixture(supported, mesh(TestMesh::overhang));
    CHECK(no_support.support_points == 0);
    CHECK(support.support_points > 0);

    DynamicPrintConfig support_ironing_off = supported;
    support_ironing_off.set_deserialize_strict({
        { "support_interface_top_layers", 2 },
        { "support_ironing", false },
    });
    const BooleanEvidence support_interface = process_fixture(support_ironing_off, mesh(TestMesh::overhang));
    DynamicPrintConfig support_ironing_on = support_ironing_off;
    support_ironing_on.set("support_ironing", true);
    const BooleanEvidence ironed_support = process_fixture(support_ironing_on, mesh(TestMesh::overhang));
    CHECK(support_interface.support_ironing_paths == 0);
    CHECK(ironed_support.support_ironing_paths > 0);

    DynamicPrintConfig anywhere = supported;
    anywhere.set("support_on_build_plate_only", false);
    const BooleanEvidence support_anywhere = process_fixture(anywhere, two_tier_fixture());
    DynamicPrintConfig plate_only = anywhere;
    plate_only.set("support_on_build_plate_only", true);
    const BooleanEvidence support_plate_only = process_fixture(plate_only, two_tier_fixture());
    CHECK(support_anywhere.support_points > 0);
    CHECK(support_plate_only.support_points > 0);
    CHECK(support_anywhere.support_points != support_plate_only.support_points);

    DynamicPrintConfig thin_bridge = boolean_config();
    thin_bridge.set_deserialize_strict({
        { "bridge_no_support", true },
        { "thick_bridges", false },
        { "support_remove_small_overhang", false },
    });
    const BooleanEvidence normal_bridge = process_fixture(thin_bridge, mesh(TestMesh::bridge));
    DynamicPrintConfig thick_bridge = thin_bridge;
    thick_bridge.set("thick_bridges", true);
    const BooleanEvidence thick_bridge_evidence = process_fixture(thick_bridge, mesh(TestMesh::bridge));
    REQUIRE(normal_bridge.bridge_paths > 0);
    REQUIRE(thick_bridge_evidence.bridge_paths > 0);
    CHECK(thick_bridge_evidence.bridge_height > normal_bridge.bridge_height);
}

TEST_CASE("Brim compensation boolean changes the generated brim outline", "[SettingsBooleanVariance]")
{
    DynamicPrintConfig uncompensated = boolean_config();
    uncompensated.set_deserialize_strict({
        { "brim_type",                    "outer_only" },
        { "brim_width",                   4.0 },
        { "elefant_foot_compensation",    0.3 },
        { "elefant_foot_compensation_layers", 1 },
        { "brim_use_efc_outline",         false },
    });
    const BooleanEvidence plain = process_fixture(uncompensated);
    DynamicPrintConfig compensated = uncompensated;
    compensated.set("brim_use_efc_outline", true);
    const BooleanEvidence adjusted = process_fixture(compensated);
    REQUIRE(plain.brim_items > 0);
    REQUIRE(adjusted.brim_items > 0);
    CHECK(std::abs(plain.brim_checksum - adjusted.brim_checksum) > 0.01);
}

TEST_CASE("Ironing, thin-wall, arc, and fuzzy booleans reach their pipeline roles", "[SettingsBooleanVariance]")
{
    DynamicPrintConfig no_ironing = boolean_config();
    no_ironing.set_deserialize_strict("ironing_type", "no ironing");
    const BooleanEvidence plain = process_fixture(no_ironing);
    DynamicPrintConfig topmost = no_ironing;
    topmost.set_deserialize_strict("ironing_type", "topmost");
    const BooleanEvidence ironed = process_fixture(topmost);
    CHECK(plain.ironing_paths == 0);
    CHECK(ironed.ironing_paths > 0);

    DynamicPrintConfig no_arc = boolean_config();
    no_arc.set("enable_arc_fitting", false);
    const BooleanEvidence linear = process_fixture(no_arc);
    DynamicPrintConfig arc = no_arc;
    arc.set("enable_arc_fitting", true);
    const BooleanEvidence arcs = process_fixture(arc);
    CHECK(linear.arc_moves == 0);
    CHECK(arcs.arc_moves > 0);

    DynamicPrintConfig no_thin = boolean_config();
    no_thin.set_deserialize_strict({
        { "wall_loops", 2 },
        { "top_shell_layers", 0 },
        { "bottom_shell_layers", 0 },
        { "sparse_infill_density", "0%" },
        { "detect_thin_wall", false },
    });
    const BooleanEvidence ordinary_wall = process_fixture(no_thin, make_cube(20, 0.25, 3));
    DynamicPrintConfig detect_thin = no_thin;
    detect_thin.set("detect_thin_wall", true);
    const BooleanEvidence thin_wall = process_fixture(detect_thin, make_cube(20, 0.25, 3));
    CHECK(thin_wall.thin_fill_paths > ordinary_wall.thin_fill_paths);

    DynamicPrintConfig fuzzy_off = boolean_config();
    fuzzy_off.set_deserialize_strict({
        { "fuzzy_skin", "external" },
        { "fuzzy_skin_first_layer", false },
        { "fuzzy_skin_thickness", 0.2 },
        { "fuzzy_skin_point_distance", 0.3 },
        { "wall_generator", "arachne" },
    });
    const BooleanEvidence smooth_first = process_fixture(fuzzy_off);
    DynamicPrintConfig fuzzy_on = fuzzy_off;
    fuzzy_on.set("fuzzy_skin_first_layer", true);
    const BooleanEvidence fuzzy_first = process_fixture(fuzzy_on);
    CHECK(std::abs(fuzzy_first.perimeter_length - smooth_first.perimeter_length) > 0.01);
}

TEST_CASE("Spiral and model transformation booleans alter stateful geometry", "[SettingsBooleanVariance]")
{
    DynamicPrintConfig ordinary = boolean_config();
    ordinary.set_deserialize_strict({
        { "wall_loops", 1 },
        { "top_shell_layers", 0 },
        { "bottom_shell_layers", 2 },
        { "sparse_infill_density", "0%" },
        { "spiral_mode", false },
    });
    const BooleanEvidence ordinary_evidence = process_fixture(ordinary, mesh(TestMesh::slopy_cube));
    DynamicPrintConfig spiral = ordinary;
    spiral.set("spiral_mode", true);
    const BooleanEvidence spiral_evidence = process_fixture(spiral, mesh(TestMesh::slopy_cube));
    CHECK(ordinary_evidence.extruding_z_moves == 0);
    CHECK(spiral_evidence.extruding_z_moves > 0);

    DynamicPrintConfig no_polyhole = boolean_config();
    no_polyhole.set_deserialize_strict({
        { "hole_to_polyhole", false },
        { "hole_to_polyhole_twisted", false },
        { "hole_to_polyhole_max_edges", 6 },
        { "hole_to_polyhole_threshold", 0.5 },
    });
    const BooleanEvidence circular_hole = process_fixture(no_polyhole, mesh(TestMesh::cube_with_hole));
    DynamicPrintConfig polyhole = no_polyhole;
    polyhole.set("hole_to_polyhole", true);
    const BooleanEvidence polygon_hole = process_fixture(polyhole, mesh(TestMesh::cube_with_hole));
    REQUIRE(circular_hole.hole_vertices > 0);
    REQUIRE(polygon_hole.hole_vertices > 0);
    CHECK(circular_hole.hole_vertices != polygon_hole.hole_vertices);

    DynamicPrintConfig untwisted = polyhole;
    untwisted.set("hole_to_polyhole_twisted", false);
    const BooleanEvidence fixed_hole = process_fixture(untwisted, mesh(TestMesh::cube_with_hole));
    DynamicPrintConfig twisted = polyhole;
    twisted.set("hole_to_polyhole_twisted", true);
    const BooleanEvidence rotating_hole = process_fixture(twisted, mesh(TestMesh::cube_with_hole));
    CHECK(std::abs(fixed_hole.hole_checksum - rotating_hole.hole_checksum) > 0.01);
}
