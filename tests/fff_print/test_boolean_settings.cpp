#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <utility>
#include <catch2/catch_all.hpp>
#include <catch2/catch_message.hpp>

#include "catch2/catch_test_macros.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/GCodeReader.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/MeshBoolean.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintConfig.hpp"

#include "test_helpers.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <initializer_list>
#include <libslic3r/TriangleMesh.hpp>
#include <libslic3r/libslic3r.h>
#include <libslic3r/Point.hpp>
#include <libslic3r/Surface.hpp>
#include <libslic3r/Polygon.hpp>
#include <limits>
#include <string>
#include <vector>

using namespace Slic3r;
using namespace Slic3r::Test;

namespace {

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

TriangleMesh circular_hole_fixture()
{
    // TestMesh::cube_with_hole is a square cutout. Build a real circular
    // through-hole so polyhole conversion and layer rotation have an
    // observable geometric input.
    TriangleMesh body = make_cube(20., 20., 6.);
    TriangleMesh drill = make_cylinder(3., 8., PI / 90.);
    drill.translate(Vec3f(10.f, 10.f, -1.f));
    MeshBoolean::minus(body, drill);
    return body;
}

struct BooleanEvidence {
    size_t layers = 0;
    size_t perimeter_paths = 0;
    size_t first_layer_perimeter_paths = 0;
    size_t top_layer_perimeter_paths = 0;
    size_t support_points = 0;
    size_t support_ironing_paths = 0;
    size_t ironing_paths = 0;
    size_t external_bridge_infill_paths = 0;
    double external_bridge_infill_flow_mm3_per_mm = 0.0;
    size_t external_perimeter_paths = 0;
    double external_perimeter_max_x = -std::numeric_limits<double>::infinity();
    size_t brim_items = 0;
    double brim_checksum = 0.0;
    double perimeter_length = 0.0;
    double overhang_winding = 0.0;
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
            if (role == erExternalPerimeter) {
                ++evidence.external_perimeter_paths;
                for (const Point3 &point : path.polyline.points)
                    evidence.external_perimeter_max_x = std::max(evidence.external_perimeter_max_x, static_cast<double>(point.x()));
            }
            if (role == erExternalPerimeter && path.polyline.points.size() > 2) {
                // A closed path has identical first/last points, so their
                // cross product cannot observe reversal.  Signed polygon
                // area remains sensitive to overhang_reverse.
                for (size_t i = 1; i < path.polyline.points.size(); ++i) {
                    const Point3 &a = path.polyline.points[i - 1];
                    const Point3 &b = path.polyline.points[i];
                    evidence.overhang_winding += static_cast<double>(a.x()) * b.y() -
                                                  static_cast<double>(a.y()) * b.x();
                }
            }
        }
        if (role == erIroning)
            ++evidence.ironing_paths;
        if (role == erBridgeInfill) {
            ++evidence.external_bridge_infill_paths;
            evidence.external_bridge_infill_flow_mm3_per_mm =
                std::max(evidence.external_bridge_infill_flow_mm3_per_mm, path.mm3_per_mm);
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

TriangleMesh thin_wall_fixture()
{
    // Keep a normal-sized body so the print remains valid, and attach a
    // sub-nozzle fin whose narrow section is visible to classic walls.
    TriangleMesh body = make_cube(20., 20., 3.);
    // The fin crosses the body's right edge and extends well beyond it. A fin
    // laid along the edge is absorbed by the union and does not form a
    // thin-wall medial axis at the junction.
    TriangleMesh fin = make_cube(10., 0.25, 3.);
    fin.translate(Vec3f(19.9f, 5.f, 0.f));
    MeshBoolean::cgal::plus(body, fin);
    return body;
}

std::vector<std::string> non_first_layer_role_sequence(const std::string &gcode)
{
    const size_t first_layer = gcode.find(";LAYER_CHANGE");
    if (first_layer == std::string::npos)
        return {};
    const size_t second_layer = gcode.find(";LAYER_CHANGE", first_layer + 1);
    if (second_layer == std::string::npos)
        return {};
    return role_sequence(gcode.substr(second_layer), { "perimeter", "infill" });
}

} // namespace

TEST_CASE("only_one_wall_top limits top layer perimeter count", "[SettingsBooleanVariance][BooleanSettings]")
{
    DynamicPrintConfig wall_base = boolean_config();
    wall_base.set_deserialize_strict({
        { "wall_loops", 3 },
        { "only_one_wall_top", false },
    });
    const BooleanEvidence normal_top = process_fixture(wall_base);

    DynamicPrintConfig one_top = wall_base;
    one_top.set_deserialize_strict({ { "only_one_wall_top", true } });
    const BooleanEvidence one_top_evidence = process_fixture(one_top);
    CHECK(one_top_evidence.top_layer_perimeter_paths < normal_top.top_layer_perimeter_paths);
}

TEST_CASE("only_one_wall_first_layer limits first layer perimeter count", "[SettingsBooleanVariance][BooleanSettings]")
{
    DynamicPrintConfig normal_first = boolean_config();
    normal_first.set_deserialize_strict({
        { "wall_loops", 3 },
        { "only_one_wall_first_layer", false },
    });
    const BooleanEvidence regular_first = process_fixture(normal_first);
    DynamicPrintConfig one_first = normal_first;
    one_first.set_deserialize_strict({ { "only_one_wall_first_layer", true } });
    const BooleanEvidence one_first_evidence = process_fixture(one_first);
    CHECK(one_first_evidence.first_layer_perimeter_paths < regular_first.first_layer_perimeter_paths);
}

TEST_CASE("extra_perimeters_on_overhangs adds overhang perimeter geometry", "[SettingsBooleanVariance][BooleanSettings]")
{
    DynamicPrintConfig overhang_base = boolean_config();
    overhang_base.set_deserialize_strict({
        { "detect_overhang_wall",          true },
        { "wall_loops",                    3 },
        { "extra_perimeters_on_overhangs", false },
        { "overhang_reverse",              false },
        { "overhang_reverse_internal_only", false },
    });
    const BooleanEvidence regular_overhang = process_fixture(overhang_base, mesh(TestMesh::overhang));
    DynamicPrintConfig extra = overhang_base;
    extra.set("extra_perimeters_on_overhangs", true);
    const BooleanEvidence extra_overhang = process_fixture(extra, mesh(TestMesh::overhang));
    CHECK(extra_overhang.perimeter_paths > regular_overhang.perimeter_paths);
}

TEST_CASE("overhang_reverse changes overhang path orientation", "[SettingsBooleanVariance][BooleanSettings]")
{
    DynamicPrintConfig overhang_base = boolean_config();
    overhang_base.set_deserialize_strict({
        { "detect_overhang_wall",          true },
        { "wall_loops",                    3 },
        { "extra_perimeters_on_overhangs", false },
        { "overhang_reverse",              false },
        { "overhang_reverse_internal_only", false },
    });
    const BooleanEvidence regular_overhang = process_fixture(overhang_base, mesh(TestMesh::overhang));
    DynamicPrintConfig reverse = overhang_base;
    reverse.set("overhang_reverse", true);
    const BooleanEvidence reversed = process_fixture(reverse, mesh(TestMesh::overhang));
    CHECK(std::abs(reversed.overhang_winding - regular_overhang.overhang_winding) > 0.01);
}

TEST_CASE("is_infill_first changes role ordering", "[SettingsBooleanVariance][BooleanSettings]")
{
    const BooleanEvidence walls_first = process_fixture(with_bool("is_infill_first", false));
    const BooleanEvidence infill_first = process_fixture(with_bool("is_infill_first", true));
    const std::vector<std::string> wall_then_fill = non_first_layer_role_sequence(walls_first.gcode);
    const std::vector<std::string> fill_then_wall = non_first_layer_role_sequence(infill_first.gcode);
    REQUIRE_FALSE(wall_then_fill.empty());
    REQUIRE_FALSE(fill_then_wall.empty());
    CHECK(wall_then_fill.front() == "perimeter");
    CHECK(fill_then_wall.front() == "infill");
}

TEST_CASE("enable_support creates support geometry when required", "[SettingsBooleanVariance][BooleanSettings]")
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
}

TEST_CASE("support_ironing creates ironing paths on support", "[SettingsBooleanVariance][BooleanSettings]")
{
    DynamicPrintConfig support_ironing_off = boolean_config();
    support_ironing_off.set_deserialize_strict({
        { "enable_support",               true },
        { "support_type",                 "normal(auto)" },
        { "support_threshold_angle",      15 },
        { "support_remove_small_overhang", false },
        { "support_interface_top_layers", 2 },
        { "support_ironing",              false },
    });
    const BooleanEvidence support_interface = process_fixture(support_ironing_off, mesh(TestMesh::overhang));
    DynamicPrintConfig support_ironing_on = support_ironing_off;
    support_ironing_on.set("support_ironing", true);
    const BooleanEvidence ironed_support = process_fixture(support_ironing_on, mesh(TestMesh::overhang));
    CHECK(support_interface.support_ironing_paths == 0);
    CHECK(ironed_support.support_ironing_paths > 0);
}

TEST_CASE("support_on_build_plate_only changes elevated support allocation", "[SettingsBooleanVariance][BooleanSettings]")
{
    DynamicPrintConfig supported = boolean_config();
    supported.set_deserialize_strict({
        { "enable_support",               true },
        { "support_type",                 "normal(auto)" },
        { "support_threshold_angle",      15 },
        { "support_remove_small_overhang", false },
    });
    DynamicPrintConfig anywhere = supported;
    anywhere.set("support_on_build_plate_only", false);
    const BooleanEvidence support_anywhere = process_fixture(anywhere, two_tier_fixture());
    DynamicPrintConfig plate_only = anywhere;
    plate_only.set("support_on_build_plate_only", true);
    const BooleanEvidence support_plate_only = process_fixture(plate_only, two_tier_fixture());
    CHECK(support_anywhere.support_points > 0);
    CHECK(support_plate_only.support_points > 0);
    CHECK(support_anywhere.support_points != support_plate_only.support_points);
}

TEST_CASE("thick_bridges changes bridge extrusion geometry", "[SettingsBooleanVariance][BooleanSettings]")
{
    DynamicPrintConfig thin_bridge = boolean_config();
    thin_bridge.set_deserialize_strict({
        { "bridge_no_support", true },
        { "thick_bridges", false },
        // Keep the ratio away from the default convergence point so the
        // rounded (thick) and normal bridge-flow contracts are distinguishable.
        { "bridge_flow", 0.75 },
        { "support_remove_small_overhang", false },
    });
    const BooleanEvidence normal_bridge = process_fixture(thin_bridge, mesh(TestMesh::bridge));
    DynamicPrintConfig thick_bridge = thin_bridge;
    thick_bridge.set("thick_bridges", true);
    const BooleanEvidence thick_bridge_evidence = process_fixture(thick_bridge, mesh(TestMesh::bridge));
    REQUIRE(normal_bridge.external_bridge_infill_paths > 0);
    REQUIRE(thick_bridge_evidence.external_bridge_infill_paths > 0);
    // Thick bridges use the rounded bridge-flow model. Width and height are
    // visualization fields and may be normalized by fill spacing; the
    // deposited volume per unit path is the stable flow contract.
    CHECK(std::abs(thick_bridge_evidence.external_bridge_infill_flow_mm3_per_mm -
                   normal_bridge.external_bridge_infill_flow_mm3_per_mm) > 0.001);
}

TEST_CASE("brim_use_efc_outline changes the generated brim outline", "[SettingsBooleanVariance][BooleanSettings]")
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

TEST_CASE("ironing_type creates topmost ironing paths", "[SettingsBooleanVariance][BooleanSettings]")
{
    DynamicPrintConfig no_ironing = boolean_config();
    no_ironing.set_deserialize_strict("ironing_type", "no ironing");
    const BooleanEvidence plain = process_fixture(no_ironing);
    DynamicPrintConfig topmost = no_ironing;
    topmost.set_deserialize_strict("ironing_type", "topmost");
    const BooleanEvidence ironed = process_fixture(topmost);
    CHECK(plain.ironing_paths == 0);
    CHECK(ironed.ironing_paths > 0);
}

TEST_CASE("enable_arc_fitting emits arcs for curved geometry", "[SettingsBooleanVariance][BooleanSettings]")
{
    DynamicPrintConfig no_arc = boolean_config();
    no_arc.set("enable_arc_fitting", false);
    const TriangleMesh curved = make_cylinder(10., 4., PI / 90.);
    const BooleanEvidence linear = process_fixture(no_arc, curved);
    DynamicPrintConfig arc = no_arc;
    arc.set("enable_arc_fitting", true);
    const BooleanEvidence arcs = process_fixture(arc, curved);
    CHECK(linear.arc_moves == 0);
    CHECK(arcs.arc_moves > 0);
}

TEST_CASE("detect_thin_wall emits an external perimeter into a narrow fin", "[SettingsBooleanVariance][BooleanSettings]")
{
    DynamicPrintConfig no_thin = boolean_config();
    no_thin.set_deserialize_strict({
        { "wall_loops", 2 },
        { "top_shell_layers", 0 },
        { "bottom_shell_layers", 0 },
        { "sparse_infill_density", "0%" },
        { "detect_thin_wall", false },
        { "wall_generator", "classic" },
    });
    const BooleanEvidence ordinary_wall = process_fixture(no_thin, thin_wall_fixture());
    DynamicPrintConfig detect_thin = no_thin;
    detect_thin.set("detect_thin_wall", true);
    const BooleanEvidence thin_wall = process_fixture(detect_thin, thin_wall_fixture());
    // Classic perimeter generation stores detected medial-axis walls in the
    // external-perimeter collection.
    CHECK(thin_wall.external_perimeter_paths > ordinary_wall.external_perimeter_paths);
    // init_print centers the fixture, so compare against the disabled control
    // rather than an absolute model coordinate.
    CHECK(thin_wall.external_perimeter_max_x > ordinary_wall.external_perimeter_max_x + static_cast<double>(scale_(5.0)));
}

TEST_CASE("fuzzy_skin_first_layer changes first layer perimeter length", "[SettingsBooleanVariance][BooleanSettings]")
{
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

TEST_CASE("spiral_mode creates continuous extruding Z motion", "[SettingsBooleanVariance][BooleanSettings]")
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
}

TEST_CASE("hole_to_polyhole changes circular hole vertex count", "[SettingsBooleanVariance][BooleanSettings]")
{
    DynamicPrintConfig no_polyhole = boolean_config();
    no_polyhole.set_deserialize_strict({
        { "hole_to_polyhole", false },
        { "hole_to_polyhole_twisted", false },
        { "hole_to_polyhole_max_edges", 6 },
        { "hole_to_polyhole_threshold", 0.5 },
    });
    const BooleanEvidence circular_hole = process_fixture(no_polyhole, circular_hole_fixture());
    DynamicPrintConfig polyhole = no_polyhole;
    polyhole.set("hole_to_polyhole", true);
    const BooleanEvidence polygon_hole = process_fixture(polyhole, circular_hole_fixture());
    REQUIRE(circular_hole.hole_vertices > 0);
    REQUIRE(polygon_hole.hole_vertices > 0);
    CHECK(circular_hole.hole_vertices != polygon_hole.hole_vertices);
}

TEST_CASE("hole_to_polyhole_twisted rotates polygon hole geometry", "[SettingsBooleanVariance][BooleanSettings]")
{
    DynamicPrintConfig polyhole = boolean_config();
    polyhole.set_deserialize_strict({
        { "hole_to_polyhole",           true },
        { "hole_to_polyhole_twisted",   false },
        { "hole_to_polyhole_max_edges", 6 },
        { "hole_to_polyhole_threshold", 0.5 },
    });
    DynamicPrintConfig untwisted = polyhole;
    untwisted.set("hole_to_polyhole_twisted", false);
    const BooleanEvidence fixed_hole = process_fixture(untwisted, circular_hole_fixture());
    DynamicPrintConfig twisted = polyhole;
    twisted.set("hole_to_polyhole_twisted", true);
    const BooleanEvidence rotating_hole = process_fixture(twisted, circular_hole_fixture());
    CHECK(std::abs(fixed_hole.hole_checksum - rotating_hole.hole_checksum) > 0.01);
}

namespace internal_wall_reversal_contracts {

struct LoopEvidence {
    bool external;
    double area_mm2;
    double length_mm;
    bool counter_clockwise;
};

using LayerLoops = std::vector<std::vector<LoopEvidence>>;

LayerLoops observe(const char *generator, const char *direction, bool reverse, bool internal_only)
{
    DynamicPrintConfig config = boolean_config();
    config.set_deserialize_strict({
        {"wall_generator", generator},
        {"wall_direction", direction},
        {"wall_loops", 3},
        {"outer_wall_line_width", 0.45},
        {"inner_wall_line_width", 0.45},
        {"initial_layer_line_width", 0.45},
        {"top_shell_layers", 0},
        {"bottom_shell_layers", 0},
        {"sparse_infill_density", "0%"},
        {"only_one_wall_top", false},
        {"only_one_wall_first_layer", false},
        {"spiral_mode", false},
        {"enable_support", false},
        {"raft_layers", 0},
        {"fuzzy_skin", "none"},
        {"detect_overhang_wall", true},
        {"extra_perimeters_on_overhangs", false},
        {"overhang_reverse_threshold", "0"},
        {"overhang_reverse", reverse},
        {"overhang_reverse_internal_only", internal_only},
    });
    REQUIRE(config.opt_serialize("wall_generator") == generator);
    REQUIRE(config.opt_serialize("wall_direction") == direction);
    REQUIRE(config.opt_bool("overhang_reverse") == reverse);
    REQUIRE(config.opt_bool("overhang_reverse_internal_only") == internal_only);

    Model model;
    Print print;
    init_print({make_cube(10., 10., 0.8)}, print, model, config);
    REQUIRE(print.validate().string.empty());
    print.process();
    REQUIRE(print.objects().size() == 1);
    const auto &layers = print.objects().front()->layers();
    REQUIRE(layers.size() == 4); // Explicit 0.2 mm initial and ordinary heights.

    LayerLoops result;
    const double one_mm = double(scale_(1.));
    for (const Layer *layer : layers) {
        std::vector<LoopEvidence> loops;
        for (const LayerRegion *region : layer->regions()) {
            // Own the flattened collection while observing its loop pointers.
            const ExtrusionEntityCollection flat = region->perimeters.flatten();
            for (const ExtrusionEntity *entity : flat.entities) {
                REQUIRE(entity->is_loop());
                const auto &loop = *static_cast<const ExtrusionLoop *>(entity);
                REQUIRE_FALSE(loop.paths.empty());
                const Polygon polygon = loop.polygon();
                REQUIRE(polygon.points.size() >= 3);
                const double signed_area = polygon.area() / (one_mm * one_mm);
                REQUIRE(std::isfinite(signed_area));
                REQUIRE(std::abs(signed_area) > 1.0);
                const bool external = loop.paths.front().role() == erExternalPerimeter;
                for (const ExtrusionPath &path : loop.paths) {
                    REQUIRE(path.role() == (external ? erExternalPerimeter : erPerimeter));
                }
                const double length = loop.length() / one_mm;
                REQUIRE(std::isfinite(length));
                REQUIRE(length > 0.);
                loops.push_back({external, std::abs(signed_area), length, signed_area > 0.});
            }
        }
        REQUIRE(std::count_if(loops.begin(), loops.end(), [](const LoopEvidence &loop) {
            return loop.external;
        }) > 0);
        REQUIRE(std::count_if(loops.begin(), loops.end(), [](const LoopEvidence &loop) {
            return !loop.external;
        }) > 0);
        std::sort(loops.begin(), loops.end(), [](const LoopEvidence &a, const LoopEvidence &b) {
            if (a.external != b.external)
                return a.external < b.external;
            return a.area_mm2 < b.area_mm2;
        });
        result.push_back(std::move(loops));
    }
    return result;
}

void require_geometry(const LayerLoops &expected, const LayerLoops &observed)
{
    REQUIRE(observed.size() == expected.size());
    for (size_t layer = 0; layer < expected.size(); ++layer) {
        REQUIRE(observed[layer].size() == expected[layer].size());
        for (size_t i = 0; i < expected[layer].size(); ++i) {
            CHECK(observed[layer][i].external == expected[layer][i].external);
            CHECK_THAT(observed[layer][i].area_mm2,
                       Catch::Matchers::WithinAbs(expected[layer][i].area_mm2, 1e-5));
            CHECK_THAT(observed[layer][i].length_mm,
                       Catch::Matchers::WithinAbs(expected[layer][i].length_mm, 1e-5));
        }
    }
}

} // namespace internal_wall_reversal_contracts

TEST_CASE("Internal-only reversal preserves exterior winding on alternating layers",
          "[BooleanSettings][OverhangInternalReversal][SettingsVariance]")
{
    using namespace internal_wall_reversal_contracts;
    const char *generator = GENERATE("classic", "arachne");
    const char *direction = GENERATE("ccw", "cw");
    CAPTURE(generator, direction);
    const LayerLoops disabled = observe(generator, direction, false, false);
    const LayerLoops disabled_internal = observe(generator, direction, false, true);
    const LayerLoops all_walls = observe(generator, direction, true, false);
    const LayerLoops internal_walls = observe(generator, direction, true, true);
    require_geometry(disabled, disabled_internal);
    require_geometry(disabled, all_walls);
    require_geometry(disabled, internal_walls);

    // The documented zero threshold reverses every even printed layer,
    // irrespective of overhang geometry. Inspect generated loop winding,
    // independently of traversal order or the production reversal helper.
    const bool configured_ccw = std::string(direction) == "ccw";
    for (size_t layer = 0; layer < disabled.size(); ++layer) {
        const bool even_printed_layer = (layer + 1) % 2 == 0;
        for (size_t i = 0; i < disabled[layer].size(); ++i) {
            CAPTURE(layer, i, disabled[layer][i].external);
            CHECK(disabled[layer][i].counter_clockwise == configured_ccw);
            CHECK(disabled_internal[layer][i].counter_clockwise == configured_ccw);
            CHECK(all_walls[layer][i].counter_clockwise == (configured_ccw != even_printed_layer));
            const bool reverse_internal = even_printed_layer && !disabled[layer][i].external;
            CHECK(internal_walls[layer][i].counter_clockwise == (configured_ccw != reverse_internal));
        }
    }
}
