#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <initializer_list>
#include <limits>
#include <string>
#include <string_view>
#include <sstream>
#include <utility>
#include <vector>
#include "libslic3r/Config.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/Model.hpp"
#include <cstddef>
#include "libslic3r/libslic3r.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_message.hpp>
#include "libslic3r/Layer.hpp"
#include "libslic3r/Surface.hpp"
#include "libslic3r/TriangleMesh.hpp"

#include "test_helpers.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Polyline.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintBase.hpp"
#include "libslic3r/Slicing.hpp"
#include "libslic3r/GCodeReader.hpp"
#include "libslic3r/TriangleSelector.hpp"

using namespace Slic3r::Test;
using namespace Slic3r;

namespace {

// The upper plate overhangs both the lower plate and open air, so branches land on the model and on
// the bed in the same slice.
TriangleMesh two_tier_mesh()
{
    TriangleMesh lower  = make_cube(30, 30, 3);
    TriangleMesh column = make_cube(8, 8, 15);
    TriangleMesh upper  = make_cube(50, 50, 3);
    // Each part overlaps the one below rather than resting on it; a coplanar join slices ambiguously.
    column.translate(11.f, 11.f, 2.f);
    upper.translate(-10.f, -10.f, 16.f);
    TriangleMesh mesh = lower;
    mesh.merge(column);
    mesh.merge(upper);
    return mesh;
}

TriangleMesh scaled(TestMesh id, float scale)
{
    TriangleMesh mesh = Slic3r::Test::mesh(id);
    mesh.scale(scale);
    return mesh;
}

// Preserve the dense XY node field without repeating the same geometry over extra layers.
TriangleMesh scaled_z(TestMesh id, float xy_scale, float z_scale)
{
    TriangleMesh mesh = Slic3r::Test::mesh(id);
    mesh.scale(Vec3f(xy_scale, xy_scale, z_scale));
    return mesh;
}

// `extra` is applied last, so a caller can add or override any key.
void slice_with_tree_support(const TriangleMesh &mesh, Slic3r::Print &print, const char *style,
                             int threshold_angle = 30, int build_plate_only = 0, int raft_layers = 0,
                             std::initializer_list<Slic3r::ConfigBase::SetDeserializeItem> extra = {})
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        { "enable_support",              1 },
        { "support_type",                "tree(auto)" },
        { "support_style",               style },
        { "support_on_build_plate_only", build_plate_only },
        { "support_threshold_angle",     threshold_angle },
        { "raft_layers",                 raft_layers },
        { "layer_height",                0.2 },
    });
    config.set_deserialize_strict(extra);
    Slic3r::Test::init_and_process_print({ mesh }, print, config);
}

Points support_points(const Slic3r::Print &print)
{
    Points points;
    for (const SupportLayer *layer : print.objects().front()->support_layers())
        layer->support_fills.collect_points(points);
    return points;
}

size_t support_point_count(const TriangleMesh &mesh, const char *style, int threshold_angle = 30,
                           int build_plate_only = 0)
{
    Slic3r::Print print;
    slice_with_tree_support(mesh, print, style, threshold_angle, build_plate_only);
    return support_points(print).size();
}

namespace tree_support_critical_regions_contracts {

struct CriticalRegionEvidence {
    size_t total_points = 0;
    size_t ordinary_overhang_segments = 0;
    size_t cantilever_segments = 0;
};

// The 1 mm shelf overhang is below TreeSupport's 3 mm cantilever threshold.  The separate
// 28 mm cap is carried by a 6 mm stem, leaving an 11 mm side overhang and a source-defined
// cantilever.  Both regions live in one ModelObject so the public object output can be compared
// without relying on private TreeSupport state.
static TriangleMesh critical_regions_fixture()
{
    TriangleMesh result = make_cube(20.0, 20.0, 4.0);

    TriangleMesh ordinary_shelf = make_cube(22.0, 20.0, 0.8);
    ordinary_shelf.translate(-1.0, 0.0, 3.6); // overlaps the base by 0.4 mm
    result.merge(ordinary_shelf);

    TriangleMesh stem = make_cube(6.0, 6.0, 8.0);
    stem.translate(41.0, 7.0, 0.0); // centered under the cap: x=41..47
    result.merge(stem);

    TriangleMesh cantilever_cap = make_cube(28.0, 28.0, 0.8);
    cantilever_cap.translate(30.0, -4.0, 7.6); // overlaps the stem by 0.4 mm
    result.merge(cantilever_cap);
    return result;
}

static bool segment_intersects_xy_window(const Point3 &a, const Point3 &b, const Point &center_offset,
                                          double min_x, double max_x, double min_y, double max_y)
{
    const double ax = unscale<double>(a.x() + center_offset.x());
    const double ay = unscale<double>(a.y() + center_offset.y());
    const double bx = unscale<double>(b.x() + center_offset.x());
    const double by = unscale<double>(b.y() + center_offset.y());
    const double dx = bx - ax;
    const double dy = by - ay;
    double t_min = 0.0;
    double t_max = 1.0;
    const auto clip = [&t_min, &t_max](double p, double q) {
        if (std::abs(p) <= 1e-12)
            return q >= 0.0;
        const double r = q / p;
        if (p < 0.0) {
            if (r > t_max) return false;
            if (r > t_min) t_min = r;
        } else {
            if (r < t_min) return false;
            if (r < t_max) t_max = r;
        }
        return true;
    };
    return clip(-dx, ax - min_x) && clip(dx, max_x - ax) &&
           clip(-dy, ay - min_y) && clip(dy, max_y - ay);
}

static size_t support_segments_in_window(const Print &print, const Point &center_offset,
                                         double min_x, double max_x, double min_y, double max_y)
{
    size_t count = 0;
    for (const SupportLayer *layer : print.objects().front()->support_layers()) {
        for_each_extrusion_path(layer->support_fills, [&](const ExtrusionPath &path) {
            for (size_t i = 1; i < path.polyline.points.size(); ++i) {
                if (segment_intersects_xy_window(path.polyline.points[i - 1], path.polyline.points[i], center_offset,
                                                 min_x, max_x, min_y, max_y)) {
                    ++count;
                    break;
                }
            }
        });
    }
    return count;
}

static CriticalRegionEvidence observe(bool critical_only)
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    // The ordinary shelf extends left of X=0. Keep both regions and their support
    // branches inside the bed so boundary clipping cannot erase the shelf witness.
    config.set_deserialize_strict({
        { "printable_area",               "-100x-100,200x-100,200x200,-100x200" },
        { "enable_support",                1 },
        { "support_type",                  "tree(auto)" },
        { "support_style",                 "tree_slim" },
        { "support_on_build_plate_only",   0 },
        { "support_critical_regions_only", critical_only },
        { "support_remove_small_overhang", 0 },
        { "support_threshold_angle",       30 },
        { "support_top_z_distance",        0.2 },
        { "support_interface_top_layers",  0 },
        { "support_interface_bottom_layers", 0 },
        { "max_bridge_length",              0.0 },
        { "bridge_no_support",              0 },
        { "thick_bridges",                  0 },
        { "tree_support_branch_distance",  5.0 },
        { "tree_support_branch_diameter",  2.0 },
        { "tree_support_branch_angle",     40.0 },
        { "support_line_width",            0.4 },
        { "line_width",                    0.42 },
        { "outer_wall_line_width",         0.42 },
        { "inner_wall_line_width",         0.42 },
        { "wall_generator",                "classic" },
        { "nozzle_diameter",               "0.4" },
        { "layer_height",                  0.2 },
        { "initial_layer_print_height",    0.2 },
        { "raft_layers",                   0 },
        { "brim_width",                    0.0 },
        { "skirt_loops",                   0 },
        { "use_relative_e_distances",      true },
        { "before_layer_change_gcode",     "G92 E0\n" },
    });

    Print print;
    Model model;
    std::vector<TriangleMesh> meshes;
    meshes.emplace_back(critical_regions_fixture());
    init_print(std::move(meshes), print, model, config, nullptr, false);
    const StringObjectException validation = print.validate();
    INFO(validation.string);
    REQUIRE(validation.string.empty());
    print.process();
    REQUIRE(!print.objects().empty());
    const PrintObject &object = *print.objects().front();
    const Points points = support_points(print);
    CriticalRegionEvidence result;
    result.total_points = points.size();
    const Point &center_offset = object.center_offset();
    // The shelf's exposed one-millimetre band is x in (-1,0).  The cap's outer band
    // x in (30,40.5) is disjoint from that shelf and is the independent 11 mm cantilever
    // witness. Segment intersection also catches a path whose vertices straddle a window.
    result.ordinary_overhang_segments = support_segments_in_window(print, center_offset, -1.1, -0.1, 2.0, 18.0);
    result.cantilever_segments = support_segments_in_window(print, center_offset, 30.0, 40.5, -3.0, 23.0);
    REQUIRE(result.total_points > 0);
    return result;
}

} // namespace tree_support_critical_regions_contracts

TEST_CASE("Critical tree support keeps cantilevers and removes ordinary overhangs",
          "[TreeSupport][SupportCriticalRegionsOnly]")
{
    using namespace tree_support_critical_regions_contracts;
    const CriticalRegionEvidence all_regions = observe(false);
    const CriticalRegionEvidence critical_only = observe(true);

    // Both regions must be reachable in the unfiltered public support output.
    REQUIRE(all_regions.ordinary_overhang_segments > 0);
    REQUIRE(all_regions.cantilever_segments > 0);
    REQUIRE(critical_only.total_points > 0);
    REQUIRE(critical_only.cantilever_segments > 0);
    // support_critical_regions_only clears ordinary overhangs but retains source-classified
    // cantilevers.  The windows are disjoint and derived from the fixture dimensions.
    REQUIRE(critical_only.ordinary_overhang_segments == 0);
}

// Index of the first differing point, or the common length when they match. An index keeps a
// failure readable; comparing the vectors themselves dumps thousands of points.
size_t first_difference(const Points &a, const Points &b)
{
    const size_t common = std::min(a.size(), b.size());
    for (size_t i = 0; i < common; ++i)
        if (a[i] != b[i])
            return i;
    return common;
}

// Slice `mesh` twice and require an identical support point sequence. Point counts and total
// length are order insensitive, so the sequence is what a reordering shows up in.
void sliced_twice_matches(const TriangleMesh &mesh, int build_plate_only, const char *style = "tree_slim",
                          std::initializer_list<Slic3r::ConfigBase::SetDeserializeItem> extra = {})
{
    Slic3r::Print first_print, second_print;
    slice_with_tree_support(mesh, first_print, style, 30, build_plate_only, 0, extra);
    slice_with_tree_support(mesh, second_print, style, 30, build_plate_only, 0, extra);
    const Points first  = support_points(first_print);
    const Points second = support_points(second_print);
    REQUIRE(first.size() > 1000); // without support the comparison below passes vacuously
    REQUIRE(second.size() == first.size());
    REQUIRE(first_difference(first, second) == first.size());
}

} // namespace

TEST_CASE("Tree support is generated for an overhang and not for a plain cube", "[TreeSupport]")
{
    REQUIRE(support_point_count(scaled(TestMesh::overhang, 2.f), "tree_slim") > 1000);
    REQUIRE(support_point_count(Slic3r::Test::cube(20), "tree_slim") == 0);
}

TEST_CASE("Restricting tree support to the build plate changes what is generated", "[TreeSupport]")
{
    const TriangleMesh mesh = two_tier_mesh();
    const size_t anywhere    = support_point_count(mesh, "tree_slim", 30, 0);
    const size_t plate_only  = support_point_count(mesh, "tree_slim", 30, 1);
    REQUIRE(anywhere > 1000);
    REQUIRE(plate_only > 1000);
    // The upper plate overhangs the lower one, so some branches would land on the model.
    REQUIRE(plate_only != anywhere);
}

TEST_CASE("Tree support layers rise monotonically within the layer height limits", "[TreeSupport]")
{
    Slic3r::Print print;
    slice_with_tree_support(scaled(TestMesh::overhang, 2.f), print, "tree_slim");
    const double nozzle = print.config().nozzle_diameter.values.front();

    size_t checked = 0;
    double previous = 0;
    bool   previous_was_adjacent = false;
    for (const SupportLayer *layer : print.objects().front()->support_layers()) {
        if (layer->print_z <= 0 || layer->height <= 0) {
            // Layers with no nodes are left at zero. Skipping one leaves a hole, so the next pair
            // spans more than one layer and its gap says nothing about the layer height limit.
            previous_was_adjacent = false;
            continue;
        }
        if (previous > 0) {
            CAPTURE(previous, layer->print_z);
            REQUIRE(layer->print_z > previous);
            if (previous_was_adjacent)
                REQUIRE(layer->print_z - previous <= nozzle + EPSILON);
        }
        previous = layer->print_z;
        previous_was_adjacent = true;
        ++checked;
    }
    REQUIRE(checked > 10);
}

TEST_CASE("A raft is still generated under tree support", "[TreeSupport]")
{
    // The mesh supports itself, so a layer count alone passes with no raft at all.
    Slic3r::Print rafted, unrafted;
    slice_with_tree_support(scaled(TestMesh::overhang, 2.f), rafted, "tree_slim", 30, 0, 3);
    slice_with_tree_support(scaled(TestMesh::overhang, 2.f), unrafted, "tree_slim", 30, 0, 0);
    const PrintObject *rafted_object   = rafted.objects().front();
    const PrintObject *unrafted_object = unrafted.objects().front();
    REQUIRE(rafted_object->support_layers().size() > unrafted_object->support_layers().size());
    // The raft goes under the object.
    REQUIRE(rafted_object->layers().front()->print_z > unrafted_object->layers().front()->print_z);
}

// drop_nodes() decides the node merges and spawns the next layer's nodes in parallel. Every one of
// those decisions has to be applied in a fixed order, or the same model gives different branches on
// each slice.
TEST_CASE("Tree support toolpaths do not depend on thread scheduling", "[TreeSupport][TreeSupportScheduling][Regression][Parallel]")
{
    // Scaled up so that a layer holds enough nodes for the parallel range to be split. At stock
    // size it stays in one chunk and the order never varies.
    SECTION("overhang")            { sliced_twice_matches(scaled_z(TestMesh::overhang, 2.f, 1.f), 0); }
    SECTION("bridge with hole")    { sliced_twice_matches(scaled_z(TestMesh::bridge_with_hole, 3.f, 1.f), 0); }
    // Build-plate-only filtering retains enough support points to keep the parallel work substantial.
    SECTION("on the build plate")  { sliced_twice_matches(scaled_z(TestMesh::overhang, 4.f, 1.f), 1); }
    // Branches resting on the model are what put nodes in a part group other than 0, which is the
    // only way to reach the prune in the second pass. tree_hybrid additionally builds polygon
    // nodes, so it is the only style that exercises the overhang merge.
    SECTION("resting on the model") { sliced_twice_matches(two_tier_mesh(), 0); }
    SECTION("hybrid on the model")  { sliced_twice_matches(two_tier_mesh(), 0, "tree_hybrid"); }
}

// Prim breaks equal-distance ties by heap address. A 1 mm branch diameter puts neighbours close
// enough to tie, and an explicit line width pins max_move_dist, so the moved tie winner reaches
// the support toolpaths.
TEST_CASE("Tree support toolpaths do not depend on the MST tie order", "[TreeSupport][Regression][Parallel]")
{
    sliced_twice_matches(two_tier_mesh(), 0, "tree_hybrid", {
        { "tree_support_branch_diameter", 1.0 },
        { "tree_support_branch_distance", 5.0 },
        { "tree_support_branch_angle",    40 },
        { "support_line_width",           0.4 },
    });
}
namespace tree_support_interactions_contracts {

struct TreeSupportInteractionEvidence {
    size_t nonempty_layers = 0;
    size_t path_count = 0;
    size_t closed_nonfirst_loop_count = 0;
    size_t first_layer_path_count = 0;
    double first_layer_area_mm2 = 0.0;
    double first_layer_length_mm = 0.0;
    double total_length_mm = 0.0;
    bool finite = true;
};

static bool is_closed_loop(const ExtrusionLoop &loop)
{
    if (loop.paths.empty())
        return false;
    for (const ExtrusionPath &path : loop.paths)
        if (path.polyline.points.size() < 2)
            return false;
    for (size_t i = 1; i < loop.paths.size(); ++i)
        if (loop.paths[i - 1].last_point3() != loop.paths[i].first_point3())
            return false;
    return loop.paths.back().last_point3() == loop.paths.front().first_point3();
}

static size_t closed_loop_count(const ExtrusionEntity &entity)
{
    if (const auto *collection = dynamic_cast<const ExtrusionEntityCollection *>(&entity)) {
        size_t count = 0;
        for (const ExtrusionEntity *child : collection->entities)
            if (child != nullptr)
                count += closed_loop_count(*child);
        return count;
    }
    if (const auto *loop = dynamic_cast<const ExtrusionLoop *>(&entity)) {
        if (is_closed_loop(*loop))
            return 1;
    }
    return 0;
}

static double support_area_mm2(const ExPolygons &polygons)
{
    double area = 0.0;
    for (const ExPolygon &polygon : polygons)
        area += std::abs(polygon.area());
    return unscale<double>(unscale<double>(area));
}

static TreeSupportInteractionEvidence observe_tree_support(const TriangleMesh &mesh, bool auto_brim, int wall_count)
{
    Print print;
    slice_with_tree_support(mesh, print, "tree_slim", 30, 0, 0, {
        { "printable_area",                "-100x-100,200x-100,200x200,-100x200" },
        { "initial_layer_print_height",    0.2 },
        { "tree_support_auto_brim",        auto_brim },
        { "tree_support_brim_width",       16.0 },
        { "tree_support_wall_count",       wall_count },
        { "support_base_pattern",          "rectilinear" },
        { "support_critical_regions_only",  false },
        { "support_remove_small_overhang",  0 },
        { "support_interface_top_layers",  0 },
        { "support_interface_bottom_layers", 0 },
        { "support_threshold_angle",        30 },
        { "support_line_width",              0.4 },
        { "line_width",                      0.4 },
        { "nozzle_diameter",                 "0.4" },
        { "raft_layers",                     0 },
    });
    REQUIRE(print.objects().size() == 1);

    TreeSupportInteractionEvidence result;
    const PrintObject &object = *print.objects().front();
    for (const SupportLayer *layer : object.support_layers()) {
        size_t layer_path_count = 0;
        double layer_length_mm = 0.0;
        for_each_extrusion_path(layer->support_fills, [&](const ExtrusionPath &path) {
            ++layer_path_count;
            const double length_mm = unscale<double>(path.length());
            result.finite = result.finite && std::isfinite(length_mm) && length_mm > 0.0;
            layer_length_mm += length_mm;
        });
        if (layer_path_count == 0)
            continue;

        ++result.nonempty_layers;
        result.path_count += layer_path_count;
        result.total_length_mm += layer_length_mm;
        if (result.first_layer_path_count == 0) {
            result.first_layer_path_count = layer_path_count;
            result.first_layer_area_mm2 = support_area_mm2(layer->base_areas);
            result.first_layer_length_mm = layer_length_mm;
        } else {
            for (const ExtrusionEntity *entity : layer->support_fills.entities)
                if (entity != nullptr)
                    result.closed_nonfirst_loop_count += closed_loop_count(*entity);
        }
    }
    result.finite = result.finite && std::isfinite(result.first_layer_area_mm2) &&
        std::isfinite(result.first_layer_length_mm) && std::isfinite(result.total_length_mm);
    return result;
}

static TriangleMesh interaction_fixture()
{
    return scaled_z(TestMesh::overhang, 1.0f, 0.5f);
}

} // namespace tree_support_interactions_contracts

TEST_CASE("Tree support auto brim and wall loops change public support geometry",
          "[TreeSupport][TreeSupportInteractions]")
{
    using namespace tree_support_interactions_contracts;
    const TriangleMesh mesh = interaction_fixture();
    const TreeSupportInteractionEvidence auto_brim = observe_tree_support(mesh, true, 0);
    const TreeSupportInteractionEvidence explicit_brim = observe_tree_support(mesh, false, 0);
    const TreeSupportInteractionEvidence one_wall = observe_tree_support(mesh, true, 1);
    const TreeSupportInteractionEvidence two_walls = observe_tree_support(mesh, true, 2);

    for (const TreeSupportInteractionEvidence &evidence : { auto_brim, explicit_brim, one_wall, two_walls }) {
        REQUIRE(evidence.finite);
        REQUIRE(evidence.nonempty_layers > 0);
        REQUIRE(evidence.first_layer_path_count > 0);
        REQUIRE(evidence.first_layer_area_mm2 > 0.0);
        REQUIRE(evidence.first_layer_length_mm > 0.0);
        REQUIRE(evidence.path_count > 0);
        REQUIRE(evidence.total_length_mm > 0.0);
    }

    // Auto-brim is bounded by MAX_BRANCH_RADIUS_FIRST_LAYER (12 mm), while the explicit
    // 16 mm setting is applied directly by draw_circles() on the first support layer.
    REQUIRE(explicit_brim.first_layer_area_mm2 > auto_brim.first_layer_area_mm2 * 1.01);
    // wall_count=2 reaches the second _make_loops() offset and therefore emits more support
    // loops than the explicit one-wall mode on this same public fixture. The zero/auto row is
    // retained as a finite positive control; zero explicitly permits infill-only output.
    REQUIRE(one_wall.closed_nonfirst_loop_count > 0);
    REQUIRE(two_walls.closed_nonfirst_loop_count > one_wall.closed_nonfirst_loop_count);
}

namespace belt_tree_landing_contracts {

struct GCodeEvidence {
    size_t support_e_lines = 0;
    size_t support_extrusion_moves = 0;
    size_t support_z_moves = 0;
    size_t positive_z_travels = 0;
    size_t support_e_only_moves = 0;
    size_t quantized_stationary_moves = 0;
    size_t invalid_support_moves = 0;
    double positive_e_mm = 0.0;
    double spatial_distance_mm = 0.0;
    bool finite = true;
};

struct Evidence {
    size_t point_count = 0;
    size_t nonempty_layers = 0;
    size_t zero_material_paths = 0;
    double total_path_mm = 0.0;
    double total_extrusion_mm3 = 0.0;
    double lowest_print_z = std::numeric_limits<double>::max();
    double lowest_plane_gap_mm = std::numeric_limits<double>::max();
    double lowest_nominal_plane_gap_mm = std::numeric_limits<double>::max();
    bool finite = true;
    size_t invalid_paths = 0;
    double first_bad_length_mm = 0.0;
    double first_bad_volume_mm3 = 0.0;
    double first_bad_mm3_per_mm = 0.0;
    double first_bad_width = 0.0;
    double first_bad_height = 0.0;
    double first_bad_print_z = 0.0;
    size_t first_bad_point_count = 0;
    int first_bad_role = 0;
    bool first_bad_force_no_extrusion = false;
    GCodeEvidence gcode;
};

static std::string_view trim_gcode_comment(std::string_view comment)
{
    while (!comment.empty() && std::isspace(static_cast<unsigned char>(comment.front())))
        comment.remove_prefix(1);
    while (!comment.empty() && std::isspace(static_cast<unsigned char>(comment.back())))
        comment.remove_suffix(1);
    return comment;
}

static GCodeEvidence parse_support_gcode(const std::string &gcode, const DynamicPrintConfig &config)
{
    GCodeEvidence result;
    GCodeReader reader;
    reader.apply_config(config);
    std::string active_role;
    reader.parse_buffer(gcode, [&result, &active_role](GCodeReader &state, const GCodeReader::GCodeLine &line) {
        const std::string_view comment = trim_gcode_comment(line.comment());
        if (comment.rfind("TYPE:", 0) == 0)
            active_role.assign(comment.substr(5));

        const bool support_role = active_role == "Support" || active_role == "Support interface" ||
                                  active_role == "Support transition";
        const bool motion = line.cmd_is("G0") || line.cmd_is("G1") || line.cmd_is("G2") || line.cmd_is("G3");
        if (motion && line.dist_Z(state) > EPSILON &&
            (!line.has_e() || line.dist_E(state) <= 0.0))
            ++result.positive_z_travels;
        if (!support_role || !motion || !line.has_e())
            return;

        ++result.support_e_lines;
        const double de = line.dist_E(state);
        const double dx = line.dist_X(state);
        const double dy = line.dist_Y(state);
        const double dz = line.dist_Z(state);
        const double distance = std::hypot(std::hypot(dx, dy), dz);
        const bool finite = std::isfinite(de) && std::isfinite(dx) && std::isfinite(dy) &&
                            std::isfinite(dz) && std::isfinite(distance);
        const bool has_spatial_axis = line.has_x() || line.has_y() || line.has_z();
        result.finite = result.finite && finite;
        if (!finite) {
            ++result.invalid_support_moves;
            return;
        }
        if (de <= 0.0)
            return;
        // E-only unretraction can occur while TYPE:Support remains active. Tiny source
        // segments can also round to identical XYZ coordinates at export precision.
        // Record both separately; neither witnesses spatial support deposition.
        if (!has_spatial_axis) {
            ++result.support_e_only_moves;
            return;
        }
        if (distance <= EPSILON) {
            ++result.quantized_stationary_moves;
            return;
        }

        ++result.support_extrusion_moves;
        result.positive_e_mm += de;
        result.spatial_distance_mm += distance;
        if (std::abs(dz) > EPSILON)
            ++result.support_z_moves;
    });
    return result;
}

static TriangleMesh belt_tree_fixture()
{
    // The same leading-overhang geometry as the existing belt regression, with an explicit
    // transformed fin so support contacts remain in the belt slicing frame.
    TriangleMesh body = make_cube(20.0, 20.0, 20.0);
    TriangleMesh leading = make_cube(20.0, 20.0, 2.0);
    Transform3d shear = Transform3d::Identity();
    shear.matrix() << 1., 0., 0.,   0.,
                      0., 1., 0., -20.,
                      0., -1., 1., 38.,
                      0., 0., 0.,   1.;
    leading.transform(shear);
    body.merge(leading);
    return body;
}

static DynamicPrintConfig belt_tree_config(int wall_count, double floor_offset)
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        { "printable_area",                 "-100x-100,300x-100,300x300,-100x300" },
        { "belt_printer",                   1 },
        { "belt_printer_infinite_y",        1 },
        { "belt_slice_rotation",            "x" },
        { "belt_slice_rotation_angle",      45 },
        { "belt_frame_tilt_decouple",        false },
        { "gcode_remap_x",                  "rev_x" },
        { "gcode_remap_y",                  "pos_z" },
        { "gcode_remap_z",                  "pos_y" },
        { "belt_support_floor_offset",     floor_offset },
        { "nozzle_diameter",               "1.0" },
        { "layer_height",                   0.8 },
        { "initial_layer_print_height",     0.8 },
        { "resolution",                     0.01 },
        { "enable_arc_fitting",             false },
        { "seam_gap",                       0.0 },
        { "min_layer_height",              "0.1" },
        { "max_layer_height",              "0.8" },
        { "initial_layer_line_width",       1.0 },
        { "line_width",                     1.0 },
        { "outer_wall_line_width",          1.0 },
        { "inner_wall_line_width",          1.0 },
        { "sparse_infill_line_width",       1.0 },
        { "internal_solid_infill_line_width", 1.0 },
        { "top_surface_line_width",         1.0 },
        { "skin_infill_line_width",         1.0 },
        { "skeleton_infill_line_width",     1.0 },
        { "bridge_line_width",              1.0 },
        { "skirt_loops",                    0 },
        { "brim_type",                      "no_brim" },
        { "brim_width",                     0.0 },
        { "raft_layers",                    0 },
        { "z_hop",                          0 },
        { "enable_support",                 1 },
        { "support_type",                   "tree(auto)" },
        { "support_style",                  "tree_slim" },
        { "support_on_build_plate_only",    0 },
        { "independent_support_layer_height", false },
        { "support_threshold_angle",        30 },
        { "support_top_z_distance",         0.2 },
        { "support_interface_top_layers",   0 },
        { "support_interface_bottom_layers",0 },
        { "support_base_pattern",            "rectilinear" },
        { "support_line_width",              1.5 },
        { "tree_support_branch_distance",   1.0 },
        { "tree_support_branch_diameter",   1.0 },
        { "tree_support_branch_angle",      60.0 },
        { "tree_support_wall_count",        wall_count },
        { "tree_support_auto_brim",         false },
        { "tree_support_brim_width",        0.0 },
        { "max_bridge_length",              0.0 },
        { "bridge_no_support",              0 },
        { "thick_bridges",                  0 },
        { "machine_start_gcode",            "T[initial_tool]\n" },
        { "layer_change_gcode",             "G92 E0\n" },
        { "use_relative_e_distances",       true },
    });
    return config;
}

static Evidence observe(int wall_count, double floor_offset)
{
    Print print;
    Model model;
    const DynamicPrintConfig config = belt_tree_config(wall_count, floor_offset);
    init_print({ belt_tree_fixture() }, print, model, config);
    REQUIRE(model.objects.size() == 1);
    REQUIRE(model.objects.front()->instances.size() == 1);
    model.objects.front()->instances.front()->set_offset(Vec3d(100., 100., 0.));
    print.apply(model, config);
    const StringObjectException validation = print.validate();
    INFO(validation.string);
    REQUIRE(validation.string.empty());
    print.set_status_silent();
    print.process();

    REQUIRE(print.objects().size() == 1);
    const PrintObject &object = *print.objects().front();
    const auto &slicing = object.slicing_parameters();
    REQUIRE(std::abs(slicing.belt_floor_shear_factor) > EPSILON);
    REQUIRE(slicing.belt_floor_from_axis == 1);
    Evidence result;
    for (const SupportLayer *layer : object.support_layers()) {
        bool layer_has_physical_path = false;
        for_each_extrusion_path(layer->support_fills, [&](const ExtrusionPath &path) {
            const double length_mm = unscale<double>(path.length());
            const double volume_mm3 = path.mm3_per_mm * length_mm;
            const bool finite_values = std::isfinite(length_mm) && std::isfinite(volume_mm3) &&
                std::isfinite(path.mm3_per_mm) && std::isfinite(double(path.width)) &&
                std::isfinite(double(path.height));
            const bool flow_metadata_valid = finite_values && path.mm3_per_mm > 0.0 &&
                path.width > 0.0f && path.height > 0.0f && path.polyline.is_valid();
            const bool nonnegative_length = finite_values && length_mm >= 0.0;
            const bool physical_path = flow_metadata_valid && nonnegative_length &&
                length_mm > 0.0 && volume_mm3 > 0.0;

            // Record invalid metadata before classifying a zero-material path. This keeps
            // negative/nonfinite regressions visible instead of silently filtering them.
            if (!flow_metadata_valid || !nonnegative_length) {
                if (result.invalid_paths == 0) {
                    result.first_bad_length_mm = length_mm;
                    result.first_bad_volume_mm3 = volume_mm3;
                    result.first_bad_mm3_per_mm = path.mm3_per_mm;
                    result.first_bad_width = path.width;
                    result.first_bad_height = path.height;
                    result.first_bad_print_z = layer->print_z;
                    result.first_bad_point_count = path.polyline.points.size();
                    result.first_bad_role = static_cast<int>(path.role());
                    result.first_bad_force_no_extrusion = path.is_force_no_extrusion();
                }
                ++result.invalid_paths;
                result.finite = false;
                return;
            }

            if (!physical_path) {
                ++result.zero_material_paths;
                return;
            }

            result.finite = result.finite && finite_values;
            if (!layer_has_physical_path) {
                layer_has_physical_path = true;
                ++result.nonempty_layers;
                result.lowest_print_z = std::min(result.lowest_print_z, double(layer->print_z));
            }
            result.total_path_mm += length_mm;
            result.total_extrusion_mm3 += volume_mm3;
            result.point_count += path.polyline.points.size();
            for (const Point3 &point : path.polyline.points) {
                const Point xy = point.to_point();
                // For the explicit X/45 degree belt, the source plane equation is
                // shear*Y + z_shift + configured floor offset. This remains an
                // independent test-owned calculation.
                const double axis_position = unscale<double>(xy.y());
                const double nominal_plane_z = slicing.belt_floor_shear_factor * axis_position +
                    slicing.belt_floor_z_shift;
                const double nominal_gap = double(layer->print_z) - nominal_plane_z;
                const double gap = nominal_gap - floor_offset;
                result.finite = result.finite && std::isfinite(gap) && std::isfinite(nominal_gap);
                result.lowest_plane_gap_mm = std::min(result.lowest_plane_gap_mm, gap);
                result.lowest_nominal_plane_gap_mm = std::min(result.lowest_nominal_plane_gap_mm, nominal_gap);
            }
        });
    }

    // Exported support moves are the physical contract. Measure full XYZ distance;
    // the canonical belt transform keeps machine Z fixed during each planar layer.
    result.gcode = parse_support_gcode(Slic3r::Test::gcode(print), config);
    result.finite = result.finite && result.gcode.finite;
    return result;
}

} // namespace belt_tree_landing_contracts

TEST_CASE("Belt tree support floor offset shifts public support landing",
          "[TreeSupport][Belt][BeltTreeLanding]")
{
    using namespace belt_tree_landing_contracts;
    const Evidence one_wall = observe(1, 0.0);
    const Evidence two_walls = observe(2, 0.0);
    const Evidence lower_floor = observe(2, -5.0);
    const Evidence raised_floor = observe(2, 5.0);

    size_t row = 0;
    for (const Evidence &evidence : { one_wall, two_walls, lower_floor, raised_floor }) {
        INFO("row=" << row << " invalid_paths=" << evidence.invalid_paths
             << " zero_material_paths=" << evidence.zero_material_paths
             << " first_bad_length_mm=" << evidence.first_bad_length_mm
             << " first_bad_volume_mm3=" << evidence.first_bad_volume_mm3
             << " mm3_per_mm=" << evidence.first_bad_mm3_per_mm
             << " width=" << evidence.first_bad_width << " height=" << evidence.first_bad_height
             << " print_z=" << evidence.first_bad_print_z
             << " point_count=" << evidence.first_bad_point_count << " role=" << evidence.first_bad_role
             << " support_extrusion_moves=" << evidence.gcode.support_extrusion_moves
             << " support_z_moves=" << evidence.gcode.support_z_moves
             << " positive_z_travels=" << evidence.gcode.positive_z_travels
             << " support_e_only_moves=" << evidence.gcode.support_e_only_moves
             << " quantized_stationary_moves=" << evidence.gcode.quantized_stationary_moves
             << " support_e_lines=" << evidence.gcode.support_e_lines
             << " invalid_support_moves=" << evidence.gcode.invalid_support_moves
             << " positive_e_mm=" << evidence.gcode.positive_e_mm
             << " spatial_distance_mm=" << evidence.gcode.spatial_distance_mm);
        REQUIRE(evidence.finite);
        REQUIRE(evidence.nonempty_layers > 0);
        REQUIRE(evidence.point_count > 2);
        REQUIRE(evidence.total_path_mm > 0.0);
        REQUIRE(evidence.total_extrusion_mm3 > 0.0);
        REQUIRE(std::isfinite(evidence.lowest_print_z));
        REQUIRE(std::isfinite(evidence.lowest_plane_gap_mm));
        REQUIRE(evidence.lowest_plane_gap_mm > -0.8);
        CHECK(evidence.lowest_plane_gap_mm < 0.8 + EPSILON);
        REQUIRE(evidence.gcode.finite);
        REQUIRE(evidence.gcode.support_extrusion_moves > 0);
        // Inverse X/45 rotation, Y/Z swap, then the 45-degree machine shear
        // give machine Z = sqrt(2) * slicing Z. It is constant within each
        // support layer; advancing the belt occurs between layers (z_hop=0).
        CHECK(evidence.gcode.support_z_moves == 0);
        REQUIRE(evidence.gcode.positive_z_travels > 0);
        REQUIRE(evidence.gcode.invalid_support_moves == 0);
        REQUIRE(std::isfinite(evidence.gcode.positive_e_mm));
        REQUIRE(std::isfinite(evidence.gcode.spatial_distance_mm));
        REQUIRE(evidence.gcode.positive_e_mm > 0.0);
        REQUIRE(evidence.gcode.spatial_distance_mm > EPSILON);
        ++row;
    }
    // Wall count changes the emitted support geometry.
    CHECK(std::abs(two_walls.total_path_mm - one_wall.total_path_mm) > 0.05);
    // Compare landing against a fixed plane: branch endpoints may move along the
    // sloped floor, so their raw slicing Z need not change monotonically. Each
    // landing has at most one 0.8 mm layer of discretization, hence 1.6 mm for
    // the difference between two rows. The reference excludes the row's offset.
    CAPTURE(two_walls.lowest_nominal_plane_gap_mm, lower_floor.lowest_nominal_plane_gap_mm,
            raised_floor.lowest_nominal_plane_gap_mm);
    CHECK_THAT(lower_floor.lowest_nominal_plane_gap_mm,
               Catch::Matchers::WithinAbs(two_walls.lowest_nominal_plane_gap_mm - 5.0, 1.6 + EPSILON));
    CHECK_THAT(raised_floor.lowest_nominal_plane_gap_mm,
               Catch::Matchers::WithinAbs(two_walls.lowest_nominal_plane_gap_mm + 5.0, 1.6 + EPSILON));
}

namespace organic_tree_density_contracts {

struct OrganicDensityEvidence {
    size_t nonempty_layers = 0;
    size_t path_count = 0;
    double total_length_mm = 0.0;
    double highest_layer_print_z = -std::numeric_limits<double>::infinity();
    double highest_layer_coverage_area_mm2 = 0.0;
    bool finite = true;
};

// Keep the broad, flat cap and narrow vertical stem small enough for a focused test while
// leaving enough layers for organic branch propagation. The highest support layer is the same
// model layer for every row because geometry and all support-height settings are held constant.
static TriangleMesh cap_overhang_fixture()
{
    TriangleMesh model = make_cube(24.0, 24.0, 1.0);
    TriangleMesh stem  = make_cube(6.0, 6.0, 6.0);
    stem.translate(9.0, 9.0, 0.5);
    TriangleMesh cap   = make_cube(24.0, 24.0, 0.8);
    cap.translate(0.0, 0.0, 5.8);
    model.merge(stem);
    model.merge(cap);
    return model;
}

static double coverage_area_mm2(const Polygons &covered)
{
    const ExPolygons merged = union_ex(covered);
    double scaled_area = 0.0;
    for (const ExPolygon &polygon : merged)
        scaled_area += std::abs(polygon.area());
    return unscale<double>(unscale<double>(scaled_area));
}

static OrganicDensityEvidence observe(const char *top_rate, double branch_distance)
{
    Print print;
    slice_with_tree_support(cap_overhang_fixture(), print, "organic", 30, 0, 0, {
        { "printable_area",                    "-100x-100,200x-100,200x200,-100x200" },
        { "initial_layer_print_height",        0.2 },
        { "skirt_loops",                       0 },
        { "brim_type",                         "no_brim" },
        { "brim_width",                        0.0 },
        { "tree_support_auto_brim",            false },
        { "tree_support_brim_width",           0.0 },
        { "tree_support_top_rate",             top_rate },
        { "tree_support_branch_distance_organic", branch_distance },
        { "tree_support_branch_diameter_angle", 5.0 },
        { "tree_support_tip_diameter",          0.8 },
        { "tree_support_branch_diameter_organic", 2.0 },
        { "tree_support_wall_count",            1 },
        { "support_base_pattern",               "hollow" },
        { "support_interface_top_layers",       0 },
        { "support_interface_bottom_layers",    0 },
        { "support_on_build_plate_only",        0 },
        { "support_threshold_angle",            30 },
        { "support_top_z_distance",             0.2 },
        { "support_line_width",                 0.4 },
        { "line_width",                         0.4 },
        { "nozzle_diameter",                    "0.4" },
        { "enable_arc_fitting",                false },
        { "resolution",                         0.01 },
        { "raft_layers",                        0 },
    });

    OrganicDensityEvidence result;
    for (const SupportLayer *layer : print.objects().front()->support_layers()) {
        if (layer == nullptr || !std::isfinite(double(layer->print_z))) {
            result.finite = false;
            continue;
        }

        size_t layer_paths = 0;
        for_each_extrusion_path(layer->support_fills, [&](const ExtrusionPath &path) {
            const double length_mm = unscale<double>(path.length());
            ++layer_paths;
            ++result.path_count;
            result.total_length_mm += length_mm;
            if (!std::isfinite(length_mm) || length_mm <= 0.0)
                result.finite = false;
        });
        if (layer_paths == 0)
            continue;
        ++result.nonempty_layers;

        result.highest_layer_print_z = std::max(result.highest_layer_print_z, double(layer->print_z));
    }

    // Offset only the top layer's paths; lower layers do not contribute to this oracle.
    Polygons highest_coverage;
    for (const SupportLayer *layer : print.objects().front()->support_layers())
        if (layer != nullptr && std::abs(layer->print_z - result.highest_layer_print_z) <= EPSILON)
            layer->support_fills.polygons_covered_by_width(highest_coverage, float(SCALED_EPSILON));
    result.highest_layer_coverage_area_mm2 = coverage_area_mm2(highest_coverage);
    result.finite = result.finite && std::isfinite(result.total_length_mm);
    result.finite = result.finite && std::isfinite(result.highest_layer_print_z);
    result.finite = result.finite && std::isfinite(result.highest_layer_coverage_area_mm2);
    return result;
}

} // namespace organic_tree_density_contracts

TEST_CASE("Organic tree top rate changes physical highest-layer support coverage",
          "[TreeSupport][OrganicTreeDensity]")
{
    using namespace organic_tree_density_contracts;
    const OrganicDensityEvidence sparse = observe("5%", 1.0);
    const OrganicDensityEvidence dense  = observe("35%", 1.0);

    for (const OrganicDensityEvidence &evidence : { sparse, dense }) {
        REQUIRE(evidence.finite);
        REQUIRE(evidence.nonempty_layers > 0);
        REQUIRE(evidence.path_count > 0);
        REQUIRE(evidence.total_length_mm > 0.0);
        REQUIRE(std::isfinite(evidence.highest_layer_print_z));
        REQUIRE(evidence.highest_layer_print_z > 0.0);
        REQUIRE(evidence.highest_layer_coverage_area_mm2 > 0.0);
    }
    CHECK_THAT(dense.highest_layer_print_z,
               Catch::Matchers::WithinAbs(sparse.highest_layer_print_z, EPSILON));
    // TreeSupport3D derives a shorter initial connect length from the higher top-rate value.
    // Unioned path-width coverage measures physical support footprint, independent of path
    // tessellation or ordering.
    REQUIRE(dense.highest_layer_coverage_area_mm2 > sparse.highest_layer_coverage_area_mm2 + 0.01);
}

TEST_CASE("Organic tree branch distance changes physical highest-layer support coverage",
          "[TreeSupport][OrganicTreeDensity]")
{
    using namespace organic_tree_density_contracts;
    const OrganicDensityEvidence close = observe("30%", 1.0);
    const OrganicDensityEvidence far   = observe("30%", 10.0);

    for (const OrganicDensityEvidence &evidence : { close, far }) {
        REQUIRE(evidence.finite);
        REQUIRE(evidence.nonempty_layers > 0);
        REQUIRE(evidence.path_count > 0);
        REQUIRE(evidence.total_length_mm > 0.0);
        REQUIRE(std::isfinite(evidence.highest_layer_print_z));
        REQUIRE(evidence.highest_layer_print_z > 0.0);
        REQUIRE(evidence.highest_layer_coverage_area_mm2 > 0.0);
    }
    CHECK_THAT(close.highest_layer_print_z,
               Catch::Matchers::WithinAbs(far.highest_layer_print_z, EPSILON));
    // The non-roof organic path uses branch_distance_organic for line spacing, so the close
    // setting must retain a larger physical footprint on the same cap.
    REQUIRE(close.highest_layer_coverage_area_mm2 > far.highest_layer_coverage_area_mm2 + 0.01);
}


namespace tree_support_thick_bridge_contracts {

struct Evidence {
    size_t support_points = 0;
    size_t positive_paths = 0;
    size_t bridge_segments = 0;
    size_t bridge_surface_count = 0;
    double bridge_support_area_mm2 = 0.0;
    double total_length_mm = 0.0;
    bool finite = true;
};

static double expolygons_area_mm2(const ExPolygons &polygons)
{
    double scaled_area = 0.0;
    for (const ExPolygon &polygon : polygons)
        scaled_area += std::abs(polygon.area());
    return unscale<double>(unscale<double>(scaled_area));
}

// The 20 mm roof is supported only at its two four-millimetre end pillars.  Its
// x=4..16 middle window is an analytic, horizontally unsupported bridge region.
static TriangleMesh thick_bridge_fixture()
{
    TriangleMesh result = make_cube(4.0, 8.0, 4.0);
    TriangleMesh right_pillar = make_cube(4.0, 8.0, 4.0);
    right_pillar.translate(16.0, 0.0, 0.0);
    result.merge(right_pillar);

    // Overlap the pillars by one layer to avoid a coplanar mesh join.
    TriangleMesh roof = make_cube(20.0, 8.0, 0.6);
    roof.translate(0.0, 0.0, 3.8);
    result.merge(roof);
    return result;
}

static Evidence observe(bool thick_bridges)
{
    Print print;
    slice_with_tree_support(thick_bridge_fixture(), print, "tree_slim", 30, 0, 0, {
        { "printable_area",                "-100x-100,200x-100,200x200,-100x200" },
        { "support_top_z_distance",        0.2 },
        { "support_interface_top_layers",  0 },
        { "support_interface_bottom_layers", 0 },
        { "support_remove_small_overhang", 0 },
        { "support_critical_regions_only", false },
        { "support_on_build_plate_only",   0 },
        { "bridge_no_support",              0 },
        { "max_bridge_length",              0.0 },
        // A round bridge is about 0.447 mm thick at this flow, extending below
        // the 0.4 mm layer-plus-gap boundary so trimming has a positive margin.
        { "bridge_flow",                    1.25 },
        { "thick_bridges",                  thick_bridges },
        { "support_line_width",             0.4 },
        { "line_width",                     0.4 },
        { "outer_wall_line_width",          0.4 },
        { "inner_wall_line_width",          0.4 },
        { "nozzle_diameter",                "0.4" },
        { "brim_type",                      "no_brim" },
        { "brim_width",                     0.0 },
        { "skirt_loops",                    0 },
        { "raft_layers",                    0 },
        { "enable_arc_fitting",             false },
    });

    REQUIRE(print.objects().size() == 1);
    const PrintObject &object = *print.objects().front();
    Evidence result;
    result.support_points = support_points(print).size();
    const Point &center_offset = object.center_offset();
    result.bridge_segments = tree_support_critical_regions_contracts::support_segments_in_window(
        print, center_offset, 4.5, 15.5, 0.5, 7.5);
    const ExPolygon bridge_window({ Point::new_scale(4.5, 0.5) - center_offset,
                                    Point::new_scale(15.5, 0.5) - center_offset,
                                    Point::new_scale(15.5, 7.5) - center_offset,
                                    Point::new_scale(4.5, 7.5) - center_offset });
    // The fixture's roof middle must be represented as a public bottom-bridge
    // surface before the support trimming oracle is meaningful.
    for (const Layer *layer : object.layers())
        for (const LayerRegion *region : layer->regions())
            for (const Surface &surface : region->fill_surfaces.surfaces)
                if (surface.surface_type == stBottomBridge &&
                    expolygons_area_mm2(intersection_ex(ExPolygons{ surface.expolygon }, bridge_window)) > 0.0)
                    ++result.bridge_surface_count;
    for (const SupportLayer *layer : object.support_layers()) {
        if (layer == nullptr)
            result.finite = false;
        else {
            // Legacy tree support exposes its footprint through extrusion paths.
            Polygons bridge_covered;
            layer->support_fills.polygons_covered_by_width(bridge_covered, float(SCALED_EPSILON));
            result.bridge_support_area_mm2 +=
                expolygons_area_mm2(intersection_ex(union_ex(bridge_covered), bridge_window));
            for_each_extrusion_path(layer->support_fills, [&result](const ExtrusionPath &path) {
                const double length_mm = unscale<double>(path.length());
                result.finite = result.finite && std::isfinite(length_mm);
                if (std::isfinite(length_mm) && length_mm > 0.0) {
                    ++result.positive_paths;
                    result.total_length_mm += length_mm;
                }
            });
        }
    }
    result.finite = result.finite && std::isfinite(result.total_length_mm);
    return result;
}

} // namespace tree_support_thick_bridge_contracts

TEST_CASE("Tree support thick bridges change trimming under a positive gap",
          "[TreeSupport][ThickBridges][SupportSettingsVariance]")
{
    using namespace tree_support_thick_bridge_contracts;
    const Evidence regular = observe(false);
    const Evidence thick   = observe(true);
    INFO("bridge support area: regular=" << regular.bridge_support_area_mm2
         << " thick=" << thick.bridge_support_area_mm2
         << "; intersecting segments: regular=" << regular.bridge_segments
         << " thick=" << thick.bridge_segments);

    for (const Evidence &evidence : {regular, thick}) {
        REQUIRE(evidence.finite);
        REQUIRE(evidence.support_points > 0);
        REQUIRE(evidence.positive_paths > 0);
        REQUIRE(evidence.total_length_mm > 0.0);
        REQUIRE(evidence.bridge_surface_count > 0);
        REQUIRE(std::isfinite(evidence.bridge_support_area_mm2));
        REQUIRE(evidence.bridge_support_area_mm2 >= 0.0);
    }
    REQUIRE(regular.bridge_segments > 0);
    REQUIRE(regular.bridge_support_area_mm2 > 0.0);
    // The analytic x=4..16 roof middle is stBottomBridge.  With a positive
    // top gap, TreeSupport adds that surface to the trimming polygons when
    // thick_bridges is enabled, so the public extruded support area in the
    // bridge window must decrease. Path count alone cannot observe this.
    REQUIRE(thick.bridge_support_area_mm2 + 0.01 < regular.bridge_support_area_mm2);
}

namespace tree_support_raft_contact_contracts {

struct RaftContactEvidence {
    double contact_z = std::numeric_limits<double>::infinity();
    double footprint_area_mm2 = 0.0;
    double footprint_width_mm = 0.0;
    double footprint_height_mm = 0.0;
    bool finite = true;
};

static RaftContactEvidence observe(double raft_expansion)
{
    Print print;
    // Isolate the raft contact: overhanging caps also contribute tree bases to
    // support_islands, which can mask the expanded contact's outer bounds.
    slice_with_tree_support(make_cube(24.0, 24.0, 2.0), print, "organic", 30, 0, 3, {
        { "printable_area",                  "-100x-100,200x-100,200x200,-100x200" },
        { "support_on_build_plate_only",     1 },
        { "support_top_z_distance",          0.2 },
        { "support_interface_top_layers",    0 },
        { "support_interface_bottom_layers", 0 },
        { "support_remove_small_overhang",   0 },
        { "tree_support_auto_brim",          false },
        { "tree_support_brim_width",         0.0 },
        { "raft_expansion",                  raft_expansion },
        { "support_line_width",              0.4 },
        { "line_width",                      0.4 },
        { "nozzle_diameter",                 "0.4" },
        { "skirt_loops",                     0 },
        { "brim_width",                      0.0 },
        { "enable_arc_fitting",              false },
    });

    REQUIRE(print.objects().size() == 1);
    const PrintObject &object = *print.objects().front();
    RaftContactEvidence result;
    std::vector<ExPolygons> footprints;
    const double expected_contact_z = double(object.slicing_parameters().raft_contact_top_z);
    for (const SupportLayer *layer : object.support_layers()) {
        if (layer == nullptr || layer->support_islands.empty())
            continue;
        if (std::abs(double(layer->print_z) - expected_contact_z) > EPSILON)
            continue;
        footprints.push_back(layer->support_islands);
    }

    REQUIRE(std::isfinite(expected_contact_z));
    result.contact_z = expected_contact_z;
    REQUIRE(!footprints.empty());
    ExPolygons all_covered;
    for (const ExPolygons &footprint : footprints)
        all_covered.insert(all_covered.end(), footprint.begin(), footprint.end());
    const ExPolygons merged = union_ex(all_covered);
    REQUIRE(!merged.empty());
    double scaled_area = 0.0;
    for (const ExPolygon &polygon : merged)
        scaled_area += std::abs(polygon.area());
    const BoundingBox bbox = get_extents(merged);
    REQUIRE(bbox.defined);
    result.footprint_area_mm2 = unscale<double>(unscale<double>(scaled_area));
    result.footprint_width_mm = unscale<double>(bbox.max.x() - bbox.min.x());
    result.footprint_height_mm = unscale<double>(bbox.max.y() - bbox.min.y());
    result.finite = std::isfinite(result.footprint_area_mm2) &&
                    std::isfinite(result.footprint_width_mm) &&
                    std::isfinite(result.footprint_height_mm);
    return result;
}

} // namespace tree_support_raft_contact_contracts

TEST_CASE("Organic tree raft expansion changes the public contact footprint",
          "[TreeSupport][TreeSupport3D][TreeSupportRaftContact]")
{
    using namespace tree_support_raft_contact_contracts;
    const RaftContactEvidence unexpanded = observe(0.0);
    const RaftContactEvidence expanded   = observe(3.0);

    for (const RaftContactEvidence &evidence : {unexpanded, expanded}) {
        REQUIRE(evidence.finite);
        REQUIRE(evidence.footprint_area_mm2 > 0.0);
        REQUIRE(evidence.footprint_width_mm > 0.0);
        REQUIRE(evidence.footprint_height_mm > 0.0);
    }
    CHECK_THAT(expanded.contact_z, Catch::Matchers::WithinAbs(unexpanded.contact_z, EPSILON));
    // Expanding each side of the square contact by 3 mm adds 6 mm to both
    // dimensions; any common raft inflation cancels in the paired difference.
    REQUIRE_THAT(expanded.footprint_width_mm - unexpanded.footprint_width_mm,
                 Catch::Matchers::WithinAbs(6.0, 0.05));
    REQUIRE_THAT(expanded.footprint_height_mm - unexpanded.footprint_height_mm,
                 Catch::Matchers::WithinAbs(6.0, 0.05));
    REQUIRE(expanded.footprint_area_mm2 > unexpanded.footprint_area_mm2 * 1.15);
}

namespace tree_support_independent_layer_height_contracts {

struct LayerHeightEvidence {
    std::size_t active_layers = 0;
    std::size_t positive_layers = 0;
    std::size_t non_nominal_layers = 0;
    double max_height_delta = 0.0;
    bool finite = true;
};

static TriangleMesh compact_two_tier_mesh()
{
    TriangleMesh mesh = two_tier_mesh();
    mesh.scale(Vec3f(1.f, 1.f, 0.25f));
    return mesh;
}

static LayerHeightEvidence observe(bool independent)
{
    Print print;
    slice_with_tree_support(compact_two_tier_mesh(), print, "tree_slim", 30, 1, 0, {
        { "support_top_z_distance",          0.3 },
        { "initial_layer_print_height",       0.2 },
        { "independent_support_layer_height", independent },
        { "min_layer_height",                0.1 },
        { "max_layer_height",                0.35 },
        { "support_remove_small_overhang",   0 },
        { "support_interface_top_layers",    0 },
        { "support_interface_bottom_layers", 0 },
        { "tree_support_auto_brim",          false },
        { "tree_support_brim_width",         0.0 },
        { "support_line_width",              0.4 },
        { "line_width",                      0.4 },
        { "outer_wall_line_width",           0.4 },
        { "inner_wall_line_width",           0.4 },
        { "nozzle_diameter",                 "0.4" },
        { "skirt_loops",                     0 },
        { "brim_width",                      0.0 },
        { "enable_arc_fitting",              false },
    });

    REQUIRE(print.objects().size() == 1);
    const PrintObject &object = *print.objects().front();
    LayerHeightEvidence result;
    double previous_z = -std::numeric_limits<double>::infinity();
    for (const SupportLayer *layer : object.support_layers()) {
        REQUIRE(layer != nullptr);
        const double height = static_cast<double>(layer->height);
        const double print_z = static_cast<double>(layer->print_z);
        result.finite = result.finite && std::isfinite(height) && std::isfinite(print_z);
        if (height <= EPSILON || print_z <= EPSILON)
            continue;
        ++result.positive_layers;
        REQUIRE(print_z > previous_z);
        previous_z = print_z;
        REQUIRE(height >= 0.1 - 1e-5);
        REQUIRE(height <= 0.35 + 1e-5);
        result.max_height_delta = std::max(result.max_height_delta, std::abs(height - 0.2));
        if (std::abs(height - 0.2) > 0.02)
            ++result.non_nominal_layers;
        if (layer->has_extrusions())
            ++result.active_layers;
    }
    REQUIRE(result.positive_layers > 0);
    REQUIRE(result.active_layers > 0);
    REQUIRE(result.finite);
    return result;
}

} // namespace tree_support_independent_layer_height_contracts

TEST_CASE("Tree support independent layer height preserves a non-grid support interval",
          "[TreeSupport][TreeSupportLayerHeight][SupportSettingsVariance]")
{
    using namespace tree_support_independent_layer_height_contracts;
    const LayerHeightEvidence independent = observe(true);
    const LayerHeightEvidence synchronized = observe(false);

    REQUIRE(independent.finite);
    REQUIRE(synchronized.finite);
    REQUIRE(independent.active_layers > 0);
    REQUIRE(synchronized.active_layers > 0);
    // The compact two-tier fixture is the existing two-tier geometry scaled only in Z by 0.25: its
    // upper roof starts at z=4.0, so the 0.3 mm contact interval is separated from the build plate
    // without spending the original 16 mm of object height. The independent planner may split that
    // interval at its requested top gap, while the synchronized planner retains the pinned 0.2 mm grid.
    REQUIRE(independent.non_nominal_layers > 0);
    REQUIRE(independent.max_height_delta > 0.05);
    CHECK(synchronized.non_nominal_layers == 0);
    CHECK(synchronized.max_height_delta <= 0.02);
}


namespace tree_support_organic_profile_contracts {

struct SupportSlice {
    double print_z_mm = 0.0;
    ExPolygons islands;
};

struct ProfileEvidence {
    std::vector<SupportSlice> slices;
    size_t physical_extrusion_paths = 0;
    double highest_support_z = -std::numeric_limits<double>::infinity();
    double top_contact_area_mm2 = 0.0;
    double largest_lower_branch_area_mm2 = 0.0;
    size_t active_layers = 0;
};

static double support_area_mm2(const ExPolygons &islands)
{
    double scaled_area = 0.0;
    for (const ExPolygon &island : islands)
        scaled_area += std::abs(island.area());
    return unscale<double>(unscale<double>(scaled_area));
}

static constexpr double lower_region_ceiling_mm = 5.8 - 0.2 - 1.0;

// Read public support slices on the existing cap-overhang fixture. The highest
// island area is the contact footprint; lower islands expose the branch cross-section.
// The branch comparison stays at least 1 mm below the cap's 5.8 mm bottom minus the
// configured 0.2 mm contact gap, outside the region shaped by the tip diameter.
static ProfileEvidence observe(double branch_diameter_mm, double tip_diameter_mm)
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        { "enable_support",                  1 },
        { "support_type",                    "tree(auto)" },
        { "support_style",                   "organic" },
        { "use_relative_e_distances",         true },
        { "before_layer_change_gcode",       "" },
        { "layer_change_gcode",              "G92 E0\n" },
        { "support_on_build_plate_only",     0 },
        { "support_top_z_distance",          0.2 },
        { "support_line_width",              0.4 },
        { "line_width",                      0.4 },
        { "nozzle_diameter",                 "0.4" },
        { "brim_type",                       "no_brim" },
        { "brim_width",                      0.0 },
        { "tree_support_brim_width",         0.0 },
        { "tree_support_auto_brim",          false },
        { "tree_support_top_rate",           "10%" },
        { "tree_support_branch_distance_organic", 5.0 },
        { "tree_support_branch_angle_organic", 40.0 },
        { "tree_support_branch_diameter_organic", branch_diameter_mm },
        { "tree_support_branch_diameter_angle", 5.0 },
        { "tree_support_tip_diameter",       tip_diameter_mm },
        { "tree_support_wall_count",         1 },
        { "support_base_pattern",            "hollow" },
        { "support_interface_top_layers",    0 },
        { "support_interface_bottom_layers", 0 },
        { "initial_layer_print_height",      0.2 },
        { "layer_height",                   0.2 },
        { "skirt_loops",                     0 },
        { "enable_arc_fitting",              false },
        { "resolution",                      0.01 },
        { "raft_layers",                     0 },
    });

    Print print;
    Model model;
    std::vector<TriangleMesh> meshes;
    meshes.emplace_back(organic_tree_density_contracts::cap_overhang_fixture());
    init_print(std::move(meshes), print, model, config, nullptr, false);
    const StringObjectException validation = print.validate();
    INFO(validation.string);
    REQUIRE(validation.string.empty());
    print.process();

    REQUIRE(print.objects().size() == 1);
    ProfileEvidence result;
    const auto layers = print.objects().front()->support_layers();
    for (const SupportLayer *layer : layers) {
        REQUIRE(layer != nullptr);
        if (layer->support_islands.empty())
            continue;
        const double z = static_cast<double>(layer->print_z);
        result.slices.push_back({z, layer->support_islands});
        result.highest_support_z = std::max(result.highest_support_z, z);
        ++result.active_layers;
        if (z > 0.0 && z < lower_region_ceiling_mm)
            result.largest_lower_branch_area_mm2 = std::max(
                result.largest_lower_branch_area_mm2, support_area_mm2(layer->support_islands));
        for_each_extrusion_path(layer->support_fills, [&result](const ExtrusionPath &path) {
            if (path.length() > 0 && path.mm3_per_mm > 0.0)
                ++result.physical_extrusion_paths;
        });
    }
    std::stable_sort(result.slices.begin(), result.slices.end(), [](const SupportSlice &lhs, const SupportSlice &rhs) {
        return lhs.print_z_mm < rhs.print_z_mm;
    });
    REQUIRE_FALSE(result.slices.empty());
    result.top_contact_area_mm2 = support_area_mm2(result.slices.back().islands);
    return result;
}

static double sliced_geometry_difference_mm2(const ProfileEvidence &lhs, const ProfileEvidence &rhs,
                                             double ceiling_mm = std::numeric_limits<double>::infinity())
{
    double total_difference_mm2 = 0.0;
    size_t left = 0, right = 0;
    while (left < lhs.slices.size() || right < rhs.slices.size()) {
        if (right == rhs.slices.size() ||
            (left < lhs.slices.size() && lhs.slices[left].print_z_mm < rhs.slices[right].print_z_mm - EPSILON)) {
            const SupportSlice &slice = lhs.slices[left++];
            if (slice.print_z_mm < ceiling_mm)
                total_difference_mm2 += support_area_mm2(slice.islands);
        } else if (left == lhs.slices.size() ||
                   rhs.slices[right].print_z_mm < lhs.slices[left].print_z_mm - EPSILON) {
            const SupportSlice &slice = rhs.slices[right++];
            if (slice.print_z_mm < ceiling_mm)
                total_difference_mm2 += support_area_mm2(slice.islands);
        } else {
            const SupportSlice &a = lhs.slices[left++];
            const SupportSlice &b = rhs.slices[right++];
            if (a.print_z_mm < ceiling_mm)
                total_difference_mm2 += support_area_mm2(xor_ex(a.islands, b.islands));
        }
    }
    return total_difference_mm2;
}

} // namespace tree_support_organic_profile_contracts

TEST_CASE("Organic branch diameter changes lower public support geometry",
          "[TreeSupport][TreeSupportOrganicProfileSettings][SupportSettingsVariance]")
{
    using namespace tree_support_organic_profile_contracts;
    const ProfileEvidence narrow = observe(1.0, 0.8);
    const ProfileEvidence wide   = observe(4.0, 0.8);

    REQUIRE(narrow.active_layers > 0);
    REQUIRE(narrow.physical_extrusion_paths > 0);
    REQUIRE(narrow.top_contact_area_mm2 > 0.0);
    REQUIRE(narrow.largest_lower_branch_area_mm2 > 0.0);
    REQUIRE(wide.active_layers > 0);
    REQUIRE(wide.physical_extrusion_paths > 0);
    REQUIRE(wide.top_contact_area_mm2 > 0.0);
    REQUIRE(wide.largest_lower_branch_area_mm2 > 0.0);
    CHECK_THAT(narrow.highest_support_z, Catch::Matchers::WithinAbs(wide.highest_support_z, EPSILON));
    // The radius also changes branch merging and routing, so total island area need
    // not grow monotonically. With the same tip and angle, the lower public geometry
    // must still respond to the configured branch diameter, beyond a numerical sliver.
    REQUIRE(sliced_geometry_difference_mm2(wide, narrow, lower_region_ceiling_mm) > 0.01);
}

TEST_CASE("Organic tip diameter changes valid public support contact geometry",
          "[TreeSupport][TreeSupportOrganicProfileSettings][SupportSettingsVariance]")
{
    using namespace tree_support_organic_profile_contracts;
    const ProfileEvidence narrow_tip = observe(2.0, 0.4);
    const ProfileEvidence wide_tip   = observe(2.0, 1.6);

    REQUIRE(narrow_tip.active_layers > 0);
    REQUIRE(narrow_tip.physical_extrusion_paths > 0);
    REQUIRE(narrow_tip.top_contact_area_mm2 > 0.0);
    REQUIRE(wide_tip.active_layers > 0);
    REQUIRE(wide_tip.physical_extrusion_paths > 0);
    REQUIRE(wide_tip.top_contact_area_mm2 > 0.0);
    CHECK_THAT(wide_tip.highest_support_z, Catch::Matchers::WithinAbs(narrow_tip.highest_support_z, EPSILON));
    // Both rows pass print validation: the smaller tip equals the 0.4 mm extrusion width,
    // and both tips remain below the 2 mm branch diameter. Compare public slices at
    // matching Z (unmatched slices differ from empty geometry), then check the top
    // contact footprint grows for the larger tip.
    REQUIRE(sliced_geometry_difference_mm2(wide_tip, narrow_tip) > 0.01);
    REQUIRE(wide_tip.top_contact_area_mm2 > narrow_tip.top_contact_area_mm2 + 0.01);
}


namespace tree_support_vertical_enforcer_contracts {

struct Evidence {
    size_t local_paths = 0;
    size_t support_paths_near_target_z = 0;
    double expected_x_mm = 0.0;
    double expected_y_mm = 0.0;
    double expected_z_mm = 0.0;
};

static Evidence observe(bool paint_vertical_facet)
{
    Model model = Slic3r::Test::model("tree-vertical-enforcer", make_cube(20.0, 20.0, 20.0));
    REQUIRE(model.objects.size() == 1);
    ModelObject *model_object = model.objects.front();
    REQUIRE(model_object->volumes.size() == 1);
    REQUIRE(model_object->instances.size() == 1);
    ModelVolume *volume = model_object->volumes.front();

    const indexed_triangle_set &mesh = volume->mesh().its;
    const BoundingBoxf3 mesh_bounds = volume->mesh().bounding_box();
    const Transform3d model_trafo = model_object->instances.front()->get_matrix() * volume->get_matrix();
    int selected_facet = -1;
    Vec3d expected_point = Vec3d::Zero();
    for (size_t facet = 0; facet < mesh.indices.size(); ++facet) {
        const stl_triangle_vertex_indices &tri = mesh.indices[facet];
        const Vec3f &a = mesh.vertices[tri(0)];
        const Vec3f &b = mesh.vertices[tri(1)];
        const Vec3f &c = mesh.vertices[tri(2)];
        const Vec3f normal = (b - a).cross(c - a);
        const bool on_left_face = std::abs(a.x() - mesh_bounds.min.x()) < 1e-5f &&
                                  std::abs(b.x() - mesh_bounds.min.x()) < 1e-5f &&
                                  std::abs(c.x() - mesh_bounds.min.x()) < 1e-5f;
        if (!on_left_face || std::abs(normal.x()) < 1e-5f ||
            std::abs(normal.y()) > 1e-5f || std::abs(normal.z()) > 1e-5f)
            continue;

        const Vec3d centroid = ((a + b + c) / 3.f).cast<double>();
        const Vec3d world = model_trafo * centroid;
        if (world.z() < 5.0 || world.z() > 9.0)
            continue;
        selected_facet = int(facet);
        expected_point = world;
        break;
    }
    REQUIRE(selected_facet >= 0);
    if (paint_vertical_facet) {
        TriangleSelector selector(volume->mesh());
        selector.set_facet(selected_facet, EnforcerBlockerType::ENFORCER);
        REQUIRE(volume->supported_facets.set(selector));
    }

    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        { "printable_area",               "-100x-100,200x-100,200x200,-100x200" },
        { "enable_support",                1 },
        { "support_type",                  "tree(auto)" },
        { "support_style",                 "tree_slim" },
        { "support_on_build_plate_only",   0 },
        { "support_threshold_angle",       30 },
        { "support_remove_small_overhang", 0 },
        { "support_top_z_distance",        0.2 },
        { "support_interface_top_layers",  0 },
        { "support_interface_bottom_layers", 0 },
        { "tree_support_branch_distance",  5.0 },
        { "tree_support_branch_diameter",  2.0 },
        { "tree_support_branch_angle",     40.0 },
        { "support_line_width",            0.4 },
        { "line_width",                    0.4 },
        { "nozzle_diameter",               "0.4" },
        { "initial_layer_print_height",    0.2 },
        { "layer_height",                  0.2 },
        { "use_relative_e_distances",        true },
        { "layer_change_gcode",              "G92 E0\n" },
        { "raft_layers",                   0 },
        { "brim_type",                     "no_brim" },
        { "brim_width",                    0.0 },
        { "skirt_loops",                   0 },
    });

    Print print;
    model_object->ensure_on_bed();
    print.auto_assign_extruders(model_object);
    print.apply(model, config);
    const StringObjectException validation = print.validate();
    INFO(validation.string);
    REQUIRE(validation.string.empty());
    print.set_status_silent();
    print.process();

    REQUIRE(print.objects().size() == 1);
    const PrintObject &print_object = *print.objects().front();
    Evidence result;
    result.expected_x_mm = expected_point.x();
    result.expected_y_mm = expected_point.y();
    result.expected_z_mm = expected_point.z();
    const Point &center_offset = print_object.center_offset();
    for (const SupportLayer *layer : print_object.support_layers()) {
        if (std::abs(double(layer->print_z) - result.expected_z_mm) > 1.0)
            continue;
        for_each_extrusion_path(layer->support_fills, [&](const ExtrusionPath &path) {
            ++result.support_paths_near_target_z;
            for (size_t idx = 1; idx < path.polyline.points.size(); ++idx) {
                const Point3 &a = path.polyline.points[idx - 1];
                const Point3 &b = path.polyline.points[idx];
                const double segment_z = double(layer->print_z) + unscale<double>(a.z());
                if (std::abs(segment_z - result.expected_z_mm) <= 1.0 &&
                    tree_support_critical_regions_contracts::segment_intersects_xy_window(
                        a, b, center_offset,
                        result.expected_x_mm - 2.0, result.expected_x_mm + 2.0,
                        result.expected_y_mm - 2.0, result.expected_y_mm + 2.0)) {
                    ++result.local_paths;
                    break;
                }
            }
        });
    }
    return result;
}

} // namespace tree_support_vertical_enforcer_contracts

TEST_CASE("Painted vertical enforcers create support at the selected facet",
          "[TreeSupport][TreeSupportPaintedEnforcer]")
{
    using namespace tree_support_vertical_enforcer_contracts;
    const Evidence plain = observe(false);
    const Evidence painted = observe(true);

    REQUIRE(plain.local_paths == 0);
    REQUIRE(painted.support_paths_near_target_z > 0);
    REQUIRE(painted.local_paths > 0);
    CHECK_THAT(painted.expected_x_mm, Catch::Matchers::WithinAbs(0.0, 1e-4));
    CHECK(painted.expected_z_mm > 5.0);
    CHECK(painted.expected_z_mm < 9.0);
}

namespace tree_support_bottom_interface_contracts {

struct ContactRow {
    double bottom_z_mm = 0.0;
    size_t base_role_paths = 0;
    size_t interface_role_paths = 0;
    double longest_interface_segment_mm = 0.0;
    double interface_angle_radians = 0.0;
    bool has_interface_angle = false;
};

struct Evidence {
    size_t contact_interface_rows = 0;
    size_t alternating_direction_pairs = 0;
    size_t last_base_interface_paths = 0;
    std::vector<ContactRow> rows;
};

static Evidence observe()
{
    DynamicPrintConfig config = Slic3r::Test::multifilament_config(3, {
        { "printable_area",                 "-100x-100,200x-100,200x200,-100x200" },
        { "enable_support",                 1 },
        { "support_type",                   "tree(auto)" },
        { "support_style",                  "tree_slim" },
        { "support_on_build_plate_only",    0 },
        { "independent_support_layer_height", false },
        { "support_threshold_angle",        30 },
        { "support_remove_small_overhang",  0 },
        { "support_top_z_distance",         0.2 },
        { "support_bottom_z_distance",      0.2 },
        { "support_interface_top_layers",   0 },
        { "support_interface_bottom_layers", 4 },
        { "support_bottom_interface_spacing", 0.0 },
        { "support_interface_pattern",      "rectilinear_interlaced" },
        { "support_base_pattern",           "default" },
        { "support_filament",               1 },
        { "support_interface_filament",     2 },
        { "tree_support_branch_distance",   5.0 },
        { "tree_support_branch_diameter",   2.0 },
        { "tree_support_branch_angle",      40.0 },
        { "support_line_width",             0.4 },
        { "line_width",                     0.4 },
        { "initial_layer_print_height",     0.2 },
        { "layer_height",                   0.2 },
        { "use_relative_e_distances",         true },
        { "layer_change_gcode",               "G92 E0\n" },
        { "raft_layers",                    0 },
        { "brim_type",                      "no_brim" },
        { "brim_width",                     0.0 },
        { "skirt_loops",                    0 },
    });

    Print print;
    Model model;
    std::vector<TriangleMesh> meshes;
    meshes.emplace_back(two_tier_mesh());
    init_print(std::move(meshes), print, model, config, nullptr, false);
    const StringObjectException validation = print.validate();
    INFO(validation.string);
    REQUIRE(validation.string.empty());
    print.set_status_silent();
    print.process();

    REQUIRE(print.objects().size() == 1);
    const PrintObject &object = *print.objects().front();
    const Point &center_offset = object.center_offset();
    Evidence result;
    for (const SupportLayer *layer : object.support_layers()) {
        ContactRow row;
        row.bottom_z_mm = double(layer->bottom_z());
        if (row.bottom_z_mm < 3.15 || row.bottom_z_mm >= 4.0)
            continue;

        double angle_x = 0.0;
        double angle_y = 0.0;
        for_each_extrusion_path(layer->support_fills, [&](const ExtrusionPath &path) {
            bool touches_lower_plate = false;
            for (size_t idx = 1; idx < path.polyline.points.size(); ++idx) {
                const Point3 &a = path.polyline.points[idx - 1];
                const Point3 &b = path.polyline.points[idx];
                if (!tree_support_critical_regions_contracts::segment_intersects_xy_window(
                        a, b, center_offset, 1.0, 29.0, 1.0, 29.0))
                    continue;
                const double dx = unscale<double>(b.x() - a.x());
                const double dy = unscale<double>(b.y() - a.y());
                const double length = std::hypot(dx, dy);
                if (length < 0.5)
                    continue;
                touches_lower_plate = true;
                if (path.role() != erSupportMaterialInterface)
                    continue;
                const double angle = std::atan2(dy, dx);
                angle_x += length * std::cos(2.0 * angle);
                angle_y += length * std::sin(2.0 * angle);
                row.longest_interface_segment_mm = std::max(row.longest_interface_segment_mm, length);
            }
            if (!touches_lower_plate)
                return;
            if (path.role() == erSupportMaterial)
                ++row.base_role_paths;
            else if (path.role() == erSupportMaterialInterface)
                ++row.interface_role_paths;
        });
        if (row.interface_role_paths > 0) {
            row.has_interface_angle = row.longest_interface_segment_mm > 0.0;
            row.interface_angle_radians = 0.5 * std::atan2(angle_y, angle_x);
            if (row.interface_angle_radians < 0.0)
                row.interface_angle_radians += PI;
            ++result.contact_interface_rows;
        }
        if (row.bottom_z_mm >= 3.75 && row.bottom_z_mm < 4.0)
            result.last_base_interface_paths += row.base_role_paths;
        result.rows.push_back(row);
    }

    std::vector<double> interface_angles;
    for (const ContactRow &row : result.rows)
        if (row.interface_role_paths > 0 && row.has_interface_angle)
            interface_angles.push_back(row.interface_angle_radians);
    for (size_t idx = 1; idx < interface_angles.size(); ++idx) {
        double delta = std::fmod(std::abs(interface_angles[idx] - interface_angles[idx - 1]), double(PI));
        delta = std::min(delta, double(PI) - delta);
        if (delta > double(PI) / 3.0)
            ++result.alternating_direction_pairs;
    }
    return result;
}

} // namespace tree_support_bottom_interface_contracts

TEST_CASE("Tree support floor contacts keep base-interface roles and interlace direction",
          "[TreeSupport][TreeSupportBottomInterface]")
{
    using namespace tree_support_bottom_interface_contracts;
    const Evidence evidence = observe();

    std::ostringstream contact_rows;
    for (const ContactRow &row : evidence.rows)
        contact_rows << " z=" << row.bottom_z_mm << "(base=" << row.base_role_paths
                     << ",interface=" << row.interface_role_paths << ")";
    INFO("contact-interface rows=" << evidence.contact_interface_rows
         << ", last-band base-role paths=" << evidence.last_base_interface_paths
         << ", alternating direction pairs=" << evidence.alternating_direction_pairs
         << "; observed contact rows:" << contact_rows.str());
    REQUIRE(evidence.contact_interface_rows >= 2);
    REQUIRE(evidence.last_base_interface_paths > 0);
    REQUIRE(evidence.alternating_direction_pairs > 0);
}

namespace tree_support_zero_gap_roof_contracts {

struct RoofEvidence {
    bool finite = true;
    bool zero_gap_resolved = false;
    size_t support_paths = 0;
    size_t interface_paths = 0;
    size_t interface_layers = 0;
    size_t invalid_paths = 0;
    double resolved_gap_mm = std::numeric_limits<double>::quiet_NaN();
    double total_support_length_mm = 0.0;
    double total_support_volume_mm3 = 0.0;
    double top_interface_z_mm = -std::numeric_limits<double>::infinity();
    double top_overhang_coverage_mm2 = 0.0;
};

static ExPolygon cap_overhang_window(const Point &center_offset)
{
    // cap_overhang_fixture() has a 24x24 cap at XY [0,24]^2 and a 6x6 stem at
    // [9,15]^2. The ring is the independently known underside that overhangs air.
    Polygon outer{ Point::new_scale(0., 0.) - center_offset,
                   Point::new_scale(24., 0.) - center_offset,
                   Point::new_scale(24., 24.) - center_offset,
                   Point::new_scale(0., 24.) - center_offset };
    Polygon stem{ Point::new_scale(9., 9.) - center_offset,
                  Point::new_scale(9., 15.) - center_offset,
                  Point::new_scale(15., 15.) - center_offset,
                  Point::new_scale(15., 9.) - center_offset };
    return ExPolygon(std::move(outer), std::move(stem));
}

static double area_mm2(const ExPolygons &polygons)
{
    double scaled_area = 0.0;
    for (const ExPolygon &polygon : polygons)
        scaled_area += std::abs(polygon.area());
    return unscale<double>(unscale<double>(scaled_area));
}

static RoofEvidence observe(double top_gap_mm)
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        { "printable_area",                    "-100x-100,200x-100,200x200,-100x200" },
        { "enable_support",                    1 },
        { "support_type",                      "tree(auto)" },
        { "support_style",                     "organic" },
        { "support_on_build_plate_only",       0 },
        { "support_threshold_angle",           30 },
        { "support_remove_small_overhang",     0 },
        { "support_top_z_distance",            top_gap_mm },
        { "support_interface_top_layers",      3 },
        { "support_interface_bottom_layers",   0 },
        { "support_interface_spacing",         0.4 },
        { "independent_support_layer_height",  false },
        { "layer_height",                      0.2 },
        { "initial_layer_print_height",        0.2 },
        { "support_line_width",                0.4 },
        { "line_width",                        0.4 },
        { "nozzle_diameter",                   "0.4" },
        { "tree_support_top_rate",             "30%" },
        { "tree_support_branch_distance_organic", 1.0 },
        { "tree_support_branch_angle_organic",  40.0 },
        { "tree_support_branch_diameter_angle", 5.0 },
        { "tree_support_tip_diameter",          0.8 },
        { "tree_support_branch_diameter_organic", 2.0 },
        { "tree_support_wall_count",           1 },
        { "support_base_pattern",              "hollow" },
        { "tree_support_auto_brim",            false },
        { "tree_support_brim_width",           0.0 },
        { "brim_type",                         "no_brim" },
        { "brim_width",                        0.0 },
        { "skirt_loops",                       0 },
        { "raft_layers",                       0 },
        { "enable_arc_fitting",                false },
        { "resolution",                        0.01 },
        { "use_relative_e_distances",          true },
        { "layer_change_gcode",                "G92 E0\n" },
    });

    Print print;
    Model model;
    std::vector<TriangleMesh> meshes;
    meshes.emplace_back(organic_tree_density_contracts::cap_overhang_fixture());
    init_print(std::move(meshes), print, model, config, nullptr, false);
    const StringObjectException validation = print.validate();
    INFO(validation.string);
    REQUIRE(validation.string.empty());
    print.process();
    REQUIRE(print.objects().size() == 1);

    const PrintObject &object = *print.objects().front();
    RoofEvidence result;
    result.zero_gap_resolved = object.slicing_parameters().zero_gap_interface_top;
    result.resolved_gap_mm = static_cast<double>(object.slicing_parameters().gap_support_object);
    const ExPolygon overhang = cap_overhang_window(object.center_offset());
    Polygons top_interface_coverage;

    for (const SupportLayer *layer : object.support_layers()) {
        if (layer == nullptr) {
            result.finite = false;
            continue;
        }
        const double layer_z = static_cast<double>(layer->print_z);
        result.finite = result.finite && std::isfinite(layer_z);
        bool layer_has_interface = false;
        Polygons layer_interface_coverage;
        for_each_extrusion_path(layer->support_fills, [&](const ExtrusionPath &path) {
            const double length_mm = unscale<double>(path.length());
            const double flow_mm3_per_mm = path.mm3_per_mm;
            const double width_mm = static_cast<double>(path.width);
            const double height_mm = static_cast<double>(path.height);
            const double volume_mm3 = length_mm * flow_mm3_per_mm;
            const bool valid_physical_path = std::isfinite(length_mm) && length_mm > 0.0 &&
                std::isfinite(flow_mm3_per_mm) && flow_mm3_per_mm > 0.0 &&
                std::isfinite(width_mm) && width_mm > 0.0 &&
                std::isfinite(height_mm) && height_mm > 0.0 &&
                std::isfinite(volume_mm3) && volume_mm3 > 0.0;
            if (!valid_physical_path) {
                ++result.invalid_paths;
                result.finite = false;
                return;
            }

            ++result.support_paths;
            result.total_support_length_mm += length_mm;
            result.total_support_volume_mm3 += volume_mm3;
            if (path.role() != erSupportMaterialInterface)
                return;

            ++result.interface_paths;
            layer_has_interface = true;
            path.polygons_covered_by_width(layer_interface_coverage, float(SCALED_EPSILON));
        });
        if (!layer_has_interface)
            continue;

        ++result.interface_layers;
        if (layer_z > result.top_interface_z_mm + EPSILON) {
            result.top_interface_z_mm = layer_z;
            top_interface_coverage.clear();
        }
        if (std::abs(layer_z - result.top_interface_z_mm) <= EPSILON)
            append(top_interface_coverage, layer_interface_coverage);
    }

    if (!top_interface_coverage.empty()) {
        const ExPolygons clipped = intersection_ex(union_ex(top_interface_coverage), ExPolygons{overhang});
        result.top_overhang_coverage_mm2 = area_mm2(clipped);
    }
    result.finite = result.finite && std::isfinite(result.total_support_length_mm);
    result.finite = result.finite && std::isfinite(result.total_support_volume_mm3);
    result.finite = result.finite && std::isfinite(result.top_interface_z_mm);
    result.finite = result.finite && std::isfinite(result.top_overhang_coverage_mm2);
    return result;
}

} // namespace tree_support_zero_gap_roof_contracts

TEST_CASE("Zero-gap organic support roof reaches the cap underside",
          "[TreeSupport][TreeSupportRoofContact]")
{
    using namespace tree_support_zero_gap_roof_contracts;
    constexpr double cap_underside_z_mm = 5.8;
    constexpr double layer_height_mm = 0.2;
    const RoofEvidence zero_gap = observe(0.0);
    const RoofEvidence positive_gap = observe(layer_height_mm);

    for (const RoofEvidence &evidence : {zero_gap, positive_gap}) {
        REQUIRE(evidence.finite);
        REQUIRE(evidence.invalid_paths == 0);
        REQUIRE(evidence.support_paths > 0);
        REQUIRE(evidence.interface_paths > 0);
        REQUIRE(evidence.interface_layers > 0);
        REQUIRE(evidence.total_support_length_mm > 0.0);
        REQUIRE(evidence.total_support_volume_mm3 > 0.0);
        REQUIRE(evidence.top_overhang_coverage_mm2 > 0.01);
    }

    CHECK(zero_gap.zero_gap_resolved);
    CHECK_FALSE(positive_gap.zero_gap_resolved);
    CHECK_THAT(zero_gap.resolved_gap_mm, Catch::Matchers::WithinAbs(0.0, 1e-6));
    CHECK_THAT(positive_gap.resolved_gap_mm, Catch::Matchers::WithinAbs(layer_height_mm, 1e-6));
    // The fixture cap starts at z=5.8; with 0.2 mm layers the positive-gap control
    // sits one full layer lower. The coverage assertion uses only the analytically
    // known cap ring outside the 6x6 stem, not path count or stored polygon ordering.
    CHECK_THAT(zero_gap.top_interface_z_mm,
               Catch::Matchers::WithinAbs(cap_underside_z_mm, 0.03));
    CHECK_THAT(positive_gap.top_interface_z_mm,
               Catch::Matchers::WithinAbs(cap_underside_z_mm - layer_height_mm, 0.03));
    CHECK_THAT(zero_gap.top_interface_z_mm - positive_gap.top_interface_z_mm,
               Catch::Matchers::WithinAbs(layer_height_mm, 0.03));
}
