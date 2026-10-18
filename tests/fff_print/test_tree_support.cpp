#include <catch2/catch_all.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <initializer_list>
#include <limits>
#include <string>
#include <string_view>
#include <utility>
#include <vector>
#include "libslic3r/Config.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/Model.hpp"
#include <cstddef>
#include "libslic3r/libslic3r.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_message.hpp>
#include "libslic3r/Layer.hpp"
#include "libslic3r/TriangleMesh.hpp"

#include "test_helpers.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Polyline.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintBase.hpp"
#include "libslic3r/Slicing.hpp"
#include "libslic3r/GCodeReader.hpp"

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
