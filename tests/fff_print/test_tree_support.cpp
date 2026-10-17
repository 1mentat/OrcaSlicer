#include <catch2/catch_all.hpp>

#include <algorithm>
#include <cmath>
#include <initializer_list>
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
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintBase.hpp"

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
