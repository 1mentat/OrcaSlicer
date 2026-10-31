#include <algorithm>
#include <catch2/catch_all.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/catch_message.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Flow.hpp"
#include "libslic3r/GCodeReader.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Geometry.hpp"
#include "libslic3r/Geometry/ConvexHull.hpp"

#include <boost/algorithm/string.hpp>

#include <cctype>
#include <cmath>
#include <cstddef>
#include "libslic3r/Polygon.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/Polyline.hpp"
#include <string>
#include "libslic3r/Model.hpp"
#include <vector>
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/libslic3r.h"
#include <string_view>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <boost/algorithm/string/predicate.hpp>
#include <initializer_list>
#include <utility>

#include "test_helpers.hpp" // get access to init_print, etc
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/BrimEarsPoint.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/ObjectID.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintBase.hpp"
#include "libslic3r/BeltBrim.hpp"
#include "libslic3r/Config.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "test_helpers.hpp"

using namespace Slic3r::Test;
using namespace Slic3r;

// Distinct brim regions (combine_brims merges touching brims into one covering >1 object).
static int brim_count(const Print &print)
{
    int n = 0;
    for (const auto &group : print.skirt_brim_groups())
        n += (int) group.brims.size();
    return n;
}

// Total brim loops across all objects.
static size_t brim_loop_count(Print &print)
{
    size_t n = 0;
    for (const auto &kv : print.get_brimMap())
        n += kv.second.items_count();
    return n;
}

static bool brim_enters_first_layer_hole(Print &print)
{
    const PrintObject *object = print.get_object(0);
    Polygons holes;
    for (const ExPolygon &slice : object->layers().front()->lslices)
        holes.insert(holes.end(), slice.holes.begin(), slice.holes.end());

    const Vec3d plate_origin = print.get_plate_origin();
    Point shift = object->instances().front().shift_without_plate_offset();
    shift += Point(scaled(plate_origin.x()), scaled(plate_origin.y()));
    for (Polygon &hole : holes)
        hole.translate(shift);

    for (const auto &kv : print.get_brimMap()) {
        Polylines brim_paths;
        kv.second.collect_polylines(brim_paths);
        for (const Polyline &path : brim_paths)
            for (const Point &point : path.points)
                if (contains(holes, point, false))
                    return true;
    }
    return false;
}

// The span is skirt_height layers, or every layer when a draft shield is on (forced even at
// height 0); per-object skirts are rejected in By object printing (no room between objects).
TEST_CASE("Skirt is emitted once per layer it spans", "[SkirtBrim]")
{
    const int object_layers = 100; // 20mm cube at 0.2mm layers
    const char *skirt_type   = GENERATE("combined", "perobject");
    const char *print_seq    = GENERATE("by layer", "by object");
    const char *draft_shield = GENERATE("disabled", "enabled");
    const int   skirt_height = GENERATE(0, 1, 3);

    DYNAMIC_SECTION(skirt_type << " | " << print_seq << " | draft=" << draft_shield << " | height=" << skirt_height) {
        auto do_slice = [&] {
            return slice_two_cubes_arranged({
                { "skirt_loops",    1 },
                { "skirt_height",   skirt_height },
                { "skirt_distance", 3 },
                { "skirt_type",     skirt_type },
                { "draft_shield",   draft_shield },
                { "print_sequence", print_seq },
                { "layer_height",   0.2 },
            });
        };
        const bool draft = std::string(draft_shield) == "enabled";
        const bool has_skirt = draft || skirt_height > 0;
        const bool unsafe_by_object = std::string(skirt_type) == "perobject"
                                   && std::string(print_seq) == "by object" && has_skirt;

        if (unsafe_by_object) {
            REQUIRE_THROWS(do_slice());
        } else {
            const int expected_layers = draft ? object_layers : skirt_height;
            CHECK(role_passes(do_slice(), "skirt") == expected_layers);
        }
    }
}

// Each per-object skirt prints right before its own object, so distant objects yield two
// non-contiguous skirt passes; close objects group into a single skirt.
TEST_CASE("Per-object skirts group when objects are close", "[SkirtBrim][SkirtType]")
{
    auto [gap, expected_skirts] = GENERATE(table<double, int>({ { 5.0, 1 }, { 60.0, 2 } }));
    DYNAMIC_SECTION("gap=" << gap) {
        const std::string gcode = slice_two_cubes_apart(gap, {
            { "skirt_loops",    1 },
            { "skirt_height",   1 },
            { "skirt_distance", 3 },
            { "skirt_type",     "perobject" },
            { "print_sequence", "by layer" },
            { "layer_height",   0.2 },
        });
        CHECK(role_passes(gcode, "skirt") == expected_skirts);
    }
}

TEST_CASE("Per-object skirt is generated per instance", "[SkirtBrim][SkirtType]")
{
    auto [skirt_type, gap, expected_groups] = GENERATE(table<std::string, double, size_t>({
        { "combined",  5.0, 1 },
        { "combined", 60.0, 1 },
        { "perobject", 5.0, 1 },
        { "perobject", 60.0, 2 },
    }));
    DYNAMIC_SECTION(skirt_type << " gap=" << gap) {
        Print print;
        Model model;
        place_two_cube_instances_apart(gap, {
            { "skirt_type",     skirt_type },
            { "skirt_height",   1 },
            { "skirt_distance", 3 },
            { "skirt_loops",    1 },
            { "gcode_flavor",   "marlin" },
            { "use_relative_e_distances", true },
            { "layer_change_gcode", "G92 E0\n" },
            { "brim_type",      "no_brim" },
            { "print_sequence", "by layer" },
            { "draft_shield",   "disabled" },
            { "min_skirt_length", 0 },
        }, print, model);
        const auto validation = print.validate();
        INFO("validation: " << validation.string << " [" << validation.opt_key << "]");
        REQUIRE(validation.string.empty());
        print.process();

        const auto &groups = print.skirt_brim_groups();
        REQUIRE(groups.size() == expected_groups);
        REQUIRE(print.skirt().items_count() == expected_groups);

        size_t total_instances = 0;
        std::set<size_t> instance_ids;
        for (const Print::SkirtBrimGroup &group : groups) {
            REQUIRE(!group.instances.empty());
            REQUIRE(group.skirt.items_count() == 1);
            const double volume = group.skirt.total_volume();
            REQUIRE(std::isfinite(volume));
            CHECK(volume > 0.);
            total_instances += group.instances.size();

            Points skirt_points;
            group.skirt.collect_points(skirt_points);
            REQUIRE(!skirt_points.empty());
            coord_t min_x = skirt_points.front().x();
            coord_t max_x = min_x;
            for (const Point &point : skirt_points) {
                min_x = std::min(min_x, point.x());
                max_x = std::max(max_x, point.x());
            }
            CHECK(max_x > min_x);
            if (gap > 50.) {
                const coord_t width = max_x - min_x;
                if (skirt_type == "combined")
                    CHECK(width > scaled(80.));
                else
                    CHECK(width < scaled(30.));
            }

            for (const ObjectInstanceID &instance : group.instances) {
                CHECK(instance.object_id == print.get_object(0)->id());
                instance_ids.insert(instance.instance_id);
            }
        }
        CHECK(total_instances == 2);
        CHECK(instance_ids == std::set<size_t>{ 0, 1 });
    }
}

TEST_CASE("Combine brims merges touching brims", "[SkirtBrim]")
{
    auto [gap, combine, expected_brims] = GENERATE(table<double, int, int>({
        { 5.0,  1, 1 },   // touching + combine -> one merged brim
        { 5.0,  0, 2 },   // touching, no combine -> separate
        { 60.0, 1, 2 },   // far apart -> nothing to merge
    }));
    DYNAMIC_SECTION("gap=" << gap << " combine_brims=" << combine) {
        Print print;
        Model model;
        place_two_cubes_apart(gap, {
            { "skirt_loops",    1 },
            { "skirt_height",   1 },
            { "skirt_distance", 3 },
            { "skirt_type",     "perobject" },
            { "print_sequence", "by layer" },
            { "brim_type",      "outer_only" },
            { "brim_width",     5 },
            { "combine_brims",  combine },
            { "layer_height",   0.2 },
        }, print, model);
        print.process();
        CHECK(brim_count(print) == expected_brims);
    }
}

TEST_CASE("Object brims are generated per instance", "[SkirtBrim]")
{
    Print print;
    Model model;
    place_two_cube_instances_apart(60, {
        { "skirt_loops",   0 },
        { "brim_type",     "outer_only" },
        { "brim_width",    5 },
        { "combine_brims", 0 },
    }, print, model);
    print.process();

    REQUIRE(print.skirt_brim_groups().size() == 1);
    REQUIRE(print.skirt_brim_groups().front().brims.size() == 2);
    for (const Print::SkirtBrimGroup::Brim &brim : print.skirt_brim_groups().front().brims) {
        REQUIRE(brim.instances.size() == 1);
        REQUIRE(brim.instances.front().object_id == print.get_object(0)->id());
    }
}

TEST_CASE("Uncombined neighboring brims precede their respective objects", "[SkirtBrim]")
{
    Print print;
    Model model;
    place_two_cubes_apart(0, {
        { "skirt_loops",   0 },
        { "brim_type",     "outer_only" },
        { "brim_width",    5 },
        { "combine_brims", 0 },
    }, print, model);
    print.process();

    REQUIRE(print.skirt_brim_groups().size() == 1);
    REQUIRE(print.skirt_brim_groups().front().brims.size() == 2);
    CHECK(role_sequence(gcode(print), { "brim", "perimeter" }) ==
          std::vector<std::string>{ "brim", "perimeter", "brim", "perimeter" });
}

TEST_CASE("Combine brims merges neighboring object instances", "[SkirtBrim]")
{
    Print print;
    Model model;
    place_two_cube_instances_apart(5, {
        { "skirt_loops",   0 },
        { "brim_type",     "outer_only" },
        { "brim_width",    5 },
        { "combine_brims", 1 },
    }, print, model);
    print.process();

    REQUIRE(print.skirt_brim_groups().size() == 1);
    REQUIRE(print.skirt_brim_groups().front().brims.size() == 1);
    REQUIRE(print.skirt_brim_groups().front().brims.front().instances.size() == 2);
    const std::vector<std::string> expected{ "brim", "perimeter" };
    CHECK(role_sequence(gcode(print), { "brim", "perimeter" }) == expected);
}

// Each object's skirt and brim come right before that object, not all skirts then all brims first.
TEST_CASE("By-layer per-object skirt and brim precede each object", "[SkirtBrim]")
{
    const std::string gcode = slice_two_cubes_apart(60, { // far apart: a skirt+brim per object
        { "skirt_loops",    1 },
        { "skirt_height",   1 },
        { "skirt_distance", 3 },
        { "skirt_type",     "perobject" },
        { "print_sequence", "by layer" },
        { "brim_type",      "outer_only" },
        { "brim_width",     5 },
        { "layer_height",   0.2 },
    });
    const std::vector<std::string> expected{ "skirt", "brim", "perimeter", "skirt", "brim", "perimeter" };
    CHECK(role_sequence(gcode, { "skirt", "brim", "perimeter" }) == expected);
}

// A square's corners are 90 degrees, so they get ears only when brim_ears_max_angle is above 90.
TEST_CASE("Brim ears appear only at corners within the max angle", "[SkirtBrim]")
{
    auto [max_angle, expect_ears] = GENERATE(table<int, bool>({ { 91, true }, { 90, false }, { 89, false } }));
    DYNAMIC_SECTION("brim_ears_max_angle=" << max_angle) {
        Print print;
        init_and_process_print({ cube(20) }, print, {
            { "skirt_loops",              0 },
            { "brim_type",                "brim_ears" },
            { "brim_width",               1 },
            { "brim_ears_max_angle",      max_angle },
            { "initial_layer_line_width", 0.5 },
        });
        if (expect_ears) CHECK(brim_loop_count(print) > 0);
        else             CHECK(brim_loop_count(print) == 0);
    }
}

TEST_CASE("Outer-only brim ears stay out of model holes", "[SkirtBrim]")
{
    const bool outer_only = GENERATE(false, true);
    DYNAMIC_SECTION("brim_ears_outer_only=" << outer_only) {
        Print print;
        init_and_process_print({ TestMesh::cube_with_concave_hole }, print, {
            { "skirt_loops",                0 },
            { "brim_type",                  "brim_ears" },
            { "brim_width",                 2 },
            { "brim_ears_max_angle",        125 },
            { "brim_ears_detection_length", 0 },
            { "brim_ears_outer_only",       outer_only },
            { "initial_layer_line_width",   0.5 },
        });

        REQUIRE(brim_loop_count(print) > 0);
        CHECK(brim_enters_first_layer_hole(print) != outer_only);
    }
}

TEST_CASE("Painted brim ear radius controls sliced size", "[SkirtBrim]")
{
    constexpr double ear_radius = 10.0;

    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        { "skirt_loops",                0 },
        { "brim_type",                  "painted" },
        { "brim_width",                 15 },
        { "brim_object_gap",            0.1 },
        { "brim_ears_outer_only",       true },
        { "initial_layer_line_width",   0.5 },
    });

    Print print;
    Model model;
    init_print({ cube(20) }, print, model, config);
    print.process();

    const PrintObject *object = print.get_object(0);
    REQUIRE(!object->layers().front()->lslices.empty());
    const Point ear_center = object->layers().front()->lslices.front().contour.points.front();

    Transform3d model_transform = model.objects.front()->instances.front()->get_transformation().get_matrix_no_offset();
    const Point &center_offset = object->center_offset();
    model_transform = model_transform.pretranslate(
        Vec3d(-unscale<double>(center_offset.x()), -unscale<double>(center_offset.y()), 0));
    Vec3d model_pos = model_transform.inverse() *
        Vec3d(unscale<double>(ear_center.x()), unscale<double>(ear_center.y()), 0);
    model_pos.z() = model.objects.front()->raw_mesh_bounding_box().min.z() - 0.0001;
    model.objects.front()->brim_points = {
        BrimPoint(model_pos.cast<float>(), float(ear_radius)),
    };

    print.apply(model, config);
    print.process();

    const Vec3d plate_origin = print.get_plate_origin();
    Point path_center = ear_center + object->instances().front().shift_without_plate_offset();
    path_center += Point(scaled(plate_origin.x()), scaled(plate_origin.y()));

    double max_path_radius = 0.0;
    for (const auto &kv : print.get_brimMap()) {
        Polylines brim_paths;
        kv.second.collect_polylines(brim_paths);
        for (const Polyline &path : brim_paths)
            for (const Point &point : path.points)
                max_path_radius = std::max(max_path_radius, unscale<double>((point - path_center).cast<double>().norm()));
    }

    REQUIRE(max_path_radius > 0.0);
    INFO("Outermost painted-ear path radius: " << max_path_radius << " mm");
    CHECK(max_path_radius > ear_radius - 0.5);
    CHECK(max_path_radius < ear_radius);
}

TEST_CASE("Outer-only painted brim ears stay out of model holes", "[SkirtBrim]")
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        { "skirt_loops",                0 },
        { "brim_type",                  "painted" },
        { "brim_ears_outer_only",       true },
        { "initial_layer_line_width",   0.5 },
    });

    Print print;
    Model model;
    init_print({ TestMesh::cube_with_concave_hole }, print, model, config);

    // Slice once to obtain exact outer and inner contour points in print
    // coordinates, then express them in the model coordinates painted ears store.
    print.process();
    const PrintObject *object = print.get_object(0);
    REQUIRE(!object->layers().front()->lslices.empty());
    REQUIRE(!object->layers().front()->lslices.front().holes.empty());

    Transform3d model_transform = model.objects.front()->instances.front()->get_transformation().get_matrix_no_offset();
    const Point &center_offset = object->center_offset();
    model_transform = model_transform.pretranslate(
        Vec3d(-unscale<double>(center_offset.x()), -unscale<double>(center_offset.y()), 0));
    const double bottom_z = model.objects.front()->raw_mesh_bounding_box().min.z() - 0.0001;
    auto painted_point = [&model_transform, bottom_z](const Point &point) {
        Vec3d model_pos = model_transform.inverse() *
            Vec3d(unscale<double>(point.x()), unscale<double>(point.y()), 0);
        model_pos.z() = bottom_z;
        return BrimPoint(model_pos.cast<float>(), 3.f);
    };

    const ExPolygon &first_slice = object->layers().front()->lslices.front();
    Polygon inner_contour = first_slice.holes.front();
    inner_contour.reverse();
    const Points inner_ear_points = inner_contour.concave_points(55. * PI / 180.);
    REQUIRE(!inner_ear_points.empty());
    model.objects.front()->brim_points = {
        painted_point(first_slice.contour.points.front()),
        painted_point(inner_ear_points.front()),
    };
    print.apply(model, config);
    print.process();

    REQUIRE(brim_loop_count(print) > 0);
    CHECK_FALSE(brim_enters_first_layer_hole(print));
}

SCENARIO("Skirt has the configured number of loops", "[SkirtBrim]") {
    GIVEN("20mm cube and default config") {
        WHEN("skirt_loops is set to 2")  {
            Print print;
            init_and_process_print({cube(20)}, print, {
                { "skirt_height",   1 },
                { "skirt_distance", 1 },
                { "skirt_loops",    2 }
            });
            THEN("Skirt Extrusion collection has 2 loops in it") {
                REQUIRE(print.skirt().items_count() == 2);
                REQUIRE(print.skirt().flatten().entities.size() == 2);
            }
        }
    }
}

SCENARIO("Brim has the configured number of loops", "[SkirtBrim]") {
    GIVEN("20mm cube and default config, 1mm first layer width") {
        WHEN("Brim is set to 6mm")  {
	        Print print;
	        init_and_process_print({cube(20)}, print, {
                    { "brim_type",                "outer_only" },
                    { "initial_layer_line_width", 1 },
                    { "brim_width",               6 }
	        });
            THEN("Brim Extrusion collection has 6 loops in it") {
                REQUIRE(brim_loop_count(print) == 6);
            }
        }
        WHEN("Brim is set to 6mm, extrusion width 0.5mm")  {
	        Print print;
	        init_and_process_print({cube(20)}, print, {
                    { "brim_type",                "outer_only" },
                    { "brim_width",               6 },
                    { "initial_layer_line_width", 0.5 }
	        });
            THEN("Brim Extrusion collection has 12 loops in it") {
                REQUIRE(brim_loop_count(print) == 12);
            }
        }
    }
}

static double first_extrusion_feedrate_for_feature(const std::string &gcode, const std::string_view feature)
{
    double feedrate = 0.0;
    bool feature_active = false;
    GCodeReader parser;
    parser.parse_buffer(gcode, [&feedrate, &feature_active, feature] (GCodeReader &self, const GCodeReader::GCodeLine &line) {
        const std::string_view comment = line.comment();
        if (comment.find("FEATURE:") != std::string_view::npos || comment.find("TYPE:") != std::string_view::npos)
            feature_active = comment.find(feature) != std::string_view::npos;

        if (feature_active && line.extruding(self) && line.dist_XY(self) > 0) {
            feedrate = line.new_F(self);
            self.quit_parsing();
        }
    });
    return feedrate;
}

TEST_CASE("Skirt height is honored", "[SkirtBrim]") {
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        { "skirt_loops",  1 },
        { "skirt_height", 5 },
        { "wall_loops",   0 },
    });

    std::string gcode;
    SECTION("printing a single object") {
        gcode = slice({ cube(20) }, config);
    }
    SECTION("printing multiple objects") {
        gcode = slice({ cube(20), cube(20) }, config);
    }

    REQUIRE(layers_with_role(gcode, "skirt").size() == (size_t) config.opt_int("skirt_height"));
}

TEST_CASE("Brim uses first layer speed", "[SkirtBrim]") {
    DynamicPrintConfig config = Slic3r::DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        { "brim_type",                    "outer_only" },
        { "brim_width",                   5 },
        { "gcode_comments",               true },
        { "initial_layer_speed",          10 },
        { "initial_layer_infill_speed",   20 },
        { "machine_start_gcode",          "" },
        { "skirt_loops",                  0 },
        { "slow_down_for_layer_cooling",  false },
        { "z_hop",                        0 }
    });

    const std::string gcode = Slic3r::Test::slice({cube(20)}, config);

    const double brim_feedrate = first_extrusion_feedrate_for_feature(gcode, "Brim");
    REQUIRE(brim_feedrate > 0.0);
    REQUIRE_THAT(brim_feedrate, Catch::Matchers::WithinAbs(600.0, 1e-3));

    const double bottom_surface_feedrate = first_extrusion_feedrate_for_feature(gcode, "Bottom surface");
    REQUIRE(bottom_surface_feedrate > 0.0);
    REQUIRE_THAT(bottom_surface_feedrate, Catch::Matchers::WithinAbs(1200.0, 1e-3));
}

SCENARIO("Skirt and brim generation", "[SkirtBrim]") {
    GIVEN("A default configuration") {
        DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
        config.set_num_extruders(4);
        config.set_deserialize_strict({
            { "initial_layer_print_height", 0.3 },
            // avoid altering speeds unexpectedly
            { "slow_down_for_layer_cooling", false },
            { "initial_layer_speed",         "100%" },
            // remove noise from top/solid layers
            { "top_shell_layers",    0 },
            { "bottom_shell_layers", 1 },
            { "machine_start_gcode", "T[initial_tool]\n" },
        });

        WHEN("Brim width is set to 5") {
            config.set_deserialize_strict({
                { "wall_loops",  0 },
                { "skirt_loops", 0 },
                { "brim_type",   "outer_only" },
                { "brim_width",  5 },
            });
            THEN("Brim is generated") {
                std::string gcode = slice({ cube(20) }, config);
                REQUIRE(! layers_with_role(gcode, "brim").empty());
            }
        }

        WHEN("brim width to 1 with layer_width of 0.5") {
            config.set_deserialize_strict({
                { "skirt_loops",              0 },
                { "initial_layer_line_width", 0.5 },
                { "brim_type",                "outer_only" },
                { "brim_width",               1 },
            });
            THEN("2 brim lines") {
                Print print;
                init_and_process_print({ cube(20) }, print, config);
                REQUIRE(brim_loop_count(print) == 2);
            }
        }

        WHEN("Object is plated with overhang support and a brim") {
            config.set_deserialize_strict({
                { "layer_height",               0.4 },
                { "initial_layer_print_height", 0.4 },
                { "skirt_loops",                1 },
                { "skirt_distance",             0 },
                { "enable_support",             1 },
                { "brim_type",                  "outer_only" },
                { "brim_width",                 5 },
            });
            THEN("Support and brim are both emitted") {
                std::string gcode = slice({ TestMesh::overhang }, config);
                REQUIRE(! layers_with_role(gcode, "support").empty());
                REQUIRE(! layers_with_role(gcode, "brim").empty());
            }
        }

        WHEN("an object with support is surrounded by a skirt") {
            config.set_deserialize_strict({
                { "enable_support", 1 },
                { "skirt_loops",    1 },
                { "skirt_distance", 2 },
                { "brim_type",      "no_brim" },
                { "z_hop",          0 },
            });
            THEN("the skirt is long enough to enclose the object and its support") {
                std::string gcode = slice({ TestMesh::overhang }, config);
                const double first_layer_z = config.opt_float("initial_layer_print_height");

                // On the first layer, accumulate the skirt loop length and collect the
                // object + support extrusion points; the skirt must enclose them.
                double skirt_length = 0.0;
                Points footprint;
                GCodeReader parser;
                parser.parse_buffer(gcode, [&](GCodeReader &self, const GCodeReader::GCodeLine &line) {
                    if (! line.extruding(self) || line.dist_XY(self) <= 0 || std::abs(self.z() - first_layer_z) > 0.01)
                        return;
                    if (line.comment().find("skirt") != std::string_view::npos)
                        skirt_length += line.dist_XY(self);
                    else
                        footprint.push_back(Point::new_scale(line.new_X(self), line.new_Y(self)));
                });

                const double hull_perimeter = unscale<double>(Geometry::convex_hull(footprint).split_at_first_point().length());
                REQUIRE(hull_perimeter > 0.0); // guard against an empty footprint passing trivially
                REQUIRE(skirt_length > hull_perimeter);
            }
        }

        WHEN("Large minimum skirt length is used.") {
            // One skirt loop around a 20mm cube is ~88mm, so 500mm forces extra loops.
            config.set_deserialize_strict({
                { "skirt_loops",      1 },
                { "min_skirt_length", 500 },
            });
            THEN("The skirt is extended to at least the minimum length") {
                std::string gcode = slice({ cube(20) }, config);
                double skirt_length = 0.0;
                GCodeReader parser;
                parser.parse_buffer(gcode, [&skirt_length](GCodeReader &self, const GCodeReader::GCodeLine &line) {
                    if (line.extruding(self) && line.comment().find("skirt") != std::string_view::npos)
                        skirt_length += line.dist_XY(self);
                });
                REQUIRE(skirt_length >= 500.0);
            }
        }
    }
}

// Belt printers ---------------------------------------------------------------
//
// On a tilted belt the brim is laid onto the belt PLANE rather than into the Z=0
// bed plane, so it is spread across many layers instead of living on the first
// one.  The discriminating measurement is the number of contiguous brim runs in
// the G-code: a flat plate brim gives a single run, a belt brim gives one per
// layer that carries a band.  Distinct Z values are useless here, because the
// machine-frame transform couples Y into Z so every belt move has its own Z.
static DynamicPrintConfig belt_brim_config()
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        { "belt_printer",               1 },
        { "belt_slice_rotation",        "x" },
        { "belt_slice_rotation_angle",  45 },
        { "gcode_remap_x",              "rev_x" },
        { "gcode_remap_y",              "pos_z" },
        { "gcode_remap_z",              "pos_y" },
        { "layer_height",               0.2 },
        { "initial_layer_print_height", 0.2 },
        { "skirt_loops",                0 },
        { "top_shell_layers",           0 },
        { "bottom_shell_layers",        1 },
        { "machine_start_gcode",        "T[initial_tool]\n" },
        { "layer_change_gcode",         "G92 E0\n" },
    });
    return config;
}

// Same belt as belt_brim_config(), but with `filaments` distinct filaments so the brim's
// tool selection can be observed.  Kept separate from belt_brim_config() so the existing
// single-filament belt tests are untouched.
static DynamicPrintConfig belt_brim_multifilament_config(unsigned int filaments,
    std::initializer_list<Slic3r::ConfigBase::SetDeserializeItem> extra = {})
{
    DynamicPrintConfig config = multifilament_config(filaments);
    config.set_deserialize_strict({
        { "belt_printer",               1 },
        { "belt_slice_rotation",        "x" },
        { "belt_slice_rotation_angle",  45 },
        { "gcode_remap_x",              "rev_x" },
        { "gcode_remap_y",              "pos_z" },
        { "gcode_remap_z",              "pos_y" },
        { "layer_height",               0.2 },
        { "initial_layer_print_height", 0.2 },
        { "skirt_loops",                0 },
        { "top_shell_layers",           0 },
        { "bottom_shell_layers",        1 },
        { "machine_start_gcode",        "T[initial_tool]\n" },
        { "layer_change_gcode",         "G92 E0\n" },
    });
    if (extra.size() > 0)
        config.set_deserialize_strict(extra);
    return config;
}

// 0-based tool indices used by extrusions whose role comment contains `role` (needs
// gcode_comments).  Mirrors tools_for_role in test_multifilament.cpp; statics do not cross
// translation units, so it is repeated here.
static std::set<int> belt_tools_for_role(const std::string &gcode, const std::string &role)
{
    std::set<int> tools;
    int current_tool = 0;
    GCodeReader reader;
    reader.parse_buffer(gcode, [&](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        const std::string cmd(line.cmd());
        if (cmd.size() >= 2 && cmd[0] == 'T' && std::isdigit((unsigned char) cmd[1]))
            current_tool = std::stoi(cmd.substr(1));
        else if (line.extruding(self) && std::string(line.comment()).find(role) != std::string::npos)
            tools.insert(current_tool);
    });
    return tools;
}

// Machine Z of the first extruding move whose role comment contains `role`, in file order;
// numeric_limits<double>::max() when the role never extrudes.
static double first_role_z(const std::string &gcode, const std::string &role)
{
    double z = std::numeric_limits<double>::max();
    GCodeReader parser;
    parser.parse_buffer(gcode, [&z, &role](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        if (line.extruding(self) && line.comment().find(role) != std::string_view::npos) {
            z = self.z();
            self.quit_parsing();
        }
    });
    return z;
}

// Number of object layers that carry a belt brim band.  Every band prints at its own
// layer Z, so this equals role_layers(gcode, "brim") (plus any apron bands below the
// first object layer).  It is not a pass count: the bands on the empty lead-in layers
// ahead of the object's first contact print back to back, so they fold into one pass.
static int nonempty_belt_brim_layers(const PrintObject &object)
{
    int n = 0;
    for (const ExtrusionEntityCollection &band : object.belt_brim_by_layer())
        if (! band.empty())
            ++ n;
    return n;
}

// Number of distinct Z heights at which `role` extrudes: one per layer that prints it.
static int role_layers(const std::string &gcode, const std::string &role)
{
    std::set<long> zs;
    GCodeReader reader;
    reader.parse_buffer(gcode, [&](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        if (! line.extruding(self) || line.dist_XY(self) <= EPSILON)
            return;
        if (line.comment().find(role) != std::string_view::npos)
            zs.insert(std::lround(self.z() * 1000.));
    });
    return int(zs.size());
}

// Apron bands below the object's first layer that print something.
static int belt_brim_apron_bands(const PrintObject &object)
{
    int n = 0;
    for (const BeltBrimBand &band : object.belt_brim_prologue())
        if (! band.fills.empty())
            ++ n;
    return n;
}

// For each active tool, the ordinal (1-based, over extruding moves) of the FIRST move whose
// role comment contains `role`.  Lets a per-object ordering check key off the object's
// unique wall filament.
static std::map<int, long> first_move_by_tool(const std::string &gcode, const std::string &role)
{
    std::map<int, long> first;
    int  tool = 0;
    long idx  = 0;
    GCodeReader reader;
    reader.parse_buffer(gcode, [&](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        const std::string cmd(line.cmd());
        if (cmd.size() >= 2 && cmd[0] == 'T' && std::isdigit((unsigned char) cmd[1])) {
            tool = std::stoi(cmd.substr(1));
            return;
        }
        if (! line.extruding(self))
            return;
        ++ idx;
        if (std::string(line.comment()).find(role) != std::string::npos && ! first.count(tool))
            first[tool] = idx;
    });
    return first;
}

// C - the band coincident with the object's FIRST contact with the belt must not be dropped:
// a belt brim has to appear at or below the object's first perimeter.  On the unfixed feature
// the first-contact band is dropped and the first brim then appears only at a later (higher)
// layer.  Machine Z is meaningful and shared between roles under the belt remap, so the first
// brim's Z must not exceed the first perimeter's.  Both with and without support.
TEST_CASE("Belt brim is laid at the object's first belt contact", "[SkirtBrim][belt]")
{
    const bool support = GENERATE(false, true);
    DYNAMIC_SECTION("enable_support=" << support) {
        DynamicPrintConfig config = belt_brim_config();
        config.set_deserialize_strict({
            { "brim_type",           "outer_only" },
            { "brim_width",          4 },
            { "leading_brim_length", 0 },
            { "extra_brim_width",    0 },
            { "brim_object_gap",     0 },
            { "enable_support",      support ? 1 : 0 },
        });
        const std::string gcode = slice({ cube(20) }, config);

        const double brim_z = first_role_z(gcode, "brim");
        const double peri_z = first_role_z(gcode, "perimeter");
        REQUIRE(brim_z < std::numeric_limits<double>::max());
        REQUIRE(peri_z < std::numeric_limits<double>::max());
        CHECK(brim_z <= peri_z + EPSILON);
    }
}

// C control - when the band's own object layer has extrusion (any interior layer of a solid
// cube), the band takes the ordinary process_layer() path and must be drawn immediately
// before that layer's perimeters, and exactly once: never dropped, never double-emitted.
TEST_CASE("Belt brim on an object layer precedes its perimeters, once", "[SkirtBrim][belt]")
{
    DynamicPrintConfig config = belt_brim_config();
    config.set_deserialize_strict({
        { "brim_type",       "outer_only" },
        { "brim_width",      4 },
        { "brim_object_gap", 0 },
    });
    Print print;
    Model model;
    init_print({ cube(20) }, print, model, config);
    const std::string gc = gcode(print);

    // Ordering: the first thing extruded is brim, then perimeter.
    const std::vector<std::string> seq = role_sequence(gc, { "brim", "perimeter" });
    REQUIRE(seq.size() >= 2);
    CHECK(seq[0] == "brim");
    CHECK(seq[1] == "perimeter");

    // Exactly once: every band prints at its own layer Z, so the number of Z heights with
    // brim equals the number of bands - not fewer, which a dropped band would give.  (A
    // double emission would print twice at one Z: the pass count below catches that for
    // the bands that sit on layers with perimeters.)
    const PrintObject &object = *print.objects().front();
    const int bands = nonempty_belt_brim_layers(object);
    REQUIRE(bands > 0);
    CHECK(role_layers(gc, "brim") == bands + belt_brim_apron_bands(object));
    CHECK(role_passes(gc, "brim") <= bands);
}

// B - single extruder (filament id 1).  Every band must survive the 1-based -> 0-based
// filament-id conversion the apron path performs: a wrong conversion drops all single-extruder
// bands, so the pass count would collapse.  The expected count is derived from the sliced
// layers, not a ratio.
TEST_CASE("Belt brim on a single extruder emits every band once", "[SkirtBrim][belt]")
{
    DynamicPrintConfig config = belt_brim_config();
    config.set_deserialize_strict({
        { "brim_type",       "outer_only" },
        { "brim_width",      4 },
        { "brim_object_gap", 0 },
    });
    Print print;
    Model model;
    init_print({ cube(20) }, print, model, config);
    const std::string gc = gcode(print);

    const PrintObject &object = *print.objects().front();
    const int expected = nonempty_belt_brim_layers(object) + belt_brim_apron_bands(object);
    REQUIRE(expected > 0);
    CHECK(role_layers(gc, "brim") == expected);
    CHECK(belt_tools_for_role(gc, "brim") == std::set<int>{ 0 });   // filament 1 -> tool 0
}

// Number of brim segments the belt brim generator produced for `object`: the lattice
// lines of every per-layer band plus the apron prologue.  Each segment is written as one
// extruding move, so this is what a G-code count has to match.  A pass count cannot see a
// band emitted twice back to back (two copies of the same band merge into one pass).
static long belt_brim_segments(const PrintObject &object)
{
    auto segments = [](const ExtrusionEntityCollection &fills) {
        long n = 0;
        for (const ExtrusionEntity *entity : fills.flatten().entities)
            for (const Polyline &pl : entity->as_polylines())
                n += long(pl.size()) - 1;
        return n;
    };
    long n = 0;
    for (const ExtrusionEntityCollection &band : object.belt_brim_by_layer())
        n += segments(band);
    for (const BeltBrimBand &band : object.belt_brim_prologue())
        n += segments(band.fills);
    return n;
}

// Number of extruding moves in the G-code whose role is `role`.
static long role_segments(const std::string &gcode, const std::string &role)
{
    long n = 0;
    GCodeReader reader;
    reader.parse_buffer(gcode, [&](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        if (line.extruding(self) && line.dist_XY(self) > EPSILON && line.comment().find(role) != std::string_view::npos)
            ++ n;
    });
    return n;
}

TEST_CASE("Belt brim writes every generated segment exactly once", "[SkirtBrim][belt]")
{
    // The pass count above cannot tell one band from the same band twice in a row; the
    // segment count can, so a brim band emitted twice back to back fails here.
    DynamicPrintConfig config = belt_brim_config();
    config.set_deserialize_strict({
        { "brim_type",           "outer_only" },
        { "brim_width",          4 },
        { "brim_object_gap",     0 },
        { "leading_brim_length", 6 },
    });
    Print print;
    Model model;
    init_print({ cube(20) }, print, model, config);
    const std::string gc = gcode(print);

    const long expected = belt_brim_segments(*print.objects().front());
    REQUIRE(expected > 100);
    CHECK(role_segments(gc, "brim") == expected);
}

// B - multi extruder (wall filament id 2).  Every belt-brim line must print on the object's
// wall filament (index 2 -> tool 1), and the total number of passes must equal the
// single-extruder baseline: no per-filament doubling.
TEST_CASE("Belt brim on a multi-extruder object uses the wall filament, no doubling", "[SkirtBrim][belt]")
{
    // Single-extruder baseline built the same way (same nozzle/flow), so the band geometry -
    // and thus the band count - is identical and only the filament assignment differs.
    DynamicPrintConfig base = belt_brim_multifilament_config(1, {
        { "brim_type",       "outer_only" },
        { "brim_width",      4 },
        { "brim_object_gap", 0 },
    });
    const int baseline = role_passes(slice({ cube(20) }, base), "brim");
    REQUIRE(baseline > 0);

    DynamicPrintConfig config = belt_brim_multifilament_config(2, {
        { "brim_type",              "outer_only" },
        { "brim_width",             4 },
        { "brim_object_gap",        0 },
        { "outer_wall_filament_id", 2 },
        { "inner_wall_filament_id", 2 },
    });
    const std::string gc = slice({ cube(20) }, config);

    CHECK(belt_tools_for_role(gc, "brim") == std::set<int>{ 1 });   // filament 2 -> tool 1
    CHECK(role_passes(gc, "brim") == baseline);
}

// B - two objects offset ALONG the belt (Y, since the tilt is about X), each with its own
// wall filament.  Each object's brim/apron must print on that object's filament AND before
// that object's own perimeters.  The object is identified by its unique tool.
TEST_CASE("Belt brim of each object precedes its perimeters on its own filament", "[SkirtBrim][belt]")
{
    DynamicPrintConfig config = belt_brim_multifilament_config(2, {
        { "brim_type",           "outer_only" },
        { "brim_width",          4 },
        { "leading_brim_length", 6 },
        { "brim_object_gap",     0 },
    });

    std::vector<TriangleMesh> meshes;
    meshes.emplace_back(cube(20));
    TriangleMesh second = cube(20);
    second.translate(0.f, 40.f, 0.f);   // offset along the belt so it lands well after the first
    meshes.emplace_back(std::move(second));

    const std::vector<std::vector<Slic3r::ConfigBase::SetDeserializeItem>> overrides {
        { { "outer_wall_filament_id", 1 }, { "inner_wall_filament_id", 1 } },
        { { "outer_wall_filament_id", 2 }, { "inner_wall_filament_id", 2 } },
    };
    Print print;
    Model model;
    init_print(std::move(meshes), print, model, config, &overrides, /*arrange=*/false);
    print.process();
    const std::string gc = gcode(print);

    // Both brims appear, each on its object's wall filament (1 -> T0, 2 -> T1).
    CHECK(belt_tools_for_role(gc, "brim") == std::set<int>{ 0, 1 });

    const std::map<int, long> brim_first = first_move_by_tool(gc, "brim");
    const std::map<int, long> peri_first = first_move_by_tool(gc, "perimeter");
    for (int tool : { 0, 1 }) {
        REQUIRE(brim_first.count(tool) == 1);
        REQUIRE(peri_first.count(tool) == 1);
        CHECK(brim_first.at(tool) < peri_first.at(tool));
    }
}

// D - the belt-brim predicate must not fire on a request that produces no belt brim.
// leading_brim_length / extra_brim_width only feed the OUTER ring, so inner_only with zero
// brim_width yields nothing and must not claim the layers the prime tower / spiral vase need.
TEST_CASE("Belt inner-only leading brim does not reject the prime tower or spiral vase", "[SkirtBrim][belt]")
{
    auto inner_leading = [](std::initializer_list<Slic3r::ConfigBase::SetDeserializeItem> extra) {
        DynamicPrintConfig config = belt_brim_config();
        config.set_deserialize_strict({
            { "brim_type",           "inner_only" },
            { "brim_width",          0 },
            { "leading_brim_length", 6 },
            { "brim_object_gap",     0 },
        });
        config.set_deserialize_strict(extra);
        return config;
    };
    auto init_inner_leading_with_prime_tower = [](Print &print, Model &model, double brim_width) {
        DynamicPrintConfig config = belt_brim_multifilament_config(2, {
            { "brim_type",                 "inner_only" },
            { "brim_width",                brim_width },
            { "leading_brim_length",       6 },
            { "brim_object_gap",           0 },
            { "enable_prime_tower",        1 },
        });
        const std::vector<std::vector<Slic3r::ConfigBase::SetDeserializeItem>> overrides {
            { { "extruder", 1 } }, { { "extruder", 2 } },
        };
        init_print({ cube(20), cube(20) }, print, model, config, &overrides);
    };

    SECTION("prime tower is left alone") {
        Print print;
        Model model;
        init_inner_leading_with_prime_tower(print, model, 0);
        CHECK_FALSE(print.objects().front()->has_belt_brim());
        CHECK(print.validate().string.empty());
    }
    SECTION("spiral vase is left alone") {
        Print print;
        Model model;
        init_print({ cube(20) }, print, model, inner_leading({ { "spiral_mode", 1 } }));
        CHECK_FALSE(print.objects().front()->has_belt_brim());
        CHECK(print.validate().string.empty());
    }
    // enable_prime_tower stays on for any multi-filament project, but a belt printer never
    // prints the classic tower, so the setting alone must not cost the print its brim.
    SECTION("a real inner brim is accepted with the prime tower setting on") {
        Print print;
        Model model;
        init_inner_leading_with_prime_tower(print, model, 4);
        CHECK(print.objects().front()->has_belt_brim());
        CHECK(print.validate().string.empty());
        CHECK_FALSE(gcode(print).empty());
    }
    // A purge tower object is accepted too: the purge plan moves every object, apron
    // bands included, onto one layer grid.
    SECTION("a brim is accepted next to a belt purge tower object") {
        DynamicPrintConfig config = belt_brim_multifilament_config(2, {
            { "brim_type",               "outer_only" },
            { "brim_width",              4 },
            { "brim_object_gap",         0 },
            { "enable_belt_purge_tower", 1 },
        });
        const std::vector<std::vector<Slic3r::ConfigBase::SetDeserializeItem>> overrides {
            { { "extruder", 1 } }, { { "extruder", 2 } },
        };
        Print print;
        Model model;
        init_print({ cube(20), cube(20) }, print, model, config, &overrides);
        model.objects.back()->config.set_key_value("belt_purge_tower_object", new ConfigOptionBool(true));
        print.apply(model, config);
        REQUIRE(print.has_belt_purge_tower());
        CHECK(print.validate().string.empty());
        CHECK_FALSE(gcode(print).empty());
    }
}

TEST_CASE("Belt brim spans many layers instead of one", "[SkirtBrim][belt]")
{
    DynamicPrintConfig config = belt_brim_config();
    config.set_deserialize_strict({
        { "brim_type",  "outer_only" },
        { "brim_width", 5 },
    });
    const std::string gcode = slice({ cube(20) }, config);
    // A plate-brim implementation would score 1 here.
    CHECK(role_passes(gcode, "brim") > 10);
}

TEST_CASE("Belt brim is absent when both widths are zero", "[SkirtBrim][belt]")
{
    // The "no effect when disabled" guard: brim_type Auto is the shipped default and
    // reports has_brim() even at width 0, so this also pins the gate that keeps the
    // flat plate brim from running on a tilted belt.
    const char *brim_type = GENERATE("auto_brim", "outer_only", "no_brim");
    DYNAMIC_SECTION("brim_type " << brim_type) {
        DynamicPrintConfig config = belt_brim_config();
        config.set_deserialize_strict({
            { "brim_type",        brim_type },
            { "brim_width",       0 },
            { "leading_brim_length", 0 },
            { "extra_brim_width", 0 },
        });
        const std::string gcode = slice({ cube(20) }, config);
        CHECK(role_passes(gcode, "brim") == 0);
    }
}

TEST_CASE("Leading brim length alone produces a belt brim", "[SkirtBrim][belt]")
{
    // Exercises the leading_brim_length-only enablement path and the downhill sweep.
    DynamicPrintConfig config = belt_brim_config();
    config.set_deserialize_strict({
        { "brim_type",        "outer_only" },
        { "brim_width",       0 },
        { "leading_brim_length", 5 },
        { "brim_object_gap",  0 },
    });
    const std::string gcode = slice({ cube(20) }, config);
    CHECK(role_passes(gcode, "brim") > 0);
}

TEST_CASE("Leading brim length reaches further ahead of the object", "[SkirtBrim][belt]")
{
    // Compared between two runs rather than against an absolute coordinate, so the
    // assertion survives any change of origin or axis remap.
    auto brim_extent = [](double extra) {
        DynamicPrintConfig config = belt_brim_config();
        config.set_deserialize_strict({
            { "brim_type",        "outer_only" },
            { "brim_width",       3 },
            { "leading_brim_length", extra },
            { "brim_object_gap",  0 },
        });
        const std::string gcode = slice({ cube(20) }, config);
        // The apron prints before the object reaches the belt, so it shows up as brim
        // extrusion at the lowest machine Z of any brim move.
        double min_z = std::numeric_limits<double>::max();
        GCodeReader parser;
        parser.parse_buffer(gcode, [&min_z](GCodeReader &self, const GCodeReader::GCodeLine &line) {
            if (line.extruding(self) && line.comment().find("brim") != std::string_view::npos)
                min_z = std::min(min_z, static_cast<double>(self.z()));
        });
        return min_z;
    };
    const double without = brim_extent(0.);
    const double with    = brim_extent(10.);
    REQUIRE(without < std::numeric_limits<double>::max());
    REQUIRE(with    < std::numeric_limits<double>::max());
    CHECK(with < without);
}

TEST_CASE("Every brim type slices on a belt printer", "[SkirtBrim][belt]")
{
    // Auto / Mouse ear / Painted collapse to outer-only rather than crashing or
    // silently producing nothing.
    const char *brim_type = GENERATE("auto_brim", "brim_ears", "painted", "outer_only",
                                     "inner_only", "outer_and_inner", "leading_edge_only", "no_brim");
    DYNAMIC_SECTION("brim_type " << brim_type) {
        DynamicPrintConfig config = belt_brim_config();
        config.set_deserialize_strict({
            { "brim_type",  brim_type },
            { "brim_width", 5 },
        });
        const std::string gcode = slice({ cube(20) }, config);
        REQUIRE(! gcode.empty());
        if (std::string(brim_type) == "no_brim")
            CHECK(role_passes(gcode, "brim") == 0);
        else if (std::string(brim_type) != "inner_only")
            // A solid cube has no holes, so inner_only legitimately yields nothing.
            CHECK(role_passes(gcode, "brim") > 0);
    }
}

// The leading-edge-only brim is the outer brim cut down to the part's first contact
// with the belt.  The cut has to be taken at the first layer that touches the belt: the
// slicing frame starts at the belt below the footprint, so layers().front() is an empty
// lead-in layer whose contact lies ahead of the part, and a cut taken there left no brim
// at all.
TEST_CASE("Leading-edge-only brim is laid at the first contact and nowhere else", "[SkirtBrim][belt][Regression]")
{
    auto brim_gcode = [](const char *brim_type) {
        DynamicPrintConfig config = belt_brim_config();
        config.set_deserialize_strict({
            { "brim_type",           brim_type },
            { "brim_width",          5 },
            { "leading_brim_length", 10 },
            { "extra_brim_width",    0 },
            { "brim_object_gap",     0 },
        });
        return slice({ cube(20) }, config);
    };
    const std::string leading = brim_gcode("leading_edge_only");
    const std::string outer   = brim_gcode("outer_only");

    const double brim_z = first_role_z(leading, "brim");
    const double peri_z = first_role_z(leading, "perimeter");
    REQUIRE(brim_z < std::numeric_limits<double>::max());
    REQUIRE(peri_z < std::numeric_limits<double>::max());
    // At the first contact: the brim starts no later than the part does...
    CHECK(brim_z <= peri_z + EPSILON);
    // ...and stops there, while the outer brim keeps following the footprint.
    const int leading_layers = role_layers(leading, "brim");
    const int outer_layers   = role_layers(outer, "brim");
    INFO("brim layers: leading-edge " << leading_layers << ", outer " << outer_layers);
    CHECK(leading_layers > 0);
    CHECK(leading_layers < outer_layers);
}

// An overhang on the leading side is sliced before the part reaches the belt, so the
// first layer with geometry is the overhang's tip, above the belt.  A leading-edge cut
// taken there lies ahead of the part: the brim shrank to a sliver well ahead of it, or
// vanished once the overhang reached further forward than the brim.  The overhang does
// not touch the belt, so it must not change the brim at all.
TEST_CASE("Leading-edge-only brim ignores an overhang ahead of the part", "[SkirtBrim][belt][Regression]")
{
    const double fin_length = GENERATE(30., 40.);
    CAPTURE(fin_length);
    // A 20 mm cube, with or without a 2 mm thick fin leaving its top edge and reaching
    // `fin` toward -Y, the end of the part that prints first.  The fin overlaps the cube
    // by 1 mm so the two shells merge instead of sharing a face.
    auto brim_layers = [](double fin) {
        indexed_triangle_set its = its_make_cube(20., 20., 20.);
        if (fin > 0.) {
            indexed_triangle_set fin_its = its_make_cube(20., fin + 1., 2.);
            its_translate(fin_its, Vec3f(0.f, float(-fin), 18.f));
            its_merge(its, fin_its);
        }
        DynamicPrintConfig config = belt_brim_config();
        config.set_deserialize_strict({
            { "brim_type",           "leading_edge_only" },
            { "brim_width",          5 },
            { "leading_brim_length", 10 },
            { "extra_brim_width",    0 },
            { "brim_object_gap",     0 },
        });
        return role_layers(slice({ TriangleMesh(std::move(its)) }, config), "brim");
    };
    const int plain    = brim_layers(0.);
    const int with_fin = brim_layers(fin_length);
    INFO("leading-edge brim layers: plain cube " << plain << ", with the fin " << with_fin);
    REQUIRE(plain > 0);
    // One layer of slack: the fin widens the part's footprint on the plate, which can
    // move the layer grid by a fraction of a layer.
    CHECK(std::abs(with_fin - plain) <= 1);
}

TEST_CASE("An untilted belt printer gets no brim", "[SkirtBrim][belt]")
{
    // Belt brim needs a tilt to have a belt plane to lie on, and the flat plate brim
    // cannot reach the G-code on any belt printer: it is emitted out of
    // skirt_brim_groups(), which _make_skirt() builds, and that returns early for every
    // belt printer.  So an untilted belt printer gets nothing - unchanged by this
    // feature.  Making the flat brim work here would mean reopening the belt skirt gate,
    // which is a separate change; Print::validate() warns instead.
    DynamicPrintConfig config = belt_brim_config();
    config.set_deserialize_strict({
        { "belt_slice_rotation", "none" },
        { "brim_type",           "outer_only" },
        { "brim_width",          5 },
    });
    const std::string gcode = slice({ cube(20) }, config);
    CHECK(role_passes(gcode, "brim") == 0);
}

TEST_CASE("Belt brim does not resurrect the skirt", "[SkirtBrim][belt]")
{
    DynamicPrintConfig config = belt_brim_config();
    config.set_deserialize_strict({
        { "brim_type",   "outer_only" },
        { "brim_width",  5 },
        { "skirt_loops", 2 },
    });
    const std::string gcode = slice({ cube(20) }, config);
    CHECK(role_passes(gcode, "skirt") == 0);
}

TEST_CASE("Belt brim lines all have the same width", "[SkirtBrim][belt]")
{
    // Each brim line's extrusion volume comes from its nozzle-to-belt clearance.  Anchoring
    // every line to a fixed fraction of its own band gives them all the same clearance, so
    // they all come out the same width.  The nominal-spacing lattice this replaced let each
    // line land wherever it fell inside its band, so the clearance - and the width with it -
    // varied by 2x, which showed up as visibly ragged brim.
    DynamicPrintConfig config = belt_brim_config();
    config.set_deserialize_strict({
        { "brim_type",       "outer_only" },
        { "brim_width",      5 },
        { "brim_object_gap", 0 },
    });
    Print print;
    init_and_process_print({ cube(20) }, print, config);
    const PrintObject *obj = print.objects().front();

    std::vector<float> widths;
    auto collect = [&widths](const ExtrusionEntityCollection &coll) {
        for (const ExtrusionEntity *ee : coll.entities)
            if (const auto *path = dynamic_cast<const ExtrusionPath *>(ee))
                widths.push_back(path->width);
    };
    for (const ExtrusionEntityCollection &band : obj->belt_brim_by_layer())
        collect(band);
    for (const BeltBrimBand &band : obj->belt_brim_prologue())
        collect(band.fills);

    REQUIRE(widths.size() > 10);
    const float lo = *std::min_element(widths.begin(), widths.end());
    const float hi = *std::max_element(widths.begin(), widths.end());
    CHECK_THAT(hi, Catch::Matchers::WithinRel(lo, 1e-4f));
}

TEST_CASE("Belt apron survives another object printing at the same Z", "[SkirtBrim][belt]")
{
    // An apron band prints below its OWN object's first layer, but with two objects on the
    // belt the second one is already printing at that print_z.  The layer then has an
    // object layer and takes the ordinary process_layer() path rather than the brim-only
    // branch, so the band must be emitted from both or it is silently dropped.  A
    // single-object print cannot exercise this.
    auto brim_passes = [](int object_count) {
        DynamicPrintConfig config = belt_brim_config();
        config.set_deserialize_strict({
            { "brim_type",           "outer_only" },
            { "brim_width",          3 },
            { "leading_brim_length", 8 },
            { "brim_object_gap",     0 },
        });
        std::vector<TriangleMesh> meshes;
        for (int i = 0; i < object_count; ++ i) {
            TriangleMesh m = cube(20);
            // Offset along the belt so the second object starts well after the first.
            m.translate(0.f, float(40 * i), 0.f);
            meshes.emplace_back(std::move(m));
        }
        Print print;
        Model model;
        init_print(std::move(meshes), print, model, config);
        print.process();
        return role_passes(gcode(print), "brim");
    };

    const int one = brim_passes(1);
    const int two = brim_passes(2);
    REQUIRE(one > 0);
    // Two identical objects should carry twice the brim.  Merely asserting `two > one`
    // would not be decisive: the FIRST object's apron survives the bug, because nothing
    // else is printing that early, so only the second object's apron goes missing.
    // Requiring close to 2x is what actually detects the dropped bands.
    CHECK(two >= 1.8 * one);
}

TEST_CASE("Belt brim coexists with support material", "[SkirtBrim][belt]")
{
    // Supports put extra layers into the same z stream as the apron bands, which is what
    // the three-way merge in collect_layers_to_print() exists to handle: a band sharing a
    // print_z with a support layer of the SAME object used to overwrite it in the
    // print-wide merge.  A smoke test - it cannot prove the collision occurred - but it
    // does exercise the merge with all three streams populated.
    DynamicPrintConfig config = belt_brim_config();
    config.set_deserialize_strict({
        { "brim_type",           "outer_only" },
        { "brim_width",          4 },
        { "leading_brim_length", 6 },
        { "brim_object_gap",     0 },
        { "enable_support",      1 },
    });
    const std::string gc = slice({ TestMesh::overhang }, config);
    REQUIRE(! gc.empty());
    CHECK(role_passes(gc, "brim") > 0);
}

// With a 0.3 mm first layer at 45 degrees the brim band on the belt is wider than one bead,
// so its lines go on the nominal lattice instead of at a fixed fraction of the band. A
// lattice line can then land where the belt is almost at the band's print_z; it must be
// moved uphill to the same 0.75 fraction the single-line case uses, not laid scraping the
// belt with its flow clamped to half a layer.
TEST_CASE("Belt brim lattice lines keep their clearance above the belt", "[SkirtBrim][belt]")
{
    DynamicPrintConfig config = belt_brim_config();
    config.set_deserialize_strict({
        { "layer_height",               0.3 },
        { "initial_layer_print_height", 0.3 },
        { "brim_type",                  "outer_only" },
        { "brim_width",                 4 },
        { "brim_object_gap",            0 },
    });
    const std::string gcode = slice({ cube(20) }, config);

    // Heights of the brim extrusions, from the ;HEIGHT: tags inside ;TYPE:Brim sections.
    std::vector<double> brim_heights;
    bool                in_brim = false;
    std::istringstream  lines(gcode);
    for (std::string line; std::getline(lines, line); ) {
        if (boost::starts_with(line, ";TYPE:"))
            in_brim = boost::starts_with(line, ";TYPE:Brim");
        else if (in_brim && boost::starts_with(line, ";HEIGHT:"))
            brim_heights.push_back(std::stod(line.substr(8)));
    }
    REQUIRE(! brim_heights.empty());
    for (const double h : brim_heights) {
        CHECK(h >= 0.75 * 0.3 - 1e-3);
        CHECK(h <= 0.3 + 1e-3);
    }
}

// The brim prints in the object's outer wall filament even when every extrusion of the object
// is offered to purging (flush_into_objects): the tool ordering registers the brim filament
// itself, so the writer always knows it.
TEST_CASE("Belt brim slices when every object is a flush target", "[SkirtBrim][belt]")
{
    DynamicPrintConfig config = belt_brim_multifilament_config(2, {
        { "brim_type",          "outer_only" },
        { "brim_width",         4 },
        { "brim_object_gap",    0 },
        { "flush_into_objects", 1 },
        { "flush_into_infill",  1 },
    });
    const std::vector<std::vector<Slic3r::ConfigBase::SetDeserializeItem>> overrides {
        { { "extruder", 1 } }, { { "extruder", 2 } },
    };
    Print print;
    Model model;
    init_print({ cube(20), cube(20) }, print, model, config, &overrides);
    REQUIRE(print.validate().string.empty());
    const std::string out = gcode(print);
    CHECK(out.find(";TYPE:Brim") != std::string::npos);
}

using namespace Slic3r;
using namespace Slic3r::Test;

namespace brim_flow_variance {
namespace {

struct Material {
    double e = 0.0;
    double xy = 0.0;
    size_t moves = 0;
};

struct Evidence {
    Material brim;
    Material first_layer_object;
    Material later_object;
    size_t layers = 0;
    size_t brim_xy_moves = 0;
    size_t brim_loops = 0;
    double brim_geometry_length = 0.0;
};

DynamicPrintConfig configuration(double brim_ratio, double first_layer_ratio)
{
    const ConfigOptionDef *definition = print_config_def.get("brim_flow_ratio");
    REQUIRE(definition != nullptr);
    REQUIRE(definition->type == coFloat);
    REQUIRE(definition->is_value_valid(brim_ratio));
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        {"layer_height", 0.2}, {"initial_layer_print_height", 0.2},
        {"wall_loops", 2}, {"top_shell_layers", 0}, {"bottom_shell_layers", 1},
        {"bottom_shell_thickness", 0.0}, {"sparse_infill_density", "20%"},
        {"minimum_sparse_infill_area", 0.0},
        {"brim_type", "outer_only"}, {"brim_width", 2.0},
        {"brim_object_gap", 0.1}, {"skirt_loops", 0}, {"raft_layers", 0},
        {"enable_support", false}, {"gcode_comments", true},
        {"gcode_flavor", "marlin"}, {"use_relative_e_distances", true},
        {"layer_change_gcode", "G92 E0\n"}, {"enable_arc_fitting", false},
        {"slow_down_for_layer_cooling", false},
        {"filament_max_volumetric_speed", "200"},
        {"set_other_flow_ratios", true}, {"first_layer_flow_ratio", first_layer_ratio},
        {"brim_flow_ratio", brim_ratio}, {"filament_flow_ratio", "1.0"},
        {"print_flow_ratio", 1.0}
    });
    return config;
}

// Accept role markers only at the beginning of the comment, so serialized
// settings and custom text cannot masquerade as a production role transition.
std::string_view role_marker(std::string_view comment)
{
    while (!comment.empty() && comment.front() == ' ')
        comment.remove_prefix(1);
    for (std::string_view prefix : {std::string_view("TYPE:"), std::string_view("FEATURE:")}) {
        if (comment.substr(0, prefix.size()) == prefix) {
            comment.remove_prefix(prefix.size());
            while (!comment.empty() && comment.front() == ' ')
                comment.remove_prefix(1);
            while (!comment.empty() && (comment.back() == ' ' || comment.back() == '\r'))
                comment.remove_suffix(1);
            return comment;
        }
    }
    return {};
}

Evidence slice(double brim_ratio, double first_layer_ratio = 1.0)
{
    const DynamicPrintConfig config = configuration(brim_ratio, first_layer_ratio);
    Print print;
    Model model;
    TriangleMesh mesh = cube(8.0);
    mesh.scale(Vec3f(1.0f, 1.0f, 0.1f));
    init_print({mesh}, print, model, config);
    const StringObjectException validation = print.validate();
    INFO(validation.string);
    REQUIRE(validation.string.empty());
    const std::string output = gcode(print);
    REQUIRE_FALSE(output.empty());
    REQUIRE(output.size() < 4u * 1024u * 1024u);
    REQUIRE(std::count(output.begin(), output.end(), '\n') < 100000);

    Evidence result;
    for (const auto &entry : print.get_brimMap()) {
        result.brim_loops += entry.second.items_count();
        for_each_extrusion_path(entry.second, [&](const ExtrusionPath &path) {
            result.brim_geometry_length += path.length();
        });
    }
    GCodeReader reader;
    reader.apply_config(config);
    std::string role;
    reader.parse_buffer(output, [&](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        if (line.raw().rfind(";LAYER_CHANGE", 0) == 0)
            ++result.layers;
        if (const std::string_view marker = role_marker(line.comment()); !marker.empty())
            role = std::string(marker);
        if (!line.cmd_is("G1"))
            return;
        const double xy = line.dist_XY(self);
        REQUIRE(std::isfinite(xy));
        if (xy <= EPSILON)
            return;
        if (role == "Brim")
            ++result.brim_xy_moves;
        if (!line.extruding(self))
            return;
        const double e = line.dist_E(self);
        REQUIRE(std::isfinite(e));
        REQUIRE(e > 0.0);
        Material *material = role == "Brim" ? &result.brim
            : result.layers == 1 ? &result.first_layer_object : &result.later_object;
        // The cube has no support, skirt, purge tower or custom extrusion.
        // Require a named role to prevent preamble moves entering the oracle.
        REQUIRE_FALSE(role.empty());
        material->e += e;
        material->xy += xy;
        ++material->moves;
    });
    REQUIRE(result.layers == 4);
    REQUIRE(result.brim_loops > 0);
    REQUIRE(result.brim_geometry_length > 0.0);
    REQUIRE(result.brim_xy_moves > 0);
    REQUIRE(result.first_layer_object.moves > 0);
    REQUIRE(result.later_object.moves > 0);
    return result;
}

double rounding_bound(const Material &base, const Material &changed, double ratio)
{
    // Relative E is emitted to five decimal places. Each independent summed
    // move contributes at most half a decimal quantum plus float parse error.
    return 0.000006 * (ratio * base.moves + changed.moves)
        + 4.0 * std::numeric_limits<float>::epsilon() * (ratio * base.e + changed.e);
}

void require_material(const Material &base, const Material &changed, double ratio)
{
    REQUIRE(base.moves > 0);
    REQUIRE(base.e > 0.0);
    const double tolerance = rounding_bound(base, changed, ratio);
    if (ratio > 0.0) {
        CHECK(changed.moves == base.moves);
        CHECK_THAT(changed.xy, Catch::Matchers::WithinAbs(base.xy, 1e-5));
    } else {
        CHECK(changed.moves == 0);
    }
    if (ratio != 1.0)
        REQUIRE(tolerance < std::abs(base.e * (ratio - 1.0)));
    CHECK_THAT(changed.e, Catch::Matchers::WithinAbs(base.e * ratio, tolerance));
}

void require_same_brim_geometry(const Evidence &base, const Evidence &changed)
{
    CHECK(changed.brim_loops == base.brim_loops);
    CHECK_THAT(changed.brim_geometry_length,
               Catch::Matchers::WithinAbs(base.brim_geometry_length, 1e-5));
}

} // namespace
} // namespace brim_flow_variance

TEST_CASE("Brim flow boundaries scale brim material independently of first layer flow",
          "[SkirtBrim][BrimFlowVariance][SettingsVariance]")
{
    using namespace brim_flow_variance;
    const Evidence base = slice(1.0);
    REQUIRE(base.brim.moves > 0);
    REQUIRE(base.brim.e > 0.0);
    for (double ratio : {0.0, 0.5, 2.0}) {
        CAPTURE(ratio);
        const Evidence changed = slice(ratio);
        require_same_brim_geometry(base, changed);
        require_material(base.brim, changed.brim, ratio);
        require_material(base.first_layer_object, changed.first_layer_object, 1.0);
        require_material(base.later_object, changed.later_object, 1.0);
    }
    // The public option contract excludes brims from the first-layer ratio.
    // Positive object extrusion proves that this control actually took effect.
    const Evidence first_layer_changed = slice(1.0, 0.6);
    require_same_brim_geometry(base, first_layer_changed);
    require_material(base.brim, first_layer_changed.brim, 1.0);
    require_material(base.first_layer_object, first_layer_changed.first_layer_object, 0.6);
    require_material(base.later_object, first_layer_changed.later_object, 1.0);
}

namespace draft_shield_single_loop_contracts {

namespace {

struct SkirtLoopEvidence {
    std::vector<size_t> loops_per_layer;
    std::vector<double> loop_xy_lengths;
    size_t non_skirt_extrusion_moves = 0;
    double non_skirt_xy_path = 0.0;
    double non_skirt_filament = 0.0;
    bool finite = true;
};

struct DraftShieldExport {
    std::string gcode;
    DynamicPrintConfig config;
};

SkirtLoopEvidence inspect_skirt_loops(const DraftShieldExport &exported)
{
    SkirtLoopEvidence evidence;
    GCodeReader reader;
    reader.apply_config(exported.config);
    size_t layer = size_t(-1);
    std::string current_role;
    bool in_skirt_run = false;
    bool travel_after_skirt = false;
    size_t active_run = size_t(-1);
    reader.parse_buffer(exported.gcode, [&evidence, &layer, &current_role, &in_skirt_run,
                                &travel_after_skirt, &active_run](GCodeReader &self,
                                                                  const GCodeReader::GCodeLine &line) {
        const std::string_view comment = line.comment();
        if (comment.find("LAYER_CHANGE") != std::string_view::npos ||
            comment.find("CHANGE_LAYER") != std::string_view::npos) {
            ++layer;
            evidence.loops_per_layer.resize(layer + 1, 0);
            current_role.clear();
            in_skirt_run = false;
            travel_after_skirt = false;
            active_run = size_t(-1);
        }

        if (const std::string_view marker = brim_flow_variance::role_marker(comment); !marker.empty()) {
            current_role = std::string(marker);
            if (current_role != "Skirt") {
                in_skirt_run = false;
                travel_after_skirt = false;
                active_run = size_t(-1);
            }
        }

        const bool motion = line.cmd_is("G0") || line.cmd_is("G1") || line.cmd_is("G2") || line.cmd_is("G3");
        if (!motion)
            return;
        const double xy = line.dist_XY(self);
        const double filament = std::abs(line.dist_E(self));
        if (!std::isfinite(xy) || !std::isfinite(filament)) {
            evidence.finite = false;
            return;
        }

        if (current_role == "Skirt" && !line.extruding(self) && xy > EPSILON) {
            travel_after_skirt = true;
            return;
        }
        if (!line.extruding(self) || xy <= EPSILON)
            return;

        if (current_role == "Skirt") {
            if (!in_skirt_run || travel_after_skirt) {
                REQUIRE(layer != size_t(-1));
                evidence.loops_per_layer[layer] += 1;
                evidence.loop_xy_lengths.push_back(0.0);
                active_run = evidence.loop_xy_lengths.size() - 1;
                in_skirt_run = true;
            }
            travel_after_skirt = false;
            REQUIRE(active_run != size_t(-1));
            evidence.loop_xy_lengths[active_run] += xy;
            return;
        }

        in_skirt_run = false;
        travel_after_skirt = false;
        active_run = size_t(-1);
        ++evidence.non_skirt_extrusion_moves;
        evidence.non_skirt_xy_path += xy;
        evidence.non_skirt_filament += filament;
    });
    return evidence;
}

DraftShieldExport export_draft_shield_case(bool single_loop)
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        {"layer_height", 0.2},
        {"initial_layer_print_height", 0.2},
        {"wall_loops", 1},
        {"top_shell_layers", 1},
        {"bottom_shell_layers", 1},
        {"skirt_loops", 3},
        {"skirt_height", 0},
        {"skirt_distance", 3.0},
        {"min_skirt_length", 0.0},
        {"skirt_type", "combined"},
        {"draft_shield", "enabled"},
        {"single_loop_draft_shield", single_loop},
        {"brim_type", "no_brim"},
        {"raft_layers", 0},
        {"enable_support", false},
        {"enable_prime_tower", false},
        {"gcode_flavor", "marlin2"},
        {"use_relative_e_distances", true},
        {"enable_arc_fitting", false},
        {"spiral_mode", false},
        {"gcode_comments", true},
        {"machine_start_gcode", ""},
        {"machine_end_gcode", ""},
        {"file_start_gcode", ""},
        {"layer_change_gcode", "G92 E0\n"},
        {"print_sequence", "by layer"},
    });

    Model model;
    Print print;
    const TriangleMesh mesh = make_cube(8.0, 8.0, 0.8);
    Slic3r::Test::init_print({mesh}, print, model, config);
    const StringObjectException validation = print.validate();
    INFO(validation.string);
    REQUIRE(validation.string.empty());
    DraftShieldExport output;
    output.config = config;
    output.gcode = Slic3r::Test::gcode(print);
    REQUIRE_FALSE(output.gcode.empty());
    return output;
}

} // namespace

TEST_CASE("Single-loop draft shields retain all first-layer loops and limit later layers",
          "[DraftShieldSingleLoop][SkirtBrim]")
{
    const DraftShieldExport all_output = export_draft_shield_case(false);
    const DraftShieldExport one_output = export_draft_shield_case(true);
    const SkirtLoopEvidence all_evidence = inspect_skirt_loops(all_output);
    const SkirtLoopEvidence one_evidence = inspect_skirt_loops(one_output);

    REQUIRE(all_evidence.finite);
    REQUIRE(one_evidence.finite);
    REQUIRE(all_evidence.loops_per_layer.size() == 4);
    REQUIRE(one_evidence.loops_per_layer.size() == 4);
    REQUIRE(all_evidence.non_skirt_extrusion_moves > 0);
    REQUIRE(one_evidence.non_skirt_extrusion_moves > 0);
    REQUIRE(all_evidence.non_skirt_xy_path > 0.0);
    REQUIRE(one_evidence.non_skirt_xy_path > 0.0);
    REQUIRE(all_evidence.non_skirt_filament > 0.0);
    REQUIRE(one_evidence.non_skirt_filament > 0.0);
    CHECK_THAT(one_evidence.non_skirt_xy_path,
               Catch::Matchers::WithinRel(all_evidence.non_skirt_xy_path, 1e-4));
    CHECK_THAT(one_evidence.non_skirt_filament,
               Catch::Matchers::WithinRel(all_evidence.non_skirt_filament, 1e-4));

    CHECK(all_evidence.loops_per_layer.front() == 3);
    CHECK(one_evidence.loops_per_layer.front() == 3);
    for (size_t layer = 1; layer < all_evidence.loops_per_layer.size(); ++layer) {
        CAPTURE(layer);
        CHECK(all_evidence.loops_per_layer[layer] == 3);
        CHECK(one_evidence.loops_per_layer[layer] == 1);
    }
    REQUIRE(all_evidence.loop_xy_lengths.size() == 12);
    REQUIRE(one_evidence.loop_xy_lengths.size() == 6);
    for (const double length : all_evidence.loop_xy_lengths)
        REQUIRE(length > 1.0);
    for (const double length : one_evidence.loop_xy_lengths)
        REQUIRE(length > 1.0);
}

} // namespace draft_shield_single_loop_contracts

namespace skirt_settings_variance {

struct Bounds {
    double min_x = std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    double min_y = std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();
};

struct SkirtEvidence {
    size_t positive_xy_moves = 0;
    double xy_length = 0.0;
    double filament = 0.0;
    double first_x = std::numeric_limits<double>::quiet_NaN();
    double first_y = std::numeric_limits<double>::quiet_NaN();
    double first_feed_mm_s = std::numeric_limits<double>::quiet_NaN();
    Bounds bounds;
    bool finite = true;
};

struct SkirtSample {
    SkirtEvidence skirt;
    Bounds model_bounds;
    double spacing = 0.0;
};

Bounds first_layer_model_bounds(const Print &print)
{
    Bounds bounds;
    const PrintObject *object = print.get_object(0);
    REQUIRE(object != nullptr);
    REQUIRE(object->instances().size() == 1);
    REQUIRE_FALSE(object->layers().empty());
    REQUIRE_FALSE(object->layers().front()->lslices.empty());
    const Vec3d plate_origin = print.get_plate_origin();
    Point world_shift = object->instances().front().shift_without_plate_offset();
    world_shift += Point(scaled(plate_origin.x()), scaled(plate_origin.y()));
    for (const ExPolygon &slice : object->layers().front()->lslices) {
        for (const Point &point : slice.contour.points) {
            const double x = unscale<double>((point + world_shift).x());
            const double y = unscale<double>((point + world_shift).y());
            bounds.min_x = std::min(bounds.min_x, x);
            bounds.max_x = std::max(bounds.max_x, x);
            bounds.min_y = std::min(bounds.min_y, y);
            bounds.max_y = std::max(bounds.max_y, y);
        }
    }
    REQUIRE(std::isfinite(bounds.min_x));
    REQUIRE(std::isfinite(bounds.max_x));
    REQUIRE(std::isfinite(bounds.min_y));
    REQUIRE(std::isfinite(bounds.max_y));
    return bounds;
}

SkirtEvidence inspect_skirt(const std::string &gcode, const DynamicPrintConfig &config)
{
    SkirtEvidence evidence;
    std::string role;
    GCodeReader reader;
    reader.apply_config(config);
    reader.parse_buffer(gcode, [&evidence, &role](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        if (const std::string_view marker = brim_flow_variance::role_marker(line.comment()); !marker.empty())
            role.assign(marker.begin(), marker.end());
        if (role != "Skirt" || !line.extruding(self))
            return;
        const double xy = line.dist_XY(self);
        if (!std::isfinite(xy) || xy <= EPSILON)
            return;
        const double start_x = self.x();
        const double start_y = self.y();
        const double end_x = line.new_X(self);
        const double end_y = line.new_Y(self);
        const double e = line.dist_E(self);
        const double feed = double(line.new_F(self)) / 60.0;
        if (!std::isfinite(start_x) || !std::isfinite(start_y) ||
            !std::isfinite(end_x) || !std::isfinite(end_y) ||
            !std::isfinite(e) || !std::isfinite(feed)) {
            evidence.finite = false;
            return;
        }
        if (evidence.positive_xy_moves == 0) {
            evidence.first_x = start_x;
            evidence.first_y = start_y;
            evidence.first_feed_mm_s = feed;
        }
        for (const auto &[x, y] : { std::pair<double, double>{start_x, start_y},
                                    std::pair<double, double>{end_x, end_y} }) {
            evidence.bounds.min_x = std::min(evidence.bounds.min_x, x);
            evidence.bounds.max_x = std::max(evidence.bounds.max_x, x);
            evidence.bounds.min_y = std::min(evidence.bounds.min_y, y);
            evidence.bounds.max_y = std::max(evidence.bounds.max_y, y);
        }
        ++evidence.positive_xy_moves;
        evidence.xy_length += xy;
        evidence.filament += e;
    });
    return evidence;
}

SkirtSample export_skirt(double distance, double angle, double speed)
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        { "layer_height", 0.2 },
        { "initial_layer_print_height", 0.2 },
        { "initial_layer_line_width", 0.5 },
        { "elefant_foot_compensation", 0.0 },
        { "xy_contour_compensation", 0.0 },
        { "xy_hole_compensation", 0.0 },
        { "filament_shrink", "100%" },
        { "printable_area", "0x0,250x0,250x210,0x210" },
        { "wall_loops", 1 },
        { "top_shell_layers", 1 },
        { "bottom_shell_layers", 1 },
        { "sparse_infill_density", "0%" },
        { "skirt_loops", 1 },
        { "skirt_height", 1 },
        { "min_skirt_length", 0.0 },
        { "skirt_type", "combined" },
        { "skirt_distance", distance },
        { "skirt_start_angle", angle },
        { "skirt_speed", speed },
        { "brim_type", "no_brim" },
        { "raft_layers", 0 },
        { "draft_shield", "disabled" },
        { "enable_support", false },
        { "enable_prime_tower", false },
        { "gcode_flavor", "marlin" },
        { "use_relative_e_distances", true },
        { "enable_arc_fitting", false },
        { "slow_down_for_layer_cooling", false },
        { "gcode_comments", true },
        { "machine_start_gcode", "" },
        { "machine_end_gcode", "" },
        { "file_start_gcode", "" },
        { "layer_change_gcode", "G92 E0\n" },
        { "print_sequence", "by layer" },
        // The first-layer generic extrusion path supplies the zero-speed fallback.
        { "initial_layer_infill_speed", 23.0 },
        { "support_speed", 41.0 },
        { "filament_max_volumetric_speed", "200" },
    });

    TriangleMesh mesh = make_cube(20.0, 20.0, 0.8);
    mesh.translate(115.f, 95.f, 0.f);
    std::vector<TriangleMesh> meshes;
    meshes.emplace_back(std::move(mesh));
    Print print;
    Model model;
    Slic3r::Test::init_print(std::move(meshes), print, model, config, nullptr, false);
    const StringObjectException validation = print.validate();
    INFO(validation.string << " [" << validation.opt_key << "]");
    REQUIRE(validation.string.empty());
    print.process();

    SkirtSample sample;
    sample.model_bounds = first_layer_model_bounds(print);
    sample.spacing = print.skirt_flow().spacing();
    REQUIRE(std::isfinite(sample.spacing));
    REQUIRE(sample.spacing > 0.0);
    const std::string output = Slic3r::Test::gcode(print);
    REQUIRE_FALSE(output.empty());
    sample.skirt = inspect_skirt(output, config);
    return sample;
}

void require_same_loop_geometry(const SkirtEvidence &reference, const SkirtEvidence &changed)
{
    REQUIRE(reference.finite);
    REQUIRE(changed.finite);
    REQUIRE(reference.positive_xy_moves > 0);
    REQUIRE(changed.positive_xy_moves > 0);
    REQUIRE(reference.filament > 0.0);
    REQUIRE(changed.filament > 0.0);
    CHECK_THAT(changed.xy_length, Catch::Matchers::WithinAbs(reference.xy_length, 0.02));
    CHECK_THAT(changed.filament, Catch::Matchers::WithinAbs(reference.filament, 0.01));
    CHECK_THAT(changed.bounds.min_x, Catch::Matchers::WithinAbs(reference.bounds.min_x, 0.01));
    CHECK_THAT(changed.bounds.max_x, Catch::Matchers::WithinAbs(reference.bounds.max_x, 0.01));
    CHECK_THAT(changed.bounds.min_y, Catch::Matchers::WithinAbs(reference.bounds.min_y, 0.01));
    CHECK_THAT(changed.bounds.max_y, Catch::Matchers::WithinAbs(reference.bounds.max_y, 0.01));
}

} // namespace skirt_settings_variance

TEST_CASE("Skirt distance endpoints preserve analytic model clearance",
          "[SkirtBrim][SettingsVariance][SkirtDistance]")
{
    using namespace skirt_settings_variance;
    constexpr Bounds known_model{115.0, 135.0, 95.0, 115.0};
    for (double distance : {0.0, 3.0, 60.0}) {
        CAPTURE(distance);
        const SkirtSample sample = export_skirt(distance, 0.0, 37.0);
        REQUIRE(sample.skirt.finite);
        REQUIRE(sample.skirt.positive_xy_moves > 0);
        REQUIRE(sample.skirt.xy_length > 0.0);
        REQUIRE(sample.skirt.filament > 0.0);
        CHECK_THAT(sample.model_bounds.min_x, Catch::Matchers::WithinAbs(known_model.min_x, 0.01));
        CHECK_THAT(sample.model_bounds.max_x, Catch::Matchers::WithinAbs(known_model.max_x, 0.01));
        CHECK_THAT(sample.model_bounds.min_y, Catch::Matchers::WithinAbs(known_model.min_y, 0.01));
        CHECK_THAT(sample.model_bounds.max_y, Catch::Matchers::WithinAbs(known_model.max_y, 0.01));

        // Print::make_skirt_offsets starts at distance-spacing/2 then applies one
        // spacing offset. Removing half-spacing from each measured centerline gap
        // recovers the configured physical clearance on all four sides.
        const double left_clearance = known_model.min_x - sample.skirt.bounds.min_x - sample.spacing / 2.0;
        const double right_clearance = sample.skirt.bounds.max_x - known_model.max_x - sample.spacing / 2.0;
        const double bottom_clearance = known_model.min_y - sample.skirt.bounds.min_y - sample.spacing / 2.0;
        const double top_clearance = sample.skirt.bounds.max_y - known_model.max_y - sample.spacing / 2.0;
        CHECK_THAT(left_clearance, Catch::Matchers::WithinAbs(distance, 0.03));
        CHECK_THAT(right_clearance, Catch::Matchers::WithinAbs(distance, 0.03));
        CHECK_THAT(bottom_clearance, Catch::Matchers::WithinAbs(distance, 0.03));
        CHECK_THAT(top_clearance, Catch::Matchers::WithinAbs(distance, 0.03));
    }
}

TEST_CASE("Skirt angle selects the registered start and zero speed uses layer fallback",
          "[SkirtBrim][SettingsVariance][SkirtAngle][SkirtSpeed]")
{
    using namespace skirt_settings_variance;
    const SkirtSample base = export_skirt(3.0, 0.0, 37.0);
    REQUIRE(base.skirt.finite);
    REQUIRE(base.skirt.positive_xy_moves > 0);
    REQUIRE(base.skirt.filament > 0.0);
    REQUIRE(std::isfinite(base.skirt.first_x));
    REQUIRE(std::isfinite(base.skirt.first_y));

    constexpr double center_x = 125.0;
    constexpr double center_y = 105.0;
    SkirtSample negative_half_turn;
    SkirtSample positive_half_turn;
    for (double angle : {-180.0, -90.0, 0.0, 90.0, 180.0}) {
        CAPTURE(angle);
        const SkirtSample sample = export_skirt(3.0, angle, 37.0);
        if (angle == -180.0)
            negative_half_turn = sample;
        if (angle == 180.0)
            positive_half_turn = sample;
        require_same_loop_geometry(base.skirt, sample.skirt);
        CHECK_THAT(sample.skirt.first_feed_mm_s, Catch::Matchers::WithinAbs(37.0, 0.05));
        const double dx = sample.skirt.first_x - center_x;
        const double dy = sample.skirt.first_y - center_y;
        REQUIRE(std::hypot(dx, dy) > 1.0);
        const double actual_angle = std::atan2(dy, dx) * 180.0 / PI;
        const double wrapped_delta = std::remainder(actual_angle - angle, 360.0);
        CHECK_THAT(wrapped_delta, Catch::Matchers::WithinAbs(0.0, 2.0));
    }
    REQUIRE(negative_half_turn.skirt.positive_xy_moves > 0);
    REQUIRE(positive_half_turn.skirt.positive_xy_moves > 0);
    CHECK_THAT(positive_half_turn.skirt.first_x,
               Catch::Matchers::WithinAbs(negative_half_turn.skirt.first_x, 0.01));
    CHECK_THAT(positive_half_turn.skirt.first_y,
               Catch::Matchers::WithinAbs(negative_half_turn.skirt.first_y, 0.01));

    const SkirtSample fallback = export_skirt(3.0, 0.0, 0.0);
    require_same_loop_geometry(base.skirt, fallback.skirt);
    CHECK_THAT(fallback.skirt.first_feed_mm_s, Catch::Matchers::WithinAbs(23.0, 0.05));
    CHECK_THAT(base.skirt.first_feed_mm_s, Catch::Matchers::WithinAbs(37.0, 0.05));
}

namespace brim_ears_gap_boundaries {

using skirt_settings_variance::Bounds;
using skirt_settings_variance::first_layer_model_bounds;

struct BrimEvidence {
    size_t positive_xy_moves = 0;
    size_t arc_moves = 0;
    size_t segments_near_nib_tip = 0;
    double xy_length = 0.0;
    double filament = 0.0;
    double minimum_model_clearance = std::numeric_limits<double>::infinity();
    double closest_segment_to_nib_tip = std::numeric_limits<double>::infinity();
    double closest_endpoint_to_nib_tip = std::numeric_limits<double>::infinity();
    Bounds bounds;
    bool finite = true;
};

struct BrimSample {
    BrimEvidence brim;
    Bounds model_bounds;
};

BrimEvidence inspect_brim(const std::string &output, const DynamicPrintConfig &config,
                          const Bounds *model_bounds = nullptr,
                          double nib_tip_x = std::numeric_limits<double>::quiet_NaN(),
                          double nib_tip_y = std::numeric_limits<double>::quiet_NaN())
{
    BrimEvidence evidence;
    GCodeReader reader;
    reader.apply_config(config);
    reader.parse_buffer(output, [&evidence, model_bounds, nib_tip_x, nib_tip_y](GCodeReader &self,
                                                                                const GCodeReader::GCodeLine &line) {
        // Keep the established emitted-description contract: generated brim
        // moves are marked by lowercase "brim" comments. TYPE/FEATURE state
        // does not identify this output consistently across the fixtures.
        if (line.comment().find("brim") == std::string_view::npos)
            return;
        if (line.cmd_is("G2") || line.cmd_is("G3")) {
            if (line.extruding(self))
                ++evidence.arc_moves;
            return;
        }
        if (!line.cmd_is("G1") || !line.extruding(self))
            return;

        const double xy = line.dist_XY(self);
        const double e = std::abs(line.dist_E(self));
        const double start_x = self.x();
        const double start_y = self.y();
        const double end_x = line.new_X(self);
        const double end_y = line.new_Y(self);
        if (!std::isfinite(xy) || !std::isfinite(e) || !std::isfinite(start_x) ||
            !std::isfinite(start_y) || !std::isfinite(end_x) || !std::isfinite(end_y)) {
            evidence.finite = false;
            return;
        }
        if (xy <= EPSILON || e <= 0.0)
            return;

        for (const auto &[x, y] : { std::pair<double, double>{start_x, start_y},
                                    std::pair<double, double>{end_x, end_y} }) {
            evidence.bounds.min_x = std::min(evidence.bounds.min_x, x);
            evidence.bounds.max_x = std::max(evidence.bounds.max_x, x);
            evidence.bounds.min_y = std::min(evidence.bounds.min_y, y);
            evidence.bounds.max_y = std::max(evidence.bounds.max_y, y);
            if (model_bounds != nullptr) {
                const double dx = std::max({ model_bounds->min_x - x, 0.0, x - model_bounds->max_x });
                const double dy = std::max({ model_bounds->min_y - y, 0.0, y - model_bounds->max_y });
                evidence.minimum_model_clearance = std::min(
                    evidence.minimum_model_clearance, std::hypot(dx, dy));
            }
            if (std::isfinite(nib_tip_x) && std::isfinite(nib_tip_y)) {
                const double endpoint_distance = std::hypot(x - nib_tip_x, y - nib_tip_y);
                evidence.closest_endpoint_to_nib_tip = std::min(
                    evidence.closest_endpoint_to_nib_tip, endpoint_distance);
            }
        }
        if (std::isfinite(nib_tip_x) && std::isfinite(nib_tip_y)) {
            // A segment can pass near the nib tip while both emitted endpoints are
            // farther away. Measure distance to the physical XY deposition segment.
            const double dx = end_x - start_x;
            const double dy = end_y - start_y;
            const double length_squared = dx * dx + dy * dy;
            if (!std::isfinite(length_squared) || length_squared <= 0.0) {
                evidence.finite = false;
                return;
            }
            const double raw_projection =
                ((nib_tip_x - start_x) * dx + (nib_tip_y - start_y) * dy) / length_squared;
            if (!std::isfinite(raw_projection)) {
                evidence.finite = false;
                return;
            }
            const double projection = std::clamp(raw_projection, 0.0, 1.0);
            const double closest_x = start_x + projection * dx;
            const double closest_y = start_y + projection * dy;
            const double segment_distance = std::hypot(closest_x - nib_tip_x, closest_y - nib_tip_y);
            if (!std::isfinite(segment_distance)) {
                evidence.finite = false;
                return;
            }
            evidence.closest_segment_to_nib_tip = std::min(
                evidence.closest_segment_to_nib_tip, segment_distance);
            if (segment_distance < 4.9)
                ++evidence.segments_near_nib_tip;
        }
        ++evidence.positive_xy_moves;
        evidence.xy_length += xy;
        evidence.filament += e;
    });
    return evidence;
}

DynamicPrintConfig outer_brim_config()
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        { "layer_height",               0.2 },
        { "initial_layer_print_height", 0.2 },
        { "skirt_loops",                0 },
        { "raft_layers",                0 },
        { "enable_support",              false },
        { "enable_prime_tower",          false },
        { "enable_arc_fitting",          false },
        { "brim_type",                  "outer_only" },
        { "brim_width",                  5.0 },
        { "brim_flow_ratio",             1.0 },
        { "gcode_comments",              true },
    });
    return config;
}

BrimSample export_outer_brim(bool set_gap, double gap)
{
    DynamicPrintConfig config = outer_brim_config();
    if (set_gap)
        config.set_deserialize_strict({ { "brim_object_gap", gap } });

    Print print;
    init_and_process_print({ make_cube(20.0, 20.0, 2.0) }, print, config);
    BrimSample sample;
    sample.model_bounds = first_layer_model_bounds(print);
    const std::string output = Slic3r::Test::gcode(print);
    REQUIRE_FALSE(output.empty());
    sample.brim = inspect_brim(output, config, &sample.model_bounds);
    return sample;
}

TriangleMesh shallow_nib_prism(const Vec3d &part_offset)
{
    // its_make_prism() defines a triangle with a 0.5 mm +Z apex and a 2 mm
    // base width. Rx(+90) then Rz(+90) maps that apex to +X; translation places
    // its base on the cube's right face at X=20, so the first-layer slice spans
    // 20.5 mm in X. The 0.5 mm tip is below the positive 1 mm DP tolerance.
    TriangleMesh nib = make_prism(2.0f, 2.0f, 0.5f);
    nib.rotate_x(float(PI / 2.0));
    nib.rotate_z(float(PI / 2.0));
    nib.translate(float(20.0 - part_offset.x()), float(10.0 - part_offset.y()),
                  float(1.0 - part_offset.z()));
    return nib;
}

BrimSample export_auto_ears(double detection_length)
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        { "layer_height",               0.2 },
        { "initial_layer_print_height", 0.2 },
        { "initial_layer_line_width",   0.5 },
        { "elefant_foot_compensation",   0.0 },
        { "xy_contour_compensation",    0.0 },
        { "xy_hole_compensation",       0.0 },
        { "skirt_loops",                0 },
        { "raft_layers",                0 },
        { "enable_support",              false },
        { "enable_prime_tower",          false },
        { "enable_arc_fitting",          false },
        { "brim_type",                  "brim_ears" },
        { "brim_width",                  5.0 },
        { "brim_flow_ratio",             1.0 },
        { "brim_object_gap",             0.0 },
        // At this triangle tip the internal angle is 126.87 degrees. Brim.cpp
        // passes (180 - max_angle) to Polygon::convex_points(), so 130 degrees
        // admits its ~53-degree turn while preserving the square-corner ears.
        { "brim_ears_max_angle",         130.0 },
        { "brim_ears_outer_only",        true },
        { "brim_ears_detection_length",  detection_length },
        { "gcode_comments",              true },
    });

    Print print;
    Model model;
    init_print({ make_cube(20.0, 20.0, 2.0) }, print, model, config, nullptr, false);
    REQUIRE(model.objects.size() == 1);
    REQUIRE(model.objects.front()->volumes.size() == 1);
    const Vec3d part_offset = model.objects.front()->volumes.front()->get_offset();
    ModelVolume *nib = model.objects.front()->add_volume(
        shallow_nib_prism(part_offset), ModelVolumeType::MODEL_PART, false);
    REQUIRE(nib != nullptr);
    nib->set_offset(part_offset);
    print.apply(model, config);
    print.set_status_silent();
    print.process();

    BrimSample sample;
    sample.model_bounds = first_layer_model_bounds(print);
    const double nib_tip_x = sample.model_bounds.max_x;
    const double nib_tip_y = (sample.model_bounds.min_y + sample.model_bounds.max_y) / 2.0;
    const double model_x_span = sample.model_bounds.max_x - sample.model_bounds.min_x;
    const double model_y_span = sample.model_bounds.max_y - sample.model_bounds.min_y;
    INFO("first-layer model bounds x=[" << sample.model_bounds.min_x << ", "
         << sample.model_bounds.max_x << "] span=" << model_x_span << " mm; y=["
         << sample.model_bounds.min_y << ", " << sample.model_bounds.max_y
         << "] span=" << model_y_span << " mm; nib tip=(" << nib_tip_x << ", " << nib_tip_y << ")");
    REQUIRE_THAT(model_x_span, Catch::Matchers::WithinAbs(20.5, 0.02));
    REQUIRE_THAT(model_y_span, Catch::Matchers::WithinAbs(20.0, 0.02));
    const std::string output = Slic3r::Test::gcode(print);
    REQUIRE_FALSE(output.empty());
    sample.brim = inspect_brim(output, config, &sample.model_bounds, nib_tip_x, nib_tip_y);
    return sample;
}

void require_same_model_bounds(const Bounds &a, const Bounds &b)
{
    CHECK_THAT(a.min_x, Catch::Matchers::WithinAbs(b.min_x, 0.01));
    CHECK_THAT(a.max_x, Catch::Matchers::WithinAbs(b.max_x, 0.01));
    CHECK_THAT(a.min_y, Catch::Matchers::WithinAbs(b.min_y, 0.01));
    CHECK_THAT(a.max_y, Catch::Matchers::WithinAbs(b.max_y, 0.01));
}

void require_valid_brim(const BrimEvidence &brim)
{
    REQUIRE(brim.finite);
    REQUIRE(brim.arc_moves == 0);
    REQUIRE(brim.positive_xy_moves > 0);
    REQUIRE(brim.xy_length > 0.0);
    REQUIRE(brim.filament > 0.0);
    REQUIRE(std::isfinite(brim.minimum_model_clearance));
    REQUIRE(std::isfinite(brim.bounds.min_x));
    REQUIRE(std::isfinite(brim.bounds.max_x));
    REQUIRE(std::isfinite(brim.bounds.min_y));
    REQUIRE(std::isfinite(brim.bounds.max_y));
}

} // namespace brim_ears_gap_boundaries

TEST_CASE("Brim object gap changes emitted clearance without changing the model",
          "[SkirtBrim][BrimEarsGapBoundaries]")
{
    using namespace brim_ears_gap_boundaries;
    const BrimSample default_gap = export_outer_brim(false, 0.0);
    const BrimSample minimum_gap = export_outer_brim(true, 0.0);
    const BrimSample middle_gap = export_outer_brim(true, 1.0);
    const BrimSample maximum_gap = export_outer_brim(true, 2.0);

    require_valid_brim(default_gap.brim);
    require_valid_brim(minimum_gap.brim);
    require_valid_brim(middle_gap.brim);
    require_valid_brim(maximum_gap.brim);
    require_same_model_bounds(default_gap.model_bounds, minimum_gap.model_bounds);
    require_same_model_bounds(minimum_gap.model_bounds, middle_gap.model_bounds);
    require_same_model_bounds(middle_gap.model_bounds, maximum_gap.model_bounds);

    CHECK_THAT(default_gap.brim.minimum_model_clearance,
               Catch::Matchers::WithinAbs(minimum_gap.brim.minimum_model_clearance, 0.03));
    CHECK(middle_gap.brim.minimum_model_clearance > minimum_gap.brim.minimum_model_clearance + 0.8);
    CHECK(maximum_gap.brim.minimum_model_clearance > middle_gap.brim.minimum_model_clearance + 0.8);
}

TEST_CASE("Brim ear detection length zero keeps a small corner in the emitted geometry",
          "[SkirtBrim][BrimEarsGapBoundaries]")
{
    using namespace brim_ears_gap_boundaries;
    const BrimSample disabled = export_auto_ears(0.0);
    const BrimSample default_length = export_auto_ears(1.0);
    const BrimSample positive_length = export_auto_ears(2.0);

    require_valid_brim(disabled.brim);
    require_valid_brim(default_length.brim);
    require_valid_brim(positive_length.brim);
    require_same_model_bounds(disabled.model_bounds, default_length.model_bounds);
    require_same_model_bounds(default_length.model_bounds, positive_length.model_bounds);
    INFO("disabled ear: nearest segment=" << disabled.brim.closest_segment_to_nib_tip
         << " mm, nearest endpoint=" << disabled.brim.closest_endpoint_to_nib_tip << " mm");
    INFO("1 mm ear: nearest segment=" << default_length.brim.closest_segment_to_nib_tip
         << " mm, nearest endpoint=" << default_length.brim.closest_endpoint_to_nib_tip << " mm");
    INFO("2 mm ear: nearest segment=" << positive_length.brim.closest_segment_to_nib_tip
         << " mm, nearest endpoint=" << positive_length.brim.closest_endpoint_to_nib_tip << " mm");
    CHECK(disabled.brim.segments_near_nib_tip > 0);
    CHECK(default_length.brim.segments_near_nib_tip == 0);
    CHECK(positive_length.brim.segments_near_nib_tip == 0);
    CHECK(disabled.brim.closest_segment_to_nib_tip < 4.9);
    CHECK(default_length.brim.closest_segment_to_nib_tip >= 4.9);
    CHECK(positive_length.brim.closest_segment_to_nib_tip >= 4.9);
}
