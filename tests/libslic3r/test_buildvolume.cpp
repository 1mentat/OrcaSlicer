#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/BuildVolume.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/GCode/GCodeProcessor.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/libslic3r.h"

#include <catch2/catch_all.hpp>
#include <catch2/catch_message.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <cstddef>
#include <vector>

using namespace Slic3r;

static std::vector<Vec2d> rect_area(double w, double d)
{
    return { { 0., 0. }, { w, 0. }, { w, d }, { 0., d } };
}

// extruder_printable_height and extruder_printable_area are independent config options, so a
// profile can leave the heights short. BuildVolume must not index past the end of the heights.
TEST_CASE("BuildVolume falls back to the bed height when extruder_printable_height is short", "[BuildVolume]")
{
    const std::vector<Vec2d>              bed     = rect_area(200., 200.);
    const std::vector<std::vector<Vec2d>> areas   = { rect_area(200., 200.), rect_area(100., 200.) };
    const std::vector<double>             heights = { 180. };

    const BuildVolume build_volume(bed, 250., areas, heights);

    REQUIRE(build_volume.get_extruder_area_count() == 2);
    // The extruder with a height of its own keeps it, and differs from the bed, so it gets its own volume.
    CHECK_THAT(build_volume.get_extruder_area_volume(0).bboxf.max.z(), Catch::Matchers::WithinAbs(180., 1e-6));
    // The extruder without one falls back to the bed's printable_height instead of reading out of range.
    CHECK_THAT(build_volume.get_extruder_area_volume(1).bboxf.max.z(), Catch::Matchers::WithinAbs(250., 1e-6));
}

TEST_CASE("BuildVolume keeps per-extruder heights when both vectors match", "[BuildVolume]")
{
    const std::vector<Vec2d>              bed     = rect_area(200., 200.);
    const std::vector<std::vector<Vec2d>> areas   = { rect_area(120., 200.), rect_area(100., 200.) };
    const std::vector<double>             heights = { 180., 200.5 };

    const BuildVolume build_volume(bed, 250., areas, heights);

    REQUIRE(build_volume.get_extruder_area_count() == 2);
    CHECK_THAT(build_volume.get_extruder_area_volume(0).bboxf.max.z(), Catch::Matchers::WithinAbs(180., 1e-6));
    CHECK_THAT(build_volume.get_extruder_area_volume(1).bboxf.max.z(), Catch::Matchers::WithinAbs(200.5, 1e-6));
}

namespace volume_geometry_contracts {

std::vector<Vec2d> rectangle(double x0, double y0, double x1, double y1)
{
    return {{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}};
}

std::vector<Vec2d> circle_bed(double radius, size_t segments, Vec2d center = Vec2d::Zero())
{
    std::vector<Vec2d> result;
    result.reserve(segments);
    for (size_t i = 0; i < segments; ++i) {
        const double angle = 2. * PI * double(i) / double(segments);
        result.emplace_back(center.x() + radius * std::cos(angle), center.y() + radius * std::sin(angle));
    }
    return result;
}

Transform3f translation(float x, float y, float z)
{
    Transform3f result = Transform3f::Identity();
    result.translation() = Vec3f(x, y, z);
    return result;
}

BuildVolume::ObjectState object_state(const BuildVolume &volume, const indexed_triangle_set &mesh,
                                      float x = 0.f, float y = 0.f, float z = 0.f,
                                      bool may_be_below_bed = false)
{
    return volume.object_state(mesh, translation(x, y, z), may_be_below_bed);
}

GCodeProcessorResult::MoveVertex extrusion_move(float x, float y, float z)
{
    GCodeProcessorResult::MoveVertex move;
    move.type = EMoveType::Extrude;
    move.extrusion_role = erExternalPerimeter;
    move.position = Vec3f(x, y, z);
    move.width = 0.4f;
    move.height = 0.2f;
    return move;
}


TEST_CASE("BuildVolume default state is invalid and empty", "[BuildVolume][BuildVolumeVariance]")
{
    const BuildVolume volume;
    CHECK_FALSE(volume.valid());
    CHECK(volume.type() == BuildVolume_Type::Invalid);
    CHECK(volume.type_name() == "Invalid");
    CHECK(volume.printable_area().empty());
    CHECK_THAT(volume.bounding_volume().min.x(), Catch::Matchers::WithinAbs(0., 1e-6));
    CHECK_THAT(volume.bounding_volume().max.z(), Catch::Matchers::WithinAbs(0., 1e-6));
}

TEST_CASE("BuildVolume classifies a rectangular bed and derives its extents", "[BuildVolume][BuildVolumeVariance]")
{
    const BuildVolume volume(rectangle(-10., -5., 30., 25.), 120., {}, {});
    REQUIRE(volume.valid());
    CHECK(volume.type() == BuildVolume_Type::Rectangle);
    CHECK_THAT(volume.bed_center().x(), Catch::Matchers::WithinAbs(10., 1e-9));
    CHECK_THAT(volume.bed_center().y(), Catch::Matchers::WithinAbs(10., 1e-9));
    CHECK_THAT(volume.printable_height(), Catch::Matchers::WithinAbs(120., 1e-9));
    CHECK_THAT(volume.bounding_volume().min.x(), Catch::Matchers::WithinAbs(-10., 1e-6));
    CHECK_THAT(volume.bounding_volume().max.y(), Catch::Matchers::WithinAbs(25., 1e-6));
    CHECK_THAT(volume.bounding_volume().max.z(), Catch::Matchers::WithinAbs(120., 1e-6));
}

TEST_CASE("BuildVolume recognizes a discretized circle and preserves its analytic radius", "[BuildVolume][BuildVolumeVariance]")
{
    const BuildVolume volume(circle_bed(25., 96, {10., 7.}), 80., {}, {});
    REQUIRE(volume.valid());
    REQUIRE(volume.type() == BuildVolume_Type::Circle);
    CHECK_THAT(unscaled<double>(volume.circle().center.x()), Catch::Matchers::WithinAbs(10., 0.01));
    CHECK_THAT(unscaled<double>(volume.circle().center.y()), Catch::Matchers::WithinAbs(7., 0.01));
    CHECK_THAT(unscaled<double>(volume.circle().radius), Catch::Matchers::WithinAbs(25., 0.01));
    CHECK(volume.convex_hull().area() > 0.);
}

TEST_CASE("BuildVolume distinguishes convex and concave analytic beds", "[BuildVolume][BuildVolumeVariance]")
{
    const BuildVolume convex({{0., 0.}, {30., 0.}, {15., 25.}}, 50., {}, {});
    const BuildVolume concave({{0., 0.}, {30., 0.}, {30., 30.}, {20., 30.}, {20., 10.},
                               {10., 10.}, {10., 30.}, {0., 30.}}, 50., {}, {});
    REQUIRE(convex.type() == BuildVolume_Type::Convex);
    REQUIRE(concave.type() == BuildVolume_Type::Custom);
    CHECK(concave.convex_hull().area() > concave.polygon().area());
    CHECK(concave.top_bottom_convex_hull_decomposition_scene().first.size() > 0);
    CHECK(convex.top_bottom_convex_hull_decomposition_scene().first.size() > 0);
}

TEST_CASE("BuildVolume rectangle bbox states separate inside collision and outside and honor ignore_bottom", "[BuildVolume][BuildVolumeVariance]")
{
    const BuildVolume volume(rectangle(0., 0., 20., 20.), 10., {}, {});
    CHECK(volume.volume_state_bbox(BoundingBoxf3{Vec3d(2., 2., 1.), Vec3d(8., 8., 9.)}) == BuildVolume::ObjectState::Inside);
    CHECK(volume.volume_state_bbox(BoundingBoxf3{Vec3d(18., 2., 1.), Vec3d(22., 8., 9.)}) == BuildVolume::ObjectState::Colliding);
    CHECK(volume.volume_state_bbox(BoundingBoxf3{Vec3d(25., 2., 1.), Vec3d(28., 8., 9.)}) == BuildVolume::ObjectState::Outside);
    // A box entirely under the bed is outside the volume unless the bottom limit is ignored (the default).
    CHECK(volume.volume_state_bbox(BoundingBoxf3{Vec3d(2., 2., -8.), Vec3d(8., 8., -2.)}, false) == BuildVolume::ObjectState::Outside);
    CHECK(volume.volume_state_bbox(BoundingBoxf3{Vec3d(2., 2., -8.), Vec3d(8., 8., -2.)}) == BuildVolume::ObjectState::Inside);
}

TEST_CASE("BuildVolume rectangular mesh states detect translated boundary contact", "[BuildVolume][BuildVolumeVariance]")
{
    const BuildVolume volume(rectangle(0., 0., 20., 20.), 10., {}, {});
    const indexed_triangle_set cube = its_make_cube(10., 10., 5.);
    CHECK(object_state(volume, cube) == BuildVolume::ObjectState::Inside);
    CHECK(object_state(volume, cube, 15.f) == BuildVolume::ObjectState::Colliding);
    CHECK(object_state(volume, cube, 25.f) == BuildVolume::ObjectState::Outside);
    CHECK(object_state(volume, cube, 0.f, 0.f, 8.f) == BuildVolume::ObjectState::Colliding);
}

TEST_CASE("BuildVolume reports a mesh entirely below the bed as Below and one crossing the bed as Inside", "[BuildVolume][BuildVolumeVariance]")
{
    const BuildVolume volume(rectangle(0., 0., 20., 20.), 10., {}, {});
    const indexed_triangle_set cube = its_make_cube(10., 10., 5.);
    CHECK(object_state(volume, cube, 0.f, 0.f, -8.f, true) == BuildVolume::ObjectState::Below);
    CHECK(object_state(volume, cube, 0.f, 0.f, -2.f, true) == BuildVolume::ObjectState::Inside);
}

TEST_CASE("BuildVolume zero printable height removes the upper Z limit", "[BuildVolume][BuildVolumeVariance]")
{
    const BuildVolume unlimited(rectangle(0., 0., 20., 20.), 0., {}, {});
    const BuildVolume limited(rectangle(0., 0., 20., 20.), 10., {}, {});
    const indexed_triangle_set cube = its_make_cube(10., 10., 5.);
    CHECK(object_state(unlimited, cube, 0.f, 0.f, 1000.f) == BuildVolume::ObjectState::Inside);
    CHECK(object_state(limited, cube, 0.f, 0.f, 1000.f) == BuildVolume::ObjectState::Outside);
}

TEST_CASE("BuildVolume circular mesh states use radial containment and height", "[BuildVolume][BuildVolumeVariance]")
{
    const BuildVolume volume(circle_bed(20., 96), 12., {}, {});
    REQUIRE(volume.type() == BuildVolume_Type::Circle);
    const indexed_triangle_set cylinder = its_make_cylinder(5., 4., PI / 36.);
    CHECK(object_state(volume, cylinder, 0.f, 0.f, 1.f) == BuildVolume::ObjectState::Inside);
    CHECK(object_state(volume, cylinder, 17.f, 0.f, 1.f) == BuildVolume::ObjectState::Colliding);
    CHECK(object_state(volume, cylinder, 30.f, 0.f, 1.f) == BuildVolume::ObjectState::Outside);
    CHECK(object_state(volume, cylinder, 0.f, 0.f, 10.f) == BuildVolume::ObjectState::Colliding);
    // Inside the bed's bounding square but more than 20 mm from the centre, so only a radial test rejects it.
    const indexed_triangle_set corner_cube = its_make_cube(2., 2., 2.);
    CHECK(object_state(volume, corner_cube, 15.f, 15.f, 1.f) == BuildVolume::ObjectState::Outside);
}

TEST_CASE("BuildVolume custom-bed mesh states separate inside collision and outside", "[BuildVolume][BuildVolumeVariance]")
{
    const BuildVolume volume({{0., 0.}, {30., 0.}, {30., 30.}, {20., 30.}, {20., 10.},
                              {10., 10.}, {10., 30.}, {0., 30.}}, 20., {}, {});
    REQUIRE(volume.type() == BuildVolume_Type::Custom);
    const indexed_triangle_set cube = its_make_cube(3., 3., 3.);
    CHECK(object_state(volume, cube, 13.f, 2.f) == BuildVolume::ObjectState::Inside);
    CHECK(object_state(volume, cube, -8.f, 2.f) == BuildVolume::ObjectState::Outside);
    // Custom-bed collision uses the convex hull; a cube whose vertices straddle that hull
    // boundary is reported as colliding rather than silently accepted.
    CHECK(object_state(volume, cube, 29.f, 29.f) == BuildVolume::ObjectState::Colliding);
}

TEST_CASE("BuildVolume treats an extruder area as the bed only when both its shape and height match", "[BuildVolume][BuildVolumeVariance]")
{
    const auto bed = rectangle(0., 0., 20., 20.);
    // Area 0 has the bed's shape but its own height. Area 1 has the bed's height but its own shape.
    // Area 2 has the bed's shape and no height entry, so it falls back to the bed height and becomes
    // identical to the bed.
    const BuildVolume volume(bed, 100., {bed, rectangle(0., 0., 10., 20.), bed}, {40., 100.});
    REQUIRE(volume.get_extruder_area_count() == 3);
    CHECK_FALSE(volume.get_extruder_area_volume(0).same_with_bed);
    CHECK_FALSE(volume.get_extruder_area_volume(1).same_with_bed);
    CHECK(volume.get_extruder_area_volume(2).same_with_bed);
    CHECK_THAT(volume.get_extruder_area_volume(0).bboxf.max.z(), Catch::Matchers::WithinAbs(40., 1e-6));
    CHECK_THAT(volume.get_extruder_area_volume(2).bboxf.max.z(), Catch::Matchers::WithinAbs(100., 1e-6));
}

TEST_CASE("BuildVolume per-extruder rectangle limits are returned as a complete boolean mask", "[BuildVolume][BuildVolumeVariance]")
{
    const auto bed = rectangle(0., 0., 20., 20.);
    const BuildVolume volume(bed, 100., {bed, rectangle(0., 0., 10., 20.), rectangle(10., 0., 20., 20.)}, {100., 100., 100.});
    const indexed_triangle_set cube = its_make_cube(4., 4., 4.);

    std::vector<bool> inside;
    CHECK(volume.check_object_state_with_extruder_areas(cube, translation(10.1f, 2.f, 0.f), inside) == BuildVolume::ObjectState::Limited);
    REQUIRE(inside.size() == 3);
    CHECK(inside[0]);
    CHECK_FALSE(inside[1]);
    CHECK(inside[2]);

    std::vector<bool> inside_bbox;
    CHECK(volume.check_volume_bbox_state_with_extruder_areas(BoundingBoxf3{Vec3d(8., 2., 1.), Vec3d(12., 6., 3.)}, inside_bbox) == BuildVolume::ObjectState::Limited);
    REQUIRE(inside_bbox.size() == 3);
    CHECK(inside_bbox[0]);
    CHECK_FALSE(inside_bbox[1]);
    CHECK_FALSE(inside_bbox[2]);
}

// Known bug: the per-extruder mask is resize()d, so a false entry from a previous call survives into the next; fixed by the follow-up fix PR.
TEST_CASE("BuildVolume per-extruder limits replace every entry of a reused mask", "[BuildVolume][BuildVolumeVariance][!shouldfail]")
{
    const auto bed = rectangle(0., 0., 20., 20.);
    const BuildVolume volume(bed, 100., {bed, rectangle(0., 0., 10., 20.), rectangle(10., 0., 20., 20.)}, {100., 100., 100.});
    const indexed_triangle_set cube = its_make_cube(4., 4., 4.);

    // 3DScene reuses one mask across volumes; nozzle 1 cannot reach the first object but can reach the second.
    std::vector<bool> inside;
    volume.check_object_state_with_extruder_areas(cube, translation(10.1f, 2.f, 0.f), inside);
    REQUIRE(inside.size() == 3);
    REQUIRE_FALSE(inside[1]);
    CHECK(volume.check_object_state_with_extruder_areas(cube, translation(2.f, 2.f, 0.f), inside) == BuildVolume::ObjectState::Limited);
    REQUIRE(inside.size() == 3);
    CHECK(inside[0]);
    CHECK(inside[1]);
    CHECK_FALSE(inside[2]);

    // The same for the bbox variant: nozzle 2 cannot reach the first box but can reach the second.
    volume.check_volume_bbox_state_with_extruder_areas(BoundingBoxf3{Vec3d(8., 2., 1.), Vec3d(12., 6., 3.)}, inside);
    REQUIRE(inside.size() == 3);
    REQUIRE_FALSE(inside[2]);
    CHECK(volume.check_volume_bbox_state_with_extruder_areas(BoundingBoxf3{Vec3d(12., 2., 1.), Vec3d(16., 6., 3.)}, inside) == BuildVolume::ObjectState::Limited);
    REQUIRE(inside.size() == 3);
    CHECK(inside[0]);
    CHECK_FALSE(inside[1]);
    CHECK(inside[2]);
}

TEST_CASE("BuildVolume per-extruder circle limits distinguish radial collision", "[BuildVolume][BuildVolumeVariance]")
{
    const auto bed = rectangle(0., 0., 40., 40.);
    const auto extruder = circle_bed(10., 96, {20., 20.});
    const BuildVolume volume(bed, 50., {extruder}, {50.});
    REQUIRE(volume.get_extruder_area_count() == 1);
    REQUIRE(volume.get_extruder_area_volume(0).type == BuildVolume_Type::Circle);
    const indexed_triangle_set cylinder = its_make_cylinder(2., 3., PI / 36.);
    CHECK(volume.check_object_state_with_extruder_area(cylinder, translation(20.f, 20.f, 0.f), 0) == BuildVolume::ObjectState::Inside);
    CHECK(volume.check_object_state_with_extruder_area(cylinder, translation(29.f, 20.f, 0.f), 0) == BuildVolume::ObjectState::Limited);
    // Inside the area's bounding square (10..30) but reaching about 12 mm from the centre, so only a radial test limits it.
    CHECK(volume.check_object_state_with_extruder_area(cylinder, translation(27.f, 27.f, 0.f), 0) == BuildVolume::ObjectState::Limited);
}

TEST_CASE("BuildVolume all_paths_inside checks extrusion moves against rectangular and circular beds", "[BuildVolume][BuildVolumeVariance]")
{
    // An undefined paths_bbox makes the rectangular bed check every move instead of the bbox shortcut.
    GCodeProcessorResult paths;
    paths.moves.push_back(extrusion_move(5.f, 5.f, 1.f));
    const BuildVolume rectangle_volume(rectangle(0., 0., 10., 10.), 10., {}, {});
    CHECK(rectangle_volume.all_paths_inside(paths, BoundingBoxf3{}));
    paths.moves.front().position.x() = 11.f;
    CHECK_FALSE(rectangle_volume.all_paths_inside(paths, BoundingBoxf3{}));

    GCodeProcessorResult circle_paths;
    circle_paths.moves.push_back(extrusion_move(1.f, 0.f, 1.f));
    const BuildVolume circle_volume(circle_bed(5., 96), 10., {}, {});
    REQUIRE(circle_volume.type() == BuildVolume_Type::Circle);
    CHECK(circle_volume.all_paths_inside(circle_paths, BoundingBoxf3{}));
    circle_paths.moves.front().position.x() = 6.f;
    CHECK_FALSE(circle_volume.all_paths_inside(circle_paths, BoundingBoxf3{}));
    // Inside the bounding square but about 5.66 mm from the centre of the 5 mm bed.
    circle_paths.moves.front().position = Vec3f(4.f, 4.f, 1.f);
    CHECK_FALSE(circle_volume.all_paths_inside(circle_paths, BoundingBoxf3{}));
}

TEST_CASE("BuildVolume all_paths_inside honors bbox-only calls and ignores non-extrusion moves", "[BuildVolume][BuildVolumeVariance]")
{
    const BuildVolume volume(rectangle(0., 0., 10., 10.), 10., {}, {});
    GCodeProcessorResult empty_paths;
    CHECK(volume.all_paths_inside(empty_paths, BoundingBoxf3{Vec3d(1., 1., 1.), Vec3d(9., 9., 9.)}));
    CHECK_FALSE(volume.all_paths_inside(empty_paths, BoundingBoxf3{Vec3d(-1., 1., 1.), Vec3d(9., 9., 9.)}));

    // A printed extrusion far outside the bed is rejected. Each variant below changes exactly one
    // field of that move, so it is accepted only because of the filter guard for that field.
    const GCodeProcessorResult::MoveVertex outside = extrusion_move(-100.f, -100.f, 1.f);
    GCodeProcessorResult::MoveVertex travel = outside;
    travel.type = EMoveType::Travel;
    GCodeProcessorResult::MoveVertex custom = outside;
    custom.extrusion_role = erCustom;
    GCodeProcessorResult::MoveVertex zero_width = outside;
    zero_width.width = 0.f;
    GCodeProcessorResult::MoveVertex zero_height = outside;
    zero_height.height = 0.f;
    const auto inside = [&volume](const GCodeProcessorResult::MoveVertex &move) {
        GCodeProcessorResult paths;
        paths.moves.push_back(move);
        return volume.all_paths_inside(paths, BoundingBoxf3{});
    };
    CHECK_FALSE(inside(outside));
    CHECK(inside(travel));
    CHECK(inside(custom));
    CHECK(inside(zero_width));
    CHECK(inside(zero_height));
}

TEST_CASE("BuildVolume bounding mesh follows scaled and unscaled bed extents", "[BuildVolume][BuildVolumeVariance]")
{
    const BuildVolume volume(rectangle(0., 0., 12., 8.), 6., {}, {});
    const indexed_triangle_set scaled_mesh = volume.bounding_mesh(true);
    const indexed_triangle_set raw_mesh = volume.bounding_mesh(false);
    BoundingBox3Base<Vec3f> scaled_bounds;
    for (const Vec3f &vertex : scaled_mesh.vertices)
        scaled_bounds.merge(vertex);
    BoundingBox3Base<Vec3f> raw_bounds;
    for (const Vec3f &vertex : raw_mesh.vertices)
        raw_bounds.merge(vertex);
    CHECK_THAT(scaled_bounds.max.x(), Catch::Matchers::WithinAbs(float(scale_(12.)), 1.f));
    CHECK_THAT(scaled_bounds.max.y(), Catch::Matchers::WithinAbs(float(scale_(8.)), 1.f));
    CHECK_THAT(scaled_bounds.max.z(), Catch::Matchers::WithinAbs(float(scale_(6.)), 1.f));
    CHECK_THAT(raw_bounds.max.x(), Catch::Matchers::WithinAbs(12.f, 1e-6));
    CHECK_THAT(raw_bounds.max.y(), Catch::Matchers::WithinAbs(8.f, 1e-6));
    CHECK_THAT(raw_bounds.max.z(), Catch::Matchers::WithinAbs(6.f, 1e-6));
}

TEST_CASE("BuildVolume shared volume is the intersection of the extruder areas and heights", "[BuildVolume][BuildVolumeVariance]")
{
    const auto bed = rectangle(0., 0., 20., 30.);
    // Each bound of the intersection comes from a different area: min X and max Y from area 0,
    // min Y and the height from area 1, max X from area 2.
    const BuildVolume volume(bed, 75., {rectangle(2., 0., 20., 25.), rectangle(0., 4., 15., 30.), rectangle(0., 0., 12., 28.)}, {60., 40., 50.});
    const auto &shared = volume.get_shared_volume();
    CHECK(shared.type == static_cast<int>(BuildVolume_Type::Rectangle));
    CHECK_THAT(shared.data[0], Catch::Matchers::WithinAbs(2., 1e-6));
    CHECK_THAT(shared.data[1], Catch::Matchers::WithinAbs(4., 1e-6));
    CHECK_THAT(shared.data[2], Catch::Matchers::WithinAbs(12., 1e-6));
    CHECK_THAT(shared.data[3], Catch::Matchers::WithinAbs(25., 1e-6));
    CHECK_THAT(shared.zs[0], Catch::Matchers::WithinAbs(0., 1e-6));
    CHECK_THAT(shared.zs[1], Catch::Matchers::WithinAbs(40., 1e-6));
}

TEST_CASE("BuildVolume type names cover every public enum value", "[BuildVolume][BuildVolumeVariance]")
{
    CHECK(BuildVolume::type_name(BuildVolume_Type::Invalid) == "Invalid");
    CHECK(BuildVolume::type_name(BuildVolume_Type::Rectangle) == "Rectangle");
    CHECK(BuildVolume::type_name(BuildVolume_Type::Circle) == "Circle");
    CHECK(BuildVolume::type_name(BuildVolume_Type::Convex) == "Convex");
    CHECK(BuildVolume::type_name(BuildVolume_Type::Custom) == "Custom");
}

TEST_CASE("Unlimited nonrectangular build volumes retain horizontal and bed-plane limits",
          "[BuildVolume][BuildVolumeUnlimited][BuildVolumeVariance]")
{
    struct Bed {
        std::vector<Vec2d> outline;
        BuildVolume_Type type;
    };
    const std::vector<Bed> beds{
        {circle_bed(20., 96, {15., 15.}), BuildVolume_Type::Circle},
        {{{0., 0.}, {30., 0.}, {15., 25.}}, BuildVolume_Type::Convex},
        {{{0., 0.}, {30., 0.}, {30., 30.}, {20., 30.}, {20., 10.},
          {10., 10.}, {10., 30.}, {0., 30.}}, BuildVolume_Type::Custom},
    };
    const indexed_triangle_set cube = its_make_cube(2., 2., 2.);
    for (const Bed &bed : beds) {
        INFO("bed type " << BuildVolume::type_name(bed.type));
        const BuildVolume unlimited(bed.outline, 0., {}, {});
        const BuildVolume limited(bed.outline, 20., {}, {});
        REQUIRE(unlimited.type() == bed.type);
        REQUIRE(limited.type() == bed.type);
        CHECK(object_state(unlimited, cube, 14.f, 3.f, 1000.f) == BuildVolume::ObjectState::Inside);
        CHECK(object_state(limited, cube, 14.f, 3.f, 1000.f) == BuildVolume::ObjectState::Outside);
        CHECK(object_state(unlimited, cube, 100.f, 3.f, 1000.f) == BuildVolume::ObjectState::Outside);
        CHECK(object_state(unlimited, cube, 14.f, 3.f, -4.f, true) == BuildVolume::ObjectState::Below);
        CHECK(object_state(unlimited, cube, 14.f, 3.f, -1.f, true) == BuildVolume::ObjectState::Inside);
    }
}

TEST_CASE("BuildVolume circular extruder areas have no upper Z limit when the printable height is zero",
          "[BuildVolume][BuildVolumeUnlimited][BuildVolumeVariance]")
{
    const auto bed = rectangle(0., 0., 40., 30.);
    const std::vector<std::vector<Vec2d>> areas{
        circle_bed(8., 96, {10., 15.}), circle_bed(8., 96, {30., 15.})};
    const BuildVolume volume(bed, 0., areas, {0., 0.});
    REQUIRE(volume.get_extruder_area_count() == 2);
    for (int nozzle = 0; nozzle < 2; ++nozzle) {
        REQUIRE(volume.get_extruder_area_volume(nozzle).type == BuildVolume_Type::Circle);
        REQUIRE_FALSE(volume.get_extruder_area_volume(nozzle).same_with_bed);
    }
    // Objects far above Z=0 still reach their nozzle because a zero printable height has no upper limit.
    const indexed_triangle_set cylinder = its_make_cylinder(2., 3., PI / 18.);
    std::vector<bool> inside_left;
    CHECK(volume.check_object_state_with_extruder_areas(cylinder, translation(10.f, 15.f, 1000.f), inside_left) ==
          BuildVolume::ObjectState::Limited);
    REQUIRE(inside_left.size() == 2);
    CHECK(inside_left[0]);
    CHECK_FALSE(inside_left[1]);
    std::vector<bool> inside_right;
    CHECK(volume.check_object_state_with_extruder_areas(cylinder, translation(30.f, 15.f, 1000.f), inside_right) ==
          BuildVolume::ObjectState::Limited);
    REQUIRE(inside_right.size() == 2);
    CHECK_FALSE(inside_right[0]);
    CHECK(inside_right[1]);
}

TEST_CASE("Bed-plane intersections expose collisions absent from the above-bed vertices",
          "[BuildVolume][BuildVolumeClipping][BuildVolumeVariance]")
{
    const BuildVolume volume({{0., 0.}, {30., 0.}, {15., 25.}}, 20., {}, {});
    REQUIRE(volume.type() == BuildVolume_Type::Convex);

    indexed_triangle_set inside_vertices;
    inside_vertices.vertices = {Vec3f(5.f, 5.f, 1.f), Vec3f(5.f, 6.f, 1.f), Vec3f(100.f, 5.f, -1.f)};
    inside_vertices.indices.emplace_back(0, 1, 2);
    // The two visible vertices are inside, but the edges meet Z=0 near X=52.5,
    // beyond the triangular bed's X<=30 boundary.
    CHECK(object_state(volume, inside_vertices, 0.f, 0.f, 0.f, true) == BuildVolume::ObjectState::Colliding);

    indexed_triangle_set outside_vertices;
    outside_vertices.vertices = {Vec3f(-5.f, 5.f, 1.f), Vec3f(-5.f, 6.f, 1.f), Vec3f(25.f, 5.f, -1.f)};
    outside_vertices.indices.emplace_back(0, 1, 2);
    // Here the two visible vertices are outside, while both intersections near
    // X=10 lie inside. The mesh therefore still intersects the printable volume.
    CHECK(object_state(volume, outside_vertices, 0.f, 0.f, 0.f, true) == BuildVolume::ObjectState::Colliding);
}

} // namespace volume_geometry_contracts
