#include "libslic3r/libslic3r.h"
#include "libslic3r/TriangleMesh.hpp"
#include <cstddef>
#include <cmath>
#include <catch2/catch_all.hpp>
#include <vector>
#include "libslic3r/Point.hpp"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "libslic3r/BuildVolume.hpp"
#include "libslic3r/BoundingBox.hpp"

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

TEST_CASE("BuildVolume treats an extruder area as the bed only when both its shape and height match", "[BuildVolume][BuildVolumeVariance]")
{
    const auto bed = rectangle(0., 0., 20., 20.);
    // Area 0 has the bed's shape but its own height. Area 2 has the bed's shape and no height
    // entry, so it falls back to the bed height and becomes identical to the bed.
    const BuildVolume volume(bed, 100., {bed, rectangle(0., 0., 10., 20.), bed}, {40., 50.});
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

TEST_CASE("BuildVolume shared volume is the intersection of the extruder areas and heights", "[BuildVolume][BuildVolumeVariance]")
{
    const auto bed = rectangle(0., 0., 20., 30.);
    const BuildVolume volume(bed, 75., {rectangle(0., 0., 10., 30.)}, {40.});
    const auto &shared = volume.get_shared_volume();
    CHECK(shared.type == static_cast<int>(BuildVolume_Type::Rectangle));
    CHECK_THAT(shared.data[0], Catch::Matchers::WithinAbs(0., 1e-6));
    CHECK_THAT(shared.data[1], Catch::Matchers::WithinAbs(0., 1e-6));
    CHECK_THAT(shared.data[2], Catch::Matchers::WithinAbs(10., 1e-6));
    CHECK_THAT(shared.data[3], Catch::Matchers::WithinAbs(30., 1e-6));
    CHECK_THAT(shared.zs[0], Catch::Matchers::WithinAbs(0., 1e-6));
    CHECK_THAT(shared.zs[1], Catch::Matchers::WithinAbs(40., 1e-6));
}

TEST_CASE("Unlimited circular extruder areas preserve the independent nozzle mask",
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

} // namespace volume_geometry_contracts
