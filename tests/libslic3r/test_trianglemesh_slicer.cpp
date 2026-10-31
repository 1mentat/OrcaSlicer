#include <algorithm>
#include <cmath>
#include <catch2/catch_all.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "libslic3r/Point.hpp"
#include "libslic3r/Polygon.hpp"
#include <tbb/global_control.h>
#include <vector>
#include <utility>

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_message.hpp>
#include <catch2/generators/catch_generators.hpp>
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/TriangleMeshSlicer.hpp"
#include "libslic3r/libslic3r.h"

using namespace Slic3r;

// The slab slicer collects each slab's intersection lines from a parallel loop over the facets.
// Its loops, and therefore the projected polygons, are derived from the order of those lines, so
// the order has to be canonical or the same mesh projects to different polygons run to run.
// The single-threaded projection is the reference; every multi-threaded run must reproduce it
// exactly, vertex order included.
TEST_CASE("Slab slicing projects the same polygons whatever the thread schedule", "[TriangleMeshSlicer]")
{
    // A dense sphere, tilted so no facet is axis aligned: thousands of upward and downward
    // facing facets spread over every slab.
    indexed_triangle_set mesh = its_make_sphere(10., 0.05);
    Transform3d trafo = Transform3d::Identity();
    trafo.rotate(Eigen::AngleAxisd(0.37, Vec3d(0.3, 0.5, 1.).normalized()));
    trafo.translate(Vec3d(1., 2., 0.));
    std::vector<float> zs;
    for (float z = -9.7f; z < 9.7f; z += 0.2f)
        zs.emplace_back(z);

    auto project = [&mesh, &trafo, &zs]() {
        std::vector<Polygons> top, bottom;
        slice_mesh_slabs(mesh, zs, trafo, &top, &bottom, nullptr, []{});
        return std::make_pair(std::move(top), std::move(bottom));
    };

    std::pair<std::vector<Polygons>, std::vector<Polygons>> reference;
    {
        tbb::global_control single_thread(tbb::global_control::max_allowed_parallelism, 1);
        reference = project();
    }
    REQUIRE(reference.first.size() == zs.size());
    REQUIRE(std::any_of(reference.first.begin(), reference.first.end(), [](const Polygons &p) { return !p.empty(); }));
    REQUIRE(std::any_of(reference.second.begin(), reference.second.end(), [](const Polygons &p) { return !p.empty(); }));

    for (int run = 0; run < 3; ++run) {
        DYNAMIC_SECTION("multi-threaded run " << run)
        {
            auto parallel = project();
            CHECK(parallel.first == reference.first);
            CHECK(parallel.second == reference.second);
        }
    }
}


namespace {

void require_xy_bounds(const Polygon &polygon, double min_x, double max_x, double min_y, double max_y)
{
    REQUIRE_FALSE(polygon.empty());
    const auto bounds = polygon.bounding_box();
    CHECK_THAT(unscaled<double>(bounds.min.x()), Catch::Matchers::WithinAbs(min_x, 1e-4));
    CHECK_THAT(unscaled<double>(bounds.max.x()), Catch::Matchers::WithinAbs(max_x, 1e-4));
    CHECK_THAT(unscaled<double>(bounds.min.y()), Catch::Matchers::WithinAbs(min_y, 1e-4));
    CHECK_THAT(unscaled<double>(bounds.max.y()), Catch::Matchers::WithinAbs(max_y, 1e-4));
}

} // namespace

TEST_CASE("Single-plane slicing honors the top-inclusive bottom-exclusive solid boundary",
          "[TriangleMeshSlicer][SinglePlaneSlice]")
{
    const indexed_triangle_set mesh = its_make_cube(10., 20., 30.);
    const float plane_z = GENERATE(-1.f, 0.f, 15.f, 30.f, 31.f);
    CAPTURE(plane_z);
    const Polygons polygons = slice_mesh(mesh, plane_z, MeshSlicingParams{});
    if (plane_z <= 0.f || plane_z > 30.f) {
        CHECK(polygons.empty());
        return;
    }
    REQUIRE(polygons.size() == 1);
    require_xy_bounds(polygons.front(), 0., 10., 0., 20.);
    CHECK_THAT(polygons.front().area() * SCALING_FACTOR * SCALING_FACTOR,
               Catch::Matchers::WithinAbs(200., 1e-4));
}

TEST_CASE("Single-plane slicing transforms both its plane classification and contour coordinates",
          "[TriangleMeshSlicer][SinglePlaneSlice]")
{
    const indexed_triangle_set mesh = its_make_cube(10., 20., 30.);
    const bool rotate_axes = GENERATE(false, true);
    CAPTURE(rotate_axes);
    MeshSlicingParams params;
    float plane_z;
    if (rotate_axes) {
        // World x=z+2, y=y+3, z=4-x.  The z=-1 section is the
        // original x=5 plane, so its independent footprint is 30 by 20.
        params.trafo.matrix() << 0., 0., 1., 2.,
                                 0., 1., 0., 3.,
                                -1., 0., 0., 4.,
                                 0., 0., 0., 1.;
        plane_z = -1.f;
    } else {
        // World x=2x+2, y=y/2+3, z=3z/2+4; the z=19 section
        // is the original z=10 plane, with a 20 by 10 footprint.
        params.trafo.matrix() << 2., 0., 0., 2.,
                                 0., 0.5, 0., 3.,
                                 0., 0., 1.5, 4.,
                                 0., 0., 0., 1.;
        plane_z = 19.f;
    }
    const Polygons polygons = slice_mesh(mesh, plane_z, params);
    REQUIRE(polygons.size() == 1);
    require_xy_bounds(polygons.front(), 2., rotate_axes ? 32. : 22., 3., rotate_axes ? 23. : 13.);
    CHECK_THAT(polygons.front().area() * SCALING_FACTOR * SCALING_FACTOR,
               Catch::Matchers::WithinAbs(rotate_axes ? 600. : 200., 1e-4));
}

TEST_CASE("Single-plane slicing preserves hole orientation and applies the selected contour mode",
          "[TriangleMeshSlicer][SinglePlaneSlice]")
{
    indexed_triangle_set mesh = its_make_cube(10., 20., 30.);
    indexed_triangle_set hole = its_make_cube(4., 6., 30.);
    for (Vec3f &vertex : hole.vertices)
        vertex += Vec3f(3.f, 7.f, 0.f);
    for (Vec3i32 &face : hole.indices)
        std::swap(face[0], face[1]);
    its_merge(mesh, hole);

    using Mode = MeshSlicingParams::SlicingMode;
    const Mode mode = GENERATE(Mode::Regular, Mode::EvenOdd, Mode::Positive, Mode::PositiveLargestContour);
    const bool lower_layer_override = GENERATE(false, true);
    CAPTURE(static_cast<unsigned int>(mode), lower_layer_override);
    MeshSlicingParams params;
    if (lower_layer_override) {
        params.slicing_mode_normal_below_layer = 1;
        params.mode_below = mode;
    } else {
        params.mode = mode;
    }
    const Polygons polygons = slice_mesh(mesh, 15.f, params);
    REQUIRE(polygons.size() == (mode == Mode::PositiveLargestContour ? 1u : 2u));
    double signed_area = 0.;
    unsigned int holes = 0;
    for (const Polygon &polygon : polygons) {
        const double area = polygon.area() * SCALING_FACTOR * SCALING_FACTOR;
        signed_area += area;
        holes += area < 0. ? 1u : 0u;
    }
    const double expected_area = mode == Mode::PositiveLargestContour ? 200. : mode == Mode::Positive ? 224. : 176.;
    CHECK_THAT(signed_area, Catch::Matchers::WithinAbs(expected_area, 1e-4));
    const unsigned int expected_holes = mode == Mode::Regular || mode == Mode::EvenOdd ? 1u : 0u;
    CHECK(holes == expected_holes);
}


namespace trianglemesh_contour_repair_contracts {

indexed_triangle_set closed_box_triangle_soup(bool reverse_right_wall, bool rotate_facets)
{
    const std::vector<Vec3f> vertices {
        Vec3f(0.f, 0.f, 0.f), Vec3f(20.f, 0.f, 0.f), Vec3f(20.f, 14.f, 0.f), Vec3f(0.f, 14.f, 0.f),
        Vec3f(0.f, 0.f, 8.f), Vec3f(20.f, 0.f, 8.f), Vec3f(20.f, 14.f, 8.f), Vec3f(0.f, 14.f, 8.f)
    };
    // Outward-wound closed rectangular shell: bottom, top, then the four side walls.
    std::vector<Vec3i32> facets {
        { 0, 2, 1 }, { 0, 3, 2 }, { 4, 5, 6 }, { 4, 6, 7 },
        { 0, 1, 5 }, { 0, 5, 4 }, { 1, 2, 6 }, { 1, 6, 5 },
        { 2, 3, 7 }, { 2, 7, 6 }, { 3, 0, 4 }, { 3, 4, 7 }
    };
    if (reverse_right_wall) {
        std::swap(facets[6][1], facets[6][2]);
        std::swap(facets[7][1], facets[7][2]);
    }
    if (rotate_facets)
        std::rotate(facets.begin(), facets.begin() + 5, facets.end());

    // Give every triangle three fresh vertex indices while retaining exactly coincident coordinates.
    // The closed surface is geometric, but its triangle connectivity is intentionally split.
    indexed_triangle_set soup;
    soup.vertices.reserve(facets.size() * 3);
    soup.indices.reserve(facets.size());
    for (const Vec3i32 &facet : facets) {
        Vec3i32 split_facet;
        for (int corner = 0; corner < 3; ++corner) {
            split_facet[corner] = static_cast<int>(soup.vertices.size());
            soup.vertices.emplace_back(vertices[facet[corner]]);
        }
        soup.indices.emplace_back(split_facet);
    }
    return soup;
}

} // namespace trianglemesh_contour_repair_contracts

TEST_CASE("Single-plane slicing closes a box with coincident split triangle vertices",
          "[TriangleMeshSlicer][SinglePlaneSlice]")
{
    const auto soup = trianglemesh_contour_repair_contracts::closed_box_triangle_soup(false, false);
    const Polygons contours = slice_mesh(soup, 4.f, MeshSlicingParams{});

    REQUIRE(contours.size() == 1);
    const Polygon &contour = contours.front();
    require_xy_bounds(contour, 0., 20., 0., 14.);
    CHECK_THAT(std::abs(contour.area()) * SCALING_FACTOR * SCALING_FACTOR,
               Catch::Matchers::WithinAbs(280., 1e-4));
}

TEST_CASE("Single-plane slicing joins reversed split facets into one contour",
          "[TriangleMeshSlicer][SinglePlaneSlice]")
{
    const auto soup = trianglemesh_contour_repair_contracts::closed_box_triangle_soup(true, true);
    const Polygons contours = slice_mesh(soup, 4.f, MeshSlicingParams{});

    REQUIRE(contours.size() == 1);
    const Polygon &contour = contours.front();
    require_xy_bounds(contour, 0., 20., 0., 14.);
    CHECK_THAT(std::abs(contour.area()) * SCALING_FACTOR * SCALING_FACTOR,
               Catch::Matchers::WithinAbs(280., 1e-4));
    // The reversed-orientation repair path documents CCW normalization for patched contours.
    CHECK(contour.area() > 0.);
}
