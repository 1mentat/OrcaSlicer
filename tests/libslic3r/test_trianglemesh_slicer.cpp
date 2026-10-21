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
TEST_CASE("Slab slicing projects the same polygons whatever the thread schedule", "[TriangleMeshSlicer][Parallel]")
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

indexed_triangle_set vertical_panel_mesh()
{
    indexed_triangle_set mesh;
    mesh.vertices = {
        Vec3f(0.f, 0.f, 0.f), Vec3f(10.f, 0.f, 0.f),
        Vec3f(10.f, 0.f, 10.f), Vec3f(0.f, 0.f, 10.f)
    };
    mesh.indices = { Vec3i32(0, 1, 2), Vec3i32(0, 2, 3) };
    return mesh;
}

indexed_triangle_set coplanar_up_and_down_mesh()
{
    indexed_triangle_set mesh;
    mesh.vertices = {
        Vec3f(0.f, 0.f, 5.f), Vec3f(10.f, 0.f, 5.f), Vec3f(0.f, 10.f, 5.f),
        Vec3f(20.f, 0.f, 5.f), Vec3f(20.f, 10.f, 5.f), Vec3f(30.f, 0.f, 5.f)
    };
    // The first triangle has +Z winding; the second has -Z winding.
    mesh.indices = { Vec3i32(0, 1, 2), Vec3i32(3, 4, 5) };
    return mesh;
}

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

TEST_CASE("Slab slicing reports vertical face centers and normals", "[TriangleMeshSlicer][VerticalPoints][TriangleMeshSlicerContracts]")
{
    std::vector<std::pair<Vec3f, Vec3f>> vertical_points;
    slice_mesh_slabs(vertical_panel_mesh(), { 0.f, 5.f, 10.f }, Transform3d::Identity(),
                     nullptr, nullptr, &vertical_points, []{});

    REQUIRE(vertical_points.size() == 2);
    std::sort(vertical_points.begin(), vertical_points.end(), [](const auto &left, const auto &right) {
        return left.first.x() < right.first.x();
    });
    CHECK_THAT(double(vertical_points[0].first.x()) * SCALING_FACTOR, Catch::Matchers::WithinAbs(10.0 / 3.0, 1e-5));
    CHECK_THAT(double(vertical_points[0].first.z()), Catch::Matchers::WithinAbs(20.0 / 3.0, 1e-5));
    CHECK_THAT(double(vertical_points[1].first.x()) * SCALING_FACTOR, Catch::Matchers::WithinAbs(20.0 / 3.0, 1e-5));
    CHECK_THAT(double(vertical_points[1].first.z()), Catch::Matchers::WithinAbs(10.0 / 3.0, 1e-5));
    for (const auto &[center, normal] : vertical_points) {
        CHECK_THAT(center.y(), Catch::Matchers::WithinAbs(0.f, 1e-5));
        CHECK_THAT(normal.norm(), Catch::Matchers::WithinAbs(1.f, 1e-5));
        CHECK_THAT(std::abs(normal.x()), Catch::Matchers::WithinAbs(0.f, 1e-5));
        CHECK_THAT(std::abs(normal.y()), Catch::Matchers::WithinAbs(1.f, 1e-5));
        CHECK_THAT(std::abs(normal.z()), Catch::Matchers::WithinAbs(0.f, 1e-5));
    }
}

TEST_CASE("Slab slicing assigns coplanar faces to their documented surface", "[TriangleMeshSlicer][Slab][TriangleMeshSlicerContracts]")
{
    const auto mesh = coplanar_up_and_down_mesh();
    const std::vector<float> zs { 5.f, 10.f };

    std::vector<Polygons> top;
    slice_mesh_slabs(mesh, zs, Transform3d::Identity(), &top, nullptr, nullptr, []{});
    REQUIRE(top.size() == zs.size());
    REQUIRE(top[0].size() == 1);
    CHECK(top[1].empty());
    require_xy_bounds(top[0].front(), 0., 10., 0., 10.);

    std::vector<Polygons> bottom;
    slice_mesh_slabs(mesh, zs, Transform3d::Identity(), nullptr, &bottom, nullptr, []{});
    REQUIRE(bottom.size() == zs.size());
    CHECK(bottom[0].empty());
    REQUIRE(bottom[1].size() == 1);
    require_xy_bounds(bottom[1].front(), 20., 30., 0., 10.);
}

TEST_CASE("Mirrored slab transforms preserve surface roles", "[TriangleMeshSlicer][Slab][Transform][TriangleMeshSlicerContracts]")
{
    Transform3d mirror = Transform3d::Identity();
    mirror.prescale(Vec3d(-1., 1., 1.));

    std::vector<Polygons> top, bottom;
    slice_mesh_slabs(coplanar_up_and_down_mesh(), { 5.f, 10.f }, mirror,
                     &top, &bottom, nullptr, []{});
    REQUIRE(top.size() == 2);
    REQUIRE(bottom.size() == 2);
    REQUIRE(top[0].size() == 1);
    CHECK(bottom[0].empty());
    CHECK(top[1].empty());
    REQUIRE(bottom[1].size() == 1);
    require_xy_bounds(top[0].front(), -10., 0., 0., 10.);
    require_xy_bounds(bottom[1].front(), -30., -20., 0., 10.);
}

TEST_CASE("Mesh projection exposes each requested surface independently", "[TriangleMeshSlicer][Projection][TriangleMeshSlicerContracts]")
{
    const auto cube = its_make_cube(10., 20., 30.);
    Polygons top;
    project_mesh(cube, Transform3d::Identity(), &top, nullptr, []{});
    REQUIRE(top.size() == 1);
    CHECK_THAT(std::abs(double(top.front().area())) * SCALING_FACTOR * SCALING_FACTOR, Catch::Matchers::WithinAbs(200.0, 1e-4));
    CHECK_THAT(unscaled<double>(top.front().bounding_box().size().x()), Catch::Matchers::WithinAbs(10., 1e-4));
    CHECK_THAT(unscaled<double>(top.front().bounding_box().size().y()), Catch::Matchers::WithinAbs(20., 1e-4));

    Polygons bottom;
    project_mesh(cube, Transform3d::Identity(), nullptr, &bottom, []{});
    REQUIRE(bottom.size() == 1);
    CHECK_THAT(std::abs(double(bottom.front().area())) * SCALING_FACTOR * SCALING_FACTOR, Catch::Matchers::WithinAbs(200.0, 1e-4));
    CHECK_THAT(unscaled<double>(bottom.front().bounding_box().size().x()), Catch::Matchers::WithinAbs(10., 1e-4));
    CHECK_THAT(unscaled<double>(bottom.front().bounding_box().size().y()), Catch::Matchers::WithinAbs(20., 1e-4));
}

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

namespace cut_mesh_plane_contracts {

TriangleMesh box_with_middle_edge_ring()
{
    std::vector<Vec3f> vertices;
    vertices.reserve(12);
    for (int layer = 0; layer < 3; ++layer) {
        const float z = 10.f * float(layer);
        vertices.emplace_back(0.f, 0.f, z);
        vertices.emplace_back(20.f, 0.f, z);
        vertices.emplace_back(20.f, 20.f, z);
        vertices.emplace_back(0.f, 20.f, z);
    }

    std::vector<Vec3i32> facets { { 0, 2, 1 }, { 0, 3, 2 }, { 8, 9, 10 }, { 8, 10, 11 } };
    for (int layer = 0; layer < 2; ++layer) {
        const int lower = 4 * layer;
        const int upper = lower + 4;
        for (int edge = 0; edge < 4; ++edge) {
            const int next = (edge + 1) % 4;
            facets.emplace_back(lower + edge, lower + next, upper + next);
            facets.emplace_back(lower + edge, upper + next, upper + edge);
        }
    }
    return TriangleMesh(vertices, facets);
}

} // namespace cut_mesh_plane_contracts

TEST_CASE("The cut keeps both halves watertight when the plane follows an existing mesh edge",
          "[TriangleMeshSlicer][TriangleMeshCutMesh]")
{
    const TriangleMesh mesh = cut_mesh_plane_contracts::box_with_middle_edge_ring();
    indexed_triangle_set upper;
    indexed_triangle_set lower;

    cut_mesh(mesh.its, 10.f, &upper, &lower);

    CHECK(its_num_open_edges(upper) == 0);
    CHECK(its_num_open_edges(lower) == 0);
    CHECK_THAT(std::abs(its_volume(upper)), Catch::Matchers::WithinAbs(4000.f, 1e-2f));
    CHECK_THAT(std::abs(its_volume(lower)), Catch::Matchers::WithinAbs(4000.f, 1e-2f));
}

TEST_CASE("The cut preserves a plane vertex when adjacent facets cross the plane",
          "[TriangleMeshSlicer][TriangleMeshCutMesh]")
{
    const TriangleMesh tetrahedron(
        std::vector<Vec3f> { { 0.f, 0.f, 10.f }, { 0.f, 10.f, 0.f }, { 10.f, 0.f, 20.f }, { 0.f, 0.f, 20.f } },
        std::vector<Vec3i32> { { 0, 1, 2 }, { 0, 2, 3 }, { 0, 3, 1 }, { 1, 3, 2 } });
    indexed_triangle_set upper;
    indexed_triangle_set lower;

    cut_mesh(tetrahedron.its, 10.f, &upper, &lower);

    CHECK(its_num_open_edges(upper) == 0);
    CHECK(its_num_open_edges(lower) == 0);
    CHECK_THAT(std::abs(its_volume(upper)), Catch::Matchers::WithinAbs(125.f, 1e-2f));
    CHECK_THAT(std::abs(its_volume(lower)), Catch::Matchers::WithinAbs(125.f / 3.f, 1e-2f));

    indexed_triangle_set upper_only;
    cut_mesh(tetrahedron.its, 10.f, &upper_only, nullptr);
    CHECK(its_num_open_edges(upper_only) == 0);
    CHECK_THAT(std::abs(its_volume(upper_only)), Catch::Matchers::WithinAbs(125.f, 1e-2f));

    indexed_triangle_set lower_only;
    cut_mesh(tetrahedron.its, 10.f, nullptr, &lower_only);
    CHECK(its_num_open_edges(lower_only) == 0);
    CHECK_THAT(std::abs(its_volume(lower_only)), Catch::Matchers::WithinAbs(125.f / 3.f, 1e-2f));
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
