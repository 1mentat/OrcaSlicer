#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <set>
#include <tuple>
#include <vector>

#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/CutSurface.hpp"
#include "libslic3r/Emboss.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/libslic3r.h"

using namespace Slic3r;
using Catch::Matchers::WithinAbs;

namespace {

indexed_triangle_set mask_fixture()
{
    return {
        {
            {2, 0, 3},
            {0, 1, 2},
        },
        {
            stl_vertex(0.f, 0.f, 0.f),
            stl_vertex(1.f, 0.f, 0.f),
            stl_vertex(0.f, 1.f, 0.f),
            stl_vertex(2.f, 2.f, 0.f),
        },
    };
}

indexed_triangle_set aoi_fixture()
{
    return {
        {
            {0, 1, 2}, // Entirely inside the query box.
            {3, 4, 5}, // Partly inside: one corner in the box, the others beyond its top and right.
            {6, 7, 8}, // Entirely outside the query box.
        },
        {
            stl_vertex(0.f, 0.f, 0.f),
            stl_vertex(4.f, 0.f, 0.f),
            stl_vertex(0.f, 4.f, 0.f),
            stl_vertex(4.f, 4.f, 0.f),
            stl_vertex(8.f, 4.f, 0.f),
            stl_vertex(4.f, 8.f, 0.f),
            stl_vertex(8.f, 8.f, 0.f),
            stl_vertex(9.f, 8.f, 0.f),
            stl_vertex(8.f, 9.f, 0.f),
        },
    };
}

void check_index(const stl_triangle_vertex_indices &triangle, int a, int b, int c)
{
    CHECK(triangle[0] == a);
    CHECK(triangle[1] == b);
    CHECK(triangle[2] == c);
}

using Corners = std::array<std::tuple<float, float, float>, 3>;

// The corner coordinates of each face, so faces can be compared across re-indexed meshes.
std::vector<Corners> face_corners(const indexed_triangle_set &mesh)
{
    std::vector<Corners> faces;
    for (const stl_triangle_vertex_indices &face : mesh.indices) {
        Corners corners;
        for (int i = 0; i < 3; ++i) {
            const Vec3f &v = mesh.vertices[face[i]];
            corners[i] = {v.x(), v.y(), v.z()};
        }
        faces.push_back(corners);
    }
    return faces;
}

// The faces of `mesh` selected by `keep`, in order.
std::vector<Corners> face_corners(const indexed_triangle_set &mesh, const std::vector<bool> &keep)
{
    const std::vector<Corners> all = face_corners(mesh);
    std::vector<Corners> kept;
    for (size_t i = 0; i < all.size(); ++i)
        if (keep[i])
            kept.push_back(all[i]);
    return kept;
}

size_t referenced_vertex_count(const indexed_triangle_set &mesh, const std::vector<bool> &keep)
{
    std::set<int> used;
    for (size_t i = 0; i < mesh.indices.size(); ++i)
        if (keep[i])
            used.insert(mesh.indices[i].begin(), mesh.indices[i].end());
    return used.size();
}

} // namespace

TEST_CASE("its_mask compacts only vertices referenced by selected triangles", "[CutSurface]")
{
    const indexed_triangle_set input = mask_fixture();

    // Vertices are renumbered in the order the selected face first uses them.
    const indexed_triangle_set selected = its_mask(input, {true, false});
    REQUIRE(selected.indices.size() == 1);
    REQUIRE(selected.vertices.size() == 3);
    check_index(selected.indices.front(), 0, 1, 2);
    CHECK(selected.vertices[0] == input.vertices[2]);
    CHECK(selected.vertices[1] == input.vertices[0]);
    CHECK(selected.vertices[2] == input.vertices[3]);

    const indexed_triangle_set empty = its_mask(input, {false, false});
    CHECK(empty.indices.empty());
    CHECK(empty.vertices.empty());
}

TEST_CASE("its_mask keeps a fully selected mesh intact", "[CutSurface]")
{
    const indexed_triangle_set input = its_make_cube(10., 20., 30.);
    const indexed_triangle_set selected = its_mask(input, std::vector<bool>(input.indices.size(), true));

    CHECK(selected.vertices.size() == input.vertices.size());
    CHECK(face_corners(selected) == face_corners(input));
    CHECK_THAT(its_volume(selected), WithinAbs(its_volume(input), 1e-4));
}

TEST_CASE("its_mask keeps only the vertices of a sparse face selection", "[CutSurface]")
{
    const indexed_triangle_set input = its_make_cube(10., 20., 30.);
    std::vector<bool> mask(input.indices.size(), false);
    for (size_t i = 0; i < mask.size(); i += 3)
        mask[i] = true;

    const indexed_triangle_set selected = its_mask(input, mask);
    CHECK(face_corners(selected) == face_corners(input, mask));
    CHECK(selected.vertices.size() == referenced_vertex_count(input, mask));
}

TEST_CASE("its_mask compacts disjoint selected faces without retaining unreferenced vertices", "[CutSurface]")
{
    // Faces 0 (corners 0, 1, 2 on the bottom) and 2 (corners 4, 5, 6 on the top) share no vertex.
    const indexed_triangle_set input = its_make_cube(10., 20., 30.);
    std::vector<bool> mask(input.indices.size(), false);
    mask[0] = true;
    mask[2] = true;

    const indexed_triangle_set selected = its_mask(input, mask);
    CHECK(face_corners(selected) == face_corners(input, mask));
    CHECK(selected.vertices.size() == 6);
}

TEST_CASE("its_cut_AoI keeps triangles inside or partly inside the projected bounds and drops one outside", "[CutSurface]")
{
    const indexed_triangle_set input = aoi_fixture();
    const indexed_triangle_set clipped = its_cut_AoI(input, BoundingBox(Point{0, 0}, Point{5, 5}), Emboss::ProjectZ(5.0));
    CHECK(face_corners(clipped) == face_corners(input, {true, true, false}));
    CHECK(clipped.vertices.size() == 6);
}

TEST_CASE("its_cut_AoI keeps the same faces when the projection is reflected", "[CutSurface]")
{
    const indexed_triangle_set input = aoi_fixture();
    const BoundingBox query(Point{0, 0}, Point{5, 5});
    const indexed_triangle_set reflected = its_cut_AoI(input, query, Emboss::ProjectZ(-5.0));
    CHECK(face_corners(reflected) == face_corners(input, {true, true, false}));
}

TEST_CASE("its_cut_AoI keeps triangles lying on the projected boundary", "[CutSurface]")
{
    // A vertical triangle standing on the box's x = 0 edge.
    const indexed_triangle_set input = {
        {{0, 1, 2}},
        {stl_vertex(0.f, 1.f, 0.f), stl_vertex(0.f, 4.f, 0.f), stl_vertex(0.f, 1.f, 1.f)}};
    const indexed_triangle_set selected = its_cut_AoI(input, BoundingBox(Point{0, 0}, Point{3, 5}), Emboss::ProjectZ(5.0));
    CHECK(face_corners(selected) == face_corners(input));
}

TEST_CASE("its_cut_AoI keeps a triangle crossing the projected bounds with every vertex outside", "[CutSurface]")
{
    // Left of, right of and below the 3 x 3 box: no single side holds all three, and the edge at y = 1
    // passes through the box.
    const indexed_triangle_set input = {
        {{0, 1, 2}},
        {stl_vertex(-2.f, 1.f, 0.f), stl_vertex(5.f, 1.f, 0.f), stl_vertex(1.5f, -3.f, 0.f)}};
    const indexed_triangle_set selected = its_cut_AoI(input, BoundingBox(Point(0, 0), Point(3, 3)), Emboss::ProjectZ(5.));
    CHECK(face_corners(selected) == face_corners(input));
}

TEST_CASE("its_cut_AoI retains the complete mesh when the projected bounds contain it", "[CutSurface]")
{
    const indexed_triangle_set input = its_make_cube(10., 20., 30.);
    const indexed_triangle_set selected = its_cut_AoI(input, BoundingBox(Point(-1, -1), Point(11, 21)), Emboss::ProjectZ(40.));
    CHECK(face_corners(selected) == face_corners(input));
}

TEST_CASE("its_cut_AoI drops every face of a mesh outside the projected bounds", "[CutSurface]")
{
    // Faces are classified in a tbb::parallel_for. A flat 100 x 100 grid (20000 faces) all outside the bounds
    // makes every face write its skip flag, and is large enough for TBB to split the range across workers, so
    // writes that share storage (such as bits of one word in a packed bit vector) really run concurrently.
    // Repeating the cut turns an occasional lost flag into a reliable failure.
    indexed_triangle_set grid;
    const int cells = 100;
    for (int y = 0; y <= cells; ++y)
        for (int x = 0; x <= cells; ++x)
            grid.vertices.emplace_back(float(x), float(y), 0.f);
    for (int y = 0; y < cells; ++y)
        for (int x = 0; x < cells; ++x) {
            const int v = y * (cells + 1) + x;
            grid.indices.emplace_back(v, v + 1, v + cells + 2);
            grid.indices.emplace_back(v, v + cells + 2, v + cells + 1);
        }
    REQUIRE(grid.indices.size() == 20000);

    int failed_runs = 0, first_failed_run = -1;
    size_t first_failed_faces = 0;
    for (int run = 0; run < 50; ++run) {
        const size_t kept = its_cut_AoI(grid, BoundingBox(Point(200, 200), Point(210, 210)), Emboss::ProjectZ(40.)).indices.size();
        if (kept > 0 && first_failed_run < 0) {
            first_failed_run = run;
            first_failed_faces = kept;
        }
        failed_runs += kept > 0;
    }
    INFO("first failed run " << first_failed_run << " kept " << first_failed_faces << " faces");
    CHECK(failed_runs == 0);
}

TEST_CASE("cut2model mirrors the front mesh and closes every contour edge", "[CutSurface]")
{
    SurfaceCut cut;
    cut.vertices = {
        stl_vertex(0.f, 0.f, 0.f),
        stl_vertex(2.f, 0.f, 0.f),
        stl_vertex(0.f, 2.f, 0.f),
    };
    cut.indices = {{0, 1, 2}};
    cut.contours = {{0, 1, 2}};

    const indexed_triangle_set result = cut2model(cut, Emboss::ProjectZ(3.0));
    REQUIRE(result.vertices.size() == 6);
    REQUIRE(result.indices.size() == 8);

    for (size_t i = 0; i < cut.vertices.size(); ++i) {
        CHECK(result.vertices[i] == cut.vertices[i]);
        CHECK(result.vertices[i + cut.vertices.size()] == Vec3f(cut.vertices[i].x(), cut.vertices[i].y(), 3.f));
    }

    // Front face, back face with reversed winding, then two triangles per contour edge.
    check_index(result.indices[0], 0, 1, 2);
    check_index(result.indices[1], 3, 5, 4);
    check_index(result.indices[2], 0, 2, 3);
    check_index(result.indices[3], 2, 5, 3);
    check_index(result.indices[4], 1, 0, 4);
    check_index(result.indices[5], 0, 3, 4);
    check_index(result.indices[6], 2, 1, 5);
    check_index(result.indices[7], 1, 4, 5);
    CHECK(its_num_open_edges(result) == 0);
}

TEST_CASE("cut2model closes multiple contour loops", "[CutSurface]")
{
    // A 4 x 4 square ring around a 2 x 2 hole; the hole's contour runs the opposite way.
    SurfaceCut cut;
    cut.vertices = {
        stl_vertex(0.f, 0.f, 0.f), stl_vertex(4.f, 0.f, 0.f), stl_vertex(4.f, 4.f, 0.f), stl_vertex(0.f, 4.f, 0.f),
        stl_vertex(1.f, 1.f, 0.f), stl_vertex(3.f, 1.f, 0.f), stl_vertex(3.f, 3.f, 0.f), stl_vertex(1.f, 3.f, 0.f)};
    cut.indices = {{0, 1, 5}, {0, 5, 4}, {1, 2, 6}, {1, 6, 5},
                   {2, 3, 7}, {2, 7, 6}, {3, 0, 4}, {3, 4, 7}};
    cut.contours = {{0, 1, 2, 3}, {4, 7, 6, 5}};

    const indexed_triangle_set result = cut2model(cut, Emboss::OrthoProject3d(Vec3d(0., 0., 5.)));
    REQUIRE(result.vertices.size() == cut.vertices.size() * 2);
    // Front and back faces plus two wall triangles for each of the 4 + 4 contour edges.
    CHECK(result.indices.size() == cut.indices.size() * 2 + 2 * (4 + 4));
    for (size_t i = 0; i < cut.vertices.size(); ++i) {
        CHECK(result.vertices[i] == cut.vertices[i]);
        CHECK(result.vertices[i + cut.vertices.size()] == cut.vertices[i] + Vec3f(0.f, 0.f, 5.f));
    }
    CHECK(its_num_open_edges(result) == 0);
    CHECK_THAT(std::abs(its_volume(result)), WithinAbs((16. - 4.) * 5., 1e-4));
}

TEST_CASE("cut2model follows a non-axis projection vector", "[CutSurface]")
{
    SurfaceCut cut;
    cut.vertices = {stl_vertex(0.f, 0.f, 0.f), stl_vertex(2.f, 0.f, 0.f), stl_vertex(0.f, 2.f, 0.f)};
    cut.indices = {{0, 1, 2}};
    cut.contours = {{0, 1, 2}};
    const Vec3d direction(1., -2., 4.);
    const indexed_triangle_set result = cut2model(cut, Emboss::OrthoProject3d(direction));

    REQUIRE(result.vertices.size() == 6);
    for (size_t i = 0; i < cut.vertices.size(); ++i)
        CHECK(result.vertices[i + cut.vertices.size()] == cut.vertices[i] + direction.cast<float>());
    CHECK(its_num_open_edges(result) == 0);
    // A sheared prism over a 2 mm^2 triangle, 4 mm tall along Z.
    CHECK_THAT(std::abs(its_volume(result)), WithinAbs(2. * 4., 1e-4));
}
