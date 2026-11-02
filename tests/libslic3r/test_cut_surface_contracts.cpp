#include <catch2/catch_message.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <boost/filesystem/operations.hpp>
#include <boost/nowide/fstream.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <set>
#include <string>
#include <tuple>
#include <vector>

#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/CutSurface.hpp"
#include "libslic3r/Emboss.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/libslic3r.h"

#include "test_utils.hpp"

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

namespace cut_surface_topology_contracts {

namespace {

Polygon rectangle(coord_t x0, coord_t y0, coord_t x1, coord_t y1)
{
    return Polygon(Points{Point(x0, y0), Point(x1, y0), Point(x1, y1), Point(x0, y1)});
}

// Every face indexes valid, distinct vertices and has non-zero area.
void check_mesh_indices(const indexed_triangle_set &mesh)
{
    size_t bad_faces = 0;
    for (const auto &face : mesh.indices) {
        REQUIRE(face[0] < int(mesh.vertices.size()));
        REQUIRE(face[1] < int(mesh.vertices.size()));
        REQUIRE(face[2] < int(mesh.vertices.size()));
        const Vec3d edge_a = mesh.vertices[face[1]].cast<double>() - mesh.vertices[face[0]].cast<double>();
        const Vec3d edge_b = mesh.vertices[face[2]].cast<double>() - mesh.vertices[face[0]].cast<double>();
        bad_faces += !(edge_a.cross(edge_b).squaredNorm() > 0.);
    }
    CHECK(bad_faces == 0);
}

// The number of contours, each of at least three valid vertices.
size_t contour_count(const SurfaceCut &cut)
{
    for (const SurfaceCut::Contour &contour : cut.contours) {
        REQUIRE(contour.size() >= 3);
        for (const SurfaceCut::Index index : contour)
            REQUIRE(index < cut.vertices.size());
    }
    return cut.contours.size();
}

double projected_area(const indexed_triangle_set &mesh)
{
    double area = 0.;
    for (const auto &face : mesh.indices) {
        const Vec3d a = mesh.vertices[face[0]].cast<double>();
        const Vec3d b = mesh.vertices[face[1]].cast<double>();
        const Vec3d c = mesh.vertices[face[2]].cast<double>();
        area += std::abs((b.x() - a.x()) * (c.y() - a.y()) - (b.y() - a.y()) * (c.x() - a.x())) / 2.;
    }
    return area;
}

BoundingBoxf3 vertex_bounds(const SurfaceCut &cut)
{
    BoundingBoxf3 box;
    for (const Vec3f &vertex : cut.vertices)
        box.merge(vertex.cast<double>());
    return box;
}

// Requires a flat patch lying on the plane z = `z`.
// cut_surface only cuts faces whose normal points along the projection direction (set_skip_by_angle keeps faces
// within 89.9 degrees of it), so for these +Z projections the candidates are the cubes' top faces. The
// projection ratio then picks the candidate closest to front + ratio * depth.
void check_on_plane(const SurfaceCut &cut, double z)
{
    const BoundingBoxf3 box = vertex_bounds(cut);
    CHECK_THAT(box.min.z(), WithinAbs(z, 1e-4));
    CHECK_THAT(box.max.z(), WithinAbs(z, 1e-4));
}

indexed_triangle_set shifted(indexed_triangle_set mesh, const Vec3f &delta)
{
    for (Vec3f &vertex : mesh.vertices)
        vertex += delta;
    return mesh;
}

// Projects shape points (1 unit = 1 mm) along +Z from z = -10, `depth` mm deep.
Emboss::OrthoProject cube_projection(float depth = 30.f)
{
    Transform3d matrix = Transform3d::Identity();
    matrix.translate(Vec3d(0., 0., -10.));
    return Emboss::OrthoProject(matrix, Vec3d(0., 0., depth));
}

// Two 10 mm cubes, at x = 0..10 and x = 20..30.
indexed_triangle_set second_cube() { return shifted(its_make_cube(10., 10., 10.), Vec3f(20.f, 0.f, 0.f)); }

// The cut of a 1..29 x 1..9 strip over both cubes: a 9 x 8 patch on each cube's top face.
void check_two_cube_patches(const SurfaceCut &cut)
{
    check_mesh_indices(cut);
    CHECK(contour_count(cut) == 2);
    CHECK_THAT(projected_area(cut), WithinAbs(2. * 9. * 8., 1e-4));
    check_on_plane(cut, 10.);
    size_t left = 0, right = 0;
    for (const Vec3f &vertex : cut.vertices) {
        left += vertex.x() <= 10.f;
        right += vertex.x() >= 20.f;
    }
    CHECK(left > 0);
    CHECK(right > 0);
    CHECK(left + right == cut.vertices.size());
}

} // namespace

TEST_CASE("cut_surface keeps the part of the facing cube side inside the projected shape", "[CutSurface]")
{
    const SurfaceCut cut = cut_surface({ExPolygon(rectangle(1, 1, 9, 9))}, {its_make_cube(10., 10., 10.)}, cube_projection(), 0.f);
    check_mesh_indices(cut);
    CHECK(contour_count(cut) == 1);
    CHECK_THAT(projected_area(cut), WithinAbs(64., 1e-4));
    // The patch lies on the cube's top face (the only face pointing along the projection), spanning the shape.
    check_on_plane(cut, 10.);
    const BoundingBoxf3 box = vertex_bounds(cut);
    CHECK_THAT(box.min.x(), WithinAbs(1., 1e-4));
    CHECK_THAT(box.max.x(), WithinAbs(9., 1e-4));
    CHECK_THAT(box.min.y(), WithinAbs(1., 1e-4));
    CHECK_THAT(box.max.y(), WithinAbs(9., 1e-4));
}

TEST_CASE("cut_surface returns no patch for a projected shape outside the model", "[CutSurface]")
{
    CHECK(cut_surface({ExPolygon(rectangle(20, 20, 30, 30))}, {its_make_cube(10., 10., 10.)}, cube_projection(), .5f).empty());
}

TEST_CASE("cut_surface keeps a shape hole as its own contour", "[CutSurface]")
{
    ExPolygon shape(rectangle(1, 1, 9, 9));
    shape.holes.emplace_back(rectangle(4, 4, 6, 6));
    shape.holes.back().reverse();
    const SurfaceCut cut = cut_surface({shape}, {its_make_cube(10., 10., 10.)}, cube_projection(), .5f);
    check_mesh_indices(cut);
    CHECK(contour_count(cut) == 2);
    CHECK_THAT(projected_area(cut), WithinAbs(64. - 4., 1e-4));
}

TEST_CASE("cut_surface accepts multiple disjoint projected shapes", "[CutSurface]")
{
    const ExPolygons shapes{ExPolygon(rectangle(1, 1, 3, 3)), ExPolygon(rectangle(6, 5, 9, 9))};
    const SurfaceCut cut = cut_surface(shapes, {its_make_cube(10., 10., 10.)}, cube_projection(), .25f);
    check_mesh_indices(cut);
    CHECK(contour_count(cut) == 2);
    CHECK_THAT(projected_area(cut), WithinAbs(2. * 2. + 3. * 4., 1e-4));
}

TEST_CASE("cut_surface handles two disjoint source meshes in one projection", "[CutSurface]")
{
    const SurfaceCut cut = cut_surface({ExPolygon(rectangle(1, 1, 29, 9))}, {its_make_cube(10., 10., 10.), second_cube()},
                                       cube_projection(), .5f);
    check_two_cube_patches(cut);
}

TEST_CASE("Surface cuts preserve disconnected components in one source mesh", "[CutSurface]")
{
    indexed_triangle_set combined = its_make_cube(10., 10., 10.);
    const indexed_triangle_set second = second_cube();
    const int offset = int(combined.vertices.size());
    combined.vertices.insert(combined.vertices.end(), second.vertices.begin(), second.vertices.end());
    for (const auto &face : second.indices)
        combined.indices.emplace_back(face[0] + offset, face[1] + offset, face[2] + offset);

    check_two_cube_patches(cut_surface({ExPolygon(rectangle(1, 1, 29, 9))}, {combined}, cube_projection(), .5f));
}

TEST_CASE("Surface cuts select the requested depth across stacked models", "[CutSurface]")
{
    // A near cube at z = 0..10 and a far one at z = 20..30 under the same shape, projected from z = -10 over 50 mm.
    // The candidates are the top faces at z = 10 and 30; the ratio picks the one closest to -10 + 50 * ratio, so
    // ratios 0 and 0.5 (targets -10 and 15) cut the near cube and ratios 0.75 and 1 (27.5 and 40) the far one.
    const ExPolygons shapes{ExPolygon(rectangle(1, 1, 9, 9))};
    const indexed_triangle_set near_model = its_make_cube(10., 10., 10.);
    const indexed_triangle_set far_model = shifted(its_make_cube(10., 10., 10.), Vec3f(0.f, 0.f, 20.f));
    const SurfaceCut front = cut_surface(shapes, {near_model, far_model}, cube_projection(50.f), 0.f);
    const SurfaceCut back = cut_surface(shapes, {near_model, far_model}, cube_projection(50.f), 1.f);
    check_mesh_indices(front);
    check_mesh_indices(back);
    CHECK(contour_count(front) == 1);
    CHECK(contour_count(back) == 1);
    CHECK_THAT(projected_area(front), WithinAbs(64., 1e-4));
    CHECK_THAT(projected_area(back), WithinAbs(64., 1e-4));
    check_on_plane(front, 10.);
    check_on_plane(back, 30.);
    check_on_plane(cut_surface(shapes, {near_model, far_model}, cube_projection(50.f), .5f), 10.);
    check_on_plane(cut_surface(shapes, {near_model, far_model}, cube_projection(50.f), .75f), 30.);
}

TEST_CASE("Surface cuts preserve multiple projected holes inside the face", "[CutSurface]")
{
    // A 1 mm inset 8 x 8 shape with two 2 x 2 holes.
    ExPolygon shape(rectangle(1, 1, 9, 9));
    shape.holes.emplace_back(rectangle(2, 2, 4, 4));
    shape.holes.back().reverse();
    shape.holes.emplace_back(rectangle(6, 6, 8, 8));
    shape.holes.back().reverse();
    const SurfaceCut cut = cut_surface({shape}, {its_make_cube(10., 10., 10.)}, cube_projection(), .5f);
    check_mesh_indices(cut);
    CHECK(contour_count(cut) == 3);
    CHECK_THAT(projected_area(cut), WithinAbs(64. - 2. * 4., 1e-4));
}

// Known bug: when the projected shape's outline coincides with the edge of the cut face, cut_surface classifies
// faces through an invalid face descriptor (out-of-range write; asserts in debug), so it crashes or corrupts the
// result; fixed by the follow-up fix PR.
TEST_CASE("Surface cuts preserve multiple holes when the shape outline matches the face edge", "[CutSurface][.]")
{
    ExPolygon shape(rectangle(0, 0, 10, 10));
    shape.holes.emplace_back(rectangle(2, 2, 4, 4));
    shape.holes.back().reverse();
    shape.holes.emplace_back(rectangle(6, 6, 8, 8));
    shape.holes.back().reverse();
    const SurfaceCut cut = cut_surface({shape}, {its_make_cube(10., 10., 10.)}, cube_projection(), .5f);
    check_mesh_indices(cut);
    CHECK(contour_count(cut) == 3);
    CHECK_THAT(projected_area(cut), WithinAbs(100. - 2. * 4., 1e-4));
}

// Shape coordinates in scaled units (1 mm = 1e6), as the emboss tool produces them, projected back to millimetres.
Polygon scaled_rectangle(double x0, double y0, double x1, double y1)
{
    return rectangle(scale_(x0), scale_(y0), scale_(x1), scale_(y1));
}

Emboss::OrthoProject scaled_cube_projection(float depth)
{
    Transform3d matrix = Transform3d::Identity();
    matrix.translate(Vec3d(0., 0., -10.));
    matrix.scale(Vec3d(SCALING_FACTOR, SCALING_FACTOR, 1.));
    return Emboss::OrthoProject(matrix, Vec3d(0., 0., depth));
}

// The cut of a 1..29 x 1..9 strip over three 10 mm cubes side by side (x = 0..10, 10..20, 20..30) is one 28 x 8
// patch, split into the three cubes' top faces. Vertices strictly inside the middle cube or an outer cube pin
// which candidate supplied each part; vertices on the shared edges (x = 10, 20) may belong to either.
void check_three_cube_cut(const SurfaceCut &cut, double middle_z)
{
    check_mesh_indices(cut);
    CHECK(contour_count(cut) >= 3);
    CHECK_THAT(projected_area(cut), WithinAbs(28. * 8., 1e-4));
    const BoundingBoxf3 box = vertex_bounds(cut);
    CHECK_THAT(box.min.x(), WithinAbs(1., 1e-4));
    CHECK_THAT(box.max.x(), WithinAbs(29., 1e-4));
    CHECK_THAT(box.min.y(), WithinAbs(1., 1e-4));
    CHECK_THAT(box.max.y(), WithinAbs(9., 1e-4));
    size_t outer = 0, middle = 0;
    for (const Vec3f &vertex : cut.vertices) {
        if (vertex.x() < 9.99f || vertex.x() > 20.01f) {
            CHECK_THAT(vertex.z(), WithinAbs(10., 1e-4));
            ++outer;
        } else if (vertex.x() > 10.01f && vertex.x() < 19.99f) {
            CHECK_THAT(vertex.z(), WithinAbs(middle_z, 1e-4));
            ++middle;
        }
    }
    CHECK(outer > 0);
    CHECK(middle > 0);
}

TEST_CASE("Surface cuts fill the gap between two outer cubes from an unselected middle cube", "[CutSurface]")
{
    const ExPolygon shape(scaled_rectangle(1., 1., 29., 9.));
    // The outer cubes are listed first and touch the shape ends; the middle cube lies entirely in the gap.
    const SurfaceCut cut = cut_surface({shape},
        {its_make_cube(10., 10., 10.), shifted(its_make_cube(10., 10., 10.), Vec3f(20.f, 0.f, 0.f)),
         shifted(its_make_cube(10., 10., 10.), Vec3f(10.f, 0.f, 0.f))},
        scaled_cube_projection(80.f), .5f);
    check_three_cube_cut(cut, 10.);
}

TEST_CASE("Surface cuts choose the closest of two stacked middle cubes to the projection ratio", "[CutSurface]")
{
    const ExPolygon shape(scaled_rectangle(1., 1., 29., 9.));
    // Two candidates cover the middle gap, with tops at z = 30 and z = 50. Ratio 0 targets the front of the
    // projection, so the lower one wins; ratio 1 targets the back, so the higher one wins.
    const std::vector<indexed_triangle_set> models{
        its_make_cube(10., 10., 10.), shifted(its_make_cube(10., 10., 10.), Vec3f(20.f, 0.f, 0.f)),
        shifted(its_make_cube(10., 10., 10.), Vec3f(10.f, 0.f, 20.f)),
        shifted(its_make_cube(10., 10., 10.), Vec3f(10.f, 0.f, 40.f))};
    check_three_cube_cut(cut_surface({shape}, models, scaled_cube_projection(100.f), 0.f), 30.);
    check_three_cube_cut(cut_surface({shape}, models, scaled_cube_projection(100.f), 1.f), 50.);
}

} // namespace cut_surface_topology_contracts

namespace cut_surface_corefine_contracts {

namespace {

// Writes an axis-aligned cube of the given side with its minimum corner at `origin` as an OFF file.
void write_cube_off(const std::string &path, float side, const Vec3f &origin)
{
    const float x0 = origin.x(), y0 = origin.y(), z0 = origin.z();
    const float x1 = x0 + side, y1 = y0 + side, z1 = z0 + side;
    boost::nowide::ofstream out(path);
    out << "OFF\n8 12 0\n"
        << x0 << " " << y0 << " " << z0 << "\n" << x1 << " " << y0 << " " << z0 << "\n"
        << x1 << " " << y1 << " " << z0 << "\n" << x0 << " " << y1 << " " << z0 << "\n"
        << x0 << " " << y0 << " " << z1 << "\n" << x1 << " " << y0 << " " << z1 << "\n"
        << x1 << " " << y1 << " " << z1 << "\n" << x0 << " " << y1 << " " << z1 << "\n"
        << "3 0 2 1\n3 0 3 2\n3 4 5 6\n3 4 6 7\n"
        << "3 0 1 5\n3 0 5 4\n3 1 2 6\n3 1 6 5\n"
        << "3 2 3 7\n3 2 7 6\n3 3 0 4\n3 3 4 7\n";
}

void write_text(const std::string &path, const std::string &text)
{
    boost::nowide::ofstream out(path);
    out << text;
}

} // namespace

// corefine_test discards the result of the corefine() call and returns true for any two readable meshes, so
// this only pins that readable files are accepted; it cannot show that corefinement ran.
TEST_CASE("corefine_test accepts two readable OFF meshes", "[CutSurface]")
{
    ScopedTemporaryFile model(".off"), shape(".off");
    write_cube_off(model.string(), 10.f, Vec3f(0.f, 0.f, 0.f));
    write_cube_off(shape.string(), 4.f, Vec3f(8.f, 3.f, 2.f));
    CHECK(corefine_test(model.string(), shape.string()));
}

TEST_CASE("corefine_test rejects a missing model or shape file", "[CutSurface]")
{
    ScopedTemporaryFile valid(".off"), missing(".off");
    write_cube_off(valid.string(), 10.f, Vec3f(0.f, 0.f, 0.f));
    // `missing` only reserves a unique name; nothing is ever written to it.
    REQUIRE_FALSE(boost::filesystem::exists(missing.path()));
    CHECK_FALSE(corefine_test(missing.string(), valid.string()));
    CHECK_FALSE(corefine_test(valid.string(), missing.string()));
}

TEST_CASE("corefine_test rejects a file that is not an OFF mesh", "[CutSurface]")
{
    ScopedTemporaryFile valid(".off"), garbage(".off");
    write_cube_off(valid.string(), 10.f, Vec3f(0.f, 0.f, 0.f));
    write_text(garbage.string(), "not an OFF mesh\n");
    CHECK_FALSE(corefine_test(garbage.string(), valid.string()));
    CHECK_FALSE(corefine_test(valid.string(), garbage.string()));
}

TEST_CASE("corefine_test rejects a truncated OFF mesh", "[CutSurface]")
{
    ScopedTemporaryFile valid(".off"), truncated(".off");
    write_cube_off(valid.string(), 10.f, Vec3f(0.f, 0.f, 0.f));
    // The header promises 8 vertices and 12 faces but only one vertex follows.
    write_text(truncated.string(), "OFF\n8 12 0\n0 0 0\n");
    CHECK_FALSE(corefine_test(truncated.string(), valid.string()));
    CHECK_FALSE(corefine_test(valid.string(), truncated.string()));
}

} // namespace cut_surface_corefine_contracts
