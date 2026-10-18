#include "libslic3r/libslic3r.h"
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "test_utils.hpp"
#include <boost/nowide/fstream.hpp>
#include <cmath>
#include <string>
#include <sstream>
#include <catch2/catch_all.hpp>
#include <catch2/generators/catch_generators.hpp>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <libslic3r/BoundingBox.hpp>
#include <libslic3r/Point.hpp>
#include <libslic3r/ExPolygon.hpp>
#include <vector>

#include "catch2/catch_test_macros.hpp"
#include "catch2/catch_message.hpp"
#include "libslic3r/CutSurface.hpp"
#include "libslic3r/Emboss.hpp"
#include "libslic3r/TriangleMesh.hpp"

using namespace Slic3r;

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
            {3, 4, 5}, // Crosses its top and right boundaries.
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

void check_index(const stl_triangle_vertex_indices &triangle, uint32_t a, uint32_t b, uint32_t c)
{
    CHECK(triangle[0] == a);
    CHECK(triangle[1] == b);
    CHECK(triangle[2] == c);
}

} // namespace

TEST_CASE("its_mask compacts only vertices referenced by selected triangles", "[CutSurface]")
{
    const indexed_triangle_set input = mask_fixture();

    const indexed_triangle_set selected = its_mask(input, {true, false});
    REQUIRE(selected.indices.size() == 1);
    REQUIRE(selected.vertices.size() == 3);
    check_index(selected.indices.front(), 0, 1, 2);
    CHECK(selected.vertices[0].isApprox(input.vertices[2]));
    CHECK(selected.vertices[1].isApprox(input.vertices[0]));
    CHECK(selected.vertices[2].isApprox(input.vertices[3]));

    const indexed_triangle_set empty = its_mask(input, {false, false});
    CHECK(empty.indices.empty());
    CHECK(empty.vertices.empty());
}

TEST_CASE("its_cut_AoI keeps triangles that intersect the projected bounds", "[CutSurface]")
{
    const indexed_triangle_set input = aoi_fixture();
    const Emboss::ProjectZ projection(5.0);
    const BoundingBox query(Point {0, 0}, Point {5, 5});

    const indexed_triangle_set clipped = its_cut_AoI(input, query, projection);
    REQUIRE(clipped.indices.size() == 2);

    for (const stl_triangle_vertex_indices &triangle : clipped.indices) {
        CAPTURE(triangle[0], triangle[1], triangle[2]);
        REQUIRE(triangle[0] < clipped.vertices.size());
        REQUIRE(triangle[1] < clipped.vertices.size());
        REQUIRE(triangle[2] < clipped.vertices.size());

        const Vec3f &a = clipped.vertices[triangle[0]];
        const Vec3f &b = clipped.vertices[triangle[1]];
        const Vec3f &c = clipped.vertices[triangle[2]];
        const float min_x = std::min({a.x(), b.x(), c.x()});
        const float max_x = std::max({a.x(), b.x(), c.x()});
        const float min_y = std::min({a.y(), b.y(), c.y()});
        const float max_y = std::max({a.y(), b.y(), c.y()});

        CHECK(min_x <= 5.f);
        CHECK(max_x >= 0.f);
        CHECK(min_y <= 5.f);
        CHECK(max_y >= 0.f);
    }
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

    const Emboss::ProjectZ projection(3.0);
    const indexed_triangle_set result = cut2model(cut, projection);
    REQUIRE(result.vertices.size() == 6);
    REQUIRE(result.indices.size() == 8);

    for (size_t i = 0; i < cut.vertices.size(); ++i) {
        CHECK(result.vertices[i].isApprox(cut.vertices[i]));
        const Vec3f expected_back(cut.vertices[i].x(), cut.vertices[i].y(), 3.f);
        CHECK(result.vertices[i + cut.vertices.size()].isApprox(expected_back));
    }

    check_index(result.indices[0], 0, 1, 2);
    check_index(result.indices[1], 3, 5, 4);
    check_index(result.indices[2], 0, 2, 3);
    check_index(result.indices[3], 2, 5, 3);
    check_index(result.indices[4], 1, 0, 4);
    check_index(result.indices[5], 0, 3, 4);
    check_index(result.indices[6], 2, 1, 5);
    check_index(result.indices[7], 1, 4, 5);
}

TEST_CASE("cut_surface clips a cube to a projected polygon without invalid topology", "[CutSurface]")
{
    const ExPolygon shape(Polygon {{1, 1}, {9, 1}, {9, 9}, {1, 9}});
    const indexed_triangle_set model = its_make_cube(10.0, 10.0, 10.0);
    Transform3d projection_transform = Transform3d::Identity();
    projection_transform.translate(Vec3d(0., 0., -10.));
    const Emboss::OrthoProject projection(projection_transform, Vec3d(0., 0., 30.));

    const SurfaceCut cut = cut_surface({shape}, {model}, projection, 0.0f);
    REQUIRE_FALSE(cut.vertices.empty());
    REQUIRE_FALSE(cut.indices.empty());

    for (const Vec3f &vertex : cut.vertices) {
        CHECK(vertex.x() >= 1.f);
        CHECK(vertex.x() <= 9.f);
        CHECK(vertex.y() >= 1.f);
        CHECK(vertex.y() <= 9.f);
        CHECK(vertex.z() >= 0.f);
        CHECK(vertex.z() <= 10.f);
    }

    for (const stl_triangle_vertex_indices &triangle : cut.indices) {
        CAPTURE(triangle[0], triangle[1], triangle[2]);
        REQUIRE(triangle[0] < cut.vertices.size());
        REQUIRE(triangle[1] < cut.vertices.size());
        REQUIRE(triangle[2] < cut.vertices.size());
        const Vec3f edge_1 = cut.vertices[triangle[1]] - cut.vertices[triangle[0]];
        const Vec3f edge_2 = cut.vertices[triangle[2]] - cut.vertices[triangle[0]];
        CHECK(edge_1.cross(edge_2).squaredNorm() > 1e-8f);
    }

    for (const SurfaceCut::Contour &contour : cut.contours) {
        REQUIRE_FALSE(contour.empty());
        for (const SurfaceCut::Index vertex_index : contour)
            CHECK(vertex_index < cut.vertices.size());
    }
}

namespace cut_surface_extended_contracts {
using namespace Slic3r;
using Catch::Matchers::WithinAbs;

namespace {

Polygon rectangle(coord_t x0, coord_t y0, coord_t x1, coord_t y1)
{
    return Polygon(Points{Point(x0, y0), Point(x1, y0), Point(x1, y1), Point(x0, y1)});
}

void check_mesh_indices(const indexed_triangle_set &mesh)
{
    for (const auto &face : mesh.indices) {
        REQUIRE(face[0] < mesh.vertices.size());
        REQUIRE(face[1] < mesh.vertices.size());
        REQUIRE(face[2] < mesh.vertices.size());
        CHECK(face[0] != face[1]);
        CHECK(face[1] != face[2]);
        CHECK(face[0] != face[2]);
        const Vec3f edge_a = mesh.vertices[face[1]] - mesh.vertices[face[0]];
        const Vec3f edge_b = mesh.vertices[face[2]] - mesh.vertices[face[0]];
        CHECK(edge_a.cross(edge_b).squaredNorm() > 1e-10f);
    }
}

void check_contours(const SurfaceCut &cut)
{
    REQUIRE_FALSE(cut.contours.empty());
    for (const SurfaceCut::Contour &contour : cut.contours) {
        REQUIRE(contour.size() >= 3);
        for (const SurfaceCut::Index index : contour)
            CHECK(index < cut.vertices.size());
    }
}

double projected_area(const indexed_triangle_set &mesh)
{
    double area = 0.;
    for (const auto &face : mesh.indices) {
        const Vec3f &a = mesh.vertices[face[0]];
        const Vec3f &b = mesh.vertices[face[1]];
        const Vec3f &c = mesh.vertices[face[2]];
        area += std::abs(double(b.x() - a.x()) * (c.y() - a.y()) -
                         double(b.y() - a.y()) * (c.x() - a.x())) / 2.;
    }
    return area;
}

indexed_triangle_set shifted(indexed_triangle_set mesh, const Vec3f &delta)
{
    for (Vec3f &vertex : mesh.vertices)
        vertex += delta;
    return mesh;
}

void write_cube_off(const std::string &path, float side)
{
    boost::nowide::ofstream out(path);
    out << "OFF\n8 12 0\n"
        << "0 0 0\n" << side << " 0 0\n" << side << " " << side << " 0\n" << "0 " << side << " 0\n"
        << "0 0 " << side << "\n" << side << " 0 " << side << "\n" << side << " " << side << " " << side << "\n"
        << "0 " << side << " " << side << "\n"
        << "3 0 2 1\n3 0 3 2\n3 4 5 6\n3 4 6 7\n"
        << "3 0 1 5\n3 0 5 4\n3 1 2 6\n3 1 6 5\n"
        << "3 2 3 7\n3 2 7 6\n3 3 0 4\n3 3 4 7\n";
}

Emboss::OrthoProject cube_projection(float depth = 30.f)
{
    Transform3d matrix = Transform3d::Identity();
    matrix.translate(Vec3d(0., 0., -10.));
    return Emboss::OrthoProject(matrix, Vec3d(0., 0., depth));
}

} // namespace

TEST_CASE("its_cut_AoI preserves selection under projection reflection", "[CutSurface][CutSurfaceExtended]")
{
    const indexed_triangle_set input = aoi_fixture();
    const BoundingBox query(Point {0, 0}, Point {5, 5});
    const indexed_triangle_set forward = its_cut_AoI(input, query, Emboss::ProjectZ(5.0));
    const indexed_triangle_set reflected = its_cut_AoI(input, query, Emboss::ProjectZ(-5.0));

    REQUIRE(forward.indices.size() == 2);
    REQUIRE(reflected.indices.size() == forward.indices.size());
    REQUIRE(reflected.vertices.size() == forward.vertices.size());
    for (size_t i = 0; i < forward.indices.size(); ++i) {
        CHECK(reflected.indices[i] == forward.indices[i]);
    }
    for (size_t i = 0; i < forward.vertices.size(); ++i)
        CHECK(reflected.vertices[i].isApprox(forward.vertices[i]));
    check_mesh_indices(reflected);
}

TEST_CASE("its_cut_AoI keeps triangles lying on the projected boundary", "[CutSurface][CutSurfaceExtended]")
{
    const indexed_triangle_set input = {
        {{0, 1, 2}},
        {stl_vertex(0.f, 1.f, 0.f), stl_vertex(0.f, 4.f, 0.f), stl_vertex(0.f, 1.f, 1.f)}};
    const indexed_triangle_set selected = its_cut_AoI(
        input, BoundingBox(Point {0, 0}, Point {3, 5}), Emboss::ProjectZ(5.0));

    REQUIRE(selected.indices.size() == 1);
    REQUIRE(selected.vertices.size() == 3);
    check_mesh_indices(selected);
    for (const Vec3f &vertex : selected.vertices)
        CHECK_THAT(vertex.x(), WithinAbs(0., 1e-6));
}

TEST_CASE("its_mask keeps all selected faces and preserves volume", "[CutSurface][CutSurfaceExtended]")
{
    const indexed_triangle_set input = its_make_cube(10., 20., 30.);
    const indexed_triangle_set selected = its_mask(input, std::vector<bool>(input.indices.size(), true));

    REQUIRE(selected.indices.size() == input.indices.size());
    REQUIRE(selected.vertices.size() <= input.vertices.size());
    CHECK_THAT(std::abs(its_volume(selected)), WithinAbs(std::abs(its_volume(input)), 1e-4));
    check_mesh_indices(selected);
}

TEST_CASE("its_mask compacts disjoint selected faces without retaining unreferenced vertices", "[CutSurface][CutSurfaceExtended]")
{
    const indexed_triangle_set input = its_make_cube(10., 10., 10.);
    std::vector<bool> mask(input.indices.size(), false);
    for (size_t i = 0; i < mask.size(); i += 3)
        mask[i] = true;

    const indexed_triangle_set selected = its_mask(input, mask);
    REQUIRE(selected.indices.size() == std::count(mask.begin(), mask.end(), true));
    REQUIRE_FALSE(selected.vertices.empty());
    CHECK(selected.vertices.size() <= selected.indices.size() * 3);
    check_mesh_indices(selected);
}

TEST_CASE("its_cut_AoI retains the complete mesh when the projected bounds contain it", "[CutSurface][CutSurfaceExtended]")
{
    const indexed_triangle_set input = its_make_cube(10., 10., 10.);
    const Emboss::ProjectZ projection(20.);
    const indexed_triangle_set selected = its_cut_AoI(input, BoundingBox(Point(-1, -1), Point(11, 11)), projection);

    REQUIRE(selected.indices.size() == input.indices.size());
    CHECK_THAT(std::abs(its_volume(selected)), WithinAbs(std::abs(its_volume(input)), 1e-4));
    check_mesh_indices(selected);
}

TEST_CASE("its_cut_AoI returns an empty compact mesh for a disjoint projected box", "[CutSurface][CutSurfaceExtended]")
{
    const indexed_triangle_set input = its_make_cube(10., 10., 10.);
    const Emboss::ProjectZ projection(20.);
    const indexed_triangle_set selected = its_cut_AoI(input, BoundingBox(Point(20, 20), Point(30, 30)), projection);

    CHECK(selected.indices.empty());
    CHECK(selected.vertices.empty());
}

TEST_CASE("its_cut_AoI keeps a triangle crossing one side of the projected bounds", "[CutSurface][CutSurfaceExtended]")
{
    indexed_triangle_set input = {
        {{0, 1, 2}},
        {stl_vertex(-2.f, 1.f, 0.f), stl_vertex(2.f, 1.f, 0.f), stl_vertex(0.f, 4.f, 0.f)}};
    const Emboss::ProjectZ projection(5.);
    const indexed_triangle_set selected = its_cut_AoI(input, BoundingBox(Point(0, 0), Point(3, 3)), projection);

    REQUIRE(selected.indices.size() == 1);
    REQUIRE(selected.vertices.size() == 3);
    check_mesh_indices(selected);
}

TEST_CASE("cut2model transforms both sides and closes multiple contour loops", "[CutSurface][CutSurfaceExtended]")
{
    SurfaceCut cut;
    cut.vertices = {
        stl_vertex(0.f, 0.f, 0.f), stl_vertex(4.f, 0.f, 0.f), stl_vertex(4.f, 4.f, 0.f), stl_vertex(0.f, 4.f, 0.f),
        stl_vertex(1.f, 1.f, 0.f), stl_vertex(3.f, 1.f, 0.f), stl_vertex(3.f, 3.f, 0.f), stl_vertex(1.f, 3.f, 0.f)};
    // Triangulate the ring, leaving the inner square open on both caps.
    cut.indices = {{0, 1, 5}, {0, 5, 4}, {1, 2, 6}, {1, 6, 5},
                   {2, 3, 7}, {2, 7, 6}, {3, 0, 4}, {3, 4, 7}};
    cut.contours = {{0, 1, 2, 3}, {4, 7, 6, 5}};

    const Emboss::OrthoProject3d projection(Vec3d(0., 0., 5.));
    const indexed_triangle_set result = cut2model(cut, projection);
    REQUIRE(result.vertices.size() == cut.vertices.size() * 2);
    REQUIRE(result.indices.size() == cut.indices.size() * 2 + 2 * (4 + 4));
    check_mesh_indices(result);
    for (size_t i = 0; i < cut.vertices.size(); ++i) {
        CHECK(result.vertices[i].isApprox(cut.vertices[i]));
        CHECK_THAT(result.vertices[i + cut.vertices.size()].z() - cut.vertices[i].z(), WithinAbs(5., 1e-6));
    }
    REQUIRE(result.indices.size() > cut.indices.size());
    CHECK(result.indices[cut.indices.size()][0] == cut.indices[0][0] + cut.vertices.size());
    CHECK(result.indices[cut.indices.size()][1] == cut.indices[0][2] + cut.vertices.size());
    CHECK(result.indices[cut.indices.size()][2] == cut.indices[0][1] + cut.vertices.size());
    CHECK_THAT(std::abs(its_volume(result)), WithinAbs((16. - 4.) * 5., 1e-4));
}

TEST_CASE("cut2model follows a non-axis projection vector", "[CutSurface][CutSurfaceExtended]")
{
    SurfaceCut cut;
    cut.vertices = {stl_vertex(0.f, 0.f, 0.f), stl_vertex(2.f, 0.f, 0.f), stl_vertex(0.f, 2.f, 0.f)};
    cut.indices = {{0, 1, 2}};
    cut.contours = {{0, 1, 2}};
    const Vec3d direction(1., -2., 4.);
    const indexed_triangle_set result = cut2model(cut, Emboss::OrthoProject3d(direction));

    REQUIRE(result.vertices.size() == 6);
    for (size_t i = 0; i < cut.vertices.size(); ++i) {
        const Vec3f delta = result.vertices[i + cut.vertices.size()] - result.vertices[i];
        CHECK(delta.isApprox(direction.cast<float>(), 1e-5f));
    }
    check_mesh_indices(result);
}

TEST_CASE("cut_surface keeps a central cube patch inside the projected shape", "[CutSurface][CutSurfaceExtended]")
{
    const ExPolygons shapes{ExPolygon(rectangle(1, 1, 9, 9))};
    const indexed_triangle_set model = its_make_cube(10., 10., 10.);
    const Emboss::OrthoProject projection = cube_projection();
    const SurfaceCut cut = cut_surface(shapes, {model}, projection, 0.f);

    REQUIRE_FALSE(cut.vertices.empty());
    REQUIRE_FALSE(cut.indices.empty());
    for (const Vec3f &vertex : cut.vertices) {
        CHECK(vertex.x() >= 1.f);
        CHECK(vertex.x() <= 9.f);
        CHECK(vertex.y() >= 1.f);
        CHECK(vertex.y() <= 9.f);
        CHECK(vertex.z() >= 0.f);
        CHECK(vertex.z() <= 10.f);
    }
    check_mesh_indices(cut);
    CHECK_THAT(projected_area(cut), WithinAbs(64.0, 1e-4));
}

TEST_CASE("cut_surface returns an empty result for an empty model or shape set", "[CutSurface][CutSurfaceExtended]")
{
    const ExPolygons shapes{ExPolygon(rectangle(1, 1, 9, 9))};
    const indexed_triangle_set model = its_make_cube(10., 10., 10.);
    const Emboss::OrthoProject projection = cube_projection();
    CHECK(cut_surface({}, {model}, projection, .5f).empty());
    CHECK(cut_surface(shapes, {}, projection, .5f).empty());
}

TEST_CASE("cut_surface returns no patch for a projected shape outside the model", "[CutSurface][CutSurfaceExtended]")
{
    const ExPolygons shapes{ExPolygon(rectangle(20, 20, 30, 30))};
    const indexed_triangle_set model = its_make_cube(10., 10., 10.);
    const Emboss::OrthoProject projection = cube_projection();
    const SurfaceCut cut = cut_surface(shapes, {model}, projection, .5f);
    CHECK(cut.empty());
}

TEST_CASE("cut_surface supports a hole as a separate projected boundary", "[CutSurface][CutSurfaceExtended]")
{
    ExPolygon shape(rectangle(1, 1, 9, 9));
    shape.holes.emplace_back(rectangle(4, 4, 6, 6));
    shape.holes.back().reverse();
    const indexed_triangle_set model = its_make_cube(10., 10., 10.);
    const Emboss::OrthoProject projection = cube_projection();
    const SurfaceCut cut = cut_surface({shape}, {model}, projection, .5f);

    REQUIRE_FALSE(cut.vertices.empty());
    REQUIRE_FALSE(cut.indices.empty());
    REQUIRE(cut.contours.size() >= 2);
    for (const auto &contour : cut.contours) {
        REQUIRE(contour.size() >= 3);
        for (const auto index : contour)
            CHECK(index < cut.vertices.size());
    }
    check_mesh_indices(cut);
    CHECK_THAT(projected_area(cut), WithinAbs(60.0, 1e-4));
}

TEST_CASE("cut_surface accepts multiple disjoint projected shapes", "[CutSurface][CutSurfaceExtended]")
{
    const ExPolygons shapes{ExPolygon(rectangle(1, 1, 3, 3)), ExPolygon(rectangle(7, 7, 9, 9))};
    const indexed_triangle_set model = its_make_cube(10., 10., 10.);
    const Emboss::OrthoProject projection = cube_projection();
    const SurfaceCut cut = cut_surface(shapes, {model}, projection, .25f);

    REQUIRE_FALSE(cut.vertices.empty());
    REQUIRE_FALSE(cut.indices.empty());
    REQUIRE(cut.contours.size() >= 2);
    check_mesh_indices(cut);
    CHECK_THAT(projected_area(cut), WithinAbs(8.0, 1e-4));
}

TEST_CASE("cut_surface handles two disjoint source meshes in one projection", "[CutSurface][CutSurfaceExtended]")
{
    const ExPolygons shapes{ExPolygon(rectangle(1, 1, 29, 9))};
    const indexed_triangle_set first = its_make_cube(10., 10., 10.);
    const indexed_triangle_set second = shifted(its_make_cube(10., 10., 10.), Vec3f(20.f, 0.f, 0.f));
    const Emboss::OrthoProject projection = cube_projection(30.f);
    const SurfaceCut cut = cut_surface(shapes, {first, second}, projection, .5f);

    REQUIRE_FALSE(cut.vertices.empty());
    REQUIRE_FALSE(cut.indices.empty());
    REQUIRE(cut.contours.size() == 2);
    check_mesh_indices(cut);
    CHECK_THAT(projected_area(cut), WithinAbs(2. * 9. * 8., 1e-4));
    bool left = false;
    bool right = false;
    for (const Vec3f &vertex : cut.vertices) {
        CHECK(vertex.z() >= 0.f);
        CHECK(vertex.z() <= 10.f);
        if (vertex.x() > 10.f)
            CHECK(vertex.x() >= 20.f);
        left = left || vertex.x() < 10.f;
        right = right || vertex.x() > 20.f;
    }
    CHECK(left);
    CHECK(right);
}

TEST_CASE("corefine_test accepts two valid OFF meshes", "[CutSurface][CutSurfaceExtended]")
{
    ScopedTemporaryFile model(".off"), shape(".off");
    write_cube_off(model.string(), 10.f);
    write_cube_off(shape.string(), 4.f);
    CHECK(corefine_test(model.string(), shape.string()));
}

TEST_CASE("corefine_test rejects missing or malformed OFF inputs", "[CutSurface][CutSurfaceExtended]")
{
    ScopedTemporaryFile malformed(".off");
    {
        boost::nowide::ofstream out(malformed.string());
        out << "not an OFF mesh\n";
    }
    CHECK_FALSE(corefine_test(malformed.string(), malformed.string()));
    CHECK_FALSE(corefine_test("/path/that/does/not/exist.off", malformed.string()));
}

TEST_CASE("Surface cuts select the requested depth across stacked models", "[CutSurface][CutSurfaceExtended][CutSurfaceTopology]")
{
    const ExPolygons shapes{ExPolygon(rectangle(1, 1, 9, 9))};
    const indexed_triangle_set near_model = its_make_cube(10., 10., 10.);
    const indexed_triangle_set far_model = shifted(its_make_cube(10., 10., 10.), Vec3f(0.f, 0.f, 20.f));
    const Emboss::OrthoProject projection = cube_projection(50.f);

    const SurfaceCut front = cut_surface(shapes, {near_model, far_model}, projection, 0.f);
    const SurfaceCut back = cut_surface(shapes, {near_model, far_model}, projection, 1.f);
    REQUIRE_FALSE(front.vertices.empty());
    REQUIRE_FALSE(front.indices.empty());
    REQUIRE_FALSE(back.vertices.empty());
    REQUIRE_FALSE(back.indices.empty());
    check_contours(front);
    check_contours(back);
    check_mesh_indices(front);
    check_mesh_indices(back);

    const auto z_bounds = [](const SurfaceCut &cut) {
        std::array<float, 2> bounds{cut.vertices.front().z(), cut.vertices.front().z()};
        for (const Vec3f &vertex : cut.vertices) {
            bounds[0] = std::min(bounds[0], vertex.z());
            bounds[1] = std::max(bounds[1], vertex.z());
        }
        return bounds;
    };
    const std::array<float, 2> front_z = z_bounds(front);
    const std::array<float, 2> back_z = z_bounds(back);
    CHECK(front_z[0] >= -1e-4f);
    CHECK(front_z[1] <= 10.f + 1e-4f);
    CHECK(back_z[0] >= 20.f - 1e-4f);
    CHECK(back_z[1] <= 30.f + 1e-4f);
}

TEST_CASE("Surface cuts preserve disconnected components in one source mesh", "[CutSurface][CutSurfaceExtended][CutSurfaceTopology]")
{
    const ExPolygons shapes{ExPolygon(rectangle(1, 1, 29, 9))};
    indexed_triangle_set combined = its_make_cube(10., 10., 10.);
    const indexed_triangle_set second = shifted(its_make_cube(10., 10., 10.), Vec3f(20.f, 0.f, 0.f));
    const uint32_t offset = static_cast<uint32_t>(combined.vertices.size());
    combined.vertices.insert(combined.vertices.end(), second.vertices.begin(), second.vertices.end());
    for (const auto &face : second.indices)
        combined.indices.emplace_back(face[0] + offset, face[1] + offset, face[2] + offset);

    const SurfaceCut cut = cut_surface(shapes, {combined}, cube_projection(30.f), .5f);
    REQUIRE_FALSE(cut.vertices.empty());
    REQUIRE_FALSE(cut.indices.empty());
    REQUIRE(cut.contours.size() == 2);
    check_contours(cut);
    check_mesh_indices(cut);
    CHECK_THAT(projected_area(cut), WithinAbs(2. * 9. * 8., 1e-4));
    bool left = false;
    bool right = false;
    for (const Vec3f &vertex : cut.vertices) {
        CHECK(vertex.z() >= 0.f);
        CHECK(vertex.z() <= 10.f);
        if (vertex.x() > 10.f)
            CHECK(vertex.x() >= 20.f);
        left = left || vertex.x() < 10.f;
        right = right || vertex.x() > 20.f;
    }
    CHECK(left);
    CHECK(right);
}

TEST_CASE("Surface cuts preserve multiple projected holes", "[CutSurface][CutSurfaceExtended][CutSurfaceTopology]")
{
    const double inset = GENERATE(0., 1.);
    CAPTURE(inset);
    // Zero inset retains the regression where the projection and cube share
    // their outer boundary; the inset row controls for an interior projection.
    ExPolygon shape(rectangle(inset, inset, 10. - inset, 10. - inset));
    shape.holes.emplace_back(rectangle(2, 2, 4, 4));
    shape.holes.back().reverse();
    shape.holes.emplace_back(rectangle(6, 6, 8, 8));
    shape.holes.back().reverse();

    const SurfaceCut cut = cut_surface({shape}, {its_make_cube(10., 10., 10.)}, cube_projection(), .5f);
    REQUIRE_FALSE(cut.vertices.empty());
    REQUIRE_FALSE(cut.indices.empty());
    CAPTURE(cut.vertices.size(), cut.indices.size(), cut.contours.size(), projected_area(cut));
    std::ostringstream contour_geometry;
    for (const SurfaceCut::Contour &contour : cut.contours) {
        Vec3f low = cut.vertices.front();
        Vec3f high = low;
        for (std::size_t i = 0; i < contour.size(); ++i) {
            REQUIRE(contour[i] < cut.vertices.size());
            const Vec3f &point = cut.vertices[contour[i]];
            if (i == 0) {
                low = point;
                high = point;
            } else {
                low = low.cwiseMin(point);
                high = high.cwiseMax(point);
            }
        }
        contour_geometry << contour.size() << " points, bounds [" << low.transpose()
                         << "] to [" << high.transpose() << "]; ";
    }
    INFO(contour_geometry.str());
    CHECK(cut.contours.size() >= 3);
    check_contours(cut);
    check_mesh_indices(cut);
    const double side = 10. - 2. * inset;
    CHECK_THAT(projected_area(cut), WithinAbs(side * side - 2. * 2. * 2., 1e-4));
}

} // namespace cut_surface_extended_contracts

namespace cut_surface_extended_contracts {
using namespace Slic3r;
using Catch::Matchers::WithinAbs;

namespace {

indexed_triangle_set shifted_strip(float x, float z)
{
    indexed_triangle_set model = its_make_cube(10., 10., 10.);
    for (Vec3f &vertex : model.vertices)
        vertex += Vec3f(x, 0.f, z);
    return model;
}

void check_three_strip_cut(const SurfaceCut &cut, double expected_area)
{
    REQUIRE_FALSE(cut.vertices.empty());
    REQUIRE_FALSE(cut.indices.empty());
    check_contours(cut);
    check_mesh_indices(cut);
    CAPTURE(cut.contours.size(), projected_area(cut));
    CHECK(cut.contours.size() >= 3);
    for (const Vec3f &vertex : cut.vertices) {
        CHECK(vertex.x() >= 1.f - 1e-4f);
        CHECK(vertex.x() <= 29.f + 1e-4f);
        CHECK(vertex.y() >= 1.f - 1e-4f);
        CHECK(vertex.y() <= 9.f + 1e-4f);
    }
    CHECK_THAT(projected_area(cut), WithinAbs(expected_area, 1e-4));
}

} // namespace

TEST_CASE("Surface cuts fill an interior projected gap from an unselected strip", "[CutSurface][CutSurfacePatchSelection]")
{
    const ExPolygon shape(rectangle(1, 1, 29, 9));
    const Emboss::OrthoProject projection = cube_projection(80.f);

    // The outer strips touch the query corners, so best-distance selection starts
    // with them. The middle strip lies entirely in the remaining interior gap.
    const SurfaceCut cut = cut_surface(
        {shape},
        {shifted_strip(0.f, 0.f), shifted_strip(20.f, 0.f), shifted_strip(10.f, 0.f)},
        projection,
        .5f);
    check_three_strip_cut(cut, 224.);
    for (const Vec3f &vertex : cut.vertices) {
        CHECK(vertex.z() >= -1e-4f);
        CHECK(vertex.z() <= 10.f + 1e-4f);
    }
}

TEST_CASE("Surface cuts choose the closest of overlapping interior strips", "[CutSurface][CutSurfacePatchSelection]")
{
    const ExPolygon shape(rectangle(1, 1, 29, 9));
    const Emboss::OrthoProject projection = cube_projection(100.f);

    // The two interior candidates cover the same projected gap. Their depth
    // centers are 25 and 45, while the selected outer strips center around 5;
    // the production ordering should choose the nearer candidate.
    const SurfaceCut cut = cut_surface(
        {shape},
        {shifted_strip(0.f, 0.f), shifted_strip(20.f, 0.f),
         shifted_strip(10.f, 20.f), shifted_strip(10.f, 40.f)},
        projection,
        .5f);
    check_three_strip_cut(cut, 224.);
    bool has_near_interior = false;
    for (const Vec3f &vertex : cut.vertices) {
        CHECK(vertex.z() >= -1e-4f);
        CHECK(vertex.z() <= 30.f + 1e-4f);
        has_near_interior = has_near_interior || (vertex.z() > 20.f - 1e-4f);
    }
    CHECK(has_near_interior);
}

} // namespace cut_surface_extended_contracts

namespace cut_surface_extended_contracts {
namespace {

indexed_triangle_set overlapping_roof(float peak)
{
    // A closed triangular prism, with its ridge at x=5. Its roof crosses the
    // cube's z=10 face twice, so clipping leaves two separate flat patches.
    indexed_triangle_set mesh;
    mesh.vertices = {
        Vec3f(1.f, -1.f, 8.f), Vec3f(9.f, -1.f, 8.f), Vec3f(5.f, -1.f, peak),
        Vec3f(1.f, 11.f, 8.f), Vec3f(9.f, 11.f, 8.f), Vec3f(5.f, 11.f, peak)
    };
    mesh.indices = {
        stl_triangle_vertex_indices(0, 1, 2), stl_triangle_vertex_indices(3, 5, 4),
        stl_triangle_vertex_indices(0, 3, 4), stl_triangle_vertex_indices(0, 4, 1),
        stl_triangle_vertex_indices(1, 4, 5), stl_triangle_vertex_indices(1, 5, 2),
        stl_triangle_vertex_indices(0, 2, 5), stl_triangle_vertex_indices(0, 5, 3)
    };
    return mesh;
}

} // namespace

TEST_CASE("Surface cuts retain the exterior patches of intersecting solids",
          "[CutSurface][CutSurfaceClipOverlap]")
{
    const float peak = GENERATE(12.f, 16.f);
    const bool reverse_models = GENERATE(false, true);
    CAPTURE(peak, reverse_models);
    const indexed_triangle_set cube = its_make_cube(10., 10., 10.);
    const indexed_triangle_set roof = overlapping_roof(peak);
    std::vector<indexed_triangle_set> models{cube, roof};
    if (reverse_models)
        std::reverse(models.begin(), models.end());

    const ExPolygon shape(rectangle(1, 1, 9, 9));
    // Use the back projection: the roof is the farther exposed surface, while
    // ratio .5 would target the z=5 midpoint and prefer the cube top at z=10.
    const SurfaceCut cut = cut_surface({shape}, models, cube_projection(30.f), 1.f);
    REQUIRE_FALSE(cut.vertices.empty());
    REQUIRE_FALSE(cut.indices.empty());
    check_mesh_indices(cut);
    check_contours(cut);
    CHECK(cut.contours.size() >= 3);

    // Independently intersect the two sloping roof planes with z=10. The
    // projected query has area64; the raised roof and two flat sides partition it.
    const double rise = double(peak) - 8.;
    const double left = 1. + 8. / rise;
    const double right = 9. - 8. / rise;
    const double raised_area_expected = (right - left) * 8.;
    double flat_area = 0.;
    double raised_area = 0.;
    float max_z = cut.vertices.front().z();
    for (const Vec3f &vertex : cut.vertices) {
        CHECK(vertex.x() >= 1.f - 1e-4f);
        CHECK(vertex.x() <= 9.f + 1e-4f);
        CHECK(vertex.y() >= 1.f - 1e-4f);
        CHECK(vertex.y() <= 9.f + 1e-4f);
        const double roof_z = 8. + rise * (4. - std::abs(double(vertex.x()) - 5.)) / 4.;
        CHECK_THAT(double(vertex.z()), WithinAbs(std::max(10., roof_z), 1e-4));
        max_z = std::max(max_z, vertex.z());
    }
    CHECK_THAT(double(max_z), WithinAbs(double(peak), 1e-4));
    for (const auto &face : cut.indices) {
        const Vec3f &a = cut.vertices[face[0]];
        const Vec3f &b = cut.vertices[face[1]];
        const Vec3f &c = cut.vertices[face[2]];
        const double area = std::abs(double(b.x() - a.x()) * (c.y() - a.y()) -
                                     double(b.y() - a.y()) * (c.x() - a.x())) / 2.;
        const double center_x = (double(a.x()) + b.x() + c.x()) / 3.;
        const bool flat = std::abs(a.z() - 10.f) < 1e-4f &&
                          std::abs(b.z() - 10.f) < 1e-4f &&
                          std::abs(c.z() - 10.f) < 1e-4f;
        if (flat) {
            flat_area += area;
            const bool inside_raised_strip = center_x > left + 1e-4 && center_x < right - 1e-4;
            CHECK_FALSE(inside_raised_strip);
        } else {
            raised_area += area;
            CHECK(center_x >= left - 1e-4);
            CHECK(center_x <= right + 1e-4);
        }
    }
    CHECK_THAT(flat_area, WithinAbs(64. - raised_area_expected, 1e-4));
    CHECK_THAT(raised_area, WithinAbs(raised_area_expected, 1e-4));
    CHECK_THAT(projected_area(cut), WithinAbs(64., 1e-4));
}

} // namespace cut_surface_extended_contracts
