#include <catch2/catch_all.hpp>

#include <algorithm>
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
