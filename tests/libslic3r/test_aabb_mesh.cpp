#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <admesh/stl.h>

#include "libslic3r/AABBMesh.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/TriangleMesh.hpp"

using namespace Slic3r;
using Catch::Matchers::WithinAbs;

namespace {

void check_vec3(const Vec3d &actual, const Vec3d &expected, double tolerance = 1e-5)
{
    CHECK_THAT(actual.x(), WithinAbs(expected.x(), tolerance));
    CHECK_THAT(actual.y(), WithinAbs(expected.y(), tolerance));
    CHECK_THAT(actual.z(), WithinAbs(expected.z(), tolerance));
}

bool lies_on_cube_boundary(const Vec3d &p, double x, double y, double z, double tolerance)
{
    const bool on_x = std::abs(p.x()) <= tolerance || std::abs(p.x() - x) <= tolerance;
    const bool on_y = std::abs(p.y()) <= tolerance || std::abs(p.y() - y) <= tolerance;
    const bool on_z = std::abs(p.z()) <= tolerance || std::abs(p.z() - z) <= tolerance;
    return on_x || on_y || on_z;
}

indexed_triangle_set translated_cube(double width, double depth, double height, const Vec3f &offset)
{
    indexed_triangle_set result = its_make_cube(width, depth, height);
    for (Vec3f &vertex : result.vertices)
        vertex += offset;
    return result;
}

indexed_triangle_set joined_cubes()
{
    indexed_triangle_set result = its_make_cube(2., 2., 2.);
    indexed_triangle_set second = translated_cube(2., 2., 2., Vec3f(4.f, 0.f, 0.f));
    const int vertex_offset = int(result.vertices.size());
    result.vertices.insert(result.vertices.end(), second.vertices.begin(), second.vertices.end());
    for (Vec3i32 face : second.indices)
        result.indices.emplace_back(face.array() + vertex_offset);
    return result;
}

} // namespace

TEST_CASE("AABBMesh exposes the indexed solid and incidence indexes", "[AABBMesh]")
{
    const indexed_triangle_set cube = its_make_cube(2., 3., 4.);
    AABBMesh mesh(cube);

    REQUIRE(mesh.get_triangle_mesh() == &cube);
    REQUIRE(mesh.vertices().size() == cube.vertices.size());
    REQUIRE(mesh.indices().size() == cube.indices.size());
    CHECK(mesh.vertices(0).isApprox(cube.vertices[0]));
    CHECK(mesh.indices(0).isApprox(cube.indices[0]));
    REQUIRE(mesh.vertex_face_index().count(0) == 5);
    REQUIRE(mesh.vertex_face_index().count(6) == 5);
    REQUIRE(mesh.face_neighbor_index().size() == cube.indices.size());
    for (const Vec3i32 &neighbors : mesh.face_neighbor_index()) {
        CHECK(neighbors[0] >= 0);
        CHECK(neighbors[1] >= 0);
        CHECK(neighbors[2] >= 0);
    }
}

TEST_CASE("AABBMesh returns the first forward cube hit with source and normal", "[AABBMesh]")
{
    const indexed_triangle_set cube = its_make_cube(2., 3., 4.);
    AABBMesh mesh(cube);
    const Vec3d source(-1., 1.1, 2.2);
    const Vec3d direction(1., 0., 0.);
    const AABBMesh::hit_result hit = mesh.query_ray_hit(source, direction);

    REQUIRE(hit.is_valid());
    REQUIRE(hit.is_hit());
    CHECK_THAT(hit.distance(), WithinAbs(1., 1e-6));
    check_vec3(hit.position(), Vec3d(0., 1.1, 2.2));
    check_vec3(hit.source(), source);
    check_vec3(hit.direction(), direction);
    CHECK_THAT(hit.normal().norm(), WithinAbs(1., 1e-6));
    CHECK_THAT(hit.normal().x(), WithinAbs(-1., 1e-6));
    CHECK_THAT(hit.normal().y(), WithinAbs(0., 1e-6));
    CHECK_THAT(hit.normal().z(), WithinAbs(0., 1e-6));
    CHECK_FALSE(hit.is_inside());
}

TEST_CASE("AABBMesh distinguishes a ray miss without exposing a fake face", "[AABBMesh]")
{
    const indexed_triangle_set cube = its_make_cube(2., 3., 4.);
    AABBMesh mesh(cube);
    const AABBMesh::hit_result hit = mesh.query_ray_hit(Vec3d(-1., 4., 2.), Vec3d(1., 0., 0.));

    REQUIRE(hit.is_valid());
    CHECK_FALSE(hit.is_hit());
    CHECK(hit.face() < 0);
    CHECK(std::isinf(hit.distance()));
}

TEST_CASE("AABBMesh reports an exit hit as inside when starting in a solid", "[AABBMesh]")
{
    const indexed_triangle_set cube = its_make_cube(2., 3., 4.);
    AABBMesh mesh(cube);
    const AABBMesh::hit_result hit = mesh.query_ray_hit(Vec3d(1., 1.5, 2.), Vec3d(0., 0., 1.));

    REQUIRE(hit.is_hit());
    CHECK_THAT(hit.distance(), WithinAbs(2., 1e-6));
    check_vec3(hit.position(), Vec3d(1., 1.5, 4.));
    CHECK_THAT(hit.normal().z(), WithinAbs(1., 1e-6));
    CHECK_THAT(hit.normal().x(), WithinAbs(0., 1e-6));
    CHECK_THAT(hit.normal().y(), WithinAbs(0., 1e-6));
    CHECK(hit.is_inside());
}

TEST_CASE("AABBMesh returns sorted entry and exit hits for an axis ray", "[AABBMesh]")
{
    const indexed_triangle_set cube = its_make_cube(2., 3., 4.);
    AABBMesh mesh(cube);
    const std::vector<AABBMesh::hit_result> hits = mesh.query_ray_hits(Vec3d(-1., 1.1, 2.2), Vec3d(1., 0., 0.));

    REQUIRE(hits.size() == 2);
    REQUIRE(hits[0].is_hit());
    REQUIRE(hits[1].is_hit());
    CHECK_THAT(hits[0].distance(), WithinAbs(1., 1e-6));
    CHECK_THAT(hits[1].distance(), WithinAbs(3., 1e-6));
    CHECK(hits[0].distance() < hits[1].distance());
    check_vec3(hits[0].position(), Vec3d(0., 1.1, 2.2));
    check_vec3(hits[1].position(), Vec3d(2., 1.1, 2.2));
    CHECK_THAT(hits[0].normal().x(), WithinAbs(-1., 1e-6));
    CHECK_THAT(hits[1].normal().x(), WithinAbs(1., 1e-6));
}

TEST_CASE("AABBMesh collapses two triangle hits on a coplanar cube seam", "[AABBMesh]")
{
    const indexed_triangle_set cube = its_make_cube(2., 3., 4.);
    AABBMesh mesh(cube);
    // y=1.5,z=2 lies on the diagonal shared by each x face's two triangles.
    // Both triangles report the same geometric crossing; callers need one hit.
    const std::vector<AABBMesh::hit_result> hits = mesh.query_ray_hits(
        Vec3d(-1., 1.5, 2.), Vec3d(1., 0., 0.));

    REQUIRE(hits.size() == 2);
    CHECK_THAT(hits[0].distance(), WithinAbs(1., 1e-6));
    CHECK_THAT(hits[1].distance(), WithinAbs(3., 1e-6));
    CHECK(hits[0].distance() < hits[1].distance());
}

TEST_CASE("AABBMesh returns no hits for a ray parallel to and outside a solid", "[AABBMesh]")
{
    const indexed_triangle_set cube = its_make_cube(2., 3., 4.);
    AABBMesh mesh(cube);
    const std::vector<AABBMesh::hit_result> hits = mesh.query_ray_hits(Vec3d(-1., 4., 2.), Vec3d(1., 0., 0.));
    CHECK(hits.empty());
}

TEST_CASE("AABBMesh squared distance returns an analytic face closest point", "[AABBMesh]")
{
    const indexed_triangle_set cube = its_make_cube(2., 3., 4.);
    AABBMesh mesh(cube);
    int face = -1;
    Vec3d closest = Vec3d::Zero();
    const double distance = mesh.squared_distance(Vec3d(3., 1.1, 2.2), face, closest);

    REQUIRE(face >= 0);
    REQUIRE(face < int(cube.indices.size()));
    CHECK_THAT(distance, WithinAbs(1., 1e-6));
    check_vec3(closest, Vec3d(2., 1.1, 2.2));
}

TEST_CASE("AABBMesh squared distance projects an interior point onto the nearest boundary", "[AABBMesh]")
{
    const indexed_triangle_set cube = its_make_cube(2., 3., 4.);
    AABBMesh mesh(cube);
    int face = -1;
    Vec3d closest = Vec3d::Zero();
    const double distance = mesh.squared_distance(Vec3d(1., 1.5, 2.), face, closest);

    REQUIRE(face >= 0);
    CHECK_THAT(distance, WithinAbs(1., 1e-6));
    CHECK(lies_on_cube_boundary(closest, 2., 3., 4., 1e-6));
    CHECK_THAT((closest - Vec3d(1., 1.5, 2.)).norm(), WithinAbs(1., 1e-6));
}

TEST_CASE("AABBMesh reports zero squared distance and the original point on a face", "[AABBMesh]")
{
    const indexed_triangle_set cube = its_make_cube(2., 3., 4.);
    AABBMesh mesh(cube);
    int face = -1;
    Vec3d closest = Vec3d::Zero();
    const Vec3d point(0., 1.1, 2.2);
    const double distance = mesh.squared_distance(point, face, closest);

    REQUIRE(face >= 0);
    CHECK_THAT(distance, WithinAbs(0., 1e-10));
    check_vec3(closest, point, 1e-5);
}

TEST_CASE("AABBMesh computes unit normals for every indexed face", "[AABBMesh]")
{
    const indexed_triangle_set cube = its_make_cube(2., 3., 4.);
    AABBMesh mesh(cube);
    REQUIRE(mesh.indices().size() == 12);
    for (size_t face_id = 0; face_id < mesh.indices().size(); ++face_id) {
        const Vec3i32 face = mesh.indices(face_id);
        const Vec3d a = mesh.vertices(face[0]).cast<double>();
        const Vec3d b = mesh.vertices(face[1]).cast<double>();
        const Vec3d c = mesh.vertices(face[2]).cast<double>();
        const Vec3d expected = (b - a).cross(c - a).normalized();
        const Vec3d actual = mesh.normal_by_face_id(int(face_id));
        CHECK_THAT(actual.norm(), WithinAbs(1., 1e-6));
        CHECK_THAT(actual.dot(expected), WithinAbs(1., 1e-6));
    }
}

TEST_CASE("AABBMesh copy construction and assignment preserve query semantics", "[AABBMesh]")
{
    const indexed_triangle_set cube = its_make_cube(2., 3., 4.);
    AABBMesh original(cube);
    AABBMesh copied(original);
    AABBMesh assigned(cube);
    assigned = original;

    for (AABBMesh *mesh : {&original, &copied, &assigned}) {
        const AABBMesh::hit_result hit = mesh->query_ray_hit(Vec3d(-1., 1.1, 2.2), Vec3d(1., 0., 0.));
        REQUIRE(hit.is_hit());
        CHECK_THAT(hit.distance(), WithinAbs(1., 1e-6));
        CHECK(mesh->get_triangle_mesh() == &cube);
        CHECK(mesh->face_neighbor_index().size() == cube.indices.size());
    }
}

TEST_CASE("AABBMesh move construction and assignment retain the indexed source", "[AABBMesh]")
{
    const indexed_triangle_set cube = its_make_cube(2., 3., 4.);
    AABBMesh original(cube);
    AABBMesh moved(std::move(original));
    AABBMesh assigned(cube);
    assigned = std::move(moved);

    const AABBMesh::hit_result hit = assigned.query_ray_hit(Vec3d(-1., 1.1, 2.2), Vec3d(1., 0., 0.));
    REQUIRE(hit.is_hit());
    CHECK_THAT(hit.distance(), WithinAbs(1., 1e-6));
    CHECK(assigned.get_triangle_mesh() == &cube);
}

TEST_CASE("AABBMesh supports an indexed mesh and TriangleMesh with identical geometry", "[AABBMesh]")
{
    const indexed_triangle_set cube = its_make_cube(2., 3., 4.);
    const TriangleMesh triangle_mesh(cube);
    AABBMesh indexed_mesh(cube);
    AABBMesh object_mesh(triangle_mesh);

    const Vec3d source(-1., 1.1, 2.2);
    const Vec3d direction(1., 0., 0.);
    const auto indexed_hit = indexed_mesh.query_ray_hit(source, direction);
    const auto object_hit = object_mesh.query_ray_hit(source, direction);
    REQUIRE(indexed_hit.is_hit());
    REQUIRE(object_hit.is_hit());
    CHECK_THAT(indexed_hit.distance(), WithinAbs(object_hit.distance(), 1e-6));
    CHECK(indexed_hit.face() == object_hit.face());
    check_vec3(indexed_hit.normal(), object_hit.normal());
}

TEST_CASE("AABBMesh finds all crossings through two disconnected solids", "[AABBMesh]")
{
    const indexed_triangle_set disjoint = joined_cubes();
    AABBMesh mesh(disjoint, true);
    const std::vector<AABBMesh::hit_result> hits = mesh.query_ray_hits(Vec3d(-1., 1., 1.), Vec3d(1., 0., 0.));

    REQUIRE(hits.size() == 4);
    CHECK_THAT(hits[0].distance(), WithinAbs(1., 1e-6));
    CHECK_THAT(hits[1].distance(), WithinAbs(3., 1e-6));
    CHECK_THAT(hits[2].distance(), WithinAbs(5., 1e-6));
    CHECK_THAT(hits[3].distance(), WithinAbs(7., 1e-6));
    for (const auto &hit : hits) {
        CHECK(hit.is_hit());
        CHECK(std::isfinite(hit.distance()));
    }
}

TEST_CASE("AABBMesh handles an empty indexed set as an empty acceleration structure", "[AABBMesh]")
{
    const indexed_triangle_set empty;
    AABBMesh mesh(empty);
    const AABBMesh::hit_result hit = mesh.query_ray_hit(Vec3d::Zero(), Vec3d(1., 0., 0.));
    const std::vector<AABBMesh::hit_result> hits = mesh.query_ray_hits(Vec3d::Zero(), Vec3d(1., 0., 0.));
    int face = -1;
    Vec3d closest = Vec3d::Ones();

    REQUIRE(hit.is_valid());
    CHECK_FALSE(hit.is_hit());
    CHECK(hits.empty());
    CHECK_THAT(mesh.squared_distance(Vec3d::Zero(), face, closest), WithinAbs(-1., 1e-12));
}

TEST_CASE("AABBMesh uses the calculated epsilon without changing ordinary cube crossings", "[AABBMesh]")
{
    const indexed_triangle_set cube = its_make_cube(2., 3., 4.);
    AABBMesh fixed_epsilon(cube, false);
    AABBMesh scaled_epsilon(cube, true);
    const Vec3d source(-1., 1.1, 2.2);
    const Vec3d direction(1., 0., 0.);
    const auto fixed_hit = fixed_epsilon.query_ray_hit(source, direction);
    const auto scaled_hit = scaled_epsilon.query_ray_hit(source, direction);

    REQUIRE(fixed_hit.is_hit());
    REQUIRE(scaled_hit.is_hit());
    CHECK_THAT(fixed_hit.distance(), WithinAbs(1., 1e-6));
    CHECK_THAT(scaled_hit.distance(), WithinAbs(1., 1e-6));
    CHECK_THAT(fixed_hit.distance(), WithinAbs(scaled_hit.distance(), 1e-6));
}
