#include <algorithm>
#include <cstddef>
#include <map>
#include <utility>

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/Point.hpp"
#include <Eigen/Core>
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/libslic3r.h"

using namespace Slic3r;
using Catch::Matchers::WithinAbs;

namespace {

bool all_indices_valid(const indexed_triangle_set& its)
{
    return std::all_of(its.indices.begin(), its.indices.end(), [&its](const Vec3i32& face) {
        return std::all_of(face.begin(), face.end(), [&its](int i) { return i >= 0 && size_t(i) < its.vertices.size(); });
    });
}

bool every_edge_has_two_incidents(const indexed_triangle_set& mesh)
{
    std::map<std::pair<int, int>, int> edge_counts;
    for (const Vec3i32& face : mesh.indices)
        for (int corner = 0; corner < 3; ++corner) {
            int first  = face[corner];
            int second = face[(corner + 1) % 3];
            if (first > second)
                std::swap(first, second);
            ++edge_counts[{first, second}];
        }
    if (edge_counts.empty())
        return false;
    for (const auto& [edge, count] : edge_counts) {
        (void) edge;
        if (count != 2)
            return false;
    }
    return true;
}

} // namespace

TEST_CASE("Primitive generators preserve analytic bounds and positive solids", "[TriangleMesh][TriangleMeshContracts]")
{
    const indexed_triangle_set frustum = its_make_frustum(6., 8., PI / 12.);
    const BoundingBoxf3 frustum_box    = bounding_box(frustum);
    CHECK_THAT(frustum_box.size().z(), WithinAbs(8., 1e-5));
    CHECK_THAT(its_volume(frustum), WithinAbs(float((7. / 12.) * PI * 36. * 8.), 15.f));
    const indexed_triangle_set torus = its_make_torus(8., 2., PI / 8.);
    const BoundingBoxf3 torus_box    = bounding_box(torus);
    CHECK_THAT(torus_box.size().x(), WithinAbs(20., 0.2));
    CHECK_THAT(torus_box.size().y(), WithinAbs(20., 0.2));
    CHECK_THAT(torus_box.size().z(), WithinAbs(4., 0.2));
    CHECK(its_volume(torus) > 0.f);
}

TEST_CASE("Specialized primitive generators produce closed positive solids with analytic bounds", "[TriangleMesh][TriangleMeshContracts]")
{
    const indexed_triangle_set dowel = its_make_frustum_dowel(3., 8., 16);
    REQUIRE_FALSE(dowel.vertices.empty());
    REQUIRE_FALSE(dowel.indices.empty());
    REQUIRE(all_indices_valid(dowel));
    REQUIRE(every_edge_has_two_incidents(dowel));
    CHECK(its_volume(dowel) > 0.f);
    const BoundingBoxf3 dowel_box = bounding_box(dowel);
    CHECK_THAT(dowel_box.min.x(), WithinAbs(-3., 1e-4));
    CHECK_THAT(dowel_box.max.x(), WithinAbs(3., 1e-4));
    CHECK_THAT(dowel_box.min.y(), WithinAbs(-3., 1e-4));
    CHECK_THAT(dowel_box.max.y(), WithinAbs(3., 1e-4));
    CHECK_THAT(dowel_box.min.z(), WithinAbs(-8., 1e-4));
    CHECK_THAT(dowel_box.max.z(), WithinAbs(8., 1e-4));

    const indexed_triangle_set pyramid = its_make_pyramid(6.f, 9.f);
    REQUIRE(all_indices_valid(pyramid));
    REQUIRE(every_edge_has_two_incidents(pyramid));
    CHECK_THAT(its_volume(pyramid), WithinAbs(108.f, 1e-4));
    const BoundingBoxf3 pyramid_box = bounding_box(pyramid);
    CHECK_THAT(pyramid_box.min.x(), WithinAbs(-3., 1e-4));
    CHECK_THAT(pyramid_box.max.x(), WithinAbs(3., 1e-4));
    CHECK_THAT(pyramid_box.min.y(), WithinAbs(-3., 1e-4));
    CHECK_THAT(pyramid_box.max.y(), WithinAbs(3., 1e-4));
    CHECK_THAT(pyramid_box.min.z(), WithinAbs(0., 1e-4));
    CHECK_THAT(pyramid_box.max.z(), WithinAbs(9., 1e-4));

    const indexed_triangle_set snap = its_make_snap(4., 8.);
    REQUIRE(all_indices_valid(snap));
    REQUIRE(every_edge_has_two_incidents(snap));
    CHECK(its_volume(snap) > 0.f);
    const BoundingBoxf3 snap_box = bounding_box(snap);
    CHECK_THAT(snap_box.min.z(), WithinAbs(0., 1e-4));
    CHECK_THAT(snap_box.max.z(), WithinAbs(8., 1e-4));
    CHECK_THAT(snap_box.min.x(), WithinAbs(-snap_box.max.x(), 1e-4));
    CHECK_THAT(snap_box.min.y(), WithinAbs(-snap_box.max.y(), 1e-4));
}

namespace pyramid_winding_contracts {
using namespace Slic3r;
using Catch::Matchers::WithinAbs;

// Known bug: its_make_pyramid winds its two base facets inward; fixed by the follow-up fix PR
TEST_CASE("Pyramid facets all face away from an interior point", "[TriangleMesh][PyramidWinding][!shouldfail]")
{
    const auto mesh = its_make_pyramid(6.f, 9.f);
    const Vec3f interior(0.f, 0.f, 2.25f);
    REQUIRE(mesh.indices.size() == 6);
    for (const auto& face : mesh.indices) {
        for (int index : face) {
            REQUIRE(index >= 0);
            REQUIRE(static_cast<std::size_t>(index) < mesh.vertices.size());
        }
        const Vec3f a = mesh.vertices[face[0]], b = mesh.vertices[face[1]], c = mesh.vertices[face[2]];
        const Vec3f normal = (b - a).cross(c - a);
        CHECK(normal.dot((a + b + c) / 3.f - interior) > 0.f);
    }
}

// Known bug: its_make_pyramid winds its two base facets inward; fixed by the follow-up fix PR
TEST_CASE("Pyramid topology pairs every directed edge with its reverse", "[TriangleMesh][PyramidWinding][!shouldfail]")
{
    const auto mesh = its_make_pyramid(6.f, 9.f);
    CHECK(its_num_open_edges(mesh) == 0);
    const auto adjacent = its_face_neighbors(mesh);
    REQUIRE(adjacent.size() == 6);
    for (const auto& neighbors : adjacent)
        for (int face : neighbors)
            CHECK(face >= 0);
}

TEST_CASE("Pyramid signed volume is invariant under translation", "[TriangleMesh][PyramidWinding]")
{
    const Vec3f displacement = GENERATE(Vec3f(0.f, 0.f, 7.f), Vec3f(0.f, 0.f, -7.f), Vec3f(11.f, -13.f, 5.f));
    auto mesh                = its_make_pyramid(6.f, 9.f);
    // One third of the 6 by 6 mm base times the 9 mm height.
    CHECK_THAT(its_volume(mesh), WithinAbs(108., 1e-4));
    for (auto& vertex : mesh.vertices)
        vertex += displacement;
    CHECK_THAT(its_volume(mesh), WithinAbs(108., 1e-3));
}

} // namespace pyramid_winding_contracts
