#include <algorithm>
#include <cstddef>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/Point.hpp"
#include <Eigen/Core>
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/ShortEdgeCollapse.hpp"
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

} // namespace

TEST_CASE("Short-edge collapse decimates a closed sphere to its face budget without orphan vertices", "[ShortEdgeCollapse]")
{
    const indexed_triangle_set source = its_make_sphere(5., PI / 12.);
    REQUIRE(source.indices.size() > 8);
    REQUIRE(its_num_open_edges(source) == 0);
    const float source_volume = its_volume(source);
    const size_t target_faces = source.indices.size() * 3 / 4;

    indexed_triangle_set result = source;
    its_short_edge_collpase(result, target_faces);

    CHECK(result.indices.size() <= target_faces);
    CHECK(result.vertices.size() < source.vertices.size());
    REQUIRE(all_indices_valid(result));
    // Only vertices still referenced by a face are kept.
    CHECK(its_compactify_vertices(result) == 0);
    // Collapsing short edges of a fine sphere keeps it a solid of nearly the same volume.
    CHECK(its_volume(result) > 0.9f * source_volume);
    CHECK(its_volume(result) <= source_volume);
}
