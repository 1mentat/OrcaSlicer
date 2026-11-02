#include "libslic3r/TextureToColor/Callbacks.hpp"
#include "libslic3r/TextureToColor/CgalUtils.hpp"
#include "libslic3r/TextureToColor/ColorUtils.hpp"
#include "libslic3r/TextureToColor/Repair.hpp"
#include "libslic3r/TextureToColor/TriMesh.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "texture_to_color_fixtures.hpp"

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/generators/catch_generators_range.hpp>
#include <catch2/catch_message.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <admesh/stl.h>

#include <cmath>
#include <cstddef>
#include <memory>
#include <vector>

using namespace Slic3r;
using namespace Slic3r::tex2color;
using namespace Slic3r::tex2color::test;
using Catch::Matchers::WithinAbs;

namespace {

// The tetrahedron with its base face flipped and an extra degenerate face.
TriMesh reversed_tetrahedron_with_degenerate_face()
{
    TriMesh mesh = closed_tetrahedron();
    mesh.indices[0] = stl_triangle_vertex_indices(0, 1, 2);
    mesh.indices.emplace_back(0, 0, 0);
    return mesh;
}

// A 10 mm cube with two separate holes: one triangle missing from the top (a three-edge hole)
// and the whole bottom square missing (a four-edge hole). The largest hole has 4 edges and the
// boundary has 7 in total, so the per-hole and total limits can be told apart.
TriMesh cube_with_two_holes()
{
    const TriangleMesh cube = make_cube(10., 10., 10.);
    TriMesh mesh(cube.its);
    bool top_removed = false;
    std::vector<stl_triangle_vertex_indices> kept;
    for (const stl_triangle_vertex_indices &face : mesh.indices) {
        const auto all_at = [&](float z) {
            return mesh.vertices[face[0]].z() == z && mesh.vertices[face[1]].z() == z && mesh.vertices[face[2]].z() == z;
        };
        if (all_at(0.f))
            continue;
        if (!top_removed && all_at(10.f)) {
            top_removed = true;
            continue;
        }
        kept.push_back(face);
    }
    mesh.indices = kept;
    return mesh;
}

} // namespace

TEST_CASE("RepairMesh applies both bounded boundary limits",
          "[TextureToColor][repair]")
{
    const TriMesh input = open_tetrahedron();
    const auto [max_hole_edges, max_boundary_edges] = GENERATE(
        table<std::size_t, std::size_t>({
            {3, 3},
            {2, 3},
            {3, 2},
            {MAX_REPAIRABLE_MESH_HOLE_EDGES, MAX_REPAIRABLE_MESH_BOUNDARY_EDGES},
        }));
    RepairSetting settings;
    settings.max_hole_edges = max_hole_edges;
    settings.max_boundary_edges = max_boundary_edges;
    std::shared_ptr<TriMesh> repaired;

    REQUIRE(RepairMesh(input, repaired, nullptr, nullptr, settings));
    REQUIRE(repaired);
    check_finite_and_valid(*repaired);
    CHECK(repaired->vertices.size() == 4);
    const bool hole_limit_allows_closure = max_hole_edges >= 3;
    const bool total_limit_allows_closure = max_boundary_edges >= 3;
    const bool should_close = hole_limit_allows_closure && total_limit_allows_closure;
    const std::size_t expected_faces = should_close ? 4 : 3;
    CHECK(repaired->indices.size() == expected_faces);
    if (should_close) {
        CHECK(color_utils::is_closed(*repaired));
        CHECK_THAT(std::abs(its_volume(*repaired)), WithinAbs(1.f / 6.f, 1e-5f));
    } else {
        CHECK_FALSE(color_utils::is_closed(*repaired));
    }
}

TEST_CASE("RepairMesh corrects face orientation and removes a degenerate face",
          "[TextureToColor][repair]")
{
    const TriMesh input = reversed_tetrahedron_with_degenerate_face();
    std::shared_ptr<TriMesh> repaired;
    // Hole filling is disabled, so a closed result can only come from reorienting the flipped
    // face in place. Dropping the conflicting face and filling the hole it leaves would end open.
    RepairSetting no_hole_filling;
    no_hole_filling.max_hole_edges = 0;
    no_hole_filling.max_boundary_edges = 0;

    REQUIRE(RepairMesh(input, repaired, nullptr, nullptr, no_hole_filling));
    REQUIRE(repaired);
    check_finite_and_valid(*repaired);
    CHECK(repaired->vertices.size() == 4);
    CHECK(repaired->indices.size() == 4);
    CHECK(color_utils::is_closed(*repaired));
    // The volume alone cannot show the winding: the flipped face lies in z = 0, through the
    // origin, and adds nothing to the signed sum.
    CHECK_THAT(std::abs(its_volume(*repaired)), WithinAbs(1.f / 6.f, 1e-5f));
}

// Known bug: orient_polygon_soup only makes the winding consistent; here it flips the three
// outward faces to match the one inward face, so the repaired solid is inside out (volume -1/6).
// Needs a fix (e.g. orient_to_bound_a_volume after building the mesh).
TEST_CASE("RepairMesh leaves a reoriented closed mesh facing outward", "[TextureToColor][repair][!shouldfail]")
{
    std::shared_ptr<TriMesh> repaired;
    REQUIRE(RepairMesh(reversed_tetrahedron_with_degenerate_face(), repaired));
    REQUIRE(repaired);
    CHECK_THAT(its_volume(*repaired), WithinAbs(1.f / 6.f, 1e-5f));
}

TEST_CASE("RepairMesh cancellation stops at each published stage",
          "[TextureToColor][repair]")
{
    const TriMesh input = open_tetrahedron();
    // Record the stages RepairMesh publishes on an uncancelled run, then cancel right after each.
    std::vector<int> stages;
    {
        std::shared_ptr<TriMesh> unused;
        REQUIRE(RepairMesh(input, unused, [&](AlgoProgress state) { stages.push_back(state.percent); }, nullptr));
    }
    // The final 100% report announces completion; there is nothing left to cancel after it.
    REQUIRE(stages.back() == 100);
    stages.pop_back();
    REQUIRE(stages.size() >= 2);
    const int stop_after = GENERATE_COPY(from_range(stages));
    CAPTURE(stop_after);
    std::vector<int> progress;
    auto sentinel = std::make_shared<TriMesh>(closed_tetrahedron());
    std::shared_ptr<TriMesh> repaired = sentinel;
    const auto record_progress = [&](AlgoProgress state) { progress.push_back(state.percent); };
    const auto cancel_at_stage = [&]() {
        return !progress.empty() && progress.back() >= stop_after;
    };

    CHECK_FALSE(RepairMesh(input, repaired, record_progress, cancel_at_stage));
    REQUIRE_FALSE(progress.empty());
    CHECK(progress.back() == stop_after);
    REQUIRE(repaired == sentinel);
    CHECK(repaired->indices.size() == 4);
}


TEST_CASE("RepairMesh limits the largest hole and the total boundary separately", "[TextureToColor][repair]")
{
    const TriMesh input = cube_with_two_holes();
    REQUIRE(input.indices.size() == 9);
    // Lock the premise: two boundary cycles, the larger of 4 edges, 7 edges in total.
    const BoundaryEdgeStats stats = ComputeBoundaryEdgeStats(cgalutils::trimesh_to_cgal(input));
    REQUIRE(stats.cycle_count == 2);
    REQUIRE(stats.max_cycle_edges == 4);
    REQUIRE(stats.total_boundary_edges == 7);
    // {max_hole_edges, max_boundary_edges, closes}. The last row allows each hole on its own
    // (4 <= 7 per hole) but not their 7-edge total. The per-hole case {3, 7}, where only the
    // 4-edge hole is over the limit, is covered by the tagged test below.
    const auto [max_hole_edges, max_boundary_edges, closes] = GENERATE(
        table<std::size_t, std::size_t, bool>({
            {4, 7, true},
            {4, 6, false},
            {7, 4, false},
        }));
    RepairSetting settings;
    settings.max_hole_edges = max_hole_edges;
    settings.max_boundary_edges = max_boundary_edges;
    std::shared_ptr<TriMesh> repaired;

    REQUIRE(RepairMesh(input, repaired, nullptr, nullptr, settings));
    REQUIRE(repaired);
    check_finite_and_valid(*repaired);
    if (closes) {
        CHECK(color_utils::is_closed(*repaired));
        CHECK_THAT(std::abs(its_volume(*repaired)), WithinAbs(1000.f, 1e-2f));
    } else {
        CHECK_FALSE(color_utils::is_closed(*repaired));
        CHECK(repaired->indices.size() == 9);
    }
}

// Known bug: RepairSetting::max_hole_edges is documented as "skip triangulating a boundary cycle whose
// half-edge count exceeds this", but RepairMesh gates hole closing globally on the largest cycle, so
// one over-limit hole stops every other hole from being filled. (The comment on
// MAX_REPAIRABLE_MESH_HOLE_EDGES says the opposite: "skips triangulation entirely".) Needs a fix or a
// documentation decision.
TEST_CASE("RepairMesh fills holes within the per-hole limit when another hole exceeds it",
          "[TextureToColor][repair][!shouldfail]")
{
    RepairSetting settings;
    settings.max_hole_edges = 3;      // the three-edge hole qualifies, the four-edge one does not
    settings.max_boundary_edges = 7;  // the total is within the aggregate limit
    std::shared_ptr<TriMesh> repaired;
    REQUIRE(RepairMesh(cube_with_two_holes(), repaired, nullptr, nullptr, settings));
    REQUIRE(repaired);
    // One triangle closes the three-edge hole; the four-edge hole stays open.
    CHECK(repaired->indices.size() == 10);
    CHECK_FALSE(color_utils::is_closed(*repaired));
}
