#include "libslic3r/TextureToColor/Callbacks.hpp"
#include "libslic3r/TextureToColor/CgalUtils.hpp"
#include "libslic3r/TextureToColor/ColorUtils.hpp"
#include "libslic3r/TextureToColor/Repair.hpp"
#include "libslic3r/TextureToColor/TriMesh.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/Point.hpp"
#include "texture_to_color_fixtures.hpp"

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/generators/catch_generators_range.hpp>
#include <catch2/catch_message.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <admesh/stl.h>
#include <CGAL/Polygon_mesh_processing/self_intersections.h>

#include <cmath>
#include <cstddef>
#include <memory>
#include <vector>
#include <utility>

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

TEST_CASE("RepairMesh leaves a reoriented closed mesh facing outward", "[TextureToColor][repair]")
{
    std::shared_ptr<TriMesh> repaired;
    REQUIRE(RepairMesh(reversed_tetrahedron_with_degenerate_face(), repaired));
    REQUIRE(repaired);
    CHECK_THAT(its_volume(*repaired), WithinAbs(1.f / 6.f, 1e-5f));
}

TEST_CASE("RepairMesh orients closed solid boundaries with nesting-aware cavity winding", "[TextureToColor][repair]")
{
    const bool nested = GENERATE(false, true);
    const bool reverse_outer = GENERATE(false, true);
    const bool reverse_inner = GENERATE(false, true);
    TriMesh input = closed_tetrahedron();
    TriMesh second = closed_tetrahedron();
    for (auto &point : input.vertices)
        point *= nested ? 4.f : 1.f;
    for (auto &point : second.vertices)
        point += nested ? Vec3f(0.5f, 0.5f, 0.5f) : Vec3f(3.f, 0.f, 0.f);
    if (reverse_outer)
        for (auto &face : input.indices)
            std::swap(face[1], face[2]);
    if (reverse_inner)
        for (auto &face : second.indices)
            std::swap(face[1], face[2]);
    its_merge(input, second);
    std::shared_ptr<TriMesh> output;
    REQUIRE(RepairMesh(input, output));
    REQUIRE(output);
    check_finite_and_valid(*output);
    REQUIRE(output->indices.size() == 8);
    CHECK(color_utils::is_closed(*output));
    // Nested shells are a cavity; disconnected non-nested shells are two solids.
    CHECK_THAT(its_volume(*output), WithinAbs(nested ? (64.f - 1.f) / 6.f : 2.f / 6.f, 1e-5f));
}

TEST_CASE("RepairMesh keeps open and intersecting results as best-effort surfaces", "[TextureToColor][repair]")
{
    const bool intersecting = GENERATE(false, true);
    TriMesh input = closed_tetrahedron();
    TriMesh second = intersecting ? closed_tetrahedron() : open_tetrahedron();
    for (auto &point : second.vertices)
        point += intersecting ? Vec3f(0.25f, 0.25f, 0.25f) : Vec3f(3.f, 0.f, 0.f);
    its_merge(input, second);
    if (intersecting) {
        REQUIRE(color_utils::is_closed(input));
        REQUIRE(CGAL::Polygon_mesh_processing::does_self_intersect(cgalutils::trimesh_to_cgal(input)));
    }
    RepairSetting no_hole_filling;
    no_hole_filling.max_hole_edges = 0;
    no_hole_filling.max_boundary_edges = 0;
    std::shared_ptr<TriMesh> output;
    REQUIRE(RepairMesh(input, output, nullptr, nullptr, no_hole_filling));
    REQUIRE(output);
    check_finite_and_valid(*output);
    CHECK(output->indices.size() == input.indices.size());
    CHECK(color_utils::is_closed(*output) == intersecting);
    if (intersecting)
        CHECK(CGAL::Polygon_mesh_processing::does_self_intersect(cgalutils::trimesh_to_cgal(*output)));
    // No positive-volume assertion is valid for these best-effort outputs.
}

TEST_CASE("RepairMesh preserves output when cancellation arrives during solid orientation", "[TextureToColor][repair]")
{
    const auto sentinel = std::make_shared<TriMesh>(closed_tetrahedron());
    std::shared_ptr<TriMesh> output = sentinel;
    std::vector<int> progress;
    int polls_at_85 = 0;
    const auto cancel_after_work = [&]() {
        if (!progress.empty() && progress.back() == 85)
            return ++polls_at_85 == 2;
        return false;
    };
    CHECK_FALSE(RepairMesh(closed_tetrahedron(), output,
                          [&](AlgoProgress state) { progress.push_back(state.percent); }, cancel_after_work));
    CHECK(polls_at_85 == 2);
    REQUIRE_FALSE(progress.empty());
    CHECK(progress.back() == 85);
    REQUIRE(output == sentinel);
    CHECK(output->indices.size() == 4);
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
    REQUIRE_FALSE(stages.empty());
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
    // 4-edge hole is over the limit, is covered by the partial-repair test below.
    const auto [max_hole_edges, max_boundary_edges, closes] = GENERATE(
        table<std::size_t, std::size_t, bool>({
            {4, 7, true},
            {4, 6, false},
            {7, 4, false},
            {3, 6, false},
            {2, 7, false},
            {0, 7, false},
            {4, 0, false},
        }));
    RepairSetting settings;
    settings.max_hole_edges = max_hole_edges;
    settings.max_boundary_edges = max_boundary_edges;
    std::shared_ptr<TriMesh> repaired;

    REQUIRE(RepairMesh(input, repaired, nullptr, nullptr, settings));
    REQUIRE(repaired);
    check_finite_and_valid(*repaired);
    if (closes) {
        CHECK(repaired->indices.size() == 12);
        CHECK(color_utils::is_closed(*repaired));
        CHECK_THAT(std::abs(its_volume(*repaired)), WithinAbs(1000.f, 1e-2f));
    } else {
        CHECK_FALSE(color_utils::is_closed(*repaired));
        CHECK(repaired->indices.size() == 9);
    }
}

TEST_CASE("RepairMesh fills holes within the per-hole limit when another hole exceeds it",
          "[TextureToColor][repair]")
{
    RepairSetting settings;
    settings.max_hole_edges = 3;      // the three-edge hole qualifies, the four-edge one does not
    settings.max_boundary_edges = 7;  // the total is within the aggregate limit
    std::shared_ptr<TriMesh> repaired;
    REQUIRE(RepairMesh(cube_with_two_holes(), repaired, nullptr, nullptr, settings));
    REQUIRE(repaired);
    check_finite_and_valid(*repaired);
    // One triangle closes the three-edge hole; the four-edge hole stays open.
    CHECK(repaired->indices.size() == 10);
    CHECK_FALSE(color_utils::is_closed(*repaired));
    const BoundaryEdgeStats stats = ComputeBoundaryEdgeStats(cgalutils::trimesh_to_cgal(*repaired));
    CHECK(stats.cycle_count == 1);
    CHECK(stats.max_cycle_edges == 4);
    CHECK(stats.total_boundary_edges == 4);
}

TEST_CASE("Unbounded boundary repair keeps the one-argument helper's policy", "[TextureToColor][repair]")
{
    auto mesh = cgalutils::trimesh_to_cgal(cube_with_two_holes());
    CloseBoundariesAndRepairManifoldness(mesh);
    const TriMesh repaired = cgalutils::cgal_to_trimesh(mesh);
    check_finite_and_valid(repaired);
    CHECK(repaired.indices.size() == 12);
    CHECK(color_utils::is_closed(repaired));
    CHECK_THAT(std::abs(its_volume(repaired)), WithinAbs(1000.f, 1e-2f));
}

TEST_CASE("Partial hole repair does not publish a mesh when cancelled", "[TextureToColor][repair]")
{
    const int stop_after = GENERATE(80, 85);
    RepairSetting settings;
    settings.max_hole_edges = 3;
    settings.max_boundary_edges = 7;
    auto sentinel = std::make_shared<TriMesh>(closed_tetrahedron());
    std::shared_ptr<TriMesh> repaired = sentinel;
    int progress = 0;
    const auto record_progress = [&](AlgoProgress state) { progress = state.percent; };
    const auto cancel = [&]() { return progress >= stop_after; };
    CHECK_FALSE(RepairMesh(cube_with_two_holes(), repaired, record_progress, cancel, settings));
    CHECK(progress == stop_after);
    REQUIRE(repaired == sentinel);
    CHECK(repaired->indices.size() == 4);
}
