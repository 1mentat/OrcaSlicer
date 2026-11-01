#include "libslic3r/TextureToColor/Callbacks.hpp"
#include "libslic3r/TextureToColor/ColorUtils.hpp"
#include "libslic3r/TextureToColor/Repair.hpp"
#include "libslic3r/TextureToColor/TriMesh.hpp"
#include "libslic3r/TriangleMesh.hpp"

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <admesh/stl.h>

#include <cmath>
#include <cstddef>
#include <memory>
#include <vector>

using namespace Slic3r;
using namespace Slic3r::tex2color;
using Catch::Matchers::WithinAbs;

namespace {

// Unit right tetrahedron, outward-facing, volume 1/6.
TriMesh closed_tetrahedron()
{
    return TriMesh(
        {
            stl_triangle_vertex_indices(0, 2, 1),
            stl_triangle_vertex_indices(0, 1, 3),
            stl_triangle_vertex_indices(1, 2, 3),
            stl_triangle_vertex_indices(2, 0, 3),
        },
        {
            stl_vertex(0.f, 0.f, 0.f),
            stl_vertex(1.f, 0.f, 0.f),
            stl_vertex(0.f, 1.f, 0.f),
            stl_vertex(0.f, 0.f, 1.f),
        });
}

// The tetrahedron without its last face: one three-edge hole.
TriMesh open_tetrahedron()
{
    TriMesh mesh = closed_tetrahedron();
    mesh.indices.pop_back();
    return mesh;
}

// The tetrahedron with its base face flipped and an extra degenerate face.
TriMesh reversed_tetrahedron_with_degenerate_face()
{
    TriMesh mesh = closed_tetrahedron();
    mesh.indices[0] = stl_triangle_vertex_indices(0, 1, 2);
    mesh.indices.emplace_back(0, 0, 0);
    return mesh;
}

void check_finite_and_valid(const TriMesh &mesh)
{
    for (const stl_vertex &vertex : mesh.vertices) {
        for (std::size_t component = 0; component < 3; ++component)
            CHECK(std::isfinite(vertex[component]));
    }
    for (const stl_triangle_vertex_indices &face : mesh.indices) {
        for (std::size_t component = 0; component < 3; ++component) {
            REQUIRE(face[component] >= 0);
            REQUIRE(static_cast<std::size_t>(face[component]) < mesh.vertices.size());
        }
    }
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

    REQUIRE(RepairMesh(input, repaired));
    REQUIRE(repaired);
    check_finite_and_valid(*repaired);
    CHECK(repaired->vertices.size() == 4);
    CHECK(repaired->indices.size() == 4);
    CHECK(color_utils::is_closed(*repaired));
    // Closing and the face count depend on consistent winding. The volume alone cannot show it:
    // the flipped face lies in z = 0, through the origin, and adds nothing to the signed sum.
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
    const int stop_after = GENERATE(30, 50, 70, 80, 85);
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

