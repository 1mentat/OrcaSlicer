#pragma once

// Small meshes, colours and checks shared by the TextureToColor tests.

#include "libslic3r/TextureToColor/TextureToColor.hpp"
#include "libslic3r/TextureToColor/TriMesh.hpp"

#include <catch2/catch_test_macros.hpp>

#include <admesh/stl.h>

#include <array>
#include <cmath>
#include <cstddef>

namespace Slic3r::tex2color::test {

using Color = std::array<std::size_t, 3>;

inline constexpr Color red{{255, 0, 0}};
inline constexpr Color green{{0, 255, 0}};
inline constexpr Color blue{{0, 0, 255}};

// True when every channel is within the 2-unit quantisation of the Lab round trip that the
// pipeline's colour clustering goes through.
inline bool matches_color(const Color &actual, const Color &expected)
{
    for (std::size_t channel = 0; channel < expected.size(); ++channel) {
        if (std::abs(static_cast<double>(actual[channel]) - static_cast<double>(expected[channel])) > 2.0)
            return false;
    }
    return true;
}

// One right triangle in the XY plane.
inline TriMesh one_triangle()
{
    return TriMesh(
        {stl_triangle_vertex_indices(0, 1, 2)},
        {stl_vertex(0.f, 0.f, 0.f), stl_vertex(1.f, 0.f, 0.f), stl_vertex(0.f, 1.f, 0.f)});
}

// Unit right tetrahedron, outward-facing, volume 1/6.
inline TriMesh closed_tetrahedron()
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
inline TriMesh open_tetrahedron()
{
    TriMesh mesh = closed_tetrahedron();
    mesh.indices.pop_back();
    return mesh;
}

// One colour, no smoothing, no oversampling beyond four faces, and no mesh repair.
inline TextureToColorSettings bounded_settings()
{
    TextureToColorSettings settings;
    settings.target_colors_num = 1;
    settings.smooth_weight = 0.0;
    settings.oversampling_iters = 0;
    settings.oversampling_min_face_count = 4;
    settings.oversampling_max_face_count = 32;
    settings.mesh_repair_decision = MeshRepairDecision::ImportWithoutRepair;
    return settings;
}

// Every vertex is finite and every face index refers to an existing vertex.
inline void check_finite_and_valid(const TriMesh &mesh)
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

} // namespace Slic3r::tex2color::test
