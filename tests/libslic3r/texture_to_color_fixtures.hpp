#pragma once

// Small meshes and checks shared by the TextureToColor tests.

#include "libslic3r/TextureToColor/TriMesh.hpp"

#include <catch2/catch_test_macros.hpp>

#include <admesh/stl.h>

#include <cmath>
#include <cstddef>

namespace Slic3r::tex2color::test {

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
