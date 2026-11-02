#include "libslic3r/Point.hpp"
#include "libslic3r/TextureToColor/ColorUtils.hpp"
#include "libslic3r/TextureToColor/TextureToColor.hpp"
#include "libslic3r/TextureToColor/TriMesh.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "texture_to_color_fixtures.hpp"

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <admesh/stl.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <functional>
#include <string>
#include <utility>
#include <vector>

using namespace Slic3r;
using namespace Slic3r::tex2color;
using namespace Slic3r::tex2color::test;
using Catch::Matchers::WithinAbs;

namespace {

Vec3d as_vec3d(const TriVertex &vertex)
{
    return Vec3d(vertex[0], vertex[1], vertex[2]);
}

// True when the mesh has a vertex at the given point.
bool has_vertex_at(const TriMesh &mesh, const TriVertex &point)
{
    return std::any_of(mesh.vertices.begin(), mesh.vertices.end(),
                       [&point](const TriVertex &v) { return (v - point).norm() <= 1e-6f; });
}

// one_triangle() with a different colour at vertex 0 than at vertices 1 and 2: only the two edges at
// vertex 0 cross the colour boundary, so only they get midpoints and the 1-2 edge stays whole.
void check_split_only_at_vertex_zero_edges(const TriMesh &out_mesh)
{
    CHECK(out_mesh.vertices.size() == 5);
    CHECK(has_vertex_at(out_mesh, TriVertex(0.5f, 0.f, 0.f)));
    CHECK(has_vertex_at(out_mesh, TriVertex(0.f, 0.5f, 0.f)));
    CHECK_FALSE(has_vertex_at(out_mesh, TriVertex(0.5f, 0.5f, 0.f)));
}

} // namespace

TEST_CASE("Vertex color clustering splits a face only at palette boundaries", "[TextureToColor]")
{
    const TriMesh mesh = one_triangle();
    const std::vector<Color> input_face_colors{red};
    const std::vector<std::array<float, 4>> vertex_colors{
        {1.f, 0.f, 0.f, 1.f},
        {0.f, 0.f, 1.f, 1.f},
        {0.f, 0.f, 1.f, 1.f},
    };

    TextureToColorSettings settings;
    settings.target_colors_num = 2;
    settings.oversampling_min_face_count = 2;
    settings.smooth_weight = 0.0;

    TriMesh out_mesh;
    std::vector<Color> out_face_colors;
    REQUIRE(ClusterAndSmooth(mesh, input_face_colors, out_mesh, out_face_colors, settings, nullptr, nullptr, vertex_colors));
    REQUIRE(out_mesh.indices.size() == 3);
    REQUIRE(out_face_colors.size() == out_mesh.indices.size());

    std::size_t red_count = 0;
    std::size_t blue_count = 0;
    for (const Color &color : out_face_colors) {
        if (color == red)
            ++red_count;
        if (color == blue)
            ++blue_count;
        const bool is_palette_color = color == red || color == blue;
        CHECK(is_palette_color);
    }
    // Only the two edges at the red vertex cross the boundary: red keeps the corner quarter
    // (v0, m01, m20), blue the other two faces.
    CHECK(red_count == 1);
    CHECK(blue_count == 2);
    check_split_only_at_vertex_zero_edges(out_mesh);
}

TEST_CASE("Vertex color splitting preserves a three-way boundary triangulation",
          "[TextureToColor]")
{
    const auto face = GENERATE(
        stl_triangle_vertex_indices(0, 1, 2),
        stl_triangle_vertex_indices(1, 2, 0),
        stl_triangle_vertex_indices(2, 0, 1));
    const TriMesh mesh(
        {face},
        {stl_vertex(0.f, 0.f, 0.f), stl_vertex(4.f, 0.f, 0.f), stl_vertex(0.f, 3.f, 0.f)});
    const std::vector<Color> input_face_colors{red};
    const std::vector<std::array<float, 4>> vertex_colors{
        {1.f, 0.f, 0.f, 1.f},
        {0.f, 1.f, 0.f, 1.f},
        {0.f, 0.f, 1.f, 1.f},
    };

    TextureToColorSettings settings;
    settings.target_colors_num = 3;
    settings.smooth_weight = 0.0;
    settings.oversampling_min_face_count = 2;
    settings.oversampling_max_face_count = 16;
    settings.mesh_repair_decision = MeshRepairDecision::ImportWithoutRepair;

    TriMesh out_mesh;
    std::vector<Color> out_face_colors;
    REQUIRE(ClusterAndSmooth(mesh, input_face_colors, out_mesh, out_face_colors, settings, nullptr, nullptr, vertex_colors));
    // Every edge is split at its midpoint and the centre triangle is cut once: 3 corners + 3 edge
    // midpoints + 1 interior point, 3 corner faces + 2 centre faces.
    REQUIRE(out_mesh.indices.size() == 5);
    REQUIRE(out_mesh.vertices.size() == 7);
    REQUIRE(out_face_colors.size() == out_mesh.indices.size());
    check_finite_and_valid(out_mesh);

    double summed_area = 0.0;
    for (const auto &face_indices : out_mesh.indices) {
        const TriVertex &a = out_mesh.vertices[static_cast<std::size_t>(face_indices[0])];
        const TriVertex &b = out_mesh.vertices[static_cast<std::size_t>(face_indices[1])];
        const TriVertex &c = out_mesh.vertices[static_cast<std::size_t>(face_indices[2])];
        const double twice_area = static_cast<double>(b.x() - a.x()) * static_cast<double>(c.y() - a.y())
                                - static_cast<double>(b.y() - a.y()) * static_cast<double>(c.x() - a.x());
        CHECK(twice_area > 0.0);
        summed_area += twice_area * 0.5;
    }
    CHECK_THAT(summed_area, WithinAbs(6.0, 1e-5));

    // All three edges are split at their midpoints. The centre triangle is cut once more, from the
    // midpoint between the two edge midpoints next to the vertex opposite the longest edge (the
    // right-angle corner (0,0); the hypotenuse is the longest edge).
    const TriVertex p0(0.f, 0.f, 0.f), p1(4.f, 0.f, 0.f), p2(0.f, 3.f, 0.f);
    const TriVertex m01 = (p0 + p1) / 2.f, m12 = (p1 + p2) / 2.f, m20 = (p2 + p0) / 2.f;
    const std::array<TriVertex, 4> expected_split_points{m01, m12, m20, TriVertex((m20 + m01) / 2.f)};
    for (const TriVertex &expected : expected_split_points) {
        bool found = false;
        for (const TriVertex &actual : out_mesh.vertices) {
            const bool same_x = std::abs(static_cast<double>(actual.x() - expected.x())) <= 1e-5;
            const bool same_y = std::abs(static_cast<double>(actual.y() - expected.y())) <= 1e-5;
            const bool same_z = std::abs(static_cast<double>(actual.z() - expected.z())) <= 1e-5;
            if (same_x && same_y && same_z) {
                found = true;
                break;
            }
        }
        CHECK(found);
    }

    std::size_t red_count = 0;
    std::size_t green_count = 0;
    std::size_t blue_count = 0;
    for (const Color &color : out_face_colors) {
        bool in_palette = false;
        if (color == red) {
            ++red_count;
            in_palette = true;
        }
        if (color == green) {
            ++green_count;
            in_palette = true;
        }
        if (color == blue) {
            ++blue_count;
            in_palette = true;
        }
        CHECK(in_palette);
    }
    CHECK(red_count == 1);
    CHECK(green_count == 2);
    CHECK(blue_count == 2);
}

TEST_CASE("ClusterAndSmooth honors the adaptive cluster cap on its generic public route",
          "[TextureToColor]")
{
    const TriMesh mesh = closed_tetrahedron();
    const std::vector<Color> input_face_colors{red, green, blue, red};

    TextureToColorSettings settings;
    settings.target_colors_num = 0;
    settings.max_cluster_k = 2;
    settings.max_color_distance = 0.0;
    settings.smooth_weight = 0.0;
    settings.oversampling_min_face_count = 4;
    settings.mesh_repair_decision = MeshRepairDecision::ImportWithoutRepair;

    TriMesh out_mesh;
    std::vector<Color> out_face_colors;
    REQUIRE(ClusterAndSmooth(mesh, input_face_colors, out_mesh, out_face_colors, settings));
    REQUIRE(out_mesh.indices.size() == mesh.indices.size());
    REQUIRE(out_mesh.vertices.size() == mesh.vertices.size());
    REQUIRE(out_face_colors.size() == out_mesh.indices.size());
    CHECK(color_utils::is_closed(out_mesh));
    CHECK_THAT(static_cast<double>(std::abs(its_volume(out_mesh))), WithinAbs(1.0 / 6.0, 1e-5));
    check_finite_and_valid(out_mesh);

    std::vector<Color> distinct_colors;
    for (const Color &color : out_face_colors) {
        bool already_present = false;
        for (const Color &distinct : distinct_colors) {
            if (distinct == color) {
                already_present = true;
                break;
            }
        }
        if (!already_present)
            distinct_colors.push_back(color);
    }
    CHECK(distinct_colors.size() == 2);
    // The cap merges the two closest input colours (red and blue, closer in Lab than either is to
    // green) into one centre and keeps green on its own.
    // (Checked to hold with the CIEDE2000 T-term and mean-hue corrections applied as well.)
    CHECK(std::any_of(distinct_colors.begin(), distinct_colors.end(), [](const Color &c) { return matches_color(c, green); }));
    for (const Color &color : distinct_colors) {
        if (matches_color(color, green))
            continue;
        CHECK(color[0] > 2);
        CHECK(color[1] <= 2);
        CHECK(color[2] > 2);
    }
}

TEST_CASE("ClusterAndSmooth requests a repair decision for an open mesh",
          "[TextureToColor]")
{
    const TriMesh input = open_tetrahedron();
    const std::vector<Color> input_colors{red, green, blue};
    TextureToColorSettings settings = bounded_settings();
    settings.mesh_repair_decision = MeshRepairDecision::Ask;
    bool decision_required = false;
    settings.mesh_repair_decision_required = &decision_required;

    TriMesh out_mesh;
    std::vector<Color> out_colors;
    CHECK_FALSE(ClusterAndSmooth(input, input_colors, out_mesh, out_colors, settings));
    CHECK(decision_required);
    CHECK(out_mesh.indices.size() == input.indices.size());
    CHECK(out_mesh.vertices.size() == input.vertices.size());
    CHECK(out_colors.empty());
}

// Only the outcome: ClusterAndSmooth exposes no hole-filling limits, so this cannot tell an in-place
// reorientation from dropping and refilling the conflicting face; the RepairMesh tests prove the
// in-place case.
TEST_CASE("ClusterAndSmooth returns a closed, consistently wound mesh for an inconsistently wound tetrahedron",
          "[TextureToColor]")
{
    TriMesh input = closed_tetrahedron();
    std::swap(input.indices[0][1], input.indices[0][2]);
    const std::vector<Color> input_colors(input.indices.size(), red);
    const TextureToColorSettings settings = bounded_settings();

    TriMesh out_mesh;
    std::vector<Color> out_colors;
    REQUIRE(ClusterAndSmooth(input, input_colors, out_mesh, out_colors, settings));
    REQUIRE(out_mesh.indices.size() == 4);
    REQUIRE(out_colors.size() == out_mesh.indices.size());
    CHECK(color_utils::is_closed(out_mesh));
    CHECK_THAT(static_cast<double>(std::abs(its_volume(out_mesh))),
               WithinAbs(1.0 / 6.0, 1e-5));
    check_finite_and_valid(out_mesh);

    // Repair makes neighbouring faces consistently oriented. The global sign is not asserted: the
    // repair can leave the solid inside out (a known RepairMesh bug, tagged in its own tests). The
    // tetrahedron's centroid gives an independent sign.
    const Vec3d interior(0.25, 0.25, 0.25);
    int orientation = 0;
    for (const TriFace &face : out_mesh.indices) {
        const Vec3d a = as_vec3d(out_mesh.vertices[static_cast<std::size_t>(face[0])]);
        const Vec3d b = as_vec3d(out_mesh.vertices[static_cast<std::size_t>(face[1])]);
        const Vec3d c = as_vec3d(out_mesh.vertices[static_cast<std::size_t>(face[2])]);
        const Vec3d face_center = (a + b + c) / 3.0;
        const double facing = (b - a).cross(c - a).dot(face_center - interior);
        REQUIRE(std::abs(facing) > 1e-9);
        if (orientation == 0)
            orientation = facing > 0.0 ? 1 : -1;
        CHECK(facing * orientation > 0.0);
    }
    for (const Color &color : out_colors)
        CHECK(color == red);
}

TEST_CASE("Repair callback rejection keeps importing the original open mesh",
          "[TextureToColor]")
{
    const TriMesh input = open_tetrahedron();
    const std::vector<Color> input_colors(input.indices.size(), red);
    TextureToColorSettings settings = bounded_settings();
    settings.mesh_repair_decision = MeshRepairDecision::RepairAndImport;
    std::size_t callback_calls = 0;
    std::size_t callback_faces = 0;
    settings.mesh_repair_callback = [&](const indexed_triangle_set &source,
                                        indexed_triangle_set &,
                                        std::function<void(const char *, unsigned)>,
                                        std::function<bool()>,
                                        std::string *error) {
        ++callback_calls;
        callback_faces = source.indices.size();
        if (error != nullptr)
            *error = "fixture declines repair";
        return false;
    };

    TriMesh out_mesh;
    std::vector<Color> out_colors;
    REQUIRE(ClusterAndSmooth(input, input_colors, out_mesh, out_colors, settings));
    CHECK(callback_calls == 1);
    CHECK(callback_faces == input.indices.size());
    CHECK(out_mesh.indices.size() == input.indices.size());
    CHECK_FALSE(color_utils::is_closed(out_mesh));
    CHECK(out_colors.size() == out_mesh.indices.size());
    check_finite_and_valid(out_mesh);
    for (const Color &color : out_colors)
        CHECK(color == red);
}

TEST_CASE("Adaptive vertex-color clustering splits only the distinct color boundary",
          "[TextureToColor]")
{
    const auto [max_cluster_k, expected_faces] = GENERATE(
        table<std::size_t, std::size_t>({{1, 1}, {2, 3}}));
    const TriMesh input = one_triangle();
    const std::vector<Color> input_face_colors{red};
    const std::vector<std::array<float, 4>> vertex_colors{
        {1.f, 0.f, 0.f, 1.f},
        {0.f, 1.f, 0.f, 1.f},
        {0.f, 1.f, 0.f, 1.f},
    };
    TextureToColorSettings settings = bounded_settings();
    settings.target_colors_num = 0;
    settings.max_cluster_k = max_cluster_k;
    settings.max_color_distance = 0.0;
    settings.oversampling_min_face_count = 2;

    TriMesh out_mesh;
    std::vector<Color> out_colors;
    REQUIRE(ClusterAndSmooth(input, input_face_colors, out_mesh, out_colors,
                             settings, nullptr, nullptr, vertex_colors));
    REQUIRE(out_mesh.indices.size() == expected_faces);
    REQUIRE(out_colors.size() == out_mesh.indices.size());
    check_finite_and_valid(out_mesh);

    double area = 0.0;
    for (const TriFace &face : out_mesh.indices) {
        const Vec3d a = as_vec3d(out_mesh.vertices[static_cast<std::size_t>(face[0])]);
        const Vec3d b = as_vec3d(out_mesh.vertices[static_cast<std::size_t>(face[1])]);
        const Vec3d c = as_vec3d(out_mesh.vertices[static_cast<std::size_t>(face[2])]);
        const double twice_area = (b - a).cross(c - a).z();
        CHECK(twice_area > 0.0);
        area += twice_area * 0.5;
    }
    CHECK_THAT(area, WithinAbs(0.5, 1e-6));

    if (max_cluster_k == 2) {
        check_split_only_at_vertex_zero_edges(out_mesh);
        std::size_t red_faces = 0;
        std::size_t green_faces = 0;
        bool all_colors_in_expected_palette = true;
        for (const Color &color : out_colors) {
            const bool is_red = matches_color(color, red);
            const bool is_green = matches_color(color, green);
            if (is_red)
                ++red_faces;
            if (is_green)
                ++green_faces;
            if (!is_red && !is_green)
                all_colors_in_expected_palette = false;
        }
        CHECK(all_colors_in_expected_palette);
        CHECK(red_faces > 0);
        CHECK(green_faces > 0);
    }
}

namespace {

// A 10 mm cube, white except for one bottom and one top triangle, which are black. The two black
// faces do not touch, so each is surrounded by white.
std::pair<TriMesh, std::vector<Color>> cube_with_two_isolated_black_faces()
{
    TriMesh cube(make_cube(10., 10., 10.).its);
    std::vector<Color> colors(cube.indices.size(), Color{{255, 255, 255}});
    const auto first_face_at = [&cube](float z) {
        for (std::size_t i = 0; i < cube.indices.size(); ++i) {
            const auto &face = cube.indices[i];
            if (cube.vertices[face[0]].z() == z && cube.vertices[face[1]].z() == z && cube.vertices[face[2]].z() == z)
                return i;
        }
        return cube.indices.size();
    };
    const std::size_t bottom = first_face_at(0.f);
    const std::size_t top = first_face_at(10.f);
    REQUIRE(bottom < colors.size());
    REQUIRE(top < colors.size());
    colors[bottom] = Color{{0, 0, 0}};
    colors[top] = Color{{0, 0, 0}};
    return {cube, colors};
}

std::size_t count_faces(const std::vector<Color> &colors, const Color &expected)
{
    return std::size_t(std::count_if(colors.begin(), colors.end(), [&](const Color &c) { return matches_color(c, expected); }));
}

} // namespace

// Smoothing relabels both isolated black faces white; with a fixed palette,
// ensure_all_cluster_centers_used() then puts black back on one face so every palette colour stays in use.
TEST_CASE("Fixed-palette smoothing changes labels but keeps every palette colour in use", "[TextureToColor]")
{
    // Plain references (not structured bindings) so the lambda below can capture them in C++17.
    const auto fixture = cube_with_two_isolated_black_faces();
    const TriMesh &cube = fixture.first;
    const std::vector<Color> &colors = fixture.second;
    const Color black{{0, 0, 0}};
    const Color white{{255, 255, 255}};
    TextureToColorSettings settings = bounded_settings();
    settings.target_colors_num = 2;
    settings.oversampling_min_face_count = cube.indices.size();

    const auto run = [&](double smooth_weight) {
        settings.smooth_weight = smooth_weight;
        TriMesh out_mesh;
        std::vector<Color> out_colors;
        REQUIRE(ClusterAndSmooth(cube, colors, out_mesh, out_colors, settings));
        REQUIRE(out_colors.size() == cube.indices.size());
        return out_colors;
    };

    const std::vector<Color> unsmoothed = run(0.0);
    CHECK(count_faces(unsmoothed, black) == 2);
    CHECK(count_faces(unsmoothed, white) == cube.indices.size() - 2);

    const std::vector<Color> smoothed = run(1.0);
    CHECK(count_faces(smoothed, black) == 1);
    CHECK(count_faces(smoothed, white) == cube.indices.size() - 1);
}

TEST_CASE("Adaptive smoothing absorbs a one-face colour that the unsmoothed result keeps",
          "[TextureToColor]")
{
    const TriMesh input = closed_tetrahedron();
    const Color minority{{0, 0, 0}};
    const Color majority{{255, 255, 255}};
    const std::vector<Color> input_face_colors{minority, majority, majority, majority};
    TextureToColorSettings settings = bounded_settings();
    settings.target_colors_num = 0;
    settings.max_cluster_k = 2;
    settings.max_color_distance = 0.0;
    // Without smoothing the adaptive palette keeps both colours: black stays on its face.
    settings.smooth_weight = 0.0;
    {
        TriMesh unsmoothed_mesh;
        std::vector<Color> unsmoothed;
        REQUIRE(ClusterAndSmooth(input, input_face_colors, unsmoothed_mesh, unsmoothed, settings));
        REQUIRE(unsmoothed.size() == input.indices.size());
        CHECK(matches_color(unsmoothed[0], minority));
    }

    // Full smoothing relabels the lone black face to its neighbours' colour.
    settings.smooth_weight = 1.0;

    TriMesh out_mesh;
    std::vector<Color> out_colors;
    REQUIRE(ClusterAndSmooth(input, input_face_colors, out_mesh, out_colors, settings));
    REQUIRE(out_mesh.indices.size() == input.indices.size());
    REQUIRE(out_colors.size() == out_mesh.indices.size());
    CHECK(color_utils::is_closed(out_mesh));
    CHECK_THAT(static_cast<double>(std::abs(its_volume(out_mesh))),
               WithinAbs(1.0 / 6.0, 1e-5));
    check_finite_and_valid(out_mesh);

    for (const Color &color : out_colors)
        CHECK(matches_color(color, majority));
}
