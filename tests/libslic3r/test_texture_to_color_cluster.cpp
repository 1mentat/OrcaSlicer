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

using Color = std::array<std::size_t, 3>;

constexpr Color red{{255, 0, 0}};
constexpr Color blue{{0, 0, 255}};
constexpr Color green{{0, 255, 0}};

TriMesh one_triangle()
{
    return TriMesh(
        {stl_triangle_vertex_indices(0, 1, 2)},
        {stl_vertex(0.f, 0.f, 0.f), stl_vertex(1.f, 0.f, 0.f), stl_vertex(0.f, 1.f, 0.f)});
}

TextureToColorSettings bounded_settings()
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


Vec3d as_vec3d(const TriVertex &vertex)
{
    return Vec3d(vertex[0], vertex[1], vertex[2]);
}

// True when every channel is within the 2-unit Lab round-trip quantisation of the expected colour.
bool matches_color(const Color &actual, const Color &expected)
{
    for (std::size_t channel = 0; channel < expected.size(); ++channel) {
        if (std::abs(static_cast<double>(actual[channel]) - static_cast<double>(expected[channel])) > 2.0)
            return false;
    }
    return true;
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
    CHECK(std::find(distinct_colors.begin(), distinct_colors.end(), green) != distinct_colors.end());
    for (const Color &color : distinct_colors) {
        if (color == green)
            continue;
        CHECK(color[0] > 0);
        CHECK(color[1] == 0);
        CHECK(color[2] > 0);
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

TEST_CASE("ClusterAndSmooth repairs an inconsistently wound tetrahedron",
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
    // repair can leave the solid inside out, which #235 tags as a known RepairMesh bug. The
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

// With a fixed palette, ensure_all_cluster_centers_used() puts every centre back into use after
// smoothing, so the one-face minority colour survives even at full smoothing weight.
TEST_CASE("ClusterAndSmooth keeps every fixed palette colour in use after full smoothing",
          "[TextureToColor]")
{
    const TriMesh input = closed_tetrahedron();
    const Color minority{{0, 0, 0}};
    const Color majority{{255, 255, 255}};
    const std::vector<Color> input_face_colors{minority, majority, majority, majority};
    TextureToColorSettings settings = bounded_settings();
    settings.target_colors_num = 2;
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
    std::size_t majority_faces = 0;
    std::size_t minority_faces = 0;
    bool all_colors_in_fixed_palette = true;
    for (const Color &color : out_colors) {
        const bool is_majority = matches_color(color, majority);
        const bool is_minority = matches_color(color, minority);
        if (is_majority)
            ++majority_faces;
        if (is_minority)
            ++minority_faces;
        if (!is_majority && !is_minority)
            all_colors_in_fixed_palette = false;
    }
    CHECK(all_colors_in_fixed_palette);
    CHECK(majority_faces == 3);
    CHECK(minority_faces == 1);
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

    // Full smoothing relabels the lone black face to its neighbours' colour, and the unused
    // centre is discarded.
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
