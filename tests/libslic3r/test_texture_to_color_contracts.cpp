#include <catch2/catch_all.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <array>
#include <admesh/stl.h>
#include <cmath>
#include <cstddef>
#include <memory>
#include <opencv2/core.hpp>
#include <vector>

#include "TextureToColor/TriMesh.hpp"
#include "Point.hpp"
#include "catch2/catch_test_macros.hpp"
#include "TextureToColor/ColorUtils.hpp"
#include "TextureToColor/Repair.hpp"
#include "TriangleMesh.hpp"
#include "libslic3r/TextureToColor/TextureToColor.hpp"

using namespace Slic3r;
using namespace Slic3r::tex2color;
using Catch::Matchers::WithinAbs;

namespace {

using Color = std::array<std::size_t, 3>;

constexpr Color red {{255, 0, 0}};
constexpr Color blue{{0, 0, 255}};
constexpr Color green{{0, 255, 0}};

TriMesh one_triangle()
{
    return TriMesh(
        {stl_triangle_vertex_indices(0, 1, 2)},
        {stl_vertex(0.f, 0.f, 0.f), stl_vertex(1.f, 0.f, 0.f), stl_vertex(0.f, 1.f, 0.f)});
}

std::vector<std::vector<Vec2f>> one_face_uv()
{
    return {{Vec2f(0.f, 0.f), Vec2f(1.f, 0.f), Vec2f(0.f, 1.f)}};
}

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

std::vector<std::vector<Vec2f>> tetrahedron_uvs()
{
    return {
        {Vec2f(0.f, 0.f), Vec2f(1.f, 0.f), Vec2f(0.f, 1.f)},
        {Vec2f(0.f, 0.f), Vec2f(1.f, 0.f), Vec2f(0.f, 1.f)},
        {Vec2f(0.f, 0.f), Vec2f(1.f, 0.f), Vec2f(0.f, 1.f)},
        {Vec2f(0.f, 0.f), Vec2f(1.f, 0.f), Vec2f(0.f, 1.f)},
    };
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

TriMesh open_tetrahedron()
{
    TriMesh mesh = closed_tetrahedron();
    mesh.indices.pop_back();
    return mesh;
}

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
            const std::size_t index = static_cast<std::size_t>(face[component]);
            REQUIRE(index < mesh.vertices.size());
        }
    }
}

} // namespace

TEST_CASE("Texture-to-color rejects invalid texture inputs and clears outputs", "[TextureToColor]")
{
    const TriMesh mesh = one_triangle();
    TriMesh color_mesh = mesh;
    std::vector<Color> face_colors{red};

    cv::Mat empty_texture;
    CHECK_FALSE(TextureToColor(mesh, one_face_uv(), empty_texture, color_mesh, face_colors));
    CHECK(color_mesh.indices.empty());
    CHECK(face_colors.empty());

    color_mesh = mesh;
    face_colors = {red};
    cv::Mat grayscale(1, 1, CV_8UC1, cv::Scalar(0));
    CHECK_FALSE(TextureToColor(mesh, one_face_uv(), grayscale, color_mesh, face_colors));
    CHECK(color_mesh.indices.empty());
    CHECK(face_colors.empty());

    color_mesh = mesh;
    face_colors = {red};
    cv::Mat rgb(1, 1, CV_8UC3, cv::Scalar(0, 0, 0));
    CHECK_FALSE(TextureToColor(mesh, {{Vec2f(0.f, 0.f), Vec2f(1.f, 0.f)}}, rgb, color_mesh, face_colors));
    CHECK(color_mesh.indices.empty());
    CHECK(face_colors.empty());
}

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
    CHECK(red_count > 0);
    CHECK(blue_count > 0);
}

TEST_CASE("TextureToColor samples a closed tetrahedron through the public route",
          "[TextureToColor][TextureToColorPublic]")
{
    const TriMesh mesh = closed_tetrahedron();
    const auto uv_coords = tetrahedron_uvs();
    const Color expected_rgb{{233, 19, 7}};
    // Weighted texture sampling and the Lab round trip quantize RGB channels.
    constexpr double kColorQuantizationTolerance = 2.0;
    cv::Mat texture(1, 1, CV_8UC3, cv::Scalar(7, 19, 233));

    TriMesh color_mesh;
    std::vector<Color> face_colors;
    REQUIRE(TextureToColor(mesh, uv_coords, texture, color_mesh, face_colors, bounded_settings()));
    REQUIRE_FALSE(color_mesh.indices.empty());
    REQUIRE(face_colors.size() == color_mesh.indices.size());
    for (const Color &color : face_colors) {
        for (std::size_t channel = 0; channel < expected_rgb.size(); ++channel)
            CHECK_THAT(static_cast<double>(color[channel]),
                       WithinAbs(static_cast<double>(expected_rgb[channel]),
                                 kColorQuantizationTolerance));
    }
    check_finite_and_valid(color_mesh);
}

TEST_CASE("TextureToColor cancellation clears outputs before returning",
          "[TextureToColor][TextureToColorPublic]")
{
    const TriMesh mesh = closed_tetrahedron();
    const auto uv_coords = tetrahedron_uvs();
    cv::Mat texture(1, 1, CV_8UC3, cv::Scalar(7, 19, 233));
    TriMesh color_mesh = mesh;
    std::vector<Color> face_colors{{233, 19, 7}};
    const auto cancel_immediately = []() { return true; };

    const bool completed = TextureToColor(
        mesh, uv_coords, texture, color_mesh, face_colors, bounded_settings(), nullptr,
        cancel_immediately);
    CHECK_FALSE(completed);
    CHECK(color_mesh.indices.empty());
    CHECK(color_mesh.vertices.empty());
    CHECK(face_colors.empty());
}

TEST_CASE("RepairMesh applies both bounded boundary limits",
          "[TextureToColor][TextureToColorRepair]")
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
          "[TextureToColor][TextureToColorRepair]")
{
    const TriMesh input = reversed_tetrahedron_with_degenerate_face();
    std::shared_ptr<TriMesh> repaired;

    REQUIRE(RepairMesh(input, repaired));
    REQUIRE(repaired);
    check_finite_and_valid(*repaired);
    CHECK(repaired->vertices.size() == 4);
    CHECK(repaired->indices.size() == 4);
    CHECK(color_utils::is_closed(*repaired));
    CHECK_THAT(std::abs(its_volume(*repaired)), WithinAbs(1.f / 6.f, 1e-5f));
}

TEST_CASE("RepairMesh cancellation stops at each published stage",
          "[TextureToColor][TextureToColorRepair]")
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

TEST_CASE("Vertex color splitting preserves a three-way boundary triangulation",
          "[TextureToColor][TextureToColorExtended]")
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

    const std::array<TriVertex, 4> expected_split_points{
        TriVertex(2.f, 0.f, 0.f),
        TriVertex(2.f, 1.5f, 0.f),
        TriVertex(0.f, 1.5f, 0.f),
        TriVertex(1.f, 0.75f, 0.f),
    };
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
          "[TextureToColor][TextureToColorExtended]")
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
}
