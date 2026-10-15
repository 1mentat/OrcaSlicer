#include <catch2/catch_all.hpp>

#include <array>
#include <cstddef>
#include <opencv2/core/types.hpp>
#include <vector>

#include <opencv2/core.hpp>

#include "TextureToColor/TriMesh.hpp"
#include "Point.hpp"
#include "catch2/catch_test_macros.hpp"
#include "libslic3r/TextureToColor/TextureToColor.hpp"

using namespace Slic3r;
using namespace Slic3r::tex2color;

namespace {

using Color = std::array<std::size_t, 3>;

constexpr Color red {{255, 0, 0}};
constexpr Color blue{{0, 0, 255}};

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
