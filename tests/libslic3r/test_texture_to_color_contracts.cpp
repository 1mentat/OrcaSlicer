#include "libslic3r/Point.hpp"
#include "libslic3r/TextureToColor/Callbacks.hpp"
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
#include <opencv2/core.hpp>
#include <opencv2/core/matx.hpp>
#include <opencv2/core/types.hpp>

#include <array>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <functional>
#include <limits>
#include <mutex>
#include <string>
#include <utility>
#include <vector>


using namespace Slic3r;
using namespace Slic3r::tex2color;
using namespace Slic3r::tex2color::test;
using Catch::Matchers::WithinAbs;

namespace {

std::vector<std::vector<Vec2f>> one_face_uv()
{
    return {{Vec2f(0.f, 0.f), Vec2f(1.f, 0.f), Vec2f(0.f, 1.f)}};
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


TextureToColorSettings repair_import_settings(std::size_t &callback_calls,
                                              std::size_t &callback_input_faces,
                                              bool &callback_progress_seen,
                                              bool &callback_saw_cancel_false,
                                              std::atomic<bool> *callback_finished = nullptr)
{
    TextureToColorSettings settings;
    settings.target_colors_num = 1;
    settings.smooth_weight = 0.0;
    settings.oversampling_iters = 1;
    settings.oversampling_max_face_count = 64;
    settings.mesh_repair_decision = MeshRepairDecision::RepairAndImport;
    settings.mesh_repair_callback = [&, callback_finished](const indexed_triangle_set &input,
                                                            indexed_triangle_set &repaired,
                                                            std::function<void(const char *, unsigned)> progress,
                                                            std::function<bool()> cancelled,
                                                            std::string *error) {
        ++callback_calls;
        callback_input_faces = input.indices.size();
        callback_saw_cancel_false = !cancelled();
        if (progress) {
            progress("private repair callback", 37);
            callback_progress_seen = true;
        }
        TriMesh fixed = closed_tetrahedron();
        repaired = fixed;
        if (callback_finished != nullptr)
            *callback_finished = true;
        (void) error;
        return true;
    };
    return settings;
}

std::vector<std::vector<Vec2f>> two_tone_open_tetra_uvs()
{
    const std::vector<Vec2f> red_uv{{0.f, 0.f}, {0.f, 0.f}, {0.f, 0.f}};
    // With TextureToColor's fractional UV addressing, 2/3 maps exactly to
    // column 2 of a four-column image and remains on the blue plateau.
    const std::vector<Vec2f> blue_uv{{2.f / 3.f, 0.f}, {2.f / 3.f, 0.f}, {2.f / 3.f, 0.f}};
    return {red_uv, blue_uv, blue_uv};
}

cv::Mat two_tone_texture()
{
    cv::Mat texture(1, 4, CV_8UC3);
    texture.at<cv::Vec3b>(0, 0) = cv::Vec3b(7, 19, 233);
    texture.at<cv::Vec3b>(0, 1) = cv::Vec3b(7, 19, 233);
    texture.at<cv::Vec3b>(0, 2) = cv::Vec3b(233, 19, 7);
    texture.at<cv::Vec3b>(0, 3) = cv::Vec3b(233, 19, 7);
    return texture;
}

} // namespace

TEST_CASE("TextureToColor preserves repaired topology after pre-repair oversampling",
          "[TextureToColor]")
{
    const TriMesh input = open_tetrahedron();
    const auto uv_coords = two_tone_open_tetra_uvs();
    const cv::Mat texture = two_tone_texture();

    std::size_t callback_calls = 0;
    std::size_t callback_input_faces = 0;
    bool callback_progress_seen = false;
    bool callback_saw_cancel_false = false;
    TextureToColorSettings settings = repair_import_settings(
        callback_calls, callback_input_faces, callback_progress_seen, callback_saw_cancel_false);
    settings.target_colors_num = 2;

    TriMesh repaired_output;
    std::vector<Color> repaired_colors;
    REQUIRE(TextureToColor(input, uv_coords, texture, repaired_output, repaired_colors,
                           settings, nullptr, []() { return false; }));
    CHECK(callback_calls == 1);
    CHECK(callback_input_faces == input.indices.size() * 4);
    CHECK(callback_progress_seen);
    CHECK(callback_saw_cancel_false);
    CHECK(repaired_output.indices.size() == 4);
    CHECK(repaired_colors.size() == repaired_output.indices.size());
    CHECK(color_utils::is_closed(repaired_output));
    check_finite_and_valid(repaired_output);

    const Color expected_red{{233, 19, 7}};
    const Color expected_blue{{7, 19, 233}};
    std::size_t red_count = 0;
    std::size_t blue_count = 0;
    for (const Color &color : repaired_colors) {
        if (matches_color(color, expected_red))
            ++red_count;
        if (matches_color(color, expected_blue))
            ++blue_count;
    }
    CHECK(red_count > 0);
    CHECK(blue_count > 0);

    std::size_t control_callback_calls = 0;
    std::size_t control_input_faces = 0;
    bool control_progress_seen = false;
    bool control_saw_cancel_false = false;
    TextureToColorSettings control = repair_import_settings(
        control_callback_calls, control_input_faces, control_progress_seen, control_saw_cancel_false);
    control.mesh_repair_decision = MeshRepairDecision::ImportWithoutRepair;
    TriMesh control_output;
    std::vector<Color> control_colors;
    REQUIRE(TextureToColor(input, uv_coords, texture, control_output, control_colors,
                           control, nullptr, []() { return false; }));
    CHECK(control_callback_calls == 0);
    CHECK(control_output.indices.size() == input.indices.size() * 4);
    CHECK_FALSE(color_utils::is_closed(control_output));
    CHECK(control_colors.size() == control_output.indices.size());
}

TEST_CASE("TextureToColor reports cancellation after the mesh repair callback",
          "[TextureToColor]")
{
    const TriMesh input = open_tetrahedron();
    const auto uv_coords = two_tone_open_tetra_uvs();
    const cv::Mat texture = two_tone_texture();
    std::size_t callback_calls = 0;
    std::size_t callback_input_faces = 0;
    bool callback_progress_seen = false;
    bool callback_saw_cancel_false = false;
    std::atomic<bool> callback_finished{false};
    TextureToColorSettings settings = repair_import_settings(
        callback_calls, callback_input_faces, callback_progress_seen, callback_saw_cancel_false, &callback_finished);

    TriMesh output = input;
    std::vector<Color> colors{{233, 19, 7}};
    // TextureToColor also reports progress from its TBB worker threads ("Computing face colors"),
    // so the recorder has to be thread-safe.
    std::mutex progress_mutex;
    std::vector<std::string> progress_messages;
    const auto record_progress = [&](AlgoProgress state) {
        const std::lock_guard<std::mutex> lock(progress_mutex);
        progress_messages.emplace_back(state.message ? state.message : "");
    };
    // The cancel callback is also polled from worker threads, hence the atomic flag.
    const auto cancel_after_repair = [&]() { return callback_finished.load(); };
    CHECK_FALSE(TextureToColor(input, uv_coords, texture, output, colors,
                               settings, record_progress, cancel_after_repair));
    CHECK(callback_calls == 1);
    CHECK(callback_input_faces == input.indices.size() * 4);
    CHECK(callback_progress_seen);
    CHECK(callback_saw_cancel_false);
    CHECK(callback_finished.load());

    // "Color clustering" is the stage label TextureToColor publishes right after repair.
    bool saw_callback_stage = false;
    bool saw_color_clustering_stage = false;
    for (const std::string &message : progress_messages) {
        if (message == "private repair callback")
            saw_callback_stage = true;
        if (message == "Color clustering")
            saw_color_clustering_stage = true;
    }
    CHECK(saw_callback_stage);
    CHECK_FALSE(saw_color_clustering_stage);
}

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

TEST_CASE("TextureToColor samples each face from its own UVs and returns RGB",
          "[TextureToColor]")
{
    // Two-tone BGR texture: columns 0-1 are (7,19,233) and columns 2-3 are (233,19,7). Faces 0 and 2 look
    // up column 0, faces 1 and 3 column 2, so a face that read another face's UVs, or a BGR/RGB mix-up,
    // would get the wrong colour.
    const TriMesh mesh = closed_tetrahedron();
    const std::vector<Vec2f> left{{0.f, 0.f}, {0.f, 0.f}, {0.f, 0.f}};
    const std::vector<Vec2f> right{{2.f / 3.f, 0.f}, {2.f / 3.f, 0.f}, {2.f / 3.f, 0.f}};
    const std::vector<std::vector<Vec2f>> uv_coords{left, right, left, right};
    const Color left_rgb{{233, 19, 7}};
    const Color right_rgb{{7, 19, 233}};
    TextureToColorSettings settings = bounded_settings();
    settings.target_colors_num = 2;

    TriMesh color_mesh;
    std::vector<Color> face_colors;
    REQUIRE(TextureToColor(mesh, uv_coords, two_tone_texture(), color_mesh, face_colors, settings));
    // No oversampling beyond the four input faces, so output face i is input face i.
    REQUIRE(color_mesh.indices.size() == 4);
    REQUIRE(face_colors.size() == 4);
    CHECK(matches_color(face_colors[0], left_rgb));
    CHECK(matches_color(face_colors[1], right_rgb));
    CHECK(matches_color(face_colors[2], left_rgb));
    CHECK(matches_color(face_colors[3], right_rgb));
    check_finite_and_valid(color_mesh);
}

// Only the immediate case: the outputs are cleared on entry, before any stage can be cancelled. The
// header does not promise what a later cancellation leaves in the outputs, so that is not asserted.
TEST_CASE("Immediate cancellation returns false with the outputs cleared on entry",
          "[TextureToColor]")
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

TEST_CASE("TextureToColor auto-oversampling stops at the configured face threshold",
          "[TextureToColor]")
{
    const auto [minimum_face_count, expected_face_count] = GENERATE(
        table<std::size_t, std::size_t>({{4, 4}, {5, 16}}));
    const TriMesh input = closed_tetrahedron();
    const auto uv_coords = tetrahedron_uvs();
    const Color expected_rgb{{233, 19, 7}};
    const cv::Mat texture(1, 1, CV_8UC3, cv::Scalar(7, 19, 233));
    TextureToColorSettings settings = bounded_settings();
    settings.oversampling_iters = 0;
    settings.oversampling_min_face_count = minimum_face_count;

    TriMesh out_mesh;
    std::vector<Color> out_colors;
    REQUIRE(TextureToColor(input, uv_coords, texture, out_mesh, out_colors, settings));
    REQUIRE(out_mesh.indices.size() == expected_face_count);
    REQUIRE(out_colors.size() == expected_face_count);
    CHECK(color_utils::is_closed(out_mesh));
    CHECK_THAT(static_cast<double>(std::abs(its_volume(out_mesh))),
               WithinAbs(1.0 / 6.0, 1e-5));
    check_finite_and_valid(out_mesh);
    for (const Color &color : out_colors) {
        for (std::size_t channel = 0; channel < expected_rgb.size(); ++channel)
            CHECK_THAT(static_cast<double>(color[channel]),
                       WithinAbs(static_cast<double>(expected_rgb[channel]), 2.0));
    }
}

TEST_CASE("TextureToColor rejects invalid face texture regions and clears outputs",
          "[TextureToColor]")
{
    // All images own their small buffers; invalid metadata must be rejected
    // before any texel is sampled.
    const int invalid = GENERATE(0, 1, 2, 3, 4, 5, 6, 7, 8);
    CAPTURE(invalid);
    const TriMesh input = one_triangle();
    auto uv_coords = one_face_uv();
    cv::Mat texture(3, 3, CV_8UC3, cv::Scalar(7, 19, 233));
    std::vector<cv::Rect> regions{{1, 1, 1, 1}};
    switch (invalid) {
    case 0: regions.push_back(regions.front()); break; // Wrong face count.
    case 1: regions[0].x = -1; break;
    case 2: regions[0].width = 0; break;
    case 3: regions[0] = cv::Rect(2, 1, 2, 1); break; // Crosses the right edge.
    case 4: regions[0] = cv::Rect(1, 2, 1, 2); break; // Crosses the bottom edge.
    case 5: texture = cv::Mat(3, 3, CV_16UC3, cv::Scalar(0)); break;
    case 6: uv_coords[0][0].x() = std::numeric_limits<float>::quiet_NaN(); break;
    case 7: uv_coords[0][0].y() = std::numeric_limits<float>::infinity(); break;
    case 8: uv_coords[0][0].x() = 1.5f; break; // Regional UVs must be local.
    }

    TriMesh output = input;
    std::vector<Color> colors{red};
    CHECK_FALSE(TextureToColor(input, uv_coords, texture, output, colors,
                               bounded_settings(), nullptr, nullptr, regions));
    CHECK(output.vertices.empty());
    CHECK(output.indices.empty());
    CHECK(colors.empty());
}

TEST_CASE("TextureToColor keeps local texture regions attached through subdivision",
          "[TextureToColor]")
{
    const bool fixed_iterations = GENERATE(true, false);
    CAPTURE(fixed_iterations);
    const TriMesh input(
        {stl_triangle_vertex_indices(0, 1, 2), stl_triangle_vertex_indices(3, 4, 5)},
        {stl_vertex(0.f, 0.f, 0.f), stl_vertex(1.f, 0.f, 0.f), stl_vertex(0.f, 1.f, 0.f),
         stl_vertex(3.f, 0.f, 0.f), stl_vertex(4.f, 0.f, 0.f), stl_vertex(3.f, 1.f, 0.f)});
    const std::vector<Vec2f> local_uv(3, Vec2f(0.25f, 0.5f));
    const std::vector<std::vector<Vec2f>> uv_coords{local_uv, local_uv};
    const std::vector<cv::Rect> regions{{1, 1, 2, 2}, {2, 4, 3, 1}};
    cv::Mat texture(5, 6, CV_8UC3, cv::Scalar(0, 255, 0));
    for (int row = 1; row <= 2; ++row) {
        texture.at<cv::Vec3b>(row, 1) = cv::Vec3b(0, 0, 255);
        texture.at<cv::Vec3b>(row, 2) = cv::Vec3b(255, 0, 0);
    }
    for (int col = 2; col <= 4; ++col)
        texture.at<cv::Vec3b>(4, col) = cv::Vec3b(255, 0, 0);
    // Local U=1/4 blends the first region's red/blue texels to (191,0,63),
    // regardless of its atlas offset or the second region's unequal extent.
    const Color left_rgb{{191, 0, 63}};
    TextureToColorSettings settings = bounded_settings();
    settings.target_colors_num = 2;
    settings.oversampling_iters = fixed_iterations ? 2 : 0;
    settings.oversampling_min_face_count = 32;
    settings.oversampling_max_face_count = 64;

    TriMesh output;
    std::vector<Color> colors;
    REQUIRE(TextureToColor(input, uv_coords, texture, output, colors,
                           settings, nullptr, nullptr, regions));
    REQUIRE(output.indices.size() == 32);
    REQUIRE(colors.size() == output.indices.size());
    check_finite_and_valid(output);
    bool left_seen = false;
    bool right_seen = false;
    for (std::size_t i = 0; i < output.indices.size(); ++i) {
        const auto &face = output.indices[i];
        const double center_x = (output.vertices[face[0]].x() + output.vertices[face[1]].x()
                               + output.vertices[face[2]].x()) / 3.;
        if (center_x < 1.5) {
            CHECK(matches_color(colors[i], left_rgb));
            left_seen = true;
        } else {
            REQUIRE(center_x > 2.5);
            CHECK(matches_color(colors[i], blue));
            right_seen = true;
        }
    }
    CHECK(left_seen);
    CHECK(right_seen);
}
