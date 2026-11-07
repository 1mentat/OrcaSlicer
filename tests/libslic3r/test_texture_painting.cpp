#include <opencv2/core/hal/interface.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/matx.hpp>
#include <opencv2/imgcodecs.hpp>
#include "libslic3r/TriangleSelector.hpp"
#include "libslic3r/BoundingBox.hpp"
#include <cmath>
#include <atomic>
#include <algorithm>
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/TexturePainting.hpp"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <array>
#include <cstddef>
#include <string>
#include <vector>

using namespace Slic3r;

namespace {

using Color = std::array<std::size_t, 3>;

} // namespace

TEST_CASE("Texture painting decodes raw RGBA pixels to contiguous BGR", "[TexturePainting]")
{
    const TextureImage image { 2, 1, 4, { 255, 0, 0, 11, 0, 255, 0, 22 } };
    std::vector<unsigned char> pixels;
    int width = 0;
    int height = 0;
    REQUIRE(decode_texture_to_pixels(image, pixels, width, height));
    CHECK(width == 2);
    CHECK(height == 1);
    CHECK(pixels == std::vector<unsigned char> { 0, 0, 255, 0, 255, 0 });

    TextureImage invalid = image;
    invalid.channels = 2;
    CHECK_FALSE(decode_texture_to_pixels(invalid, pixels, width, height));
}

TEST_CASE("Texture painting chooses the nearest filament for every cluster", "[TexturePainting]")
{
    const std::vector<Color> clusters { { 0, 0, 0 }, { 255, 255, 255 } };
    const std::vector<std::array<float, 4>> filaments {
        { 0.f, 0.f, 0.f, 1.f }, { 1.f, 1.f, 1.f, 1.f },
    };
    const auto matches = match_clusters_to_filaments(clusters, filaments, { "black", "white" });
    REQUIRE(matches.size() == 2);
    CHECK(matches[0].filament_index == 0);
    CHECK(matches[1].filament_index == 1);
    CHECK_THAT(matches[0].delta_e, Catch::Matchers::WithinAbs(0., 1e-12));
    CHECK_THAT(matches[1].delta_e, Catch::Matchers::WithinAbs(0., 1e-12));
}

namespace {

constexpr Color RED{{255, 0, 0}};
constexpr Color BLUE{{0, 0, 255}};

void require_valid_painted_geometry(const PaintedMesh &mesh)
{
    REQUIRE_FALSE(mesh.vertices.empty());
    REQUIRE_FALSE(mesh.indices.empty());
    REQUIRE(mesh.face_colors.size() == mesh.indices.size());
    for (const auto &vertex : mesh.vertices)
        for (float coordinate : vertex)
            REQUIRE(std::isfinite(coordinate));
    for (const auto &face : mesh.indices)
        for (int index : face) {
            REQUIRE(index >= 0);
            REQUIRE(static_cast<std::size_t>(index) < mesh.vertices.size());
        }
}

void check_color(const Color &actual, const Color &expected)
{
    CHECK(actual[0] == expected[0]);
    CHECK(actual[1] == expected[1]);
    CHECK(actual[2] == expected[2]);
}

void check_clustered_red(const Color &actual)
{
    // The texture-to-color path averages seven samples with float Gaussian
    // weights before clustering.  Their finite-precision sum can land just
    // below one, and the resulting channel is intentionally quantized to an
    // integer.  Keep the direct decode/sample assertions exact while allowing
    // that documented one-unit clustered-path quantization.
    CHECK(actual[0] >= 254);
    CHECK(actual[0] <= 255);
    CHECK(actual[1] == 0);
    CHECK(actual[2] == 0);
}

TexturedMesh multi_material_fixture()
{
    TexturedMesh mesh;
    mesh.vertices = {
        { 0.f, 0.f, 0.f }, { 1.f, 0.f, 0.f }, { 0.f, 1.f, 0.f },
        { 2.f, 0.f, 0.f }, { 2.f, 1.f, 0.f },
    };
    mesh.indices = { { 0, 1, 2 }, { 0, 2, 3 }, { 0, 3, 4 } };
    mesh.material_ids = { 0, 1, 2 };
    mesh.material_texture_map = { 0, 1, -1 };
    mesh.material_colors = {
        { 1.f, 0.f, 0.f, 1.f }, { 0.f, 0.f, 1.f, 1.f }, { 0.25f, 0.5f, 0.75f, 1.f },
    };
    // Each texture is 2x1 RGB: UV (0,0) samples its first pixel exactly.
    mesh.textures = {
        { 2, 1, 3, { 255, 0, 0, 0, 255, 0 } },
        { 2, 1, 3, { 0, 0, 255, 255, 255, 0 } },
    };
    mesh.uv_coords.resize(9, { 0.f, 0.f });
    mesh.uv_indices = {
        { 0, 0, 0 }, { 3, 3, 3 }, { 6, 6, 6 },
    };
    return mesh;
}

double triangle_area(const std::array<float, 3> &a,
                    const std::array<float, 3> &b,
                    const std::array<float, 3> &c)
{
    const double ab[3] = { b[0] - a[0], b[1] - a[1], b[2] - a[2] };
    const double ac[3] = { c[0] - a[0], c[1] - a[1], c[2] - a[2] };
    const double cross[3] = {
        ab[1] * ac[2] - ab[2] * ac[1],
        ab[2] * ac[0] - ab[0] * ac[2],
        ab[0] * ac[1] - ab[1] * ac[0],
    };
    return 0.5 * std::sqrt(cross[0] * cross[0] + cross[1] * cross[1] + cross[2] * cross[2]);
}

double painted_area(const PaintedMesh &mesh)
{
    require_valid_painted_geometry(mesh);
    double area = 0.;
    for (const auto &face : mesh.indices)
        area += triangle_area(mesh.vertices[face[0]], mesh.vertices[face[1]], mesh.vertices[face[2]]);
    REQUIRE(std::isfinite(area));
    return area;
}

PaintedMesh run_vertex_color_case(const std::vector<std::array<float, 4>> &vertex_colors)
{
    TexturedMesh input;
    input.vertices = { { 0.f, 0.f, 0.f }, { 2.f, 0.f, 0.f }, { 0.f, 1.f, 0.f } };
    input.indices = { { 0, 1, 2 } };
    input.precomputed_face_colors = { RED };
    input.precomputed_vertex_colors = vertex_colors;

    TexturePaintingSettings settings;
    settings.target_colors_num = 3;
    PaintedMesh output;
    REQUIRE(face_colors_to_painting(input, output, settings));
    REQUIRE(output.face_colors.size() == output.indices.size());
    REQUIRE(output.vertices.size() >= input.vertices.size());
    CHECK_THAT(painted_area(output), Catch::Matchers::WithinRel(1., 1e-5));
    for (const Color &color : output.face_colors) {
        CHECK(std::find(output.cluster_colors.begin(), output.cluster_colors.end(), color) != output.cluster_colors.end());
    }
    return output;
}

} // namespace

TEST_CASE("Texture painting samples mapped textures and material colors", "[TexturePainting]")
{
    std::vector<Color> colors;
    REQUIRE(sample_original_face_colors(multi_material_fixture(), colors));
    REQUIRE(colors.size() == 3);
    check_color(colors[0], RED);
    check_color(colors[1], BLUE);
    check_color(colors[2], { 63, 127, 191 });
}

TEST_CASE("Texture painting converts a solid RGB texture on a closed mesh", "[TexturePainting]")
{
    const TriangleMesh cube_mesh = make_cube(10., 10., 10.);
    TexturedMesh input;
    input.vertices.reserve(cube_mesh.its.vertices.size());
    for (const Vec3f &vertex : cube_mesh.its.vertices)
        input.vertices.push_back({ vertex.x(), vertex.y(), vertex.z() });
    input.indices.reserve(cube_mesh.its.indices.size());
    for (const Vec3i32 &face : cube_mesh.its.indices)
        input.indices.push_back({ face[0], face[1], face[2] });
    input.uvs.assign(input.vertices.size(), { 0.f, 0.f });
    input.textures = { { 1, 1, 3, { 255, 0, 0 } } };

    REQUIRE_FALSE(input.vertices.empty());
    REQUIRE_FALSE(input.indices.empty());
    for (const auto &vertex : input.vertices)
        for (float coordinate : vertex)
            REQUIRE(std::isfinite(coordinate));
    for (const auto &face : input.indices)
        for (int index : face) {
            REQUIRE(index >= 0);
            REQUIRE(static_cast<std::size_t>(index) < input.vertices.size());
        }

    TexturePaintingSettings settings;
    settings.target_colors_num = 1;
    settings.smooth_weight = 0.;
    settings.oversampling_iters = 1;
    PaintedMesh output;
    REQUIRE(texture_to_painting(input, output, settings));
    REQUIRE(output.face_colors.size() == output.indices.size());
    REQUIRE_FALSE(output.indices.empty());
    require_valid_painted_geometry(output);
    for (const Color &color : output.face_colors)
        check_clustered_red(color);
}

TEST_CASE("Texture painting preserves area while splitting vertex-color boundaries", "[TexturePainting]")
{
    const auto uniform = run_vertex_color_case({
        { 1.f, 0.f, 0.f, 1.f }, { 1.f, 0.f, 0.f, 1.f }, { 1.f, 0.f, 0.f, 1.f },
    });
    CHECK(uniform.indices.size() == 1);

    const auto two_clusters = run_vertex_color_case({
        { 1.f, 0.f, 0.f, 1.f }, { 0.f, 0.f, 1.f, 1.f }, { 0.f, 0.f, 1.f, 1.f },
    });
    CHECK(two_clusters.indices.size() == 3);

    const auto three_clusters = run_vertex_color_case({
        { 1.f, 0.f, 0.f, 1.f }, { 0.f, 1.f, 0.f, 1.f }, { 0.f, 0.f, 1.f, 1.f },
    });
    CHECK(three_clusters.indices.size() == 5);
}

TEST_CASE("Texture painting honors cancellation before processing", "[TexturePainting]")
{
    TexturedMesh input;
    input.vertices = { { 0.f, 0.f, 0.f }, { 1.f, 0.f, 0.f }, { 0.f, 1.f, 0.f } };
    input.indices = { { 0, 1, 2 } };
    input.precomputed_face_colors = { RED };
    input.precomputed_vertex_colors = {
        { 1.f, 0.f, 0.f, 1.f }, { 1.f, 0.f, 0.f, 1.f }, { 1.f, 0.f, 0.f, 1.f },
    };
    std::atomic<bool> cancelled{false};
    PaintedMesh output;
    CHECK_FALSE(face_colors_to_painting(input, output, {}, nullptr, [&]() {
        cancelled.store(true, std::memory_order_relaxed);
        return true;
    }));
    CHECK(cancelled.load(std::memory_order_relaxed));
}

TEST_CASE("Texture painting rejects incomplete import payloads", "[TexturePainting]")
{
    TexturedMesh empty;
    PaintedMesh painted;
    CHECK_FALSE(texture_to_painting(empty, painted));
    CHECK_FALSE(face_colors_to_painting(empty, painted));

    std::vector<Color> sampled;
    CHECK_FALSE(sample_original_face_colors(empty, sampled));

    Model model;
    ModelObject *object = model.add_object();
    REQUIRE(object != nullptr);
    ModelVolume *volume = object->add_volume(make_cube(2., 2., 2.));
    REQUIRE(volume != nullptr);
    object->add_instance();
    CHECK_FALSE(apply_painted_mesh_to_volume(painted, {}, *volume));
}

namespace {

void require_valid_indexed_geometry(const indexed_triangle_set &mesh)
{
    REQUIRE_FALSE(mesh.vertices.empty());
    REQUIRE_FALSE(mesh.indices.empty());
    for (const auto &vertex : mesh.vertices)
        REQUIRE(vertex.allFinite());
    for (const auto &face : mesh.indices) {
        REQUIRE(face.minCoeff() >= 0);
        REQUIRE(static_cast<std::size_t>(face.maxCoeff()) < mesh.vertices.size());
    }
}

} // namespace

TEST_CASE("Texture painting replaces a volume and keeps its world placement", "[TexturePainting]")
{
    Model model;
    const TriangleMesh cube_mesh = make_cube(10., 10., 10.);
    require_valid_indexed_geometry(cube_mesh.its);
    ModelObject *object = model.add_object();
    REQUIRE(object != nullptr);
    ModelVolume *volume = object->add_volume(cube_mesh);
    REQUIRE(volume != nullptr);
    REQUIRE(object->add_instance() != nullptr);

    const indexed_triangle_set cube = cube_mesh.its;
    PaintedMesh painted;
    painted.vertices.reserve(cube.vertices.size());
    for (const Vec3f &vertex : cube.vertices)
        painted.vertices.push_back({ vertex.x(), vertex.y(), vertex.z() });
    painted.indices.reserve(cube.indices.size());
    painted.face_colors.assign(cube.indices.size(), RED);
    for (const Vec3i32 &face : cube.indices)
        painted.indices.push_back({ face[0], face[1], face[2] });
    painted.cluster_colors = { RED };

    FilamentMatch match;
    match.cluster_index = 0;
    match.filament_index = 0;
    require_valid_painted_geometry(painted);
    require_valid_indexed_geometry(volume->mesh().its);
    REQUIRE(volume->source.mesh_offset.allFinite());
    REQUIRE(volume->get_matrix().matrix().allFinite());
    REQUIRE(apply_painted_mesh_to_volume(painted, { match }, *volume));

    // Reacquire the owned mesh after apply replaces it; keep no mesh reference across apply.
    require_valid_indexed_geometry(volume->mesh().its);
    REQUIRE(volume->get_matrix().matrix().allFinite());
    const BoundingBoxf3 world_box = volume->mesh().transformed_bounding_box(volume->get_matrix());
    REQUIRE(world_box.defined);
    REQUIRE(world_box.min.allFinite());
    REQUIRE(world_box.max.allFinite());
    CHECK_THAT(world_box.min.x(), Catch::Matchers::WithinAbs(0., 1e-5));
    CHECK_THAT(world_box.max.x(), Catch::Matchers::WithinAbs(10., 1e-5));
    CHECK_THAT(world_box.min.z(), Catch::Matchers::WithinAbs(0., 1e-5));
    CHECK_THAT(world_box.max.z(), Catch::Matchers::WithinAbs(10., 1e-5));
    CHECK(volume->is_mm_painted());
    REQUIRE(volume->mmu_segmentation_facets.get_data().used_states.size() >
            static_cast<std::size_t>(EnforcerBlockerType::Extruder1));
    CHECK(volume->mmu_segmentation_facets.get_data().used_states[static_cast<std::size_t>(EnforcerBlockerType::Extruder1)]);
}

namespace texture_atlas_contracts {
namespace {

using RGB = Color;

TextureImage solid_texture(int width, int height, RGB rgb)
{
    REQUIRE(width > 0);
    REQUIRE(width <= 4);
    REQUIRE(height > 0);
    REQUIRE(height <= 4);
    TextureImage image;
    image.width = width;
    image.height = height;
    image.channels = 3;
    image.data.resize(static_cast<size_t>(width) * height * 3);
    for (int y = 0; y < height; ++y)
        for (int x = 0; x < width; ++x) {
            const size_t offset = (static_cast<size_t>(y) * width + x) * 3;
            image.data[offset + 0] = static_cast<unsigned char>(rgb[0]);
            image.data[offset + 1] = static_cast<unsigned char>(rgb[1]);
            image.data[offset + 2] = static_cast<unsigned char>(rgb[2]);
        }
    return image;
}

TexturedMesh two_material_faces()
{
    TexturedMesh mesh;
    mesh.vertices = {
        { 0.f, 0.f, 0.f }, { 1.f, 0.f, 0.f }, { 0.f, 1.f, 0.f },
        { 3.f, 0.f, 0.f }, { 4.f, 0.f, 0.f }, { 3.f, 1.f, 0.f },
    };
    mesh.indices = { { 0, 1, 2 }, { 3, 4, 5 } };
    mesh.uvs.assign(mesh.vertices.size(), { 0.f, 0.f });
    mesh.material_ids = { 0, 1 };
    mesh.material_texture_map = { 0, 1 };
    mesh.material_colors = {
        { 0.f, 1.f, 0.f, 1.f },
        { 1.f, 1.f, 0.f, 1.f },
    };
    mesh.textures = {
        solid_texture(1, 1, { 255, 0, 0 }),
        solid_texture(1, 1, { 0, 0, 255 }),
    };
    return mesh;
}

TexturePaintingSettings atlas_settings()
{
    TexturePaintingSettings settings;
    settings.target_colors_num = 2;
    settings.smooth_weight = 0.;
    settings.oversampling_iters = 1;
    return settings;
}

PaintedMesh paint(const TexturedMesh &input, TexturePaintingSettings settings = atlas_settings())
{
    PaintedMesh output;
    REQUIRE(texture_to_painting(input, output, settings));
    require_valid_painted_geometry(output);
    REQUIRE(output.indices.size() == output.face_colors.size());
    REQUIRE_FALSE(output.indices.empty());
    REQUIRE(output.vertices.size() >= 3);
    return output;
}

std::vector<unsigned char> encode_image(const cv::Mat &bgr, const char *extension)
{
    REQUIRE(extension != nullptr);
    REQUIRE_FALSE(bgr.empty());
    REQUIRE(bgr.type() == CV_8UC3);
    REQUIRE(bgr.rows > 0);
    REQUIRE(bgr.rows <= 4);
    REQUIRE(bgr.cols > 0);
    REQUIRE(bgr.cols <= 4);
    std::vector<unsigned char> bytes;
    REQUIRE(cv::imencode(extension, bgr, bytes));
    REQUIRE_FALSE(bytes.empty());
    return bytes;
}

TextureImage encoded_solid_texture(int width, int height, RGB rgb, const char *extension)
{
    REQUIRE(width > 0);
    REQUIRE(width <= 4);
    REQUIRE(height > 0);
    REQUIRE(height <= 4);
    cv::Mat bgr(height, width, CV_8UC3);
    REQUIRE_FALSE(bgr.empty());
    for (int y = 0; y < height; ++y)
        for (int x = 0; x < width; ++x)
            bgr.at<cv::Vec3b>(y, x) = cv::Vec3b(
                static_cast<unsigned char>(rgb[2]),
                static_cast<unsigned char>(rgb[1]),
                static_cast<unsigned char>(rgb[0]));
    return TextureImage{-1, -1, 0, encode_image(bgr, extension)};
}

} // namespace

TEST_CASE("Texture atlas decodes an encoded PNG in the single-texture route", "[TexturePainting][TextureAtlas]")
{
    TexturedMesh mesh = two_material_faces();
    mesh.material_ids.clear();
    mesh.material_texture_map.clear();
    mesh.textures = { encoded_solid_texture(4, 4, { 255, 0, 0 }, ".png") };
    REQUIRE_FALSE(mesh.textures.front().data.empty());

    const PaintedMesh output = paint(mesh);
    REQUIRE_FALSE(output.face_colors.empty());
    for (const RGB &color : output.face_colors) {
        CHECK(color[0] > 180);
        CHECK(color[1] < 40);
        CHECK(color[2] < 40);
    }
}

} // namespace texture_atlas_contracts
