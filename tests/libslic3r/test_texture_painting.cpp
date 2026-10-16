#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/matx.hpp>
#include <opencv2/imgcodecs.hpp>
#include <limits>
#include "catch2/matchers/catch_matchers.hpp"
#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <array>
#include <algorithm>
#include <cstddef>
#include <cmath>
#include <vector>

#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/TexturePainting.hpp"
#include "libslic3r/TriangleSelector.hpp"
#include "libslic3r/TriangleMesh.hpp"

using namespace Slic3r;

namespace {

using Color = std::array<std::size_t, 3>;

constexpr Color RED  { { 255, 0, 0 } };
constexpr Color BLUE { { 0, 0, 255 } };

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
    double area = 0.;
    for (const auto &face : mesh.indices)
        area += triangle_area(mesh.vertices[face[0]], mesh.vertices[face[1]], mesh.vertices[face[2]]);
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

    TexturePaintingSettings settings;
    settings.target_colors_num = 1;
    settings.smooth_weight = 0.;
    settings.oversampling_iters = 1;
    PaintedMesh output;
    REQUIRE(texture_to_painting(input, output, settings));
    REQUIRE(output.face_colors.size() == output.indices.size());
    REQUIRE_FALSE(output.indices.empty());
    for (const Color &color : output.face_colors)
        check_clustered_red(color);
}

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

TEST_CASE("Texture painting replaces a volume and keeps its world placement", "[TexturePainting]")
{
    Model model;
    ModelObject *object = model.add_object();
    ModelVolume *volume = object->add_volume(make_cube(10., 10., 10.));
    object->add_instance();

    const indexed_triangle_set cube = make_cube(10., 10., 10.).its;
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
    REQUIRE(apply_painted_mesh_to_volume(painted, { match }, *volume));

    const BoundingBoxf3 world_box = volume->mesh().transformed_bounding_box(volume->get_matrix());
    CHECK_THAT(world_box.min.x(), Catch::Matchers::WithinAbs(0., 1e-5));
    CHECK_THAT(world_box.max.x(), Catch::Matchers::WithinAbs(10., 1e-5));
    CHECK_THAT(world_box.min.z(), Catch::Matchers::WithinAbs(0., 1e-5));
    CHECK_THAT(world_box.max.z(), Catch::Matchers::WithinAbs(10., 1e-5));
    CHECK(volume->is_mm_painted());
    CHECK(volume->mmu_segmentation_facets.get_data().used_states[static_cast<std::size_t>(EnforcerBlockerType::Extruder1)]);
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
    bool cancelled = false;
    PaintedMesh output;
    CHECK_FALSE(face_colors_to_painting(input, output, {}, nullptr, [&]() {
        cancelled = true;
        return true;
    }));
    CHECK(cancelled);
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
    ModelVolume *volume = object->add_volume(make_cube(2., 2., 2.));
    object->add_instance();
    CHECK_FALSE(apply_painted_mesh_to_volume(painted, {}, *volume));
}

namespace texture_atlas_contracts {

namespace {

using RGB = std::array<std::size_t, 3>;

TextureImage solid_texture(int width, int height, RGB rgb)
{
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

TextureImage vertical_texture(int width, int height, RGB top, RGB bottom)
{
    TextureImage image;
    image.width = width;
    image.height = height;
    image.channels = 3;
    image.data.resize(static_cast<size_t>(width) * height * 3);
    for (int y = 0; y < height; ++y) {
        const RGB &rgb = y < height / 2 ? top : bottom;
        for (int x = 0; x < width; ++x) {
            const size_t offset = (static_cast<size_t>(y) * width + x) * 3;
            image.data[offset + 0] = static_cast<unsigned char>(rgb[0]);
            image.data[offset + 1] = static_cast<unsigned char>(rgb[1]);
            image.data[offset + 2] = static_cast<unsigned char>(rgb[2]);
        }
    }
    return image;
}

TextureImage horizontal_texture(int width, RGB left, RGB right)
{
    TextureImage image;
    image.width = width;
    image.height = 1;
    image.channels = 3;
    image.data.resize(static_cast<size_t>(width) * 3);
    for (int x = 0; x < width; ++x) {
        const RGB &rgb = x < width / 2 ? left : right;
        image.data[static_cast<size_t>(x) * 3 + 0] = static_cast<unsigned char>(rgb[0]);
        image.data[static_cast<size_t>(x) * 3 + 1] = static_cast<unsigned char>(rgb[1]);
        image.data[static_cast<size_t>(x) * 3 + 2] = static_cast<unsigned char>(rgb[2]);
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
    REQUIRE(output.indices.size() == output.face_colors.size());
    REQUIRE_FALSE(output.indices.empty());
    REQUIRE(output.vertices.size() >= 3);
    return output;
}

double face_center_x(const PaintedMesh &mesh, size_t face_id)
{
    const auto &f = mesh.indices[face_id];
    return (mesh.vertices[f[0]][0] + mesh.vertices[f[1]][0] + mesh.vertices[f[2]][0]) / 3.;
}

const RGB *color_near_x(const PaintedMesh &mesh, double x, double tolerance = 0.75)
{
    for (size_t i = 0; i < mesh.face_colors.size(); ++i)
        if (std::abs(face_center_x(mesh, i) - x) < tolerance)
            return &mesh.face_colors[i];
    return nullptr;
}

void require_red_blue_by_side(const PaintedMesh &mesh)
{
    bool red = false;
    bool blue = false;
    for (size_t i = 0; i < mesh.face_colors.size(); ++i) {
        const RGB &c = mesh.face_colors[i];
        const double x = face_center_x(mesh, i);
        if (x < 1.5) {
            CHECK(c[0] > 180);
            CHECK(c[1] < 40);
            CHECK(c[2] < 40);
            red = true;
        } else if (x > 2.5) {
            CHECK(c[0] < 40);
            CHECK(c[1] < 40);
            CHECK(c[2] > 180);
            blue = true;
        }
    }
    REQUIRE(red);
    REQUIRE(blue);
}

double triangle_area(const std::array<float, 3> &a,
                     const std::array<float, 3> &b,
                     const std::array<float, 3> &c)
{
    const double abx = b[0] - a[0];
    const double aby = b[1] - a[1];
    const double abz = b[2] - a[2];
    const double acx = c[0] - a[0];
    const double acy = c[1] - a[1];
    const double acz = c[2] - a[2];
    const double cx = aby * acz - abz * acy;
    const double cy = abz * acx - abx * acz;
    const double cz = abx * acy - aby * acx;
    return 0.5 * std::sqrt(cx * cx + cy * cy + cz * cz);
}

double painted_area(const PaintedMesh &mesh)
{
    double result = 0.;
    for (const auto &f : mesh.indices)
        result += triangle_area(mesh.vertices[f[0]], mesh.vertices[f[1]], mesh.vertices[f[2]]);
    return result;
}


std::vector<unsigned char> encode_image(const cv::Mat &bgr, const char *extension)
{
    std::vector<unsigned char> bytes;
    if (!cv::imencode(extension, bgr, bytes))
        return {};
    return bytes;
}

TextureImage encoded_solid_texture(int width, int height, RGB rgb, const char *extension)
{
    cv::Mat bgr(height, width, CV_8UC3);
    for (int y = 0; y < height; ++y)
        for (int x = 0; x < width; ++x)
            bgr.at<cv::Vec3b>(y, x) = cv::Vec3b(
                static_cast<unsigned char>(rgb[2]),
                static_cast<unsigned char>(rgb[1]),
                static_cast<unsigned char>(rgb[0]));
    return TextureImage{-1, -1, 0, encode_image(bgr, extension)};
}


TextureImage rgba_texture(RGB rgb, unsigned char alpha)
{
    TextureImage image;
    image.width = 1;
    image.height = 1;
    image.channels = 4;
    image.data = {
        static_cast<unsigned char>(rgb[0]),
        static_cast<unsigned char>(rgb[1]),
        static_cast<unsigned char>(rgb[2]),
        alpha,
    };
    return image;
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

TEST_CASE("Texture atlas decodes encoded PNGs for separate material rows", "[TexturePainting][TextureAtlas]")
{
    TexturedMesh mesh = two_material_faces();
    mesh.textures = {
        encoded_solid_texture(4, 4, { 255, 0, 0 }, ".png"),
        encoded_solid_texture(2, 6, { 0, 0, 255 }, ".png"),
    };
    REQUIRE_FALSE(mesh.textures[0].data.empty());
    REQUIRE_FALSE(mesh.textures[1].data.empty());

    const PaintedMesh output = paint(mesh);
    require_red_blue_by_side(output);
}


TEST_CASE("Texture atlas converts RGBA material textures without using alpha as color", "[TexturePainting][TextureAtlas]")
{
    TexturedMesh mesh = two_material_faces();
    mesh.textures = {
        rgba_texture({ 255, 0, 0 }, 7),
        rgba_texture({ 0, 0, 255 }, 231),
    };

    const PaintedMesh output = paint(mesh);
    require_red_blue_by_side(output);
}

TEST_CASE("Texture atlas wraps negative U against a nonuniform texture", "[TexturePainting][TextureAtlas]")
{
    TexturedMesh mesh = two_material_faces();
    mesh.textures[0] = horizontal_texture(2, { 255, 0, 0 }, { 0, 255, 0 });
    mesh.textures[1] = solid_texture(2, 1, { 0, 0, 255 });
    mesh.uvs.assign(mesh.vertices.size(), { 0.f, 0.f });
    mesh.uvs[0] = { -0.5f, 0.f };
    mesh.uvs[1] = { -0.5f, 0.f };
    mesh.uvs[2] = { -0.5f, 0.f };

    const PaintedMesh output = paint(mesh);
    const RGB *left = color_near_x(output, 1. / 3.);
    REQUIRE(left != nullptr);
    CHECK((*left)[0] > 50);
    CHECK((*left)[1] > 50);
    CHECK((*left)[2] < 80);
}

TEST_CASE("Texture atlas wraps negative V against a nonuniform texture", "[TexturePainting][TextureAtlas]")
{
    TexturedMesh mesh = two_material_faces();
    mesh.textures[0] = vertical_texture(1, 2, { 255, 0, 0 }, { 0, 255, 0 });
    mesh.textures[1] = solid_texture(1, 2, { 0, 0, 255 });
    mesh.uvs.assign(mesh.vertices.size(), { 0.f, 0.f });
    mesh.uvs[0] = { 0.f, -0.5f };
    mesh.uvs[1] = { 0.f, -0.5f };
    mesh.uvs[2] = { 0.f, -0.5f };

    const PaintedMesh output = paint(mesh);
    const RGB *left = color_near_x(output, 1. / 3.);
    REQUIRE(left != nullptr);
    CHECK((*left)[0] > 50);
    CHECK((*left)[1] > 50);
    CHECK((*left)[2] < 80);
}

TEST_CASE("Texture atlas honors distinct per-face UV values", "[TexturePainting][TextureAtlas]")
{
    TexturedMesh mesh = two_material_faces();
    mesh.textures[0] = horizontal_texture(2, { 255, 0, 0 }, { 0, 255, 0 });
    mesh.textures[1] = solid_texture(2, 1, { 0, 0, 255 });
    mesh.uvs.assign(mesh.vertices.size(), { 0.f, 0.f });
    mesh.uv_coords = {
        { 0.25f, 0.f }, { 0.75f, 0.f }, { 0.75f, 0.f },
        { 0.f, 0.f }, { 0.f, 0.f }, { 0.f, 0.f },
    };
    mesh.uv_indices = { { 0, 1, 2 }, { 3, 4, 5 } };

    const PaintedMesh output = paint(mesh);
    const RGB *left = color_near_x(output, 1. / 3.);
    REQUIRE(left != nullptr);
    CHECK((*left)[0] > 50);
    CHECK((*left)[1] > 50);
    CHECK((*left)[2] < 80);
}

TEST_CASE("Texture atlas preserves encoded PNG mapping after texture permutation", "[TexturePainting][TextureAtlas]")
{
    TexturedMesh first = two_material_faces();
    first.textures = {
        encoded_solid_texture(3, 2, { 255, 0, 0 }, ".png"),
        encoded_solid_texture(2, 5, { 0, 0, 255 }, ".png"),
    };
    TexturedMesh permuted = first;
    std::swap(permuted.textures[0], permuted.textures[1]);
    permuted.material_texture_map = { 1, 0 };

    const PaintedMesh a = paint(first);
    const PaintedMesh b = paint(permuted);
    REQUIRE(a.face_colors.size() == b.face_colors.size());
    for (size_t i = 0; i < a.face_colors.size(); ++i)
        CHECK(a.face_colors[i] == b.face_colors[i]);
}

TEST_CASE("Texture atlas keeps encoded PNG cancellation before conversion", "[TexturePainting][TextureAtlas]")
{
    TexturedMesh mesh = two_material_faces();
    mesh.textures = {
        encoded_solid_texture(4, 4, { 255, 0, 0 }, ".png"),
        encoded_solid_texture(4, 4, { 0, 0, 255 }, ".png"),
    };
    bool cancelled = false;
    PaintedMesh output;
    CHECK_FALSE(texture_to_painting(mesh, output, atlas_settings(), nullptr, [&]() {
        cancelled = true;
        return true;
    }));
    CHECK(cancelled);
}

TEST_CASE("Texture atlas gives base colors an isolated color swatch", "[TexturePainting][TextureAtlas]")
{
    TexturedMesh mesh = two_material_faces();
    mesh.material_texture_map = { -1, 0 };
    mesh.material_colors = {
        { 0.8f, 0.2f, 0.1f, 1.f },
        { 1.f, 0.f, 0.f, 1.f },
    };
    mesh.textures = {
        encoded_solid_texture(2, 4, { 0, 0, 255 }, ".png"),
        encoded_solid_texture(3, 2, { 0, 0, 0 }, ".png"),
    };
    const PaintedMesh output = paint(mesh);
    bool base_color_seen = false;
    bool blue = false;
    for (size_t i = 0; i < output.face_colors.size(); ++i) {
        const RGB &color = output.face_colors[i];
        if (face_center_x(output, i) < 1.5) {
            CHECK(color[0] > 150);
            CHECK(color[1] > 20);
            CHECK(color[1] < 80);
            CHECK(color[2] < 60);
            base_color_seen = true;
        } else if (face_center_x(output, i) > 2.5) {
            CHECK(color[0] < 60);
            CHECK(color[1] < 60);
            CHECK(color[2] > 150);
            blue = true;
        }
    }
    REQUIRE(base_color_seen);
    REQUIRE(blue);
}

TEST_CASE("Texture atlas uses the first decoded row for a material-less encoded-image face", "[TexturePainting][TextureAtlas]")
{
    TexturedMesh mesh = two_material_faces();
    mesh.material_ids = { -1, 1 };
    mesh.material_texture_map = { 0, 1 };
    mesh.textures = {
        encoded_solid_texture(2, 3, { 255, 0, 0 }, ".png"),
        encoded_solid_texture(2, 5, { 0, 0, 255 }, ".png"),
    };
    const PaintedMesh output = paint(mesh);
    bool first_face_red = false;
    for (size_t i = 0; i < output.face_colors.size(); ++i) {
        if (face_center_x(output, i) < 1.5) {
            CHECK(output.face_colors[i][0] > 180);
            CHECK(output.face_colors[i][1] < 40);
            CHECK(output.face_colors[i][2] < 40);
            first_face_red = true;
        }
    }
    REQUIRE(first_face_red);
}

TEST_CASE("Texture atlas keeps each mapped texture attached to its face", "[TexturePainting][TextureAtlas]")
{
    const PaintedMesh output = paint(two_material_faces());
    require_red_blue_by_side(output);
}

TEST_CASE("Texture atlas preserves input surface area while subdividing", "[TexturePainting][TextureAtlas]")
{
    const PaintedMesh output = paint(two_material_faces());
    CHECK_THAT(painted_area(output), Catch::Matchers::WithinAbs(1., 1e-5));
}

TEST_CASE("Texture atlas remaps the vertical coordinate for unequal texture heights", "[TexturePainting][TextureAtlas]")
{
    TexturedMesh mesh = two_material_faces();
    mesh.textures[0] = vertical_texture(1, 2, { 255, 0, 0 }, { 255, 0, 0 });
    mesh.textures[1] = solid_texture(1, 8, { 0, 0, 0 });
    mesh.uvs.assign(mesh.vertices.size(), { 0.f, 0.5f });
    const PaintedMesh output = paint(mesh);
    const RGB *left = color_near_x(output, 1. / 3.);
    REQUIRE(left != nullptr);
    CHECK((*left)[0] > 100);
    CHECK((*left)[1] < 80);
    CHECK((*left)[2] < 80);
}

TEST_CASE("Texture atlas keeps UVs near a stacked texture edge in that texture", "[TexturePainting][TextureAtlas]")
{
    TexturedMesh mesh = two_material_faces();
    mesh.textures[0] = solid_texture(1, 2, { 255, 0, 0 });
    mesh.textures[1] = solid_texture(1, 2, { 0, 0, 255 });
    mesh.uvs.assign(mesh.vertices.size(), { 0.f, 0.99f });
    const PaintedMesh output = paint(mesh);
    const RGB *left = color_near_x(output, 1. / 3.);
    const RGB *right = color_near_x(output, 10. / 3.);
    REQUIRE(left != nullptr);
    REQUIRE(right != nullptr);
    CHECK((*left)[0] > 180);
    CHECK((*left)[1] < 40);
    CHECK((*left)[2] < 40);
    CHECK((*right)[0] < 40);
    CHECK((*right)[1] < 40);
    CHECK((*right)[2] > 180);
}

TEST_CASE("Texture atlas remaps the horizontal coordinate inside a narrow texture", "[TexturePainting][TextureAtlas]")
{
    TexturedMesh mesh = two_material_faces();
    mesh.textures[0] = horizontal_texture(2, { 255, 0, 0 }, { 0, 255, 0 });
    mesh.textures[1] = solid_texture(8, 1, { 0, 0, 0 });
    mesh.uvs.assign(mesh.vertices.size(), { 0.5f, 0.f });
    const PaintedMesh output = paint(mesh);
    const RGB *left = color_near_x(output, 1. / 3.);
    REQUIRE(left != nullptr);
    CHECK((*left)[0] > 50);
    CHECK((*left)[1] > 50);
    CHECK((*left)[2] < 80);
}

TEST_CASE("Texture atlas wraps tiled UVs on both axes", "[TexturePainting][TextureAtlas]")
{
    TexturedMesh mesh = two_material_faces();
    mesh.textures[0] = solid_texture(2, 2, { 255, 0, 0 });
    mesh.textures[1] = solid_texture(2, 2, { 0, 0, 255 });
    mesh.uvs.assign(mesh.vertices.size(), { -0.25f, 1.25f });
    const PaintedMesh negative = paint(mesh);
    mesh.uvs.assign(mesh.vertices.size(), { 0.75f, 0.25f });
    const PaintedMesh wrapped = paint(mesh);
    REQUIRE(negative.face_colors.size() == wrapped.face_colors.size());
    REQUIRE(negative.indices.size() == wrapped.indices.size());
    for (size_t i = 0; i < negative.face_colors.size(); ++i)
        CHECK(negative.face_colors[i] == wrapped.face_colors[i]);
}

TEST_CASE("Texture atlas uses explicit per-face UVs when vertex UVs are absent", "[TexturePainting][TextureAtlas]")
{
    TexturedMesh mesh = two_material_faces();
    mesh.uvs.clear();
    mesh.uv_coords = {
        { 0.f, 0.f }, { 0.f, 0.f }, { 0.f, 0.f },
        { 0.f, 0.f }, { 0.f, 0.f }, { 0.f, 0.f },
    };
    mesh.uv_indices = { { 0, 1, 2 }, { 3, 4, 5 } };
    const PaintedMesh output = paint(mesh);
    require_red_blue_by_side(output);
}

TEST_CASE("Texture atlas gives an untextured material a dedicated base-color swatch", "[TexturePainting][TextureAtlas]")
{
    TexturedMesh mesh = two_material_faces();
    mesh.material_texture_map = { -1, 0 };
    mesh.material_colors = {
        { 0.f, 1.f, 0.f, 1.f },
        { 1.f, 0.f, 0.f, 1.f },
    };
    mesh.textures = { solid_texture(1, 1, { 0, 0, 255 }), solid_texture(1, 1, { 0, 0, 0 }) };
    const PaintedMesh output = paint(mesh);
    bool green = false;
    bool blue = false;
    for (size_t i = 0; i < output.face_colors.size(); ++i) {
        const RGB &c = output.face_colors[i];
        if (face_center_x(output, i) < 1.5) {
            CHECK(c[0] < 60);
            CHECK(c[1] > 150);
            CHECK(c[2] < 60);
            green = true;
        } else if (face_center_x(output, i) > 2.5) {
            CHECK(c[0] < 60);
            CHECK(c[1] < 60);
            CHECK(c[2] > 150);
            blue = true;
        }
    }
    REQUIRE(green);
    REQUIRE(blue);
}

TEST_CASE("Texture atlas uses the first usable texture for a material-less face", "[TexturePainting][TextureAtlas]")
{
    TexturedMesh mesh = two_material_faces();
    mesh.material_ids = { -1, 1 };
    mesh.material_texture_map = { 0, 1 };
    const PaintedMesh output = paint(mesh);
    bool first_face_red = false;
    for (size_t i = 0; i < output.face_colors.size(); ++i) {
        if (face_center_x(output, i) < 1.5) {
            CHECK(output.face_colors[i][0] > 180);
            CHECK(output.face_colors[i][1] < 40);
            CHECK(output.face_colors[i][2] < 40);
            first_face_red = true;
        }
    }
    REQUIRE(first_face_red);
}

TEST_CASE("Texture atlas falls back to base color for an invalid texture mapping", "[TexturePainting][TextureAtlas]")
{
    TexturedMesh mesh = two_material_faces();
    mesh.material_texture_map = { 7, 1 };
    mesh.material_colors[0] = { 1.f, 1.f, 0.f, 1.f };
    const PaintedMesh output = paint(mesh);
    bool yellow = false;
    for (size_t i = 0; i < output.face_colors.size(); ++i) {
        if (face_center_x(output, i) < 1.5) {
            CHECK(output.face_colors[i][0] > 150);
            CHECK(output.face_colors[i][1] > 150);
            CHECK(output.face_colors[i][2] < 80);
            yellow = true;
        }
    }
    REQUIRE(yellow);
}

TEST_CASE("Texture atlas material order is a metamorphic permutation", "[TexturePainting][TextureAtlas]")
{
    TexturedMesh first = two_material_faces();
    TexturedMesh permuted = first;
    std::swap(permuted.textures[0], permuted.textures[1]);
    permuted.material_texture_map = { 1, 0 };
    const PaintedMesh a = paint(first);
    const PaintedMesh b = paint(permuted);
    REQUIRE(a.face_colors.size() == b.face_colors.size());
    for (size_t i = 0; i < a.face_colors.size(); ++i)
        CHECK(a.face_colors[i] == b.face_colors[i]);
}

TEST_CASE("Texture atlas is deterministic across repeated builds", "[TexturePainting][TextureAtlas]")
{
    const TexturedMesh input = two_material_faces();
    const PaintedMesh first = paint(input);
    const PaintedMesh second = paint(input);
    REQUIRE(first.indices.size() == second.indices.size());
    REQUIRE(first.vertices.size() == second.vertices.size());
    CHECK(first.indices == second.indices);
    CHECK(first.face_colors == second.face_colors);
}

TEST_CASE("Texture atlas rejects empty or undecodable texture inputs", "[TexturePainting][TextureAtlas]")
{
    TexturedMesh empty = two_material_faces();
    empty.textures.clear();
    PaintedMesh output;
    CHECK_FALSE(texture_to_painting(empty, output, atlas_settings()));

    TexturedMesh undecodable = two_material_faces();
    undecodable.textures[0].channels = 2;
    undecodable.textures[1].channels = 2;
    CHECK_FALSE(texture_to_painting(undecodable, output, atlas_settings()));

    TexturedMesh no_faces = two_material_faces();
    no_faces.indices.clear();
    CHECK_FALSE(texture_to_painting(no_faces, output, atlas_settings()));
}

TEST_CASE("Texture atlas forwards cancellation before expensive color conversion", "[TexturePainting][TextureAtlas]")
{
    bool cancelled = false;
    PaintedMesh output;
    CHECK_FALSE(texture_to_painting(two_material_faces(), output, atlas_settings(), nullptr, [&]() {
        cancelled = true;
        return true;
    }));
    CHECK(cancelled);
}

TEST_CASE("Texture atlas rejects raw images whose dimensions exceed their pixel buffers", "[TexturePainting][TextureAtlas]")
{
    for (const int width : {2, std::numeric_limits<int>::max()}) {
        TexturedMesh input = two_material_faces();
        for (TextureImage &image : input.textures) {
            image.width = width;
            image.height = 2;
            image.data.resize(3);
        }
        PaintedMesh output;
        REQUIRE_FALSE(texture_to_painting(input, output, atlas_settings()));
    }
}

} // namespace texture_atlas_contracts
