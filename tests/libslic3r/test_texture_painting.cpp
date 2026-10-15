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
