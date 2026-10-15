#include <cmath>
#include <set>
#include <utility>
#include "libslic3r/Point.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/TextureToColor/TriMesh.hpp"
#include <catch2/catch_all.hpp>

#include <array>
#include <cstddef>
#include <vector>

#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "catch2/catch_test_macros.hpp"
#include "catch2/matchers/catch_matchers.hpp"
#include "libslic3r/TextureToColor/ColorUtils.hpp"

using namespace Slic3r;
using namespace Slic3r::tex2color;
using namespace Slic3r::tex2color::color_utils;

namespace {

using Color = std::array<std::size_t, 3>;

constexpr Color red  {{255, 0, 0}};
constexpr Color green{{0, 255, 0}};
constexpr Color blue {{0, 0, 255}};

bool contains_color(const std::vector<Color> &colors, const Color &needle)
{
    for (const Color &color : colors) {
        if (color == needle)
            return true;
    }
    return false;
}

} // namespace

TEST_CASE("RGB color distances preserve their documented metrics", "[ColorUtils]")
{
    const Color black{{0, 0, 0}};
    const Color white{{255, 255, 255}};
    const ColorDouble half{{0.5, 0.25, 0.125}};
    const ColorDouble other{{0.25, 0.125, 0.0625}};

    CHECK_THAT(calc_rgb_color_difference_by_squared_rgb(black, black), Catch::Matchers::WithinAbs(0.0, 1e-12));
    CHECK_THAT(calc_rgb_color_difference_by_squared_rgb(black, white), Catch::Matchers::WithinAbs(3.0 * 255.0 * 255.0, 1e-12));
    CHECK_THAT(calc_rgb_color_difference_by_squared_rgb(red, green), Catch::Matchers::WithinAbs(2.0 * 255.0 * 255.0, 1e-12));
    CHECK_THAT(calc_rgb_color_difference_by_squared_rgb(black, white),
               Catch::Matchers::WithinAbs(calc_rgb_color_difference_by_squared_rgb(white, black), 1e-12));
    CHECK_THAT(calc_rgb_color_difference_by_squared_rgb_double(half, other),
               Catch::Matchers::WithinAbs(calc_rgb_color_difference_by_squared_rgb_double(other, half), 1e-12));

    const double ciede_black_white = calc_rgb_color_difference_by_ciede2000(black, white);
    const double ciede_black_gray  = calc_rgb_color_difference_by_ciede2000(black, Color{{128, 128, 128}});
    CHECK(ciede_black_white > ciede_black_gray);
    CHECK(ciede_black_gray > 0.0);
    CHECK_THAT(calc_rgb_color_difference_by_ciede2000(black, black), Catch::Matchers::WithinAbs(0.0, 1e-12));

    const ColorDouble red01{{1.0, 0.0, 0.0}};
    const ColorDouble blue01{{0.0, 0.0, 1.0}};
    CHECK_THAT(calc_rgb_color_difference_by_ciede2000(red, blue),
               Catch::Matchers::WithinAbs(calc_rgb_color_difference_by_ciede2000_srgb01(red01, blue01), 1e-12));
}

TEST_CASE("Nearest color mapping uses the closest specified RGB center", "[ColorUtils]")
{
    const std::vector<Color> samples{
        Color{{250, 8, 2}},
        Color{{3, 7, 247}},
        Color{{12, 240, 8}},
        Color{{255, 0, 0}},
    };
    const std::vector<Color> centers{red, blue, green};

    const std::vector<Color> mapped = cluster_to_specified_colors(samples, centers);
    REQUIRE(mapped.size() == samples.size());
    CHECK(mapped[0] == red);
    CHECK(mapped[1] == blue);
    CHECK(mapped[2] == green);
    CHECK(mapped[3] == red);

    std::vector<Color> reversed(samples.rbegin(), samples.rend());
    const std::vector<Color> reversed_mapped = cluster_to_specified_colors(reversed, centers);
    REQUIRE(reversed_mapped.size() == mapped.size());
    for (std::size_t i = 0; i < mapped.size(); ++i)
        CHECK(reversed_mapped[i] == mapped[mapped.size() - i - 1]);
}

TEST_CASE("K-means clustering is deterministic and handles degenerate cardinalities", "[ColorUtils]")
{
    ClusterParameters parameters;
    parameters.color_difference_method = ColorDifferenceMethod::RGB;
    parameters.cluster_k = 2;
    parameters.max_iter = 20;

    const std::vector<Color> ordered{red, blue, red, blue};
    const std::vector<Color> permuted{blue, red, blue, red};
    const std::vector<Color> first = cluster_k_means(ordered, parameters);
    const std::vector<Color> second = cluster_k_means(permuted, parameters);
    REQUIRE(first.size() == parameters.cluster_k);
    CHECK(second == first);
    CHECK(contains_color(first, red));
    CHECK(contains_color(first, blue));

    parameters.cluster_k = 4;
    const std::vector<Color> fewer_unique = cluster_k_means({green, red, green}, parameters);
    REQUIRE(fewer_unique.size() == 2);
    CHECK(fewer_unique[0] == green);
    CHECK(fewer_unique[1] == red);

    parameters.cluster_k = 0;
    CHECK(cluster_k_means({red, blue}, parameters).empty());
    parameters.cluster_k = 2;
    CHECK(cluster_k_means({}, parameters).empty());
}

TEST_CASE("Adaptive clustering returns stable exact results for trivial palettes", "[ColorUtils]")
{
    ClusterParameters parameters;
    parameters.color_difference_method = ColorDifferenceMethod::RGB;
    parameters.max_color_distance = 0.0;
    parameters.max_cluster_k = 8;
    parameters.max_iter = 20;

    const std::vector<Color> same{{42, 84, 126}, {42, 84, 126}, {42, 84, 126}};
    CHECK(cluster_adaptive(same, parameters) == std::vector<Color>{same[0]});

    const std::vector<Color> separated = cluster_adaptive({red, blue}, parameters);
    REQUIRE(separated.size() == 2);
    CHECK(contains_color(separated, red));
    CHECK(contains_color(separated, blue));

    const std::vector<Color> empty = cluster_adaptive({}, parameters);
    CHECK(empty.empty());
}

TEST_CASE("Nearest color ID reports an empty palette and selects exact colors", "[ColorUtils]")
{
    std::size_t id = 91;
    CHECK_FALSE(calc_nearest_color_id({}, red, id));
    CHECK(id == 91);

    const std::vector<Color> centers{red, blue, green};
    REQUIRE(calc_nearest_color_id(centers, blue, id));
    CHECK(id == 1);
    REQUIRE(calc_nearest_color_id(centers, Color{{245, 4, 3}}, id));
    CHECK(id == 0);
}

namespace {

using Color = std::array<std::size_t, 3>;

constexpr Color RED  {{255, 0, 0}};
constexpr Color BLUE {{0, 0, 255}};

TriMesh cube_mesh(double side = 10.)
{
    const TriangleMesh mesh = make_cube(side, side, side);
    return TriMesh(mesh.its);
}

TriMesh two_cubes_mesh()
{
    TriangleMesh mesh = make_cube(10., 10., 10.);
    TriangleMesh second = make_cube(10., 10., 10.);
    second.translate(Vec3f(30.f, 0.f, 0.f));
    mesh.merge(second);
    return TriMesh(mesh.its);
}

TriMesh open_square_mesh()
{
    TriVertices vertices{
        TriVertex(0.f, 0.f, 0.f), TriVertex(10.f, 0.f, 0.f),
        TriVertex(10.f, 10.f, 0.f), TriVertex(0.f, 10.f, 0.f)};
    TriFaces faces{
        TriFace(0, 1, 2), TriFace(0, 2, 3)};
    return TriMesh(std::move(faces), std::move(vertices));
}

std::vector<Vec2f> indexed_uvs(const TriMesh &mesh)
{
    std::vector<Vec2f> uvs;
    uvs.reserve(mesh.vertices.size());
    for (const TriVertex &vertex : mesh.vertices)
        uvs.emplace_back(vertex.x(), vertex.y() + vertex.z());
    return uvs;
}

void require_finite_mesh(const TriMesh &mesh)
{
    for (const TriVertex &vertex : mesh.vertices) {
        CHECK(std::isfinite(vertex.x()));
        CHECK(std::isfinite(vertex.y()));
        CHECK(std::isfinite(vertex.z()));
    }
}

} // namespace

TEST_CASE("Color mesh closure distinguishes watertight and open geometry", "[ColorUtils]")
{
    CHECK(is_closed(cube_mesh()));
    CHECK_FALSE(is_closed(open_square_mesh()));
}

TEST_CASE("Connected color mesh components retain faces, vertices, and UVs", "[ColorUtils]")
{
    const TriMesh input = two_cubes_mesh();
    const std::vector<Vec2f> uvs = indexed_uvs(input);
    std::vector<TriMesh> components;
    std::vector<std::vector<Vec2f>> component_uvs;

    REQUIRE(get_components(input, uvs, components, component_uvs));
    REQUIRE(components.size() == 2);
    REQUIRE(component_uvs.size() == components.size());

    std::size_t total_faces = 0;
    for (std::size_t i = 0; i < components.size(); ++i) {
        CHECK(components[i].indices.size() == 12);
        CHECK(components[i].vertices.size() == 8);
        CHECK(component_uvs[i].size() == components[i].vertices.size());
        CHECK(is_closed(components[i]));
        for (std::size_t vertex_id = 0; vertex_id < components[i].vertices.size(); ++vertex_id) {
            const TriVertex &vertex = components[i].vertices[vertex_id];
            const Vec2f &uv = component_uvs[i][vertex_id];
            CHECK_THAT(uv.x(), Catch::Matchers::WithinAbs(vertex.x(), 1e-6));
            CHECK_THAT(uv.y(), Catch::Matchers::WithinAbs(vertex.y() + vertex.z(), 1e-6));
        }
        total_faces += components[i].indices.size();
    }
    CHECK(total_faces == input.indices.size());

    std::vector<TriMesh> rejected_components;
    std::vector<std::vector<Vec2f>> rejected_uvs;
    std::vector<Vec2f> incomplete_uvs = uvs;
    incomplete_uvs.pop_back();
    CHECK_FALSE(get_components(input, incomplete_uvs, rejected_components, rejected_uvs));
    CHECK(rejected_components.empty());
    CHECK(rejected_uvs.empty());
}

TEST_CASE("Mesh color clustering assigns every face to a supplied center", "[ColorUtils]")
{
    const TriMesh mesh = cube_mesh();
    std::vector<Color> input_face_colors;
    input_face_colors.reserve(mesh.indices.size());
    for (std::size_t i = 0; i < mesh.indices.size(); ++i)
        input_face_colors.push_back(i % 2 == 0 ? RED : BLUE);
    std::vector<Color> face_colors = input_face_colors;

    std::vector<std::size_t> face_ids;
    REQUIRE(mesh_cluster(mesh, {RED, BLUE}, face_colors, face_ids));
    REQUIRE(face_colors.size() == mesh.indices.size());
    REQUIRE(face_ids.size() == mesh.indices.size());
    for (std::size_t i = 0; i < face_colors.size(); ++i) {
        const std::size_t expected_id = input_face_colors[i] == RED ? 0 : 1;
        CHECK(face_ids[i] == expected_id);
        CHECK(face_colors[i] == input_face_colors[i]);
    }

    std::vector<Color> one_center_colors(mesh.indices.size(), Color{{12, 34, 56}});
    std::vector<std::size_t> one_center_ids;
    REQUIRE(mesh_cluster(mesh, {Color{{12, 34, 56}}}, one_center_colors, one_center_ids));
    for (const Color &color : one_center_colors)
        CHECK(color == Color{{12, 34, 56}});

    std::vector<Color> malformed_colors(1, RED);
    CHECK_FALSE(mesh_cluster(mesh, {RED, BLUE}, malformed_colors, face_ids));
    CHECK_FALSE(mesh_cluster(mesh, {}, one_center_colors, face_ids));
}

TEST_CASE("Color region smoothing preserves topology and legal labels", "[ColorUtils]")
{
    TriMesh mesh = cube_mesh();
    const std::size_t face_count = mesh.indices.size();
    std::vector<std::size_t> labels(face_count);
    for (std::size_t i = 0; i < labels.size(); ++i)
        labels[i] = i % 2;
    const std::vector<std::size_t> original_labels = labels;

    SmoothParameters no_smoothing;
    no_smoothing.smooth_weight = 0.;
    REQUIRE(smooth_region(mesh, labels, no_smoothing));
    CHECK(mesh.indices.size() == face_count);
    CHECK(labels.size() == face_count);
    CHECK(labels == original_labels);
    CHECK(is_closed(mesh));
    require_finite_mesh(mesh);
    for (std::size_t label : labels)
        CHECK(label < 2);

    TriMesh smoothed = cube_mesh();
    std::vector<Color> colors(face_count);
    for (std::size_t i = 0; i < colors.size(); ++i)
        colors[i] = i % 2 == 0 ? RED : BLUE;
    SmoothParameters geometry_smoothing;
    geometry_smoothing.smooth_weight = 0.8;
    REQUIRE(smooth_region(smoothed, colors, geometry_smoothing));
    CHECK(smoothed.indices.size() == face_count);
    CHECK(is_closed(smoothed));
    require_finite_mesh(smoothed);
    const std::set<Color> allowed_colors{RED, BLUE};
    for (const Color &color : colors) {
        CHECK(allowed_colors.count(color) == 1);
    }

    TriMesh malformed_mesh = cube_mesh();
    std::vector<std::size_t> wrong_count(face_count - 1, 0);
    CHECK_FALSE(smooth_region(malformed_mesh, wrong_count, geometry_smoothing));
}
