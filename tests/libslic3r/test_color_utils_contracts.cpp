#include <algorithm>
#include <limits>
#include <numeric>
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

namespace extended_color_contracts {
namespace {

using Color = std::array<std::size_t, 3>;
constexpr Color red{{255, 0, 0}};
constexpr Color green{{0, 255, 0}};
constexpr Color blue{{0, 0, 255}};
constexpr Color black{{0, 0, 0}};
constexpr Color white{{255, 255, 255}};

TriMesh cube_mesh(double side = 10.)
{
    const TriangleMesh cube = make_cube(side, side, side);
    return TriMesh(cube.its);
}

TriMesh two_cubes_mesh()
{
    TriangleMesh first  = make_cube(10., 10., 10.);
    TriangleMesh second = make_cube(10., 10., 10.);
    second.translate(Vec3f(20.f, 0.f, 0.f));
    first.merge(second);
    return TriMesh(first.its);
}

TriMesh open_square_mesh()
{
    return TriMesh({stl_triangle_vertex_indices(0, 1, 2), stl_triangle_vertex_indices(0, 2, 3)},
                   {stl_vertex(0.f, 0.f, 0.f), stl_vertex(10.f, 0.f, 0.f), stl_vertex(10.f, 10.f, 0.f), stl_vertex(0.f, 10.f, 0.f)});
}

double signed_volume(const TriMesh& mesh)
{
    double sum = 0.;
    for (const auto& face : mesh.indices) {
        const auto& a = mesh.vertices[face[0]];
        const auto& b = mesh.vertices[face[1]];
        const auto& c = mesh.vertices[face[2]];
        sum += static_cast<double>(a.x()) * (static_cast<double>(b.y()) * c.z() - static_cast<double>(b.z()) * c.y());
        sum += static_cast<double>(a.y()) * (static_cast<double>(b.z()) * c.x() - static_cast<double>(b.x()) * c.z());
        sum += static_cast<double>(a.z()) * (static_cast<double>(b.x()) * c.y() - static_cast<double>(b.y()) * c.x());
    }
    return sum / 6.;
}

void require_finite_mesh(const TriMesh& mesh)
{
    for (const auto& v : mesh.vertices) {
        CHECK(std::isfinite(v.x()));
        CHECK(std::isfinite(v.y()));
        CHECK(std::isfinite(v.z()));
    }
}

std::vector<Color> sorted_colors(std::vector<Color> colors)
{
    std::sort(colors.begin(), colors.end());
    return colors;
}

bool contains_color(const std::vector<Color>& colors, const Color& needle)
{
    return std::find(colors.begin(), colors.end(), needle) != colors.end();
}

} // namespace

TEST_CASE("CIEDE sRGB01 clamps channels and agrees with the byte boundary", "[ColorUtils][ColorUtilsExtended]")
{
    const ColorDouble byte_boundary{{1., 128. / 255., 0.}};
    const ColorDouble clamped{{1.5, 128. / 255., -0.5}};
    const ColorDouble reference{{0., 0., 1.}};
    const double expected = calc_rgb_color_difference_by_ciede2000_srgb01(byte_boundary, reference);
    const double actual   = calc_rgb_color_difference_by_ciede2000_srgb01(clamped, reference);
    CHECK_THAT(actual, Catch::Matchers::WithinAbs(expected, 1e-12));
    CHECK_THAT(calc_rgb_color_difference_by_ciede2000_srgb01(byte_boundary, byte_boundary), Catch::Matchers::WithinAbs(0., 1e-12));
}

TEST_CASE("CIEDE remains symmetric across neutral and hue-wrap colors", "[ColorUtils][ColorUtilsExtended]")
{
    const std::vector<std::pair<Color, Color>> pairs{
        {Color{{1, 2, 3}}, Color{{3, 2, 1}}},
        {Color{{255, 1, 0}}, Color{{255, 0, 1}}},
        {Color{{8, 8, 8}}, Color{{9, 9, 9}}},
        {Color{{250, 220, 12}}, Color{{12, 220, 250}}},
    };
    for (const auto& [a, b] : pairs) {
        const double ab = calc_rgb_color_difference_by_ciede2000(a, b);
        const double ba = calc_rgb_color_difference_by_ciede2000(b, a);
        CHECK(std::isfinite(ab));
        CHECK(ab >= 0.);
        CHECK_THAT(ab, Catch::Matchers::WithinAbs(ba, 1e-12));
    }
}

TEST_CASE("Squared color distance obeys translation and scale identities", "[ColorUtils][ColorUtilsExtended]")
{
    const ColorDouble a{{2., 4., 8.}};
    const ColorDouble b{{1., 2., 6.}};
    const ColorDouble a_shift{{9., 15., 21.}};
    const ColorDouble b_shift{{8., 13., 19.}};
    CHECK_THAT(calc_rgb_color_difference_by_squared_rgb_double(a, b), Catch::Matchers::WithinAbs(9., 1e-12));
    CHECK_THAT(calc_rgb_color_difference_by_squared_rgb_double(a_shift, b_shift),
               Catch::Matchers::WithinAbs(calc_rgb_color_difference_by_squared_rgb_double(a, b), 1e-12));
    const ColorDouble a_scaled{{a[0] * 3., a[1] * 3., a[2] * 3.}};
    const ColorDouble b_scaled{{b[0] * 3., b[1] * 3., b[2] * 3.}};
    CHECK_THAT(calc_rgb_color_difference_by_squared_rgb_double(a_scaled, b_scaled),
               Catch::Matchers::WithinAbs(9. * calc_rgb_color_difference_by_squared_rgb_double(a, b), 1e-12));
}

TEST_CASE("Specified-color clustering is a nearest-center projection with first-tie stability", "[ColorUtils][ColorUtilsExtended]")
{
    const std::vector<Color> centers{black, white};
    const std::vector<Color> input{Color{{0, 0, 1}}, Color{{254, 254, 253}}, Color{{128, 128, 128}}, Color{{127, 127, 127}}};
    const std::vector<Color> projected = cluster_to_specified_colors(input, centers);
    REQUIRE(projected.size() == input.size());
    CHECK(projected[0] == black);
    CHECK(projected[1] == white);
    CHECK(projected[2] == white);
    CHECK(projected[3] == black);
    const std::vector<Color> duplicate_centers{red, red, blue};
    const std::vector<Color> duplicate_projection = cluster_to_specified_colors({red, blue}, duplicate_centers);
    REQUIRE(duplicate_projection.size() == 2);
    CHECK(duplicate_projection[0] == red);
    CHECK(duplicate_projection[1] == blue);
    const Color near_black{{2, 0, 0}};
    const std::vector<Color> tie_input{Color{{1, 0, 0}}};
    CHECK(cluster_to_specified_colors(tie_input, {black, near_black}) == std::vector<Color>{black});
    CHECK(cluster_to_specified_colors(tie_input, {near_black, black}) == std::vector<Color>{near_black});
}

TEST_CASE("K-means Lab mode returns bounded centers and exposes cancellation as a partial result", "[ColorUtils][ColorUtilsExtended]")
{
    ClusterParameters parameters;
    parameters.color_difference_method = ColorDifferenceMethod::Lab;
    parameters.cluster_k               = 2;
    parameters.max_iter                = 20;
    const std::vector<Color> input{red, red, blue, blue, green};
    const std::vector<Color> centers = cluster_k_means(input, parameters);
    REQUIRE(centers.size() == 2);
    for (const Color& center : centers) {
        CHECK(center[0] <= 255);
        CHECK(center[1] <= 255);
        CHECK(center[2] <= 255);
    }
    parameters.cluster_k                  = 3;
    const std::vector<Color> fewer_unique = cluster_k_means({red, blue, red}, parameters);
    REQUIRE(fewer_unique.size() == 2);
    CHECK(contains_color(fewer_unique, red));
    CHECK(contains_color(fewer_unique, blue));
    parameters.cluster_k               = 2;
    parameters.cancel_callback         = [] { return true; };
    const std::vector<Color> cancelled = cluster_k_means(input, parameters);
    REQUIRE(cancelled.size() == parameters.cluster_k);
    for (const Color& center : cancelled) {
        CHECK(center[0] <= 255);
        CHECK(center[1] <= 255);
        CHECK(center[2] <= 255);
    }
}

TEST_CASE("K-means max-iteration zero still returns FPS input representatives", "[ColorUtils][ColorUtilsExtended]")
{
    ClusterParameters parameters;
    parameters.color_difference_method = ColorDifferenceMethod::RGB;
    parameters.cluster_k               = 3;
    parameters.max_iter                = 0;
    const std::vector<Color> input{Color{{0, 10, 20}}, Color{{90, 100, 110}}, Color{{240, 230, 220}}};
    const std::vector<Color> centers = cluster_k_means(input, parameters);
    REQUIRE(centers.size() == parameters.cluster_k);
    CHECK(sorted_colors(centers) == sorted_colors(input));
}

TEST_CASE("Adaptive clustering changes cardinality at a strict distance bound", "[ColorUtils][ColorUtilsExtended]")
{
    ClusterParameters parameters;
    parameters.color_difference_method = ColorDifferenceMethod::RGB;
    parameters.max_iter                = 20;
    parameters.max_cluster_k           = 8;
    parameters.max_color_distance      = 0.;
    const std::vector<Color> exact     = cluster_adaptive({red, blue, green}, parameters);
    REQUIRE(exact.size() == 3);
    CHECK(contains_color(exact, red));
    CHECK(contains_color(exact, green));
    CHECK(contains_color(exact, blue));
    parameters.max_color_distance  = 1e9;
    const std::vector<Color> broad = cluster_adaptive({red, blue, green}, parameters);
    REQUIRE(broad.size() == 1);
    parameters.max_cluster_k          = 1;
    parameters.max_color_distance     = 0.;
    const std::vector<Color> fallback = cluster_adaptive({red, blue}, parameters);
    REQUIRE(fallback.size() == 1);
    CHECK(fallback[0][0] <= 255);
    parameters.max_cluster_k          = 2;
    parameters.max_color_distance     = 1.;
    const std::vector<Color> boundary = cluster_adaptive({black, Color{{1, 0, 0}}}, parameters);
    REQUIRE(boundary.size() == 1);
    CHECK(boundary[0] == black);
    parameters.max_cluster_k           = 8;
    const std::vector<Color> singleton = cluster_adaptive({green}, parameters);
    REQUIRE(singleton.size() == 1);
    CHECK(singleton[0] == green);
    CHECK(cluster_adaptive({}, parameters).empty());
}

TEST_CASE("Adaptive clustering cancellation is fail-closed during binary search", "[ColorUtils][ColorUtilsExtended]")
{
    ClusterParameters parameters;
    parameters.color_difference_method = ColorDifferenceMethod::Lab;
    parameters.max_cluster_k           = 8;
    parameters.max_iter                = 30;
    std::size_t callbacks              = 0;
    parameters.cancel_callback         = [&callbacks] {
        ++callbacks;
        return callbacks >= 2;
    };
    const std::vector<Color> result = cluster_adaptive({red, green, blue, white}, parameters);
    CHECK(result.empty());
    CHECK(callbacks >= 2);
}

TEST_CASE("Closed meshes and disconnected components retain geometric and UV invariants", "[ColorUtils][ColorUtilsExtended]")
{
    const TriMesh input = two_cubes_mesh();
    std::vector<Vec2f> uvs;
    for (const auto& v : input.vertices)
        uvs.emplace_back(v.x() * 0.1f, v.y() * 0.2f + v.z() * 0.01f);
    std::vector<TriMesh> components;
    std::vector<std::vector<Vec2f>> component_uvs;
    REQUIRE(get_components(input, uvs, components, component_uvs));
    REQUIRE(component_uvs.size() == components.size());
    REQUIRE(components.size() == 2);
    std::size_t faces = 0;
    for (std::size_t i = 0; i < components.size(); ++i) {
        REQUIRE(component_uvs[i].size() == components[i].vertices.size());
        CHECK(is_closed(components[i]));
        CHECK(its_num_open_edges(components[i]) == 0);
        faces += components[i].indices.size();
        for (std::size_t j = 0; j < components[i].vertices.size(); ++j) {
            const auto& v = components[i].vertices[j];
            CHECK_THAT(component_uvs[i][j].x(), Catch::Matchers::WithinAbs(v.x() * 0.1, 1e-5));
            CHECK_THAT(component_uvs[i][j].y(), Catch::Matchers::WithinAbs(v.y() * 0.2 + v.z() * 0.01, 1e-5));
        }
    }
    CHECK(faces == input.indices.size());
}

TEST_CASE("Component extraction clears stale outputs on a UV cardinality error", "[ColorUtils][ColorUtilsExtended]")
{
    const TriMesh input = cube_mesh();
    std::vector<TriMesh> components{cube_mesh()};
    std::vector<std::vector<Vec2f>> component_uvs{{Vec2f(1.f, 2.f)}};
    std::vector<Vec2f> malformed(input.vertices.size() - 1, Vec2f(0.f, 0.f));
    CHECK_FALSE(get_components(input, malformed, components, component_uvs));
    CHECK(components.empty());
    CHECK(component_uvs.empty());
}

TEST_CASE("Mesh clustering maps one center without inspecting stale face IDs", "[ColorUtils][ColorUtilsExtended]")
{
    const TriMesh mesh = cube_mesh();
    std::vector<Color> colors(mesh.indices.size(), Color{{12, 34, 56}});
    std::vector<std::size_t> ids{99, 98};
    REQUIRE(mesh_cluster(mesh, {Color{{9, 8, 7}}}, colors, ids));
    REQUIRE(colors.size() == mesh.indices.size());
    for (const Color& color : colors)
        CHECK(color == Color{{9, 8, 7}});
    CHECK(ids.size() == 2);

    std::vector<Color> two_center_colors(mesh.indices.size());
    for (std::size_t i = 0; i < two_center_colors.size(); ++i)
        two_center_colors[i] = i % 2 == 0 ? red : blue;
    std::vector<std::size_t> two_center_ids;
    REQUIRE(mesh_cluster(mesh, {red, blue}, two_center_colors, two_center_ids));
    REQUIRE(two_center_ids.size() == mesh.indices.size());
    for (std::size_t i = 0; i < two_center_ids.size(); ++i) {
        CHECK(two_center_ids[i] == (i % 2 == 0 ? 0 : 1));
        CHECK(two_center_colors[i] == (i % 2 == 0 ? red : blue));
    }

    // This face is perceptually close to both centers, but still nearest to
    // the second one.  It shares an edge with a definitely seeded first face;
    // the first-stage "or" criterion must retain that nearest assignment
    // instead of deferring it to adjacency fallback.
    const TriMesh adjacent = open_square_mesh();
    const Color near_first{{49, 245, 71}};
    const Color near_second{{24, 231, 52}};
    const Color ambiguous{{17, 236, 2}};
    std::vector<Color> ambiguous_colors{near_first, ambiguous};
    std::vector<std::size_t> ambiguous_ids;
    REQUIRE(mesh_cluster(adjacent, {near_first, near_second}, ambiguous_colors, ambiguous_ids));
    REQUIRE(ambiguous_ids.size() == 2);
    CHECK(ambiguous_ids[0] == 0);
    CHECK(ambiguous_ids[1] == 1);
    CHECK(ambiguous_colors[0] == near_first);
    CHECK(ambiguous_colors[1] == near_second);
}

TEST_CASE("Mesh clustering rejects empty and mismatched face inputs without mutating colors", "[ColorUtils][ColorUtilsExtended]")
{
    const TriMesh mesh = cube_mesh();
    const std::vector<Color> original(mesh.indices.size(), red);
    std::vector<Color> malformed = original;
    malformed.pop_back();
    std::vector<std::size_t> ids;
    CHECK_FALSE(mesh_cluster(mesh, {red, blue}, malformed, ids));
    CHECK(malformed == std::vector<Color>(original.begin(), original.end() - 1));
    std::vector<Color> empty_centers = original;
    CHECK_FALSE(mesh_cluster(mesh, {}, empty_centers, ids));
    CHECK(empty_centers == original);
}

TEST_CASE("Smoothing at zero and below the topology threshold preserves labels and shape", "[ColorUtils][ColorUtilsExtended]")
{
    const TriMesh original = cube_mesh();
    std::vector<std::size_t> initial(original.indices.size(), 0);
    for (std::size_t i = 0; i < initial.size(); i += 3)
        initial[i] = 1;
    for (const double weight : {0., 0.2, 0.3}) {
        TriMesh mesh                    = original;
        std::vector<std::size_t> labels = initial;
        const double before             = std::abs(signed_volume(mesh));
        SmoothParameters parameters;
        parameters.smooth_weight = weight;
        REQUIRE(smooth_region(mesh, labels, parameters));
        CHECK(labels.size() == mesh.indices.size());
        CHECK_THAT(std::abs(signed_volume(mesh)), Catch::Matchers::WithinRel(before, 1e-8));
        CHECK(is_closed(mesh));
        for (std::size_t label : labels)
            CHECK(label <= 1);
        if (weight == 0.)
            CHECK(labels == initial);
        if (weight == 0.3) {
            const std::size_t initial_ones  = static_cast<std::size_t>(std::count(initial.begin(), initial.end(), 1));
            const std::size_t smoothed_ones = static_cast<std::size_t>(std::count(labels.begin(), labels.end(), 1));
            CHECK(smoothed_ones < initial_ones);
        }
    }
}

TEST_CASE("Color smoothing preserves the supplied RGB palette while changing labels only through it", "[ColorUtils][ColorUtilsExtended]")
{
    TriMesh mesh = cube_mesh();
    std::vector<Color> colors;
    colors.reserve(mesh.indices.size());
    for (std::size_t i = 0; i < mesh.indices.size(); ++i)
        colors.push_back(i % 3 == 0 ? red : (i % 3 == 1 ? green : blue));
    const std::set<Color> palette(colors.begin(), colors.end());
    TriMesh unchanged_mesh                    = mesh;
    const std::vector<Color> unchanged_colors = colors;
    SmoothParameters no_smoothing;
    no_smoothing.smooth_weight = 0.;
    REQUIRE(smooth_region(unchanged_mesh, colors, no_smoothing));
    CHECK(colors == unchanged_colors);

    SmoothParameters parameters;
    parameters.smooth_weight = 0.8;
    REQUIRE(smooth_region(mesh, colors, parameters));
    REQUIRE(colors.size() == mesh.indices.size());
    CHECK(is_closed(mesh));
    for (const Color& color : colors)
        CHECK(palette.count(color) == 1);
    require_finite_mesh(mesh);
}

TEST_CASE("Remeshing rejects malformed labels before changing geometry", "[ColorUtils][ColorUtilsExtended]")
{
    TriMesh mesh           = cube_mesh();
    const TriMesh original = mesh;
    std::vector<std::size_t> labels(mesh.indices.size() - 1, 0);
    CHECK_FALSE(remesh_mesh(mesh, labels, 1.0));
    CHECK(mesh.vertices == original.vertices);
    CHECK(mesh.indices == original.indices);
}

TEST_CASE("Remeshing a closed colored cube preserves volume, closure, and label cardinality", "[ColorUtils][ColorUtilsExtended]")
{
    TriMesh mesh        = cube_mesh(8.);
    const double before = std::abs(signed_volume(mesh));
    std::vector<std::size_t> labels(mesh.indices.size(), 0);
    for (std::size_t i = 0; i < labels.size(); i += 4)
        labels[i] = 1;
    REQUIRE(remesh_mesh(mesh, labels, 1.4));
    CHECK(mesh.indices.size() > 0);
    CHECK(mesh.vertices.size() > 0);
    CHECK(labels.size() == mesh.indices.size());
    CHECK(is_closed(mesh));
    CHECK_THAT(std::abs(signed_volume(mesh)), Catch::Matchers::WithinRel(before, 0.2));
    for (std::size_t label : labels)
        CHECK(label <= 1);
    require_finite_mesh(mesh);
}

TEST_CASE("Remeshing target ratios order the resulting tessellation by edge scale", "[ColorUtils][ColorUtilsExtended]")
{
    TriMesh coarse = cube_mesh(8.);
    TriMesh fine   = cube_mesh(8.);
    std::vector<std::size_t> coarse_labels(coarse.indices.size(), 0);
    std::vector<std::size_t> fine_labels(fine.indices.size(), 0);
    REQUIRE(remesh_mesh(coarse, coarse_labels, 1.4));
    REQUIRE(remesh_mesh(fine, fine_labels, 0.6));
    REQUIRE(coarse.indices.size() > 0);
    REQUIRE(fine.indices.size() > 0);
    CHECK(fine.indices.size() > coarse.indices.size());
    CHECK(is_closed(coarse));
    CHECK(is_closed(fine));
}

TEST_CASE("Nearest-color IDs preserve first occurrence on an exact tie", "[ColorUtils][ColorUtilsExtended]")
{
    std::size_t id = 999;
    const Color equal_a{{20, 20, 20}};
    const Color equal_b{{20, 20, 20}};
    REQUIRE(calc_nearest_color_id({equal_a, equal_b, white}, equal_a, id));
    CHECK(id == 0);
    REQUIRE(calc_nearest_color_id({white}, equal_a, id));
    CHECK(id == 0);
    REQUIRE(calc_nearest_color_id({red, blue, green}, blue, id));
    CHECK(id == 1);
}

} // namespace extended_color_contracts
