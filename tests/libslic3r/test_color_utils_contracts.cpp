#include "libslic3r/Point.hpp"
#include "libslic3r/TextureToColor/ColorUtils.hpp"
#include "libslic3r/TextureToColor/TriMesh.hpp"
#include "libslic3r/TriangleMesh.hpp"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <set>
#include <utility>
#include <vector>

using namespace Slic3r;
using namespace Slic3r::tex2color;
using namespace Slic3r::tex2color::color_utils;

namespace {

using Color = std::array<std::size_t, 3>;

constexpr Color red{{255, 0, 0}};
constexpr Color green{{0, 255, 0}};
constexpr Color blue{{0, 0, 255}};
constexpr Color black{{0, 0, 0}};
constexpr Color white{{255, 255, 255}};

bool contains_color(const std::vector<Color> &colors, const Color &needle)
{
    return std::find(colors.begin(), colors.end(), needle) != colors.end();
}

std::vector<Color> sorted_colors(std::vector<Color> colors)
{
    std::sort(colors.begin(), colors.end());
    return colors;
}

// Five reds on one axis: r = 0, 10, 20, 200, 210 (g = b = 0). Their centroid is r = 88, so farthest-
// point sampling seeds with the nearest point, r = 20, then takes the farthest from it, r = 210, then
// the farthest from both, r = 0. Converged 2-means splits {0, 10, 20} | {200, 210} into r = 10 and 205.
std::vector<Color> five_reds()
{
    return {Color{{0, 0, 0}}, Color{{10, 0, 0}}, Color{{20, 0, 0}}, Color{{200, 0, 0}}, Color{{210, 0, 0}}};
}

// Independent sRGB grey code -> CIE L* and back (D65, standard sRGB transfer).
double grey_to_lightness(double code)
{
    const double v   = code / 255.0;
    const double lin = v <= 0.04045 ? v / 12.92 : std::pow((v + 0.055) / 1.055, 2.4);
    return lin > 216.0 / 24389.0 ? 116.0 * std::cbrt(lin) - 16.0 : lin * 24389.0 / 27.0;
}

int lightness_to_grey(double lightness)
{
    const double fy  = (lightness + 16.0) / 116.0;
    const double lin = fy * fy * fy > 216.0 / 24389.0 ? fy * fy * fy : lightness * 27.0 / 24389.0;
    const double v   = lin <= 0.0031308 ? 12.92 * lin : 1.055 * std::pow(lin, 1.0 / 2.4) - 0.055;
    return int(std::lround(v * 255.0));
}

} // namespace

TEST_CASE("RGB color distances preserve their documented metrics", "[ColorUtils]")
{
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

TEST_CASE("CIEDE sRGB01 clamps channels and agrees with the byte boundary", "[ColorUtils]")
{
    const ColorDouble byte_boundary{{1., 128. / 255., 0.}};
    const ColorDouble clamped{{1.5, 128. / 255., -0.5}};
    const ColorDouble reference{{0., 0., 1.}};
    const double expected = calc_rgb_color_difference_by_ciede2000_srgb01(byte_boundary, reference);
    const double actual   = calc_rgb_color_difference_by_ciede2000_srgb01(clamped, reference);
    CHECK_THAT(actual, Catch::Matchers::WithinAbs(expected, 1e-12));
    // The same colours as bytes, (255, 128, 0) against (0, 0, 255), give the same distance.
    CHECK_THAT(calc_rgb_color_difference_by_ciede2000(Color{{255, 128, 0}}, blue), Catch::Matchers::WithinAbs(expected, 1e-9));
    CHECK_THAT(calc_rgb_color_difference_by_ciede2000_srgb01(byte_boundary, byte_boundary), Catch::Matchers::WithinAbs(0., 1e-12));
}

TEST_CASE("CIEDE remains symmetric across neutral and hue-wrap colors", "[ColorUtils]")
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

TEST_CASE("Squared color distance obeys translation and scale identities", "[ColorUtils]")
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

TEST_CASE("Specified-color clustering is a nearest-center projection with first-tie stability", "[ColorUtils]")
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

TEST_CASE("K-means Lab mode averages clusters in Lab, unlike RGB mode", "[ColorUtils]")
{
    // Greys 0, 10, 20, 30 and 200 split into {0, 10, 20, 30} and {200} in either mode. Greys have no
    // chroma, so the hue terms of the CIEDE2000 distance play no part. The modes differ in how they
    // average: RGB mode takes the mean code (15), Lab mode the mean L* converted back, which is larger
    // because L* is not linear in the sRGB code near black.
    const std::vector<Color> greys{Color{{0, 0, 0}}, Color{{10, 10, 10}}, Color{{20, 20, 20}}, Color{{30, 30, 30}}, Color{{200, 200, 200}}};
    const double mean_lightness = (grey_to_lightness(0) + grey_to_lightness(10) + grey_to_lightness(20) + grey_to_lightness(30)) / 4.0;
    const int lab_mean = lightness_to_grey(mean_lightness);
    REQUIRE(lab_mean >= 15 + 2);   // far enough from the RGB mean for the +-1 tolerance below

    ClusterParameters parameters;
    parameters.cluster_k = 2;
    parameters.max_iter  = 20;

    parameters.color_difference_method = ColorDifferenceMethod::RGB;
    const std::vector<Color> rgb = sorted_colors(cluster_k_means(greys, parameters));
    REQUIRE(rgb.size() == 2);
    CHECK(rgb[0] == Color{{15, 15, 15}});
    CHECK(rgb[1] == Color{{200, 200, 200}});

    parameters.color_difference_method = ColorDifferenceMethod::Lab;
    const std::vector<Color> lab = sorted_colors(cluster_k_means(greys, parameters));
    REQUIRE(lab.size() == 2);
    for (std::size_t channel = 0; channel < 3; ++channel) {
        CHECK(std::abs(int(lab[0][channel]) - lab_mean) <= 1);
        CHECK(std::abs(int(lab[1][channel]) - 200) <= 1);
    }
}

TEST_CASE("K-means returns the unique colours when k exceeds them", "[ColorUtils]")
{
    ClusterParameters parameters;
    parameters.cluster_k = 3;
    parameters.max_iter  = 20;
    for (const auto method : {ColorDifferenceMethod::RGB, ColorDifferenceMethod::Lab}) {
        parameters.color_difference_method = method;
        CHECK(sorted_colors(cluster_k_means({red, blue, red}, parameters)) == sorted_colors({red, blue}));
    }
}

TEST_CASE("K-means max-iteration zero returns the farthest-point seeds", "[ColorUtils]")
{
    ClusterParameters parameters;
    parameters.color_difference_method = ColorDifferenceMethod::RGB;
    parameters.max_iter                = 0;
    // Two seeds: the point nearest the centroid, then the farthest from it.
    parameters.cluster_k = 2;
    CHECK(sorted_colors(cluster_k_means(five_reds(), parameters)) == std::vector<Color>{Color{{20, 0, 0}}, Color{{210, 0, 0}}});
    // Three seeds add the point farthest from both.
    parameters.cluster_k = 3;
    CHECK(sorted_colors(cluster_k_means(five_reds(), parameters)) == std::vector<Color>{Color{{0, 0, 0}}, Color{{20, 0, 0}}, Color{{210, 0, 0}}});
}

TEST_CASE("Cancelled k-means returns its seeds instead of the converged centres", "[ColorUtils]")
{
    ClusterParameters parameters;
    parameters.color_difference_method = ColorDifferenceMethod::RGB;
    parameters.cluster_k               = 2;
    parameters.max_iter                = 20;
    const std::vector<Color> converged = cluster_k_means(five_reds(), parameters);
    CHECK(sorted_colors(converged) == std::vector<Color>{Color{{10, 0, 0}}, Color{{205, 0, 0}}});

    std::size_t cancel_queries = 0;
    parameters.cancel_callback = [&cancel_queries] {
        ++cancel_queries;
        return true;
    };
    const std::vector<Color> cancelled = cluster_k_means(five_reds(), parameters);
    CHECK(cancel_queries > 0);
    CHECK(sorted_colors(cancelled) == std::vector<Color>{Color{{20, 0, 0}}, Color{{210, 0, 0}}});
}

TEST_CASE("Adaptive clustering changes cardinality at a strict distance bound", "[ColorUtils]")
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
    // The single centre is the farthest-point seed nearest the centroid of red and blue.
    CHECK(fallback[0] == blue);
    // k = 1 already meets the 1.0 bound. The result is black rather than the (0.5, 0, 0) mean because a
    // single centre is the farthest-point seed, the point nearest the centroid (black wins the tie).
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

TEST_CASE("Adaptive clustering cancellation is fail-closed during binary search", "[ColorUtils]")
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

TEST_CASE("Nearest-color IDs preserve first occurrence on an exact tie", "[ColorUtils]")
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

TEST_CASE("CIEDE2000 public RGB APIs match the independent reference fixtures",
          "[ColorUtils]")
{
    // Expected values follow Sharma, Wu, and Dalal (2005),
    // https://hajim.rochester.edu/ece/sites/gsharma/ciede2000/ciede2000noteCRNA.pdf
    // (Eqs. 2-22, including independent signed delta-h and mean-hue branches,
    // Cbar^7, and the 63-degree final T-term angle). The published Table I
    // pairs 1-34 reproduce within 5e-5 under the independent reference.
    const double green_magenta_expected = 111.41433059755204;
    const Color magenta{{255, 0, 255}};
    CHECK_THAT(calc_rgb_color_difference_by_ciede2000(green, magenta),
               Catch::Matchers::WithinAbs(green_magenta_expected, 1e-6));
    CHECK_THAT(calc_rgb_color_difference_by_ciede2000(magenta, green),
               Catch::Matchers::WithinAbs(green_magenta_expected, 1e-6));

    const ColorDouble green01{{0.0, 1.0, 0.0}};
    const ColorDouble magenta01{{1.0, 0.0, 1.0}};
    CHECK_THAT(calc_rgb_color_difference_by_ciede2000_srgb01(green01, magenta01),
               Catch::Matchers::WithinAbs(green_magenta_expected, 1e-6));
    CHECK_THAT(calc_rgb_color_difference_by_ciede2000_srgb01(magenta01, green01),
               Catch::Matchers::WithinAbs(green_magenta_expected, 1e-6));

    // Black/white is a symmetric magnitude control independent of the
    // green/magenta hue-sector discriminator.
    const double black_white_expected = 100.00000386666865;
    CHECK_THAT(calc_rgb_color_difference_by_ciede2000(black, white),
               Catch::Matchers::WithinAbs(black_white_expected, 1e-6));
    CHECK_THAT(calc_rgb_color_difference_by_ciede2000(white, black),
               Catch::Matchers::WithinAbs(black_white_expected, 1e-6));

    // This explicit pair exercises Eq. 14's second wrap branch: the adjusted
    // hue separation exceeds 180 degrees and the adjusted hue sum is above
    // 360 degrees, so the mean must subtract 360 degrees. It is independent
    // of the green/magenta T-term discriminator and catches an unconditional
    // wrap addition even after the published 63-degree T term is restored.
    const double mean_hue_wrap_expected = 45.73921346241746;
    const Color mean_hue_wrap_a{{96, 32, 255}};
    const Color mean_hue_wrap_b{{160, 96, 64}};
    CHECK_THAT(calc_rgb_color_difference_by_ciede2000(mean_hue_wrap_a, mean_hue_wrap_b),
               Catch::Matchers::WithinAbs(mean_hue_wrap_expected, 1e-6));
    CHECK_THAT(calc_rgb_color_difference_by_ciede2000(mean_hue_wrap_b, mean_hue_wrap_a),
               Catch::Matchers::WithinAbs(mean_hue_wrap_expected, 1e-6));
}

namespace {

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

// Re-split the first cube's top square along its other diagonal, so the fixture no longer matches
// the default triangulation and a component that re-triangulated its faces would differ.
void flip_first_top_diagonal(TriMesh &mesh)
{
    std::vector<std::size_t> top;
    for (std::size_t i = 0; i < mesh.indices.size() && top.size() < 2; ++i) {
        const TriFace &face = mesh.indices[i];
        bool on_top = true;
        for (int corner = 0; corner < 3; ++corner)
            on_top = on_top && mesh.vertices[face[corner]].z() == 10.f && mesh.vertices[face[corner]].x() <= 10.f;
        if (on_top)
            top.push_back(i);
    }
    REQUIRE(top.size() == 2);
    const TriFace a = mesh.indices[top[0]];
    const TriFace b = mesh.indices[top[1]];
    const auto in = [](const TriFace &face, int v) { return face[0] == v || face[1] == v || face[2] == v; };
    // Rotate a so it starts with its vertex that b lacks: a = (u1, x, y) with x-y the shared diagonal.
    int r = 0;
    while (in(b, a[r]))
        ++r;
    const int u1 = a[r], x = a[(r + 1) % 3], y = a[(r + 2) % 3];
    int u2 = -1;
    for (int corner = 0; corner < 3; ++corner)
        if (!in(a, b[corner]))
            u2 = b[corner];
    REQUIRE(u2 >= 0);
    mesh.indices[top[0]] = TriFace(u1, x, u2);
    mesh.indices[top[1]] = TriFace(u1, u2, y);
}

// A face as its three corner positions, order-insensitive.
using FaceKey = std::array<std::array<float, 3>, 3>;

FaceKey face_key(const TriMesh &mesh, const TriFace &face)
{
    FaceKey key;
    for (int corner = 0; corner < 3; ++corner) {
        const TriVertex &v = mesh.vertices[face[corner]];
        key[std::size_t(corner)] = {v.x(), v.y(), v.z()};
    }
    std::sort(key.begin(), key.end());
    return key;
}

std::multiset<FaceKey> face_keys(const TriMesh &mesh)
{
    std::multiset<FaceKey> keys;
    for (const TriFace &face : mesh.indices)
        keys.insert(face_key(mesh, face));
    return keys;
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

// UV = (x + 0.1 y, y + 0.1 z). Injective on the cubes' corner grid, so every vertex has its own UV and
// a vertex given another vertex's UV is caught.
Vec2f uv_of(const TriVertex &vertex)
{
    return Vec2f(vertex.x() + 0.1f * vertex.y(), vertex.y() + 0.1f * vertex.z());
}

std::vector<Vec2f> indexed_uvs(const TriMesh &mesh)
{
    std::vector<Vec2f> uvs;
    uvs.reserve(mesh.vertices.size());
    for (const TriVertex &vertex : mesh.vertices)
        uvs.push_back(uv_of(vertex));
    return uvs;
}

} // namespace

TEST_CASE("Color mesh closure distinguishes watertight and open geometry", "[ColorUtils]")
{
    CHECK(is_closed(cube_mesh()));
    CHECK_FALSE(is_closed(open_square_mesh()));
}

TEST_CASE("Connected color mesh components retain faces, vertices, and UVs", "[ColorUtils]")
{
    TriMesh input = two_cubes_mesh();
    flip_first_top_diagonal(input);
    REQUIRE(face_keys(input) != face_keys(two_cubes_mesh()));   // the fixture really differs from the default split
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
            CHECK_THAT(uv.x(), Catch::Matchers::WithinAbs(uv_of(vertex).x(), 1e-5));
            CHECK_THAT(uv.y(), Catch::Matchers::WithinAbs(uv_of(vertex).y(), 1e-5));
        }
        total_faces += components[i].indices.size();
    }
    CHECK(total_faces == input.indices.size());
    // The components hold exactly the input triangles, compared by corner positions, so a component
    // that re-split a square along the other diagonal would not match.
    std::multiset<FaceKey> component_faces;
    for (const TriMesh &component : components)
        for (const TriFace &face : component.indices)
            component_faces.insert(face_key(component, face));
    CHECK(component_faces == face_keys(input));

    // One component is the cube at x = 0..10, the other the one moved to x = 30..40.
    std::vector<float> min_x;
    for (const TriMesh &component : components) {
        float lo = component.vertices.front().x(), hi = lo;
        for (const TriVertex &vertex : component.vertices) {
            lo = std::min(lo, vertex.x());
            hi = std::max(hi, vertex.x());
        }
        CHECK_THAT(hi - lo, Catch::Matchers::WithinAbs(10.f, 1e-5f));
        min_x.push_back(lo);
    }
    std::sort(min_x.begin(), min_x.end());
    CHECK_THAT(min_x[0], Catch::Matchers::WithinAbs(0.f, 1e-5f));
    CHECK_THAT(min_x[1], Catch::Matchers::WithinAbs(30.f, 1e-5f));
}

TEST_CASE("Component extraction clears stale outputs on a UV cardinality error", "[ColorUtils]")
{
    const TriMesh input = cube_mesh();
    std::vector<TriMesh> components{cube_mesh()};
    std::vector<std::vector<Vec2f>> component_uvs{{Vec2f(1.f, 2.f)}};
    std::vector<Vec2f> malformed(input.vertices.size() - 1, Vec2f(0.f, 0.f));
    CHECK_FALSE(get_components(input, malformed, components, component_uvs));
    CHECK(components.empty());
    CHECK(component_uvs.empty());
}

