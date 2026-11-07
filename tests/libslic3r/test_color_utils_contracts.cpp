#include <algorithm>
#include <limits>
#include <numeric>
#include <catch2/catch_all.hpp>
#include "catch2/catch_test_macros.hpp"
#include "catch2/matchers/catch_matchers.hpp"
#include <catch2/catch_test_macros.hpp>
#include "libslic3r/Point.hpp"
#include "libslic3r/TextureToColor/ColorUtils.hpp"
#include "libslic3r/TextureToColor/TriMesh.hpp"
#include "libslic3r/TriangleMesh.hpp"

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
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

TEST_CASE("CIEDE sRGB01 clamps channels and agrees with the byte boundary", "[ColorUtils][ColorUtilsExtended]")
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

TEST_CASE("CIEDE2000 public RGB APIs match the independent reference fixtures",
          "[ColorUtils][ColorUtilsExtended][ColorUtilsCIEDEReference]")
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
            const auto source_vertex = std::find_if(input.vertices.begin(), input.vertices.end(), [&vertex](const TriVertex &candidate) {
                return candidate[0] == vertex[0] && candidate[1] == vertex[1] && candidate[2] == vertex[2];
            });
            REQUIRE(source_vertex != input.vertices.end());
            const std::size_t source_vertex_id = std::size_t(source_vertex - input.vertices.begin());
            CHECK_THAT(uv.x(), Catch::Matchers::WithinAbs(uvs[source_vertex_id].x(), 1e-5));
            CHECK_THAT(uv.y(), Catch::Matchers::WithinAbs(uvs[source_vertex_id].y(), 1e-5));
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

    std::vector<TriMesh> rejected_components{cube_mesh()};
    std::vector<std::vector<Vec2f>> rejected_uvs{{Vec2f(1.f, 2.f)}};
    std::vector<Vec2f> incomplete_uvs = uvs;
    incomplete_uvs.pop_back();
    CHECK_FALSE(get_components(input, incomplete_uvs, rejected_components, rejected_uvs));
    CHECK(rejected_components.empty());
    CHECK(rejected_uvs.empty());
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


namespace {

TriMesh ambiguous_fan_mesh()
{
    TriVertices vertices{
        TriVertex(0.f, 0.f, 0.f), TriVertex(2.f, 0.f, 0.f), TriVertex(0.f, 2.f, 0.f),
        TriVertex(1.f, -2.f, 0.f), TriVertex(3.f, 1.f, 0.f)};
    TriFaces faces{
        TriFace(0, 1, 2), // ambiguous center: exactly two adjacent seed faces
        TriFace(1, 0, 3), // red seed
        TriFace(2, 1, 4)  // blue seed
    };
    return TriMesh(std::move(faces), std::move(vertices));
}

TriMesh disconnected_open_squares_mesh()
{
    TriVertices vertices{
        TriVertex(0.f, 0.f, 0.f), TriVertex(10.f, 0.f, 0.f), TriVertex(10.f, 10.f, 0.f), TriVertex(0.f, 10.f, 0.f),
        TriVertex(30.f, 0.f, 0.f), TriVertex(40.f, 0.f, 0.f), TriVertex(40.f, 10.f, 0.f), TriVertex(30.f, 10.f, 0.f)};
    TriFaces faces{
        TriFace(0, 1, 2), TriFace(0, 2, 3),
        TriFace(4, 5, 6), TriFace(4, 6, 7)};
    return TriMesh(std::move(faces), std::move(vertices));
}

constexpr Color kRedCenter  {{220, 40, 40}};
constexpr Color kBlueCenter {{40, 40, 220}};
constexpr Color kThirdCenter{{130, 130, 130}};
constexpr Color kPatchA     {{31, 113, 170}};
constexpr Color kPatchB     {{196, 149, 45}};

TriMesh weighted_component_mesh(const bool swap_patch_areas)
{
    // The four faces form a connected strip: red seed -> patch A -> patch B ->
    // blue seed.  The two variants swap the patch areas (2/8 versus 8/2) while
    // retaining the same face adjacency and seed colors.
    TriVertices vertices;
    if (!swap_patch_areas) {
        vertices = {
            TriVertex(0.f, 0.f, 0.f), TriVertex(2.f, 0.f, 0.f), TriVertex(0.f, 2.f, 0.f),
            TriVertex(2.f, 2.f, 0.f), TriVertex(0.f, 10.f, 0.f), TriVertex(4.f, 10.f, 0.f)};
    } else {
        vertices = {
            TriVertex(0.f, 0.f, 0.f), TriVertex(2.f, 0.f, 0.f), TriVertex(0.f, 2.f, 0.f),
            TriVertex(2.f, 8.f, 0.f), TriVertex(0.f, 4.f, 0.f), TriVertex(4.f, 16.f, 0.f)};
    }
    return TriMesh(
        TriFaces{TriFace(0, 1, 2), TriFace(1, 3, 2), TriFace(2, 3, 4), TriFace(3, 5, 4)},
        std::move(vertices));
}

double triangle_area(const TriMesh &mesh, const std::size_t face_id)
{
    const auto &face = mesh.indices[face_id];
    const auto &a = mesh.vertices[face[0]];
    const auto &b = mesh.vertices[face[1]];
    const auto &c = mesh.vertices[face[2]];
    return std::abs(static_cast<double>(b.x() - a.x()) * static_cast<double>(c.y() - a.y())
                  - static_cast<double>(b.y() - a.y()) * static_cast<double>(c.x() - a.x())) * 0.5;
}

} // namespace

TEST_CASE("Mesh color clustering snaps every face to its nearest supplied centre", "[ColorUtils]")
{
    // Alternate faces are slightly off red and off blue (well under 3 dE), so each must snap to its
    // centre rather than keep its own colour.
    const TriMesh mesh = cube_mesh();
    const Color off_red{{250, 4, 6}};
    const Color off_blue{{5, 3, 250}};
    std::vector<Color> face_colors;
    face_colors.reserve(mesh.indices.size());
    for (std::size_t i = 0; i < mesh.indices.size(); ++i)
        face_colors.push_back(i % 2 == 0 ? off_red : off_blue);

    std::vector<std::size_t> face_ids;
    REQUIRE(mesh_cluster(mesh, {red, blue}, face_colors, face_ids));
    REQUIRE(face_colors.size() == mesh.indices.size());
    REQUIRE(face_ids.size() == mesh.indices.size());
    for (std::size_t i = 0; i < face_colors.size(); ++i) {
        CHECK(face_ids[i] == (i % 2 == 0 ? 0u : 1u));
        CHECK(face_colors[i] == (i % 2 == 0 ? red : blue));
    }
}

TEST_CASE("Mesh clustering with a single centre recolours every face", "[ColorUtils]")
{
    const TriMesh mesh = cube_mesh();
    std::vector<Color> colors(mesh.indices.size(), Color{{12, 34, 56}});
    std::vector<std::size_t> ids;
    REQUIRE(mesh_cluster(mesh, {Color{{9, 8, 7}}}, colors, ids));
    REQUIRE(colors.size() == mesh.indices.size());
    for (const Color &color : colors)
        CHECK(color == Color{{9, 8, 7}});
}

TEST_CASE("Mesh clustering with a single centre reports centre 0 for every face", "[ColorUtils]")
{
    const TriMesh mesh = cube_mesh();
    std::vector<Color> colors(mesh.indices.size(), Color{{12, 34, 56}});
    std::vector<std::size_t> ids;
    REQUIRE(mesh_cluster(mesh, {Color{{9, 8, 7}}}, colors, ids));
    REQUIRE(ids.size() == mesh.indices.size());
    for (const std::size_t id : ids)
        CHECK(id == 0);
}

TEST_CASE("Mesh clustering seeds a face by the distance ratio when it is beyond the absolute threshold", "[ColorUtils]")
{
    // A dull red face next to a blue seed. It is more than 3 dE from red, so only the ratio test
    // (nearest centre at least 1.5 times closer than the second) seeds it red. Without that test it
    // would be left for the flood and inherit its blue neighbour.
    const TriMesh adjacent = open_square_mesh();
    const Color dull_red{{220, 30, 30}};
    const double to_red = calc_rgb_color_difference_by_ciede2000(dull_red, red);
    const double to_blue = calc_rgb_color_difference_by_ciede2000(dull_red, blue);
    REQUIRE(to_red > 3.0);
    REQUIRE(to_red * 1.5 < to_blue);
    std::vector<Color> pair_colors{blue, dull_red};
    std::vector<std::size_t> pair_ids;
    REQUIRE(mesh_cluster(adjacent, {red, blue}, pair_colors, pair_ids));
    REQUIRE(pair_ids.size() == 2);
    CHECK(pair_ids[0] == 1);
    CHECK(pair_ids[1] == 0);
    CHECK(pair_colors[0] == blue);
    CHECK(pair_colors[1] == red);
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

TEST_CASE("Mesh clustering propagates ambiguous faces through connected and disconnected regions",
          "[ColorUtils][ColorUtilsExtended][ColorUtilsFlood]")
{
    const Color gray{{128, 128, 128}};
    const Color blue_leaning{{140, 140, 180}};
    const double gray_to_red = calc_rgb_color_difference_by_ciede2000(gray, red);
    const double gray_to_blue = calc_rgb_color_difference_by_ciede2000(gray, blue);
    const double blue_to_red = calc_rgb_color_difference_by_ciede2000(blue_leaning, red);
    const double blue_to_blue = calc_rgb_color_difference_by_ciede2000(blue_leaning, blue);
    const double gray_min = std::min(gray_to_red, gray_to_blue);
    const double gray_max = std::max(gray_to_red, gray_to_blue);
    const double blue_min = std::min(blue_to_red, blue_to_blue);
    const double blue_max = std::max(blue_to_red, blue_to_blue);
    // These are setup preconditions for the public seed/flood route.  They
    // deliberately fail closed if a future color-math change makes either
    // ambiguous fixture a direct seed or too far from the 1.5 ratio boundary.
    REQUIRE(gray_to_red > 3.0);
    REQUIRE(gray_to_blue > 3.0);
    REQUIRE(gray_min * 1.5 >= gray_max);
    REQUIRE(blue_to_red > 3.0);
    REQUIRE(blue_to_blue > 3.0);
    REQUIRE(blue_min * 1.5 >= blue_max);
    REQUIRE(blue_to_blue < blue_to_red);

    // One valid neighbor is a strict majority. The neutral face is neither a
    // first-pass seed nor a direct color center, so the public flood route must
    // inherit the definitive red neighbor.
    {
        const TriMesh mesh = open_square_mesh();
        std::vector<Color> colors{red, gray};
        std::vector<std::size_t> ids;
        REQUIRE(mesh_cluster(mesh, {red, blue}, colors, ids));
        REQUIRE(ids.size() == mesh.indices.size());
        CHECK(ids[0] == 0);
        CHECK(ids[1] == 0);
        CHECK(colors[0] == red);
        CHECK(colors[1] == red);

        std::vector<Color> blue_colors{blue, gray};
        std::vector<std::size_t> blue_ids;
        REQUIRE(mesh_cluster(mesh, {red, blue}, blue_colors, blue_ids));
        REQUIRE(blue_ids.size() == mesh.indices.size());
        CHECK(blue_ids[0] == 1);
        CHECK(blue_ids[1] == 1);
        CHECK(blue_colors[0] == blue);
        CHECK(blue_colors[1] == blue);
    }

    // The center has one red and one blue adjacent seed, so majority flooding
    // cannot decide it. It ends up blue, its nearest adjacent centre. (Outcome only: both the
    // nearest-adjacent flood rule and the final component pass would pick blue here.)
    {
        const TriMesh mesh = ambiguous_fan_mesh();
        std::vector<Color> colors{blue_leaning, red, blue};
        std::vector<std::size_t> ids;
        REQUIRE(mesh_cluster(mesh, {red, blue}, colors, ids));
        REQUIRE(ids.size() == mesh.indices.size());
        CHECK(ids[0] == 1);
        CHECK(ids[1] == 0);
        CHECK(ids[2] == 1);
        CHECK(colors[0] == blue);
        CHECK(colors[1] == red);
        CHECK(colors[2] == blue);
    }

    // Each disconnected two-face component has one seed and one ambiguous
    // face, so one valid neighbor is a strict majority in both components.
    {
        const TriMesh mesh = disconnected_open_squares_mesh();
        std::vector<Color> colors{red, gray, blue, gray};
        std::vector<std::size_t> ids;
        REQUIRE(mesh_cluster(mesh, {red, blue}, colors, ids));
        REQUIRE(ids.size() == mesh.indices.size());
        CHECK(ids[0] == 0);
        CHECK(ids[1] == 0);
        CHECK(ids[2] == 1);
        CHECK(ids[3] == 1);
        CHECK(colors[0] == red);
        CHECK(colors[1] == red);
        CHECK(colors[2] == blue);
        CHECK(colors[3] == blue);
    }
}

TEST_CASE("Mesh clustering weights an unresolved connected component by face area",
          "[ColorUtils][ColorUtilsExtended][ColorUtilsStep3]")
{

    const std::vector<Color> centers{kRedCenter, kBlueCenter, kThirdCenter};
    const std::vector<Color> input_colors{kRedCenter, kPatchA, kPatchB, kBlueCenter};
    REQUIRE(input_colors[0] == kRedCenter);
    REQUIRE(input_colors[3] == kBlueCenter);

    // Production CIEDE checks are route preconditions only. The fixed output
    // expectations below use independently calculated weighted RGB colors.
    const auto require_step3_patch = [&](const Color &patch) {
        const double red_distance   = calc_rgb_color_difference_by_ciede2000(patch, kRedCenter);
        const double blue_distance  = calc_rgb_color_difference_by_ciede2000(patch, kBlueCenter);
        const double third_distance = calc_rgb_color_difference_by_ciede2000(patch, kThirdCenter);
        const double second_distance = std::min(red_distance, blue_distance);
        REQUIRE(third_distance < second_distance);
        REQUIRE(second_distance - third_distance > 0.5);
        REQUIRE(third_distance > 3.0);
        REQUIRE(third_distance * 1.5 - second_distance > 2.0);
    };
    require_step3_patch(kPatchA);
    require_step3_patch(kPatchB);

    const auto weighted_average = [](const Color &a, const Color &b, const std::size_t area_a, const std::size_t area_b) {
        const std::size_t total = area_a + area_b;
        return Color{{(area_a * a[0] + area_b * b[0]) / total,
                      (area_a * a[1] + area_b * b[1]) / total,
                      (area_a * a[2] + area_b * b[2]) / total}};
    };
    const Color original_average = weighted_average(kPatchA, kPatchB, 2, 8);
    const Color swapped_average = weighted_average(kPatchA, kPatchB, 8, 2);
    // The weighted averages are worked out above by hand. The margins below are measured with the
    // production CIEDE2000 and only document how far each average is from the decision boundary:
    // (163,141,70) is 29.05 closer to red than to blue, (64,120,145) 28.49 closer to blue than to red.
    REQUIRE(original_average == Color{{163, 141, 70}});
    REQUIRE(swapped_average == Color{{64, 120, 145}});
    const double original_margin = calc_rgb_color_difference_by_ciede2000(original_average, kBlueCenter)
                                 - calc_rgb_color_difference_by_ciede2000(original_average, kRedCenter);
    const double swapped_margin = calc_rgb_color_difference_by_ciede2000(swapped_average, kRedCenter)
                                - calc_rgb_color_difference_by_ciede2000(swapped_average, kBlueCenter);
    CHECK_THAT(original_margin, Catch::Matchers::WithinAbs(29.0498, 0.25));
    CHECK_THAT(swapped_margin, Catch::Matchers::WithinAbs(28.4872, 0.25));

    for (const bool swap_patch_areas : {false, true}) {
        const TriMesh mesh = weighted_component_mesh(swap_patch_areas);
        REQUIRE(mesh.vertices.size() == 6);
        REQUIRE(mesh.indices.size() == 4);
        REQUIRE(triangle_area(mesh, 0) > 0.0);
        REQUIRE(triangle_area(mesh, 1) > 0.0);
        REQUIRE(triangle_area(mesh, 2) > 0.0);
        REQUIRE(triangle_area(mesh, 3) > 0.0);
        const double patch_a_area = triangle_area(mesh, 1);
        const double patch_b_area = triangle_area(mesh, 2);
        if (!swap_patch_areas) {
            CHECK_THAT(patch_a_area, Catch::Matchers::WithinAbs(2.0, 1e-6));
            CHECK_THAT(patch_b_area, Catch::Matchers::WithinAbs(8.0, 1e-6));
        } else {
            CHECK_THAT(patch_a_area, Catch::Matchers::WithinAbs(8.0, 1e-6));
            CHECK_THAT(patch_b_area, Catch::Matchers::WithinAbs(2.0, 1e-6));
        }

        const TriMesh original_mesh = mesh;
        std::vector<Color> face_colors = input_colors;
        std::vector<std::size_t> face_ids;
        REQUIRE(mesh_cluster(mesh, centers, face_colors, face_ids));
        REQUIRE(face_ids.size() == mesh.indices.size());
        REQUIRE(face_colors.size() == mesh.indices.size());
        CHECK(face_ids[0] == 0);
        CHECK(face_ids[3] == 1);
        CHECK(face_colors[0] == kRedCenter);
        CHECK(face_colors[3] == kBlueCenter);
        if (!swap_patch_areas) {
            CHECK(face_ids[1] == 0);
            CHECK(face_ids[2] == 0);
            CHECK(face_colors[1] == kRedCenter);
            CHECK(face_colors[2] == kRedCenter);
        } else {
            CHECK(face_ids[1] == 1);
            CHECK(face_ids[2] == 1);
            CHECK(face_colors[1] == kBlueCenter);
            CHECK(face_colors[2] == kBlueCenter);
        }
        CHECK(mesh.vertices == original_mesh.vertices);
        CHECK(mesh.indices == original_mesh.indices);
    }
}

namespace {

void check_finite_mesh(const TriMesh &mesh)
{
    for (const TriVertex &vertex : mesh.vertices) {
        CHECK(std::isfinite(vertex.x()));
        CHECK(std::isfinite(vertex.y()));
        CHECK(std::isfinite(vertex.z()));
    }
}

double signed_volume(const TriMesh &mesh)
{
    double sum = 0.;
    for (const auto &face : mesh.indices) {
        const auto &a = mesh.vertices[face[0]];
        const auto &b = mesh.vertices[face[1]];
        const auto &c = mesh.vertices[face[2]];
        sum += static_cast<double>(a.x()) * (static_cast<double>(b.y()) * c.z() - static_cast<double>(b.z()) * c.y());
        sum += static_cast<double>(a.y()) * (static_cast<double>(b.z()) * c.x() - static_cast<double>(b.x()) * c.z());
        sum += static_cast<double>(a.z()) * (static_cast<double>(b.x()) * c.y() - static_cast<double>(b.y()) * c.x());
    }
    return sum / 6.;
}
} // namespace

// The cube's 8 vertices are all feature vertices, so geometry/boundary smoothing is not exercised here:
// the mesh must stay unchanged while only the label smoothing threshold varies.
TEST_CASE("Label smoothing leaves labels unchanged below the threshold and removes minority faces at weight 0.3", "[ColorUtils]")
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
        const std::size_t initial_ones  = static_cast<std::size_t>(std::count(initial.begin(), initial.end(), 1));
        const std::size_t smoothed_ones = static_cast<std::size_t>(std::count(labels.begin(), labels.end(), 1));
        if (weight < 0.3) {
            CHECK(labels == initial);
            CHECK(smoothed_ones == initial_ones);
        } else {
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
    CHECK(colors != unchanged_colors);   // smoothing at 0.8 does relabel faces
    REQUIRE(colors.size() == mesh.indices.size());
    CHECK(is_closed(mesh));
    for (const Color& color : colors)
        CHECK(palette.count(color) == 1);
    check_finite_mesh(mesh);
}

TEST_CASE("Smoothing rejects a label count that does not match the faces", "[ColorUtils]")
{
    TriMesh mesh = cube_mesh();
    const TriMesh original = mesh;
    std::vector<std::size_t> wrong_count(mesh.indices.size() - 1, 0);
    SmoothParameters parameters;
    parameters.smooth_weight = 0.8;
    CHECK_FALSE(smooth_region(mesh, wrong_count, parameters));
    CHECK(mesh.vertices == original.vertices);
    CHECK(mesh.indices == original.indices);
}

TEST_CASE("Remeshing rejects a label count that does not match the faces before changing geometry", "[ColorUtils]")
{
    TriMesh mesh           = cube_mesh(6.);
    const TriMesh original = mesh;
    const int delta        = GENERATE(-1, 1);
    std::vector<std::size_t> labels(static_cast<std::size_t>(static_cast<int>(mesh.indices.size()) + delta), 5);
    const std::vector<std::size_t> original_labels = labels;
    CHECK_FALSE(remesh_mesh(mesh, labels, 0.7));
    CHECK(mesh.vertices == original.vertices);
    CHECK(mesh.indices == original.indices);
    CHECK(labels == original_labels);
}

namespace {

// Labels 7 for faces whose centroid lies above `height`, 3 otherwise. remesh_mesh constrains the edges
// whose two faces carry different labels. On this cube those are the side-face diagonals and any
// vertical edge whose adjacent side triangles differ in label; the cube has no vertices at mid-height, so
// there are no edges at the split, and the top and bottom edges (same label on both sides) stay
// unconstrained, so the remesher may only approximate them.
std::vector<std::size_t> labels_split_at_height(const TriMesh &mesh, float height)
{
    std::vector<std::size_t> labels(mesh.indices.size(), 3);
    for (std::size_t i = 0; i < labels.size(); ++i) {
        const auto &face = mesh.indices[i];
        const float z    = (mesh.vertices[face[0]].z() + mesh.vertices[face[1]].z() + mesh.vertices[face[2]].z()) / 3.f;
        if (z > height)
            labels[i] = 7;
    }
    return labels;
}

double mean_edge_length(const TriMesh &mesh)
{
    double sum = 0.;
    for (const auto &face : mesh.indices)
        for (int i = 0; i < 3; ++i)
            sum += (mesh.vertices[face[i]] - mesh.vertices[face[(i + 1) % 3]]).cast<double>().norm();
    return sum / (3. * double(mesh.indices.size()));
}

} // namespace

// Isotropic remeshing only approximates the input surface, so the unconstrained sharp edges may be
// rounded. A 10% volume tolerance allows that. Measured for reference: a single-label cube (no
// constrained edges) loses 50% of its volume at ratio 0.6 and all of it at 1.4, this fixture loses ~0%.
TEST_CASE("Remeshing a closed two-label cube approximately keeps its volume, closure, and both labels", "[ColorUtils]")
{
    const double ratio    = GENERATE(0.6, 1.4);
    constexpr double side = 8.;
    TriMesh mesh          = cube_mesh(side);
    std::vector<std::size_t> labels = labels_split_at_height(mesh, 4.f);
    REQUIRE(remesh_mesh(mesh, labels, ratio));
    REQUIRE(mesh.indices.size() > 0);
    CHECK(labels.size() == mesh.indices.size());
    CHECK(is_closed(mesh));
    CHECK_THAT(std::abs(signed_volume(mesh)), Catch::Matchers::WithinRel(side * side * side, 0.1));
    const auto low  = std::count(labels.begin(), labels.end(), std::size_t(3));
    const auto high = std::count(labels.begin(), labels.end(), std::size_t(7));
    CHECK(low > 0);
    CHECK(high > 0);
    CHECK(low + high == static_cast<std::ptrdiff_t>(labels.size()));
    check_finite_mesh(mesh);
}

TEST_CASE("Remeshing with a smaller target edge ratio yields shorter edges and more faces", "[ColorUtils]")
{
    TriMesh coarse = cube_mesh(8.);
    TriMesh fine   = cube_mesh(8.);
    std::vector<std::size_t> coarse_labels = labels_split_at_height(coarse, 4.f);
    std::vector<std::size_t> fine_labels   = labels_split_at_height(fine, 4.f);
    REQUIRE(remesh_mesh(coarse, coarse_labels, 1.4));
    REQUIRE(remesh_mesh(fine, fine_labels, 0.6));
    REQUIRE(coarse.indices.size() > 0);
    CHECK(fine.indices.size() > coarse.indices.size());
    CHECK(mean_edge_length(fine) < mean_edge_length(coarse));
    CHECK(is_closed(coarse));
    CHECK(is_closed(fine));
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
