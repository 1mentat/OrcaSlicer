#include <libslic3r/TextureBake/TextureBakeDisplace.hpp>
#include <catch2/catch_message.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

#include "libslic3r/Point.hpp"
#include "libslic3r/TextureBake/TextureBakeIndex.hpp"
#include "libslic3r/TextureBake/TextureBakeRelocate.hpp"

using namespace Slic3r;
using namespace Slic3r::TextureBake;
using Catch::Matchers::WithinAbs;

namespace {

void add_triangle(TriSoup &soup, const Vec3f &a, const Vec3f &b, const Vec3f &c)
{
    soup.pos.insert(soup.pos.end(), { a, b, c });
    soup.nrm.insert(soup.nrm.end(), 3, Vec3f(0.f, 0.f, 1.f));
}

TriSoup planar_grid(int nx = 6, int ny = 6, float spacing = 1.f)
{
    TriSoup soup;
    soup.pos.reserve(size_t(nx * ny * 6));
    soup.nrm.reserve(size_t(nx * ny * 6));
    for (int y = 0; y < ny; ++y) {
        for (int x = 0; x < nx; ++x) {
            const float x0 = (float(x) - float(nx) * .5f) * spacing;
            const float y0 = (float(y) - float(ny) * .5f) * spacing;
            const Vec3f p00(x0, y0, 0.f);
            const Vec3f p10(x0 + spacing, y0, 0.f);
            const Vec3f p11(x0 + spacing, y0 + spacing, 0.f);
            const Vec3f p01(x0, y0 + spacing, 0.f);
            add_triangle(soup, p00, p10, p11);
            add_triangle(soup, p00, p11, p01);
        }
    }
    return soup;
}

TriSoup single_triangle()
{
    TriSoup soup;
    add_triangle(soup, Vec3f(-1.f, -1.f, 0.f), Vec3f(1.f, -1.f, 0.f), Vec3f(-1.f, 1.f, 0.f));
    return soup;
}

HeightSampleFn x_field()
{
    return [](const Vec3f &p, const Vec3f &, const Vec3f &) { return p.x(); };
}

HeightSampleFn sloped_field()
{
    return [](const Vec3f &p, const Vec3f &, const Vec3f &) { return p.x() + .25f * p.y(); };
}

double value_range(const TriSoup &soup, const HeightSampleFn &sample)
{
    double lo = std::numeric_limits<double>::max();
    double hi = -lo;
    for (const Vec3f &p : soup.pos) {
        const double value = sample(p, Vec3f::UnitZ(), Vec3f::UnitZ());
        lo = std::min(lo, value);
        hi = std::max(hi, value);
    }
    return hi - lo;
}

double contour_target(const TriSoup &soup, const HeightSampleFn &sample, double level)
{
    double lo = std::numeric_limits<double>::max();
    double hi = -lo;
    for (const Vec3f &p : soup.pos) {
        const double value = sample(p, Vec3f::UnitZ(), Vec3f::UnitZ());
        lo = std::min(lo, value);
        hi = std::max(hi, value);
    }
    return lo + (hi - lo) * level;
}

double contour_error(const TriSoup &soup, const HeightSampleFn &sample, double target)
{
    double sum = 0.;
    for (const Vec3f &p : soup.pos) {
        const double delta = double(sample(p, Vec3f::UnitZ(), Vec3f::UnitZ())) - target;
        sum += delta * delta;
    }
    return sum;
}

void require_finite_geometry(const TriSoup &soup)
{
    REQUIRE(soup.pos.size() % 3 == 0);
    REQUIRE(soup.nrm.size() == soup.pos.size());
    for (size_t i = 0; i < soup.pos.size(); ++i) {
        CHECK(soup.pos[i].allFinite());
        CHECK(soup.nrm[i].allFinite());
        CHECK_THAT(soup.nrm[i].norm(), WithinAbs(1.f, 1e-5));
    }
    for (size_t t = 0; t < soup.triangle_count(); ++t) {
        const Vec3f &a = soup.pos[t * 3];
        const Vec3f &b = soup.pos[t * 3 + 1];
        const Vec3f &c = soup.pos[t * 3 + 2];
        const Vec3f cross = (b - a).cross(c - a);
        CHECK(cross.allFinite());
        // A valid planar input must retain positive oriented area: the move
        // guard rejects both flipped and collapsed triangles.
        CHECK(cross.z() > 0.f);
    }
}

void check_same_positions(const TriSoup &a, const TriSoup &b)
{
    REQUIRE(a.pos.size() == b.pos.size());
    for (size_t i = 0; i < a.pos.size(); ++i)
        CHECK_THAT((a.pos[i] - b.pos[i]).norm(), WithinAbs(0., 1e-7));
}

void check_welded_copies(const TriSoup &input, const TriSoup &output)
{
    REQUIRE(input.pos.size() == output.pos.size());
    for (size_t i = 0; i < input.pos.size(); ++i)
        for (size_t j = i + 1; j < input.pos.size(); ++j)
            if ((input.pos[i] - input.pos[j]).norm() < 1e-7f)
                CHECK_THAT((output.pos[i] - output.pos[j]).norm(), WithinAbs(0., 1e-6));
}

} // namespace

TEST_CASE("Texture contour relocation handles empty, disabled, null, and constant fields", "[TextureBakeRelocate]")
{
    const TriSoup input = planar_grid();
    RelocateSettings settings;
    settings.iterations = 2;

    const RelocateResult empty = relocate_to_contours({}, x_field(), settings);
    CHECK(empty.geometry.empty());
    CHECK(empty.moved == 0);
    CHECK(empty.rejected == 0);

    RelocateSettings disabled = settings;
    disabled.iterations = 0;
    const RelocateResult no_iterations = relocate_to_contours(input, x_field(), disabled);
    check_same_positions(no_iterations.geometry, input);
    CHECK(no_iterations.moved == 0);

    const RelocateResult no_sample = relocate_to_contours(input, HeightSampleFn{}, settings);
    check_same_positions(no_sample.geometry, input);
    CHECK(no_sample.moved == 0);

    const RelocateResult flat = relocate_to_contours(input,
        [](const Vec3f &, const Vec3f &, const Vec3f &) { return 7.f; }, settings);
    check_same_positions(flat.geometry, input);
    CHECK(flat.moved == 0);
    require_finite_geometry(flat.geometry);
}

TEST_CASE("Texture contour relocation improves a planar linear contour with tangent moves", "[TextureBakeRelocate]")
{
    const TriSoup input = planar_grid();
    const HeightSampleFn sample = sloped_field();
    RelocateSettings settings;
    settings.iterations = 1;
    settings.max_move_fraction = .2;
    settings.min_gradient_fraction = .01;
    settings.gradient_step_fraction = .2;
    const double target = contour_target(input, sample, settings.contour_level);
    const double before = contour_error(input, sample, target);

    const RelocateResult result = relocate_to_contours(input, sample, settings);
    REQUIRE(result.moved > 0);
    CHECK(contour_error(result.geometry, sample, target) < before);
    require_finite_geometry(result.geometry);
    check_welded_copies(input, result.geometry);
    for (size_t i = 0; i < input.pos.size(); ++i) {
        const Vec3f displacement = result.geometry.pos[i] - input.pos[i];
        CHECK_THAT(displacement.dot(Vec3f::UnitZ()), WithinAbs(0., 1e-6));
        CHECK(displacement.norm() <= .2 * std::sqrt(2.f) + 1e-5f);
    }
}

TEST_CASE("Texture contour relocation honors low, middle, and high contour levels", "[TextureBakeRelocate]")
{
    const TriSoup input = planar_grid();
    const HeightSampleFn sample = x_field();
    const double level = GENERATE(0., .5, 1.);
    RelocateSettings settings;
    settings.iterations = 2;
    settings.max_move_fraction = .15;
    settings.min_gradient_fraction = .01;
    settings.contour_level = level;
    const double target = contour_target(input, sample, level);
    const double before = contour_error(input, sample, target);
    const RelocateResult result = relocate_to_contours(input, sample, settings);
    CHECK(result.moved > 0);
    CHECK(contour_error(result.geometry, sample, target) < before);
    // The edge levels must direct the whole patch toward the requested side. A
    // midpoint-only implementation can still lower the aggregate error on this
    // symmetric grid, so check the signed mean independently for the two edges.
    double mean_x = 0.;
    for (const Vec3f &p : result.geometry.pos)
        mean_x += double(p.x());
    mean_x /= double(result.geometry.pos.size());
    if (level == 0.)
        CHECK(mean_x < -1e-3);
    else if (level == 1.)
        CHECK(mean_x > 1e-3);
    require_finite_geometry(result.geometry);
}

TEST_CASE("Texture contour relocation respects iterations and move caps", "[TextureBakeRelocate]")
{
    const TriSoup input = planar_grid();
    const HeightSampleFn sample = x_field();
    RelocateSettings one;
    one.iterations = 1;
    one.max_move_fraction = .1;
    one.min_gradient_fraction = .01;
    const double target = contour_target(input, sample, one.contour_level);
    const double before = contour_error(input, sample, target);

    RelocateSettings many = one;
    many.iterations = 4;
    const RelocateResult one_result = relocate_to_contours(input, sample, one);
    const RelocateResult many_result = relocate_to_contours(input, sample, many);
    CHECK(contour_error(one_result.geometry, sample, target) < before);
    CHECK(contour_error(many_result.geometry, sample, target) <= contour_error(one_result.geometry, sample, target));

    RelocateSettings zero_cap = one;
    zero_cap.max_move_fraction = 0.;
    const RelocateResult capped = relocate_to_contours(input, sample, zero_cap);
    check_same_positions(capped.geometry, input);
    require_finite_geometry(capped.geometry);
    require_finite_geometry(many_result.geometry);
}

TEST_CASE("Texture contour relocation gates weak gradients and zero finite differences", "[TextureBakeRelocate]")
{
    const TriSoup input = planar_grid();
    const HeightSampleFn sample = x_field();
    RelocateSettings disabled_gradient;
    disabled_gradient.iterations = 2;
    disabled_gradient.min_gradient_fraction = 2.;
    const RelocateResult weak = relocate_to_contours(input, sample, disabled_gradient);
    check_same_positions(weak.geometry, input);
    CHECK(weak.moved == 0);

    RelocateSettings disabled_difference = disabled_gradient;
    disabled_difference.min_gradient_fraction = 0.;
    disabled_difference.gradient_step_fraction = 0.;
    const RelocateResult no_difference = relocate_to_contours(input, sample, disabled_difference);
    check_same_positions(no_difference.geometry, input);
    CHECK(no_difference.moved == 0);

    RelocateSettings enabled = disabled_difference;
    enabled.gradient_step_fraction = .5;
    const RelocateResult active = relocate_to_contours(input, sample, enabled);
    CHECK(active.moved > 0);
    CHECK(contour_error(active.geometry, sample, 0.) < contour_error(input, sample, 0.));

    // A nonconstant field can still have a stationary point. The zero-gradient guard
    // must skip that vertex without allowing 0/0 to enter the proposal or output.
    const HeightSampleFn quadratic = [](const Vec3f &p, const Vec3f &, const Vec3f &) {
        return p.x() * p.x();
    };
    RelocateSettings stationary = enabled;
    stationary.contour_level = .5;
    const RelocateResult stationary_result = relocate_to_contours(input, quadratic, stationary);
    require_finite_geometry(stationary_result.geometry);
}

TEST_CASE("Texture contour relocation freezes locked-face vertices", "[TextureBakeRelocate]")
{
    const TriSoup input = planar_grid();
    std::vector<uint8_t> locked(input.triangle_count(), 0);
    locked[0] = 1;
    RelocateSettings settings;
    settings.iterations = 2;
    settings.max_move_fraction = .2;
    settings.min_gradient_fraction = .01;
    const RelocateResult result = relocate_to_contours(input, x_field(), settings, locked);
    REQUIRE(result.moved > 0);
    for (size_t i = 0; i < 3; ++i)
        CHECK_THAT((result.geometry.pos[i] - input.pos[i]).norm(), WithinAbs(0., 1e-7));
    bool another_vertex_moved = false;
    for (size_t i = 3; i < input.pos.size(); ++i)
        another_vertex_moved |= (result.geometry.pos[i] - input.pos[i]).norm() > 1e-6f;
    CHECK(another_vertex_moved);
    require_finite_geometry(result.geometry);
}

TEST_CASE("Texture contour relocation preserves welded copies and auxiliary weights", "[TextureBakeRelocate]")
{
    TriSoup input = planar_grid(4, 4);
    input.exclude_weight.resize(input.pos.size());
    for (size_t i = 0; i < input.exclude_weight.size(); ++i)
        input.exclude_weight[i] = float(i % 5) / 4.f;
    const RelocateResult result = relocate_to_contours(input, sloped_field());
    // Auxiliary weights are copied verbatim, so exact equality is intentional.
    REQUIRE(result.geometry.exclude_weight == input.exclude_weight);
    check_welded_copies(input, result.geometry);
    require_finite_geometry(result.geometry);
}

TEST_CASE("Texture contour relocation rejects moves that would invert triangles", "[TextureBakeRelocate]")
{
    const TriSoup input = planar_grid(3, 3);
    RelocateSettings settings;
    settings.iterations = 5;
    settings.max_move_fraction = 3.;
    settings.min_gradient_fraction = 0.;
    const RelocateResult result = relocate_to_contours(input, sloped_field(), settings);
    CHECK(result.rejected > 0);
    require_finite_geometry(result.geometry);
    CHECK(contour_error(result.geometry, sloped_field(), contour_target(input, sloped_field(), .5))
          < contour_error(input, sloped_field(), contour_target(input, sloped_field(), .5)));
}

TEST_CASE("Texture contour relocation handles a single triangle and malformed lock lengths", "[TextureBakeRelocate]")
{
    const TriSoup input = single_triangle();
    RelocateSettings settings;
    settings.iterations = 1;
    settings.max_move_fraction = .1;
    settings.min_gradient_fraction = .01;
    const RelocateResult result = relocate_to_contours(input, x_field(), settings, { 1, 0, 1, 0, 1 });
    REQUIRE(result.geometry.pos.size() == input.pos.size());
    require_finite_geometry(result.geometry);
    CHECK(result.moved == 0);
    check_same_positions(result.geometry, input);
}

TEST_CASE("Texture contour relocation keeps normal and topology contracts after repeated passes", "[TextureBakeRelocate]")
{
    const TriSoup input = planar_grid(5, 4);
    RelocateSettings settings;
    settings.iterations = 8;
    settings.max_move_fraction = .25;
    settings.min_gradient_fraction = .01;
    settings.contour_level = .35;
    const RelocateResult result = relocate_to_contours(input, sloped_field(), settings);
    REQUIRE(result.geometry.triangle_count() == input.triangle_count());
    REQUIRE(result.geometry.pos.size() == input.pos.size());
    require_finite_geometry(result.geometry);
    check_welded_copies(input, result.geometry);
}
