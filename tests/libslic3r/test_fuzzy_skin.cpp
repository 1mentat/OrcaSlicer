#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "libslic3r/Feature/FuzzySkin/FuzzySkin.hpp"
#include "libslic3r/PerimeterGenerator.hpp"
#include <algorithm>
#include <array>
#include <cstddef>
#include <catch2/catch_message.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include "libslic3r/Arachne/utils/ExtrusionJunction.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/libslic3r.h"
#include <libnoise/noise.h>
#include <libnoise/module/modulebase.h>
#include <libnoise/module/perlin.h>
#include <libnoise/module/billow.h>
#include <libnoise/module/ridgedmulti.h>
#include <libnoise/module/voronoi.h>
#include <cmath>
#include <memory>
#include <vector>

using namespace Slic3r;
using namespace Slic3r::Feature::FuzzySkin;
using Catch::Matchers::WithinAbs;

namespace {
FuzzySkinConfig ripple_config()
{
    FuzzySkinConfig c{};
    c.type = FuzzySkinType::All;
    c.thickness = scaled<coord_t>(0.4);
    c.point_distance = scaled<coord_t>(0.5);
    c.fuzzy_first_layer = true;
    c.noise_type = NoiseType::Ripple;
    c.mode = FuzzySkinMode::Displacement;
    c.ripples_per_layer = 4;
    c.ripple_offset = 50.;
    c.layers_between_ripple_offset = 3;
    return c;
}
Points square(double y = -10.)
{
    return {Point::new_scale(-10., y), Point::new_scale(10., y),
            Point::new_scale(10., y + 20.), Point::new_scale(-10., y + 20.)};
}
Points rippled(const Points &input, const FuzzySkinConfig &config)
{
    Points out = input;
    fuzzy_polyline(out, true, 1., config);
    return out;
}
void near_points(const Points &a, const Points &b, double tolerance = 3e-6)
{
    REQUIRE(a.size() == b.size());
    for (size_t i = 0; i < a.size(); ++i) {
        INFO("point=" << i);
        CHECK_THAT(unscale_(a[i].x()), WithinAbs(unscale_(b[i].x()), tolerance));
        CHECK_THAT(unscale_(a[i].y()), WithinAbs(unscale_(b[i].y()), tolerance));
    }
}
Arachne::ExtrusionJunctions junction_square()
{
    Arachne::ExtrusionJunctions out;
    for (const Point &p : square())
        out.emplace_back(p, scaled<coord_t>(0.45), 7);
    out.push_back(out.front());
    return out;
}
Points positions(const Arachne::ExtrusionJunctions &line)
{
    Points out;
    for (const auto &j : line) out.push_back(j.p);
    return out;
}
} // namespace

TEST_CASE("Ripple samples have the configured spacing and bounded transverse amplitude", "[FuzzySkin][FuzzyRipple]")
{
    auto cfg = ripple_config();
    cfg.ripples_per_layer = GENERATE(1, 4, 15);
    const Points out = rippled(square(), cfg);
    REQUIRE(out.size() == 160);
    double max_displacement = 0.;
    for (size_t i = 0; i < out.size(); ++i) {
        const double x = unscale_(out[i].x()), y = unscale_(out[i].y());
        const double t = double(i % 40) * 0.5;
        double displacement = 0.;
        switch (i / 40) {
        case 0: CHECK_THAT(x, WithinAbs(-10. + t, 2e-6)); displacement = y + 10.; break;
        case 1: CHECK_THAT(y, WithinAbs(-10. + t, 2e-6)); displacement = 10. - x; break;
        case 2: CHECK_THAT(x, WithinAbs(10. - t, 2e-6)); displacement = 10. - y; break;
        default: CHECK_THAT(y, WithinAbs(10. - t, 2e-6)); displacement = x + 10.; break;
        }
        CHECK(std::abs(displacement) <= 0.400002);
        max_displacement = std::max(max_displacement, std::abs(displacement));
    }
    CHECK_THAT(max_displacement, WithinAbs(0.4, 2e-6));
}

TEST_CASE("Ripple layer groups share phase and a half wavelength inverts displacement", "[FuzzySkin][FuzzyRipple]")
{
    auto cfg = ripple_config();
    cfg.layers_between_ripple_offset = GENERATE(1, 3, 7);
    const Points base = rippled(square(), cfg);
    cfg.layer_id = cfg.layers_between_ripple_offset - 1;
    near_points(base, rippled(square(), cfg));
    cfg.layer_id = cfg.layers_between_ripple_offset;
    const Points inverse = rippled(square(), cfg);
    cfg.thickness = 0;
    const Points center = rippled(square(), cfg);
    REQUIRE(base.size() == inverse.size());
    REQUIRE(base.size() == center.size());
    bool changed = false;
    for (size_t i = 0; i < base.size(); ++i) {
        CHECK_THAT(unscale_(base[i].x() + inverse[i].x()), WithinAbs(2 * unscale_(center[i].x()), 3e-6));
        CHECK_THAT(unscale_(base[i].y() + inverse[i].y()), WithinAbs(2 * unscale_(center[i].y()), 3e-6));
        changed = changed || (base[i] - inverse[i]).norm() > scaled<double>(0.1);
    }
    CHECK(changed);
}

TEST_CASE("Ripple full wavelength and four quarter shifts return to the same pattern", "[FuzzySkin][FuzzyRipple]")
{
    auto cfg = ripple_config();
    const Points base = rippled(square(), cfg);
    cfg.ripple_offset = GENERATE(25., 50., 100.);
    cfg.layer_id = int(100. / cfg.ripple_offset) * cfg.layers_between_ripple_offset;
    near_points(base, rippled(square(), cfg));
}

TEST_CASE("Ripple zero offset and nonpositive layer groups disable phase changes", "[FuzzySkin][FuzzyRipple]")
{
    auto cfg = ripple_config();
    const Points base = rippled(square(), cfg);
    cfg.layer_id = 19;
    cfg.layers_between_ripple_offset = GENERATE(0, -1, 3);
    if (cfg.layers_between_ripple_offset == 3) cfg.ripple_offset = 0.;
    near_points(base, rippled(square(), cfg));
}

TEST_CASE("Ripple negative layer indices retain the initial phase", "[FuzzySkin][FuzzyRipple]")
{
    auto cfg = ripple_config();
    const Points base = rippled(square(), cfg);
    cfg.layer_id = GENERATE(-1, -20);
    near_points(base, rippled(square(), cfg));
}

TEST_CASE("Ripple amplitude scales displacement about the unchanged sampling grid", "[FuzzySkin][FuzzyRipple]")
{
    auto cfg = ripple_config();
    cfg.layer_id = 3;
    const Points one = rippled(square(), cfg);
    cfg.thickness *= 2;
    const Points two = rippled(square(), cfg);
    cfg.thickness = 0;
    const Points zero = rippled(square(), cfg);
    REQUIRE(one.size() == two.size());
    REQUIRE(one.size() == zero.size());
    for (size_t i = 0; i < one.size(); ++i) {
        CHECK_THAT(unscale_(two[i].x() - zero[i].x()), WithinAbs(2 * unscale_(one[i].x() - zero[i].x()), 3e-6));
        CHECK_THAT(unscale_(two[i].y() - zero[i].y()), WithinAbs(2 * unscale_(one[i].y() - zero[i].y()), 3e-6));
    }
}

TEST_CASE("Ripple world anchor is invariant under horizontal translation", "[FuzzySkin][FuzzyRipple]")
{
    const double y = GENERATE(-10., 5., -35., 0.);
    const auto cfg = ripple_config();
    const Points base = rippled(square(y), cfg);
    Points moved = square(y);
    const Point delta = Point::new_scale(127., 0.);
    for (Point &p : moved) p += delta;
    moved = rippled(moved, cfg);
    for (Point &p : moved) p -= delta;
    near_points(base, moved);
}

TEST_CASE("Ripple duplicate vertices do not create samples or alter the contour", "[FuzzySkin][FuzzyRipple]")
{
    const auto cfg = ripple_config();
    const Points base = rippled(square(), cfg);
    Points duplicated;
    for (const Point &p : square()) {
        duplicated.push_back(p);
        duplicated.push_back(p);
    }
    near_points(base, rippled(duplicated, cfg));
}

TEST_CASE("Ripple disabled counts and subresolution steps retain the original contour", "[FuzzySkin][FuzzyRipple]")
{
    auto cfg = ripple_config();
    const int variant = GENERATE(0, 1, 2);
    if (variant == 0) cfg.ripples_per_layer = 0;
    if (variant == 1) cfg.ripples_per_layer = -2;
    if (variant == 2) cfg.point_distance = 0;
    const Points input = square();
    // The disabled path copies original integer coordinates verbatim.
    CHECK(rippled(input, cfg) == input);
}

TEST_CASE("Ripple degenerate contours remain unchanged", "[FuzzySkin][FuzzyRipple]")
{
    const auto cfg = ripple_config();
    const size_t count = GENERATE(size_t(0), size_t(1), size_t(2), size_t(5));
    const Points input(count, Point::new_scale(3., 4.));
    CHECK(rippled(input, cfg) == input);
}

TEST_CASE("Ripple polygon and constant-width Arachne paths agree at every sample", "[FuzzySkin][FuzzyRipple]")
{
    auto cfg = ripple_config();
    cfg.layer_id = GENERATE(0, 3, 8);
    auto line = junction_square();
    fuzzy_extrusion_line(line, 1., 0.2, cfg, true);
    near_points(rippled(square(), cfg), positions(line));
    for (const auto &j : line) {
        CHECK(j.w == scaled<coord_t>(0.45));
        CHECK(j.perimeter_index == 7);
    }
}

TEST_CASE("Ripple Arachne paths preserve each segment destination width and perimeter metadata", "[FuzzySkin][FuzzyRipple]")
{
    auto cfg = ripple_config();
    auto line = junction_square();
    for (size_t i = 0; i < line.size(); ++i) {
        line[i].w = scaled<coord_t>(0.2 + double(i) * 0.1);
        line[i].perimeter_index = i + 4;
    }
    const auto original = line;
    fuzzy_extrusion_line(line, 1., 0.2, cfg, true);
    REQUIRE(line.size() == 160);
    for (size_t i = 0; i < line.size(); ++i) {
        const size_t destination = i / 40 + 1;
        CHECK(line[i].w == original[destination].w);
        CHECK(line[i].perimeter_index == original[destination].perimeter_index);
    }
}


TEST_CASE("Fuzzy skin type, layer, loop, and contour gates are exhaustive", "[FuzzySkin][FuzzyRipple]")
{
    struct TypeExpectation {
        FuzzySkinType type;
        bool contour_outer;
        bool contour_inner;
        bool hole_outer;
        bool hole_inner;
    };
    // These are the user-facing style contracts: External applies to the
    // outer contour, Hole applies to the outer hole loop, All applies to both
    // outer loops, and AllWalls applies to every loop. None and Disabled are
    // hard off switches.
    const std::array<TypeExpectation, 6> expected{{
        {FuzzySkinType::None,          false, false, false, false},
        {FuzzySkinType::External,      true,  false, false, false},
        {FuzzySkinType::Hole,          false, false, true,  false},
        {FuzzySkinType::All,           true,  false, true,  false},
        {FuzzySkinType::AllWalls,      true,  true,  true,  true },
        {FuzzySkinType::Disabled_fuzzy,false, false, false, false},
    }};
    const auto &row = expected[GENERATE(size_t(0), size_t(1), size_t(2), size_t(3), size_t(4), size_t(5))];
    const bool first_layer_enabled = GENERATE(false, true);
    const int layer_id = GENERATE(-1, 0, 1);
    const size_t loop_idx = GENERATE(size_t(0), size_t(1));
    const bool is_contour = GENERATE(false, true);
    FuzzySkinConfig cfg = ripple_config();
    cfg.type = row.type;
    cfg.fuzzy_first_layer = first_layer_enabled;
    bool expected_result = false;
    if (first_layer_enabled || layer_id > 0) {
        if (is_contour)
            expected_result = loop_idx == 0 ? row.contour_outer : row.contour_inner;
        else
            expected_result = loop_idx == 0 ? row.hole_outer : row.hole_inner;
    }
    CAPTURE(int(row.type), first_layer_enabled, layer_id, loop_idx, is_contour);
    CHECK(should_fuzzify(cfg, layer_id, loop_idx, is_contour) == expected_result);

}

TEST_CASE("Fuzzy skin first-layer gating treats zero and negative layers consistently", "[FuzzySkin][FuzzyRipple]")
{
    auto cfg = ripple_config();
    cfg.type = FuzzySkinType::External;
    cfg.fuzzy_first_layer = false;
    CHECK_FALSE(should_fuzzify(cfg, 0, 0, true));
    CHECK_FALSE(should_fuzzify(cfg, -4, 0, true));
    cfg.fuzzy_first_layer = true;
    CHECK(should_fuzzify(cfg, 0, 0, true));
    CHECK(should_fuzzify(cfg, -4, 0, true));
}

TEST_CASE("Ripple phase responds antisymmetrically to positive and negative quarter offsets", "[FuzzySkin][FuzzyRipple]")
{
    auto positive = ripple_config();
    positive.ripple_offset = 25.;
    positive.layer_id = positive.layers_between_ripple_offset;
    auto negative = positive;
    negative.ripple_offset = -25.;
    auto center = positive;
    center.thickness = 0;
    const Points plus = rippled(square(), positive);
    const Points minus = rippled(square(), negative);
    const Points zero = rippled(square(), center);
    REQUIRE(plus.size() == minus.size());
    REQUIRE(plus.size() == zero.size());
    for (size_t i = 0; i < plus.size(); ++i) {
        INFO("point=" << i);
        CHECK_THAT(unscale_(plus[i].x() + minus[i].x()), WithinAbs(2. * unscale_(zero[i].x()), 3e-6));
        CHECK_THAT(unscale_(plus[i].y() + minus[i].y()), WithinAbs(2. * unscale_(zero[i].y()), 3e-6));
    }
}

TEST_CASE("Ripple sampling count follows the configured point distance", "[FuzzySkin][FuzzyRipple]")
{
    auto cfg = ripple_config();
    cfg.point_distance = scaled<coord_t>(1.);
    const Points one_mm = rippled(square(), cfg);
    cfg.point_distance = scaled<coord_t>(2.);
    const Points two_mm = rippled(square(), cfg);
    cfg.point_distance = scaled<coord_t>(0.25);
    const Points quarter_mm = rippled(square(), cfg);
    CHECK(one_mm.size() == 80);
    CHECK(two_mm.size() == 40);
    CHECK(quarter_mm.size() == 320);
    CHECK(one_mm.size() > two_mm.size());
    CHECK(quarter_mm.size() > one_mm.size());
}

TEST_CASE("Ripple Arachne guards preserve short and collapsed junction lists", "[FuzzySkin][FuzzyRipple]")
{
    auto cfg = ripple_config();
    for (const size_t count : {size_t(0), size_t(1), size_t(2)}) {
        Arachne::ExtrusionJunctions line;
        for (size_t i = 0; i < count; ++i)
            line.emplace_back(Point::new_scale(double(i), 2.), scaled<coord_t>(0.45), i + 1);
        const auto before = line;
        fuzzy_extrusion_line(line, 1., 0.2, cfg, true);
        REQUIRE(line.size() == before.size());
        for (size_t i = 0; i < line.size(); ++i) {
            CHECK(line[i].p == before[i].p);
            CHECK(line[i].w == before[i].w);
            CHECK(line[i].perimeter_index == before[i].perimeter_index);
        }
    }
    Arachne::ExtrusionJunctions collapsed;
    for (size_t i = 0; i < 3; ++i)
        collapsed.emplace_back(Point::new_scale(2., 2.), scaled<coord_t>(0.45), i + 5);
    const auto before = collapsed;
    fuzzy_extrusion_line(collapsed, 1., 0.2, cfg, false);
    REQUIRE(collapsed.size() == before.size());
    for (size_t i = 0; i < collapsed.size(); ++i) {
        CHECK(collapsed[i].p == before[i].p);
        CHECK(collapsed[i].w == before[i].w);
        CHECK(collapsed[i].perimeter_index == before[i].perimeter_index);
    }
}

TEST_CASE("Ripple preserves the input for subresolution and nonpositive sample settings", "[FuzzySkin][FuzzyRipple]")
{
    auto cfg = ripple_config();
    const Points input = square();
    for (const double distance : {0., -1., 1e-9}) {
        cfg.point_distance = scaled<coord_t>(distance);
        CHECK(rippled(input, cfg) == input);
    }
}

namespace {

std::unique_ptr<noise::module::Module> reference_noise(NoiseType noise_type, const FuzzySkinConfig &cfg)
{
    if (noise_type == NoiseType::Perlin) {
        auto module = std::make_unique<noise::module::Perlin>();
        module->SetFrequency(1. / cfg.noise_scale);
        module->SetOctaveCount(cfg.noise_octaves);
        module->SetPersistence(cfg.noise_persistence);
        return module;
    }
    if (noise_type == NoiseType::Billow) {
        auto module = std::make_unique<noise::module::Billow>();
        module->SetFrequency(1. / cfg.noise_scale);
        module->SetOctaveCount(cfg.noise_octaves);
        module->SetPersistence(cfg.noise_persistence);
        return module;
    }
    if (noise_type == NoiseType::RidgedMulti) {
        auto module = std::make_unique<noise::module::RidgedMulti>();
        module->SetFrequency(1. / cfg.noise_scale);
        module->SetOctaveCount(cfg.noise_octaves);
        return module;
    }
    if (noise_type == NoiseType::Voronoi) {
        auto module = std::make_unique<noise::module::Voronoi>();
        module->SetFrequency(1. / cfg.noise_scale);
        module->SetDisplacement(1.0);
        return module;
    }
    return nullptr;
}

Arachne::ExtrusionJunctions straight_noise_line(const FuzzySkinConfig &cfg, double slice_z)
{
    const coord_t width = scaled<coord_t>(0.45);
    Arachne::ExtrusionJunctions line{
        {Point(0, 0), width, 0},
        {Point(scaled<coord_t>(40.), 0), width, 0},
    };
    fuzzy_extrusion_line(line, slice_z, 0.2, cfg, false);
    return line;
}

} // namespace

TEST_CASE("Non-Ripple fuzzy noise forwards the configured public displacement",
          "[FuzzySkin][FuzzySkinNoiseReference]")
{
    const NoiseType noise_type = GENERATE(NoiseType::Perlin, NoiseType::Billow, NoiseType::RidgedMulti, NoiseType::Voronoi);
    CAPTURE(static_cast<int>(noise_type));

    FuzzySkinConfig cfg{};
    cfg.type              = FuzzySkinType::External;
    cfg.thickness         = scaled<coord_t>(0.25);
    cfg.point_distance    = scaled<coord_t>(0.75);
    cfg.fuzzy_first_layer = true;
    cfg.noise_type        = noise_type;
    cfg.noise_scale       = 1.0;
    cfg.noise_octaves     = 4;
    cfg.noise_persistence = 0.5;
    cfg.mode              = FuzzySkinMode::Displacement;

    constexpr double slice_z = 1.7;
    const auto actual = straight_noise_line(cfg, slice_z);
    REQUIRE(actual.size() > 10);
    const auto expected = reference_noise(noise_type, cfg);
    REQUIRE(expected != nullptr);

    size_t checked = 0;
    for (const auto &junction : actual) {
        const double x = unscale<double>(junction.p.x());
        if (x <= 1.0 || x >= 39.0)
            continue;
        const double expected_offset = expected->GetValue(x, 0., slice_z) * unscale<double>(cfg.thickness);
        CHECK(std::isfinite(expected_offset));
        CHECK_THAT(unscale<double>(junction.p.y()), WithinAbs(expected_offset, 3e-6));
        CHECK(junction.w == scaled<coord_t>(0.45));
        ++checked;
    }
    REQUIRE(checked > 10);
}

namespace fuzzy_noise_analytic_dispatch {

struct Probe {
    NoiseType type;
    double x0;
    double x1;
    double y;
    double minimum_noise;
    double maximum_noise;
};

FuzzySkinConfig one_octave_config(NoiseType type)
{
    FuzzySkinConfig cfg{};
    cfg.type              = FuzzySkinType::External;
    cfg.thickness         = scaled<coord_t>(0.5);
    cfg.point_distance    = scaled<coord_t>(0.002);
    cfg.fuzzy_first_layer = true;
    cfg.noise_type        = type;
    cfg.noise_scale       = 1.0;
    cfg.noise_octaves     = 1;
    cfg.noise_persistence = 0.5;
    cfg.mode              = FuzzySkinMode::Displacement;
    return cfg;
}

Points horizontal_probe(const Probe &probe)
{
    return {Point::new_scale(probe.x0, probe.y), Point::new_scale(probe.x1, probe.y)};
}

} // namespace fuzzy_noise_analytic_dispatch

TEST_CASE("One-octave fuzzy noise follows independent lattice-cell displacement bounds",
          "[FuzzySkin][FuzzySkinAnalyticDispatch]")
{
    using namespace fuzzy_noise_analytic_dispatch;
    const std::array<Probe, 4> probes{{
        // The libnoise 1.0 gradient interpolation on y=1,z=0 reduces to
        // g=2.12*((1-s)*a*x+s*b*(x-1)), s=3*x*x-2*x*x*x,
        // with table gradients a=-0.141751, b=0.976342. Over [0.55,0.65],
        // g lies in [-0.608,-0.575]; Billow is 2*abs(g)-0.5.
        {NoiseType::Billow,      0.55,  0.65,  1.0,  0.64,  0.72},
        {NoiseType::Perlin,      0.55,  0.65,  1.0, -0.62, -0.56},
        // On y=z=0 the gradients are a=-0.763874, b=-0.224209.
        // Over [0.001,0.010], g lies in [-0.01605,-0.00161], making
        // RidgedMulti=1.25*(1-abs(g))^2-1 lie in [0.2101,0.2460].
        {NoiseType::RidgedMulti, 0.001, 0.010, 0.0,  0.20,   0.26},
        {NoiseType::Perlin,      0.001, 0.010, 0.0, -0.018, -0.0005},
    }};
    const Probe &probe = probes[GENERATE(size_t(0), size_t(1), size_t(2), size_t(3))];
    CAPTURE(static_cast<int>(probe.type), probe.x0, probe.x1, probe.y);

    const FuzzySkinConfig cfg = one_octave_config(probe.type);
    Points actual = horizontal_probe(probe);
    fuzzy_polyline(actual, false, 0., cfg);
    REQUIRE(actual.size() >= 3);

    const double thickness = unscale<double>(cfg.thickness);
    for (const Point &point : actual) {
        const double x = unscale<double>(point.x());
        const double y = unscale<double>(point.y());
        CHECK(x >= probe.x0 - 2e-6);
        CHECK(x <= probe.x1 + 2e-6);
        // Sampling is random, but the bounds hold over each entire segment.
        // Positive Billow/RidgedMulti and negative Perlin controls distinguish
        // dispatch without constructing a reference libnoise module.
        CHECK(y > probe.y + probe.minimum_noise * thickness);
        CHECK(y < probe.y + probe.maximum_noise * thickness);
    }
}
