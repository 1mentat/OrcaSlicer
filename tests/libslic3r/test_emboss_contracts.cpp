#include "libslic3r/Polygon.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/libslic3r.h"
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "catch2/catch_test_macros.hpp"
#include "catch2/matchers/catch_matchers.hpp"
#include "libslic3r/Emboss.hpp"

using namespace Slic3r;

namespace emboss_layout_contracts {

using Catch::Matchers::WithinAbs;

namespace {
Point xy(double x, double y) { return Point(scaled(x), scaled(y)); }
// A 20 mm square around the origin, counter-clockwise.
Polygon square() { return Polygon{Points{xy(-10, -10), xy(10, -10), xy(10, 10), xy(-10, 10)}}; }
// A text line along the square's bottom edge, starting at its midpoint.
Emboss::TextLine baseline() { return Emboss::TextLine{square(), PolygonPoint{0, xy(0, -10)}, 0.f}; }
// A 2-unit glyph box (in font units) centred at x = center.
BoundingBox glyph_box(int center) { return BoundingBox(Point(center - 1, -1), Point(center + 1, 1)); }
} // namespace

TEST_CASE("Emboss normals point outward along each side of a rectangle", "[Emboss][EmbossLayout]")
{
    const Polygon polygon = square();
    const PolygonPoints locations{{0, xy(0, -10)}, {1, xy(10, 0)}, {2, xy(0, 10)}, {3, xy(-10, 0)}};
    const auto angles = Emboss::calculate_angles(int32_t(scaled(1.)), locations, polygon);
    REQUIRE(angles.size() == 4);
    const std::vector<Vec2d> expected{{0, -1}, {1, 0}, {0, 1}, {-1, 0}};
    for (size_t i = 0; i < angles.size(); ++i) {
        CHECK_THAT(std::cos(angles[i]), WithinAbs(expected[i].x(), 1e-8));
        CHECK_THAT(std::sin(angles[i]), WithinAbs(expected[i].y(), 1e-8));
    }
}

TEST_CASE("Emboss normals bisect a right-angle corner", "[Emboss][EmbossLayout]")
{
    const double angle = Emboss::calculate_angle(int32_t(scaled(2.)), PolygonPoint{1, xy(10, -10)}, square());
    CHECK_THAT(std::cos(angle), WithinAbs(std::sqrt(0.5), 1e-8));
    CHECK_THAT(std::sin(angle), WithinAbs(-std::sqrt(0.5), 1e-8));
}

TEST_CASE("Emboss normal calculation handles crossing into either adjacent edge", "[Emboss][EmbossLayout]")
{
    // An 8 mm chord from 5 mm off the bottom edge's midpoint: one sample crosses the near corner and meets
    // the side edge where the 8 mm circle cuts it, sqrt(64 - 25) = sqrt(39) mm above the corner; the other
    // stays on the bottom edge 8 mm away. The normal is perpendicular to the line between the two samples,
    // so it tilts away from straight down (the bottom-edge normal and the clamp-to-corner result).
    const double rise = std::sqrt(39.);
    const double expected_tilt = std::atan2(-13., rise);  // -64.34 degrees
    SECTION("the forward sample crosses the right corner") {
        const double angle = Emboss::calculate_angle(int32_t(scaled(8.)), PolygonPoint{0, xy(5, -10)}, square());
        CHECK_THAT(angle, WithinAbs(expected_tilt, 1e-6));
    }
    SECTION("the reverse sample crosses the left corner") {
        const double angle = Emboss::calculate_angle(int32_t(scaled(8.)), PolygonPoint{0, xy(-5, -10)}, square());
        CHECK_THAT(angle, WithinAbs(-PI - expected_tilt, 1e-6)); // -115.66 degrees, the mirror image
    }
}

TEST_CASE("An empty list of emboss samples produces no normal angles", "[Emboss][EmbossLayout]")
{
    CHECK(Emboss::calculate_angles(int32_t(scaled(1.)), PolygonPoints{}, square()).empty());
    CHECK(Emboss::sample_slice(baseline(), BoundingBoxes{}, 1.).empty());
}

TEST_CASE("Emboss glyph centers sample both sides of the baseline origin", "[Emboss][EmbossLayout]")
{
    const BoundingBoxes boxes{glyph_box(-4), glyph_box(0), glyph_box(4)};
    const auto samples = Emboss::sample_slice(baseline(), boxes, 1.);
    REQUIRE(samples.size() == 3);
    for (size_t i = 0; i < samples.size(); ++i) {
        CHECK(samples[i].index == 0);
        CHECK(samples[i].point == xy(double(i) * 4. - 4., -10));
    }
}

TEST_CASE("Emboss glyph samples scale font coordinates into millimeters", "[Emboss][EmbossLayout]")
{
    const BoundingBoxes boxes{glyph_box(2), glyph_box(4), glyph_box(6)};
    const auto samples = Emboss::sample_slice(baseline(), boxes, 0.5);
    REQUIRE(samples.size() == 3);
    for (size_t i = 0; i < samples.size(); ++i)
        CHECK(samples[i].point == xy(double(i) + 1., -10));
}

TEST_CASE("Emboss sampling supports a line containing only left-aligned glyphs", "[Emboss][EmbossLayout]")
{
    const BoundingBoxes boxes{glyph_box(-6), glyph_box(-4), glyph_box(-2)};
    const auto samples = Emboss::sample_slice(baseline(), boxes, 1.);
    REQUIRE(samples.size() == 3);
    for (size_t i = 0; i < samples.size(); ++i)
        CHECK(samples[i].point == xy(double(i) * 2. - 6., -10));
}

TEST_CASE("Whitespace glyphs retain an existing baseline cursor", "[Emboss][EmbossLayout]")
{
    // A whitespace glyph has no box; it takes the previous glyph's position.
    const BoundingBoxes boxes{glyph_box(2), BoundingBox(), glyph_box(6)};
    const auto samples = Emboss::sample_slice(baseline(), boxes, 1.);
    REQUIRE(samples.size() == 3);
    CHECK(samples[0].point == xy(2, -10));
    CHECK(samples[1].point == samples[0].point);
    CHECK(samples[2].point == xy(6, -10));
}

TEST_CASE("Emboss glyph sampling crosses a corner at the requested chord distance", "[Emboss][EmbossLayout]")
{
    // A 13 mm chord from (0,-10) reaches a side edge sqrt(13^2 - 10^2) = sqrt(69) mm above its base.
    SECTION("a glyph right of the origin lands on the right edge") {
        const auto samples = Emboss::sample_slice(baseline(), BoundingBoxes{glyph_box(13)}, 1.);
        REQUIRE(samples.size() == 1);
        CHECK(samples[0].index == 1);
        CHECK(samples[0].point.x() == scaled(10.));
        CHECK_THAT(unscale<double>(samples[0].point.y()), WithinAbs(-10. + std::sqrt(69.), 2e-6));
    }
    SECTION("a glyph left of the origin lands on the left edge") {
        const auto samples = Emboss::sample_slice(baseline(), BoundingBoxes{glyph_box(-13)}, 1.);
        REQUIRE(samples.size() == 1);
        CHECK(samples[0].index == 3);
        CHECK(samples[0].point.x() == scaled(-10.));
        CHECK_THAT(unscale<double>(samples[0].point.y()), WithinAbs(-10. + std::sqrt(69.), 2e-6));
    }
}

} // namespace emboss_layout_contracts
