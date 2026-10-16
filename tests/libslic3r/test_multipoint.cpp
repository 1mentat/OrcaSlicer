#include <catch2/generators/catch_generators.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numeric>
#include <utility>
#include <vector>

#include "libslic3r/MultiPoint.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/libslic3r.h"

using namespace Slic3r;

namespace {

double effective_area(const Points &pts, size_t curr, size_t prev, size_t next)
{
    const Vec2d curr_to_next = (pts[next] - pts[curr]).cast<double>();
    const Vec2d prev_to_next = (pts[prev] - pts[curr]).cast<double>();
    return .5 * std::abs(cross2(curr_to_next, prev_to_next));
}

// Independent O(n^2) reference: remove the least effective-area active point,
// carry forward the monotone threshold, then retain endpoints and areas > tolerance.
Points reference_visivalingam(const Points &pts, double tolerance)
{
    REQUIRE(pts.size() >= 2);
    const size_t n = pts.size();
    std::vector<size_t> prev(n), next(n);
    std::iota(prev.begin(), prev.end(), 0);
    std::iota(next.begin(), next.end(), 0);
    std::vector<double> final_area(n, 0.);
    std::vector<uint8_t> active(n, 1);
    double minimum = -std::numeric_limits<double>::max();

    for (size_t i = 1; i + 1 < n; ++i) {
        prev[i] = i - 1;
        next[i] = i + 1;
    }
    while (true) {
        size_t selected = n;
        double selected_area = std::numeric_limits<double>::infinity();
        for (size_t i = 1; i + 1 < n; ++i) {
            if (!active[i])
                continue;
            const double area = effective_area(pts, i, prev[i], next[i]);
            if (area < selected_area || (area == selected_area && i < selected)) {
                selected = i;
                selected_area = area;
            }
        }
        if (selected == n)
            break;
        minimum = std::max(minimum, selected_area);
        final_area[selected] = minimum;
        active[selected] = 0;
        const size_t p = prev[selected], q = next[selected];
        if (p != 0)
            next[p] = q;
        if (q + 1 < n)
            prev[q] = p;
    }

    Points result;
    result.reserve(n);
    result.push_back(pts.front());
    for (size_t i = 1; i + 1 < n; ++i)
        if (final_area[i] > tolerance)
            result.push_back(pts[i]);
    result.push_back(pts.back());
    return result;
}

void check_exact(const Points &actual, const Points &expected)
{
    REQUIRE(actual == expected);
    REQUIRE(actual.size() == expected.size());
    REQUIRE(actual.front() == expected.front());
    REQUIRE(actual.back() == expected.back());
}

Points make_polyline()
{
    return { Point(0, 0), Point(10, 1), Point(20, 9), Point(30, 2), Point(40, 14),
              Point(50, 3), Point(60, 0) };
}

Points transform(const Points &in, coord_t dx, coord_t dy, int scale)
{
    Points out = in;
    for (Point &p : out)
        p = Point(p.x() * scale + dx, p.y() * scale + dy);
    return out;
}

} // namespace

TEST_CASE("Visvalingam keeps endpoints and handles two and three points", "[MultiPoint][Visvalingam]")
{
    const Points two = { Point(11, -4), Point(30, 9) };
    check_exact(MultiPoint::visivalingam(two, 0.), two);

    const Points collinear = { Point(0, 0), Point(10, 0), Point(20, 0) };
    check_exact(MultiPoint::visivalingam(collinear, 0.), reference_visivalingam(collinear, 0.));

    const Points corner = { Point(0, 0), Point(10, 8), Point(20, 0) };
    check_exact(MultiPoint::visivalingam(corner, 10.), reference_visivalingam(corner, 10.));
    check_exact(MultiPoint::visivalingam(corner, 100.), reference_visivalingam(corner, 100.));
    // The triangle has base 20 and height 8, hence area 80. Equality removes it.
    check_exact(MultiPoint::visivalingam(corner, 80.), Points{corner.front(), corner.back()});
}

TEST_CASE("Visvalingam matches an independent elimination oracle", "[MultiPoint][Visvalingam]")
{
    const Points input = make_polyline();
    const double tolerance = GENERATE(0., 1., 20., 80., 100000.);
    check_exact(MultiPoint::visivalingam(input, tolerance), reference_visivalingam(input, tolerance));
}

TEST_CASE("Visvalingam is translation invariant", "[MultiPoint][Visvalingam]")
{
    const Points input = make_polyline();
    const Points translated = transform(input, 123456, -765432, 1);
    const Points expected = transform(reference_visivalingam(input, 20.), 123456, -765432, 1);
    check_exact(MultiPoint::visivalingam(translated, 20.), expected);
}

TEST_CASE("Visvalingam scales tolerance with area", "[MultiPoint][Visvalingam]")
{
    const Points input = make_polyline();
    const Points scaled = transform(input, 0, 0, 3);
    const Points expected = transform(reference_visivalingam(input, 20.), 0, 0, 3);
    check_exact(MultiPoint::visivalingam(scaled, 20. * 9.), expected);
}

TEST_CASE("Visvalingam reverses the retained order", "[MultiPoint][Visvalingam]")
{
    const Points input = make_polyline();
    Points reversed = input;
    std::reverse(reversed.begin(), reversed.end());
    Points expected = reference_visivalingam(input, 20.);
    std::reverse(expected.begin(), expected.end());
    check_exact(MultiPoint::visivalingam(reversed, 20.), expected);
}

TEST_CASE("Visvalingam preserves a salient high-area turn", "[MultiPoint][Visvalingam]")
{
    const Points input = { Point(0, 0), Point(10, 1), Point(20, 2), Point(30, 40),
                           Point(40, 3), Point(50, 2), Point(60, 0) };
    const Points result = MultiPoint::visivalingam(input, 100.);
    check_exact(result, reference_visivalingam(input, 100.));
    CHECK(std::find(result.begin(), result.end(), Point(30, 40)) != result.end());
}

TEST_CASE("Visvalingam removes a shallow run before its neighbors", "[MultiPoint][Visvalingam]")
{
    const Points input = { Point(0, 0), Point(10, 2), Point(20, 4), Point(30, 5),
                           Point(40, 9), Point(50, 10), Point(60, 10) };
    check_exact(MultiPoint::visivalingam(input, 5.), reference_visivalingam(input, 5.));
}

TEST_CASE("Visvalingam is deterministic for repeated equal input", "[MultiPoint][Visvalingam]")
{
    const Points input = make_polyline();
    const Points first = MultiPoint::visivalingam(input, 20.);
    for (int i = 0; i < 4; ++i)
        check_exact(MultiPoint::visivalingam(input, 20.), first);
}

TEST_CASE("Visvalingam rejects only internal points at a large tolerance", "[MultiPoint][Visvalingam]")
{
    const Points input = make_polyline();
    const Points result = MultiPoint::visivalingam(input, 1e12);
    REQUIRE(result.size() == 2);
    CHECK(result.front() == input.front());
    CHECK(result.back() == input.back());
}

TEST_CASE("Visvalingam retains a high-area tail node", "[MultiPoint][Visvalingam]")
{
    const Points input = { Point(0, 0), Point(10, 1), Point(20, 2), Point(30, 3),
                           Point(40, 4), Point(50, 100), Point(60, 0) };
    const Points expected = reference_visivalingam(input, 20.);
    check_exact(MultiPoint::visivalingam(input, 20.), expected);
    REQUIRE(expected.size() >= 3);
    CHECK(expected[expected.size() - 2] == input[input.size() - 2]);
}
