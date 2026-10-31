#include <catch2/generators/catch_generators.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numeric>
#include <utility>
#include <vector>

#include "libslic3r/MultiPoint.hpp"
#include "libslic3r/Line.hpp"
#include "libslic3r/Polyline.hpp"
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/libslic3r.h"

using namespace Slic3r;
using Catch::Matchers::WithinAbs;

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

// At this tolerance the oracle drops (10, 1) and (50, 3) from make_polyline() but keeps the
// other three interior points, so neither an identity nor an endpoints-only result passes.
constexpr double partial_tolerance = 80.;

Points partial_reference(const Points &input)
{
    Points expected = reference_visivalingam(input, partial_tolerance);
    REQUIRE(expected.size() > 2);
    REQUIRE(expected.size() < input.size());
    return expected;
}

Points transform(const Points &in, coord_t dx, coord_t dy, int scale)
{
    Points out = in;
    for (Point &p : out)
        p = Point(p.x() * scale + dx, p.y() * scale + dy);
    return out;
}

} // namespace

// Known bug: visivalingam() indexes a reserved but empty area vector (segfault); fixed by the follow-up fix PR
TEST_CASE("Visvalingam keeps endpoints and handles two and three points", "[MultiPoint][Visvalingam][.]")
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

// Known bug: visivalingam() indexes a reserved but empty area vector (segfault), and its heap orders node
// addresses instead of being a min-heap on effective area; fixed by the follow-up fix PR
TEST_CASE("Visvalingam matches an independent elimination oracle", "[MultiPoint][Visvalingam][.]")
{
    const Points input = make_polyline();
    const double tolerance = GENERATE(0., 1., 20., 80., 100000.);
    check_exact(MultiPoint::visivalingam(input, tolerance), reference_visivalingam(input, tolerance));
}

// Known bug: visivalingam() indexes a reserved but empty area vector (segfault); fixed by the follow-up fix PR
TEST_CASE("Visvalingam is translation invariant", "[MultiPoint][Visvalingam][.]")
{
    const Points input = make_polyline();
    const Points translated = transform(input, 123456, -765432, 1);
    const Points expected = transform(partial_reference(input), 123456, -765432, 1);
    check_exact(MultiPoint::visivalingam(translated, partial_tolerance), expected);
}

// Known bug: visivalingam() indexes a reserved but empty area vector (segfault); fixed by the follow-up fix PR
TEST_CASE("Visvalingam scales tolerance with area", "[MultiPoint][Visvalingam][.]")
{
    const Points input = make_polyline();
    const Points scaled = transform(input, 0, 0, 3);
    const Points expected = transform(partial_reference(input), 0, 0, 3);
    check_exact(MultiPoint::visivalingam(scaled, partial_tolerance * 9.), expected);
}

// Known bug: visivalingam() indexes a reserved but empty area vector (segfault); fixed by the follow-up fix PR
TEST_CASE("Visvalingam reverses the retained order", "[MultiPoint][Visvalingam][.]")
{
    const Points input = make_polyline();
    Points reversed = input;
    std::reverse(reversed.begin(), reversed.end());
    Points expected = partial_reference(input);
    std::reverse(expected.begin(), expected.end());
    check_exact(MultiPoint::visivalingam(reversed, partial_tolerance), expected);
}

// Known bug: visivalingam() indexes a reserved but empty area vector (segfault), and its heap orders node
// addresses instead of being a min-heap on effective area; fixed by the follow-up fix PR
TEST_CASE("Visvalingam preserves a salient high-area turn", "[MultiPoint][Visvalingam][.]")
{
    const Points input = { Point(0, 0), Point(10, 1), Point(20, 2), Point(30, 40),
                           Point(40, 3), Point(50, 2), Point(60, 0) };
    const Points result = MultiPoint::visivalingam(input, 100.);
    check_exact(result, reference_visivalingam(input, 100.));
    CHECK(std::find(result.begin(), result.end(), Point(30, 40)) != result.end());
}

// Known bug: visivalingam() indexes a reserved but empty area vector (segfault), and its heap orders node
// addresses instead of being a min-heap on effective area; fixed by the follow-up fix PR
TEST_CASE("Visvalingam removes a shallow run before its neighbors", "[MultiPoint][Visvalingam][.]")
{
    const Points input = { Point(0, 0), Point(10, 2), Point(20, 4), Point(30, 5),
                           Point(40, 9), Point(50, 10), Point(60, 10) };
    check_exact(MultiPoint::visivalingam(input, 5.), reference_visivalingam(input, 5.));
}

// Known bug: visivalingam() indexes a reserved but empty area vector (segfault); fixed by the follow-up fix PR
TEST_CASE("Visvalingam is deterministic for repeated equal input", "[MultiPoint][Visvalingam][.]")
{
    const Points input = make_polyline();
    const Points expected = partial_reference(input);
    for (int i = 0; i < 4; ++i)
        check_exact(MultiPoint::visivalingam(input, partial_tolerance), expected);
}

// Known bug: visivalingam() indexes a reserved but empty area vector (segfault); fixed by the follow-up fix PR
TEST_CASE("Visvalingam rejects only internal points at a large tolerance", "[MultiPoint][Visvalingam][.]")
{
    const Points input = make_polyline();
    const Points result = MultiPoint::visivalingam(input, 1e12);
    REQUIRE(result.size() == 2);
    CHECK(result.front() == input.front());
    CHECK(result.back() == input.back());
}

// Known bug: visivalingam() indexes a reserved but empty area vector (segfault), and its heap orders node
// addresses instead of being a min-heap on effective area; fixed by the follow-up fix PR
TEST_CASE("Visvalingam retains a high-area tail node", "[MultiPoint][Visvalingam][.]")
{
    const Points input = { Point(0, 0), Point(10, 1), Point(20, 2), Point(30, 3),
                           Point(40, 4), Point(50, 100), Point(60, 0) };
    const Points expected = reference_visivalingam(input, 20.);
    check_exact(MultiPoint::visivalingam(input, 20.), expected);
    REQUIRE(expected.size() >= 3);
    CHECK(expected[expected.size() - 2] == input[input.size() - 2]);
}

TEST_CASE("MultiPoint tolerance lookup and duplicate cleanup preserve point identity", "[MultiPoint][MultiPointGap]")
{
    Polyline path{Point(0, 0), Point(100, 0), Point(100, 100)};
    CHECK(path.find_point(Point(100, 0)) == 1);
    CHECK(path.find_point(Point(101, 1), 2.0) == 1);
    CHECK(path.find_point(Point(102, 0), 2.0) == -1);
    CHECK(path.find_point(Point(103, 0), 2.0) == -1);
    CHECK(path.has_boundary_point(Point(50, 0)));
    // SCALED_EPSILON is 100 coordinate units; this offset is intentionally well outside it.
    CHECK_FALSE(path.has_boundary_point(Point(50, 1000)));

    Polyline duplicates{Point(0, 0), Point(10, 0), Point(10, 0), Point(10, 0), Point(20, 0)};
    CHECK(duplicates.has_duplicate_points());
    CHECK(duplicates.remove_duplicate_points());
    const Points expected_deduplicated{Point(0, 0), Point(10, 0), Point(20, 0)};
    CHECK(duplicates.points == expected_deduplicated);
    CHECK_FALSE(duplicates.has_duplicate_points());
    CHECK_FALSE(duplicates.remove_duplicate_points());
}

TEST_CASE("MultiPoint segment intersections return every hit and the nearest hit", "[MultiPoint][MultiPointGap]")
{
    // A vertical probe crosses both arms of a U, with its connecting side away from the probe.
    const Polyline path{Point(0, 0), Point(200, 0), Point(200, 100), Point(0, 100)};
    const Line probe(Point(50, -20), Point(50, 120));

    Point first;
    REQUIRE(path.intersection(probe, &first));
    CHECK(first == Point(50, 0));

    Point nearest;
    REQUIRE(path.first_intersection(probe, &nearest));
    CHECK(nearest == Point(50, 0));

    const Line reversed_probe(Point(50, 120), Point(50, -20));
    Point reverse_nearest;
    REQUIRE(path.first_intersection(reversed_probe, &reverse_nearest));
    CHECK(reverse_nearest == Point(50, 100));

    Points hits;
    REQUIRE(path.intersections(probe, &hits));
    REQUIRE(hits.size() == 2);
    CHECK(hits[0] == Point(50, 0));
    CHECK(hits[1] == Point(50, 100));

    const Line disjoint(Point(250, -20), Point(250, 120));
    CHECK_FALSE(path.intersection(disjoint, &first));
    CHECK_FALSE(path.first_intersection(disjoint, &nearest));
    CHECK_FALSE(path.intersections(disjoint, &hits));
    CHECK(hits.size() == 2);
}

TEST_CASE("MultiPoint line distance handles projections, endpoints, and degenerate segments", "[MultiPoint][MultiPointGap]")
{
    // The first traversal's nearest distance is 4; every second-traversal candidate is
    // strictly farther, so a first-half minimum mutation cannot be masked.
    const Points horizontal{Point(0, 0), Point(10, 0)};
    const Points above{Point(5, 4), Point(5, 7)};
    CHECK_THAT(MultiPoint::minimumDistanceBetweenLinesDefinedByPoints(horizontal, above),
               WithinAbs(4.0, 1e-9));
    CHECK_THAT(MultiPoint::minimumDistanceBetweenLinesDefinedByPoints(above, horizontal),
               WithinAbs(4.0, 1e-9));

    const Points point_segment{Point(0, 0), Point(0, 0)};
    const Points remote_point{Point(3, 4), Point(3, 4)};
    CHECK_THAT(MultiPoint::minimumDistanceBetweenLinesDefinedByPoints(point_segment, remote_point),
               WithinAbs(5.0, 1e-9));
}

TEST_CASE("MultiPoint concave hull keeps a concavity up to its tolerance boundary", "[MultiPoint][MultiPointGap]")
{
    // For the concavity point (6,2), the source's boundary ratio is exactly
    // 80 / sqrt(6^2 + 8^2) = 8.  The strict boundary keeps the point; a
    // slightly larger tolerance removes it.
    const Points concave{Point(0, 0), Point(10, 0), Point(10, 10), Point(6, 2), Point(0, 10)};
    const Points retained = MultiPoint::concave_hull_2d(concave, 0.0);
    CHECK(retained == concave);
    const Points boundary = MultiPoint::concave_hull_2d(concave, 8.0);
    CHECK(boundary == concave);
    const Points smoothed = MultiPoint::concave_hull_2d(concave, 8.1);
    const Points expected_smoothed{Point(0, 0), Point(10, 0), Point(10, 10), Point(0, 10)};
    CHECK(smoothed == expected_smoothed);
}

TEST_CASE("MultiPoint3 keeps Z coordinates while matching, rotating, and deduplicating", "[MultiPoint][MultiPointGap]")
{
    Polyline3 path(Points3{Point3(0, 0, 5), Point3(10, 0, 7), Point3(10, 10, 9), Point3(10, 10, 9)});
    CHECK(path.find_point(Point3(10, 0, 7)) == 1);
    CHECK(path.find_point(Point3(10, 1, 7), 2.0) == 1);
    CHECK(path.find_point(Point3(10, 1, 7), 1.0) == -1);
    CHECK(path.remove_duplicate_points());
    CHECK(path.points.size() == 3);
    CHECK_FALSE(path.remove_duplicate_points());

    const BoundingBox3 bounds = path.bounding_box();
    CHECK(bounds.defined);
    CHECK(bounds.min == Point3(0, 0, 5));
    CHECK(bounds.max == Point3(10, 10, 9));

    path.rotate(std::acos(-1.0) / 2.0, Point3(10, 0, 100));
    CHECK(path.first_point() == Point3(10, -10, 5));
    CHECK(path.points[1] == Point3(10, 0, 7));
    CHECK(path.points[2] == Point3(0, 0, 9));
}
