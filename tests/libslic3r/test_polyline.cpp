#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "libslic3r/Polyline.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/Line.hpp"
#include "libslic3r/ArcFitter.hpp"
#include "libslic3r/Circle.hpp"
#include "libslic3r/Exception.hpp"
#include "libslic3r/libslic3r.h"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <exception>
#include <limits>
#include <utility>
#include <vector>

using namespace Slic3r;
using Catch::Matchers::WithinAbs;
namespace {
Point p(double x, double y) { return Point(scaled(x), scaled(y)); }
Polyline elbow() { return Polyline{Points{p(0, 0), p(10, 0), p(10, 10)}}; }
Polyline arc()
{
    Polyline path{Points{p(10, 0), p(6, 8), p(0, 10)}};
    path.fitting_result.push_back(
        {0, 2, EMovePathType::Arc_move_ccw, ArcSegment(p(0, 0), scaled(10.), p(10, 0), p(0, 10), ArcDirection::Arc_Dir_CCW)});
    return path;
}
void require_linear_fit(const Polyline& path)
{
    REQUIRE(path.fitting_result.size() == 1);
    CHECK(path.fitting_result[0].path_type == EMovePathType::Linear_move);
    CHECK(path.fitting_result[0].start_point_index == 0);
    CHECK(path.fitting_result[0].end_point_index == path.points.size() - 1);
}
} // namespace

TEST_CASE("Polyline leftmost point selects the smallest horizontal coordinate", "[Polyline][PolylineOperations]")
{
    const Polyline path{Points{p(5, 2), p(-3, 7), p(8, 9), p(-1, -4)}};
    CHECK(path.leftmost_point() == p(-3, 7));
    CHECK(leftmost_point(Polylines{path, Polyline{Points{p(-5, 3), p(-4, 1)}}}) == p(-5, 3));
    CHECK_THROWS_AS(leftmost_point(Polylines{}), InvalidArgument);
}
TEST_CASE("Polyline index splitting preserves both endpoints and the shared vertex", "[Polyline][PolylineOperations]")
{
    const Polyline source = elbow();
    Polyline before, after;
    REQUIRE(source.split_at_index(1, &before, &after));
    CHECK(before.points == Points{p(0, 0), p(10, 0)});
    CHECK(after.points == Points{p(10, 0), p(10, 10)});
    CHECK(source.points == elbow().points);
}
TEST_CASE("Polyline index splitting at either endpoint retains the whole other side", "[Polyline][PolylineOperations]")
{
    const Polyline source = elbow();
    Polyline before, after;
    REQUIRE(source.split_at_index(0, &before, &after));
    CHECK(before.points == Points{source.first_point()});
    CHECK(after.points == source.points);
    REQUIRE(source.split_at_index(2, &before, &after));
    CHECK(before.points == source.points);
    CHECK(after.points == Points{source.last_point()});
}
TEST_CASE("Invalid polyline split indices leave both destination paths unchanged", "[Polyline][PolylineOperations]")
{
    const Polyline source = elbow();
    Polyline before = source, after = source;
    CHECK_FALSE(source.split_at_index(3, &before, &after));
    CHECK(before.points == source.points);
    CHECK(after.points == source.points);
}
TEST_CASE("Polyline length splitting interpolates within the first segment", "[Polyline][PolylineOperations]")
{
    const Polyline source = elbow();
    Polyline before, after;
    REQUIRE(source.split_at_length(scaled(4.), &before, &after));
    CHECK(before.points == Points{p(0, 0), p(4, 0)});
    CHECK(after.points == Points{p(4, 0), p(10, 0), p(10, 10)});
    CHECK_THAT(before.length() + after.length(), WithinAbs(source.length(), 1.));
}
TEST_CASE("Polyline length splitting interpolates after a bend", "[Polyline][PolylineOperations]")
{
    const Polyline source = elbow();
    Polyline before, after;
    REQUIRE(source.split_at_length(scaled(14.), &before, &after));
    CHECK(before.points == Points{p(0, 0), p(10, 0), p(10, 4)});
    CHECK(after.points == Points{p(10, 4), p(10, 10)});
}
TEST_CASE("Polyline length splitting at an existing vertex avoids duplicate points", "[Polyline][PolylineOperations]")
{
    const Polyline source = elbow();
    Polyline before, after;
    REQUIRE(source.split_at_length(scaled(10.), &before, &after));
    CHECK(before.points == Points{p(0, 0), p(10, 0)});
    CHECK(after.points == Points{p(10, 0), p(10, 10)});
}
TEST_CASE("Polyline length splitting accepts zero and total length", "[Polyline][PolylineOperations]")
{
    const Polyline source = elbow();
    Polyline before, after;
    REQUIRE(source.split_at_length(0., &before, &after));
    CHECK(before.points == Points{source.first_point()});
    CHECK(after.points == source.points);
    REQUIRE(source.split_at_length(source.length(), &before, &after));
    CHECK(before.points == source.points);
    CHECK(after.points == Points{source.last_point()});
}
TEST_CASE("Out of range lengths and empty paths do not overwrite split outputs", "[Polyline][PolylineOperations]")
{
    const Polyline source = elbow();
    Polyline before = source, after = source;
    CHECK_FALSE(source.split_at_length(-1., &before, &after));
    CHECK_FALSE(source.split_at_length(source.length() + 1., &before, &after));
    CHECK_FALSE(Polyline{}.split_at_length(0., &before, &after));
    CHECK(before.points == source.points);
    CHECK(after.points == source.points);
}
TEST_CASE("Splitting at a supplied polyline vertex returns that shared vertex", "[Polyline][PolylineOperations]")
{
    const Polyline source = elbow();
    Polyline before, after;
    Point at = p(10, 0);
    source.split_at(at, &before, &after);
    CHECK(at == p(10, 0));
    CHECK(before.points == Points{p(0, 0), p(10, 0)});
    CHECK(after.points == Points{p(10, 0), p(10, 10)});
}
TEST_CASE("Splitting inside a polyline segment inserts the requested shared point", "[Polyline][PolylineOperations]")
{
    const Polyline source = elbow();
    Polyline before, after;
    Point at = p(10, 4);
    source.split_at(at, &before, &after);
    CHECK(before.points == Points{p(0, 0), p(10, 0), p(10, 4)});
    CHECK(after.points == Points{p(10, 4), p(10, 10)});
}
TEST_CASE("Arc splitting retains curved paths and rebases their fitting indices", "[Polyline][PolylineOperations]")
{
    const Polyline source = arc();
    Polyline before, after;
    REQUIRE(source.split_at_index(1, &before, &after));
    REQUIRE(before.fitting_result.size() == 1);
    REQUIRE(after.fitting_result.size() == 1);
    for (const Polyline* part : {&before, &after}) {
        CHECK(part->fitting_result.front().path_type == EMovePathType::Arc_move_ccw);
        CHECK(part->fitting_result.front().start_point_index == 0);
        CHECK(part->fitting_result.front().end_point_index == 1);
        CHECK(part->fitting_result.front().arc_data.start_point == part->first_point());
        CHECK(part->fitting_result.front().arc_data.end_point == part->last_point());
    }
    CHECK(before.last_point() == after.first_point());
    CHECK_THAT(before.fitting_result[0].arc_data.length + after.fitting_result[0].arc_data.length,
               WithinAbs(source.fitting_result[0].arc_data.length, 1.));
}
TEST_CASE("Linear fitting metadata is rebased when a path is split", "[Polyline][PolylineOperations]")
{
    Polyline source = elbow();
    source.reset_to_linear_move();
    Polyline before, after;
    REQUIRE(source.split_at_index(1, &before, &after));
    require_linear_fit(before);
    require_linear_fit(after);
}
TEST_CASE("Reversing a curved polyline reverses direction and preserves arc length", "[Polyline][PolylineOperations]")
{
    Polyline path           = arc();
    const Polyline original = path;
    path.reverse();
    CHECK(path.first_point() == original.last_point());
    CHECK(path.last_point() == original.first_point());
    REQUIRE(path.fitting_result.size() == 1);
    CHECK(path.fitting_result[0].path_type == EMovePathType::Arc_move_cw);
    CHECK_THAT(path.fitting_result[0].arc_data.length, WithinAbs(original.fitting_result[0].arc_data.length, 1.));
    path.reverse();
    CHECK(path.points == original.points);
    CHECK(path.fitting_result[0].path_type == EMovePathType::Arc_move_ccw);
}
TEST_CASE("Appending a fitted polyline preserves its source and offsets fitting indices", "[Polyline][PolylineOperations]")
{
    Polyline destination{Points{p(-10, 0), p(10, 0)}};
    const Polyline source = arc();
    destination.append(source);
    CHECK(source.points == arc().points);
    CHECK(destination.points == Points{p(-10, 0), p(10, 0), p(6, 8), p(0, 10)});
    REQUIRE(destination.fitting_result.size() == 2);
    CHECK(destination.fitting_result[0].path_type == EMovePathType::Linear_move);
    CHECK(destination.fitting_result[0].start_point_index == 0);
    CHECK(destination.fitting_result[0].end_point_index == 1);
    CHECK(destination.fitting_result[1].path_type == EMovePathType::Arc_move_ccw);
    CHECK(destination.fitting_result[1].start_point_index == 1);
    CHECK(destination.fitting_result[1].end_point_index == 3);
}
TEST_CASE("Appending to an empty polyline copies all curved geometry", "[Polyline][PolylineOperations]")
{
    Polyline destination;
    const Polyline source = arc();
    destination.append(source);
    CHECK(destination.points == source.points);
    REQUIRE(destination.fitting_result.size() == 1);
    CHECK(destination.fitting_result[0].path_type == EMovePathType::Arc_move_ccw);
    destination.append(Polyline{Points{p(3, 4)}});
    CHECK(destination.points == source.points);
}
TEST_CASE("Appending ordinary points after an arc creates a linear continuation", "[Polyline][PolylineOperations]")
{
    Polyline path = arc();
    path.append(p(-2, 10));
    path.append(p(-4, 10));
    REQUIRE(path.fitting_result.size() == 2);
    CHECK(path.fitting_result[1].path_type == EMovePathType::Linear_move);
    CHECK(path.fitting_result[1].start_point_index == 2);
    CHECK(path.fitting_result[1].end_point_index == 4);
}
TEST_CASE("Clipping a polyline removes exactly the requested terminal distance", "[Polyline][PolylineOperations]")
{
    Polyline path = elbow();
    path.reset_to_linear_move();
    path.clip_end(scaled(4.));
    CHECK(path.points == Points{p(0, 0), p(10, 0), p(10, 6)});
    require_linear_fit(path);
    path.clip_start(scaled(3.));
    CHECK(path.points == Points{p(3, 0), p(10, 0), p(10, 6)});
    require_linear_fit(path);
    path.clip_end(scaled(30.));
    CHECK(path.points.empty());
    CHECK(path.fitting_result.empty());
}
TEST_CASE("Polyline duplicate cleanup preserves separated repeated vertices", "[Polyline][PolylineOperations]")
{
    Polyline path{Points{p(0, 0), p(0, 0), p(1, 0), p(1, 0), p(0, 0)}};
    CHECK(remove_same_neighbor(path));
    CHECK(path.points == Points{p(0, 0), p(1, 0), p(0, 0)});
    CHECK_FALSE(remove_same_neighbor(path));
    Polyline empty;
    CHECK_FALSE(remove_same_neighbor(empty));
}
TEST_CASE("Polyline collection cleanup removes paths with fewer than two vertices", "[Polyline][PolylineOperations]")
{
    Polylines paths{Polyline{}, elbow(), Polyline{Points{p(0, 0)}}};
    CHECK(remove_degenerate(paths));
    REQUIRE(paths.size() == 1);
    CHECK(paths.front().points == elbow().points);
    CHECK_FALSE(remove_degenerate(paths));
}
TEST_CASE("Polyline collection duplicate cleanup drops paths that collapse to one vertex", "[Polyline][PolylineOperations]")
{
    Polylines paths{elbow(), Polyline{Points{p(2, 2), p(2, 2)}}};
    CHECK(remove_same_neighbor(paths));
    REQUIRE(paths.size() == 1);
    CHECK(paths.front().points == elbow().points);
    CHECK_FALSE(remove_same_neighbor(paths));
}
TEST_CASE("Polyline projection identifies the closest segment and its clamped foot", "[Polyline][PolylineOperations]")
{
    const Points path   = elbow().points;
    const auto vertical = foot_pt(path, p(14, 6));
    CHECK(vertical.first == 1);
    CHECK(vertical.second == p(10, 6));
    const auto horizontal = foot_pt(path, p(3, -5));
    CHECK(horizontal.first == 0);
    CHECK(horizontal.second == p(3, 0));
    const auto endpoint = foot_pt(path, p(-8, -2));
    CHECK(endpoint.first == 0);
    CHECK(endpoint.second == p(0, 0));
    CHECK(foot_pt(Points{}, p(0, 0)).first == -1);
    CHECK(foot_pt(Points{p(0, 0)}, p(1, 1)).first == -1);
}
// Known bug: equally_spaced_lines() measures from the last split instead of along the path; fixed by the follow-up fix PR
TEST_CASE("Equally spaced line pieces partition a long straight segment", "[Polyline][PolylineOperations][!shouldfail]")
{
    const Polyline source{Points{p(0, 0), p(12, 0)}};
    const Polylines pieces = source.equally_spaced_lines(scaled(5.));
    REQUIRE(pieces.size() == 3);
    CHECK(pieces[0].points == Points{p(0, 0), p(5, 0)});
    CHECK(pieces[1].points == Points{p(5, 0), p(10, 0)});
    CHECK(pieces[2].points == Points{p(10, 0), p(12, 0)});
}
// Known bug: equally_spaced_lines() measures from the last split instead of along the path; fixed by the follow-up fix PR
TEST_CASE("Equally spaced line pieces preserve bends and total path length", "[Polyline][PolylineOperations][!shouldfail]")
{
    const Polyline source  = elbow();
    const Polylines pieces = source.equally_spaced_lines(scaled(6.));
    REQUIRE(pieces.size() == 4);
    double total = 0.;
    for (size_t i = 0; i < pieces.size(); ++i) {
        CHECK_THAT(pieces[i].length(), WithinAbs(scaled(i == 3 ? 2. : 6.), 2.));
        total += pieces[i].length();
        if (i > 0)
            CHECK(pieces[i - 1].last_point() == pieces[i].first_point());
    }
    CHECK_THAT(total, WithinAbs(source.length(), 3.));
    CHECK(pieces.front().first_point() == source.first_point());
    CHECK(pieces.back().last_point() == source.last_point());
    CHECK(pieces[1].points == Points{p(6, 0), p(10, 0), p(10, 2)});
}

TEST_CASE("Equally spaced line pieces handle exact boundaries and repeated vertices", "[Polyline][PolylineOperations]")
{
    const Polyline source{Points{p(0, 0), p(0, 0), p(5, 0), p(10, 0)}};
    const auto pieces = source.equally_spaced_lines(scaled(5.));
    REQUIRE(pieces.size() == 2);
    CHECK(pieces[0].points == Points{p(0, 0), p(5, 0)});
    CHECK(pieces[1].points == Points{p(5, 0), p(10, 0)});
    CHECK(Polyline{Points{p(0, 0)}}.equally_spaced_lines(scaled(5.)).empty());
    CHECK(Polyline{Points{p(0, 0), p(0, 0)}}.equally_spaced_lines(scaled(5.)).empty());
}
// Known bug: equally_spaced_lines() reads first_point() of an empty polyline (segfault); fixed by the follow-up fix PR
TEST_CASE("Equally spaced pieces of an empty polyline are empty", "[Polyline][PolylineOperations][.]")
{
    CHECK(Polyline{}.equally_spaced_lines(scaled(5.)).empty());
}
// Known bug: equally_spaced_lines() measures from the last split instead of along the path; fixed by the follow-up fix PR
TEST_CASE("Equally spaced line pieces retain a short path and its internal vertices", "[Polyline][PolylineOperations][!shouldfail]")
{
    const Polyline source = elbow();
    const auto pieces     = source.equally_spaced_lines(scaled(30.));
    REQUIRE(pieces.size() == 1);
    CHECK(pieces[0].points == source.points);
}
// Known bug: equally_spaced_lines() loops forever on a zero or negative distance, appending pieces that never
// advance along the path; fixed by the follow-up fix PR
TEST_CASE("Equally spaced pieces terminate for a non-positive distance", "[Polyline][PolylineOperations][.]")
{
    // Only termination is asserted: returning a result and reporting an error are both acceptable.
    const Polyline source = elbow();
    for (const double distance : {0., -1.}) {
        try {
            (void) source.equally_spaced_lines(distance);
        } catch (const std::exception &) {
        }
    }
    SUCCEED();
}
// Known bug: equally_spaced_lines() measures from the last split instead of along the path; fixed by the follow-up fix PR
TEST_CASE("Equally spaced line pieces preserve a diagonal path within coordinate rounding", "[Polyline][PolylineOperations][!shouldfail]")
{
    const Polyline source{Points{p(0, 0), p(9, 12)}};
    const auto pieces = source.equally_spaced_lines(scaled(4.));
    REQUIRE(pieces.size() == 4);
    double total = 0.;
    for (size_t i = 0; i < pieces.size(); ++i) {
        CHECK_THAT(pieces[i].length(), WithinAbs(scaled(i == 3 ? 3. : 4.), 2.));
        total += pieces[i].length();
        if (i > 0)
            CHECK(pieces[i - 1].last_point() == pieces[i].first_point());
    }
    CHECK_THAT(total, WithinAbs(source.length(), 3.));
    CHECK(pieces.back().last_point() == source.last_point());
}

// Known bug: equally_spaced_lines() measures from the last split instead of along the path; fixed by the follow-up fix PR
TEST_CASE("Unit spacing splits a straight path at every coordinate unit", "[Polyline][PolylineOperations][!shouldfail]")
{
    const Polyline source{Points{Point(0, 0), Point(10, 0)}};
    const auto pieces = source.equally_spaced_lines(1.);
    REQUIRE(pieces.size() == 10);
    for (size_t i = 0; i < pieces.size(); ++i) {
        CHECK(pieces[i].points == Points{Point(static_cast<coord_t>(i), 0), Point(static_cast<coord_t>(i + 1), 0)});
        if (i > 0)
            CHECK(pieces[i - 1].last_point() == pieces[i].first_point());
    }
}

// Known bug: equally_spaced_lines() loops forever when a sub-unit split rounds back onto the previous point;
// fixed by the follow-up fix PR
TEST_CASE("Sub-unit spacing terminates with a contiguous partition of non-empty pieces", "[Polyline][PolylineOperations][.]")
{
    // Pieces end on integer coordinates, so each non-empty piece is at least one unit long and a
    // 20 unit path yields at most 20 pieces.
    const Polyline source{Points{Point(0, 0), Point(10, 0), Point(10, 10)}};
    for (double distance : {0.4, std::numeric_limits<double>::min()}) {
        const auto pieces = source.equally_spaced_lines(distance);
        REQUIRE_FALSE(pieces.empty());
        CHECK(pieces.size() <= 20);
        CHECK(pieces.front().first_point() == source.first_point());
        CHECK(pieces.back().last_point() == source.last_point());
        for (size_t i = 0; i < pieces.size(); ++i) {
            CHECK(pieces[i].length() > 0.);
            if (i > 0)
                CHECK(pieces[i - 1].last_point() == pieces[i].first_point());
        }
    }
}

// Known bug: equally_spaced_lines() measures from the last split instead of along the path; fixed by the follow-up fix PR
TEST_CASE("A split rounding onto a bend does not consume the rest of its segment", "[Polyline][PolylineOperations][!shouldfail]")
{
    const Polyline source{Points{Point(0, 0), Point(3000000, 4000000), Point(100000000, 4000000)}};
    const double distance = 5000000.5;
    const auto pieces     = source.equally_spaced_lines(distance);
    REQUIRE(pieces.size() == 21);
    double total = 0.;
    for (size_t i = 0; i < pieces.size(); ++i) {
        if (i + 1 < pieces.size())
            CHECK_THAT(pieces[i].length(), WithinAbs(distance, 2.));
        else
            CHECK(pieces[i].length() <= distance + 2.);
        if (i > 0)
            CHECK(pieces[i - 1].last_point() == pieces[i].first_point());
        total += pieces[i].length();
    }
    CHECK_THAT(total, WithinAbs(source.length(), 2.));
    CHECK(pieces.front().first_point() == source.first_point());
    CHECK(pieces.back().last_point() == source.last_point());
}

TEST_CASE("Equally spaced pieces ignore zero-length runs between real segments", "[Polyline][PolylineOperations]")
{
    const Polyline source{Points{Point(0, 0), Point(0, 0), Point(4, 0), Point(4, 0), Point(8, 0)}};
    const auto pieces = source.equally_spaced_lines(4.);
    REQUIRE(pieces.size() == 2);
    CHECK(pieces[0].points == Points{Point(0, 0), Point(4, 0)});
    CHECK(pieces[1].points == Points{Point(4, 0), Point(8, 0)});
}

TEST_CASE("Foot projection past a repeated vertex selects the nearest real segment", "[Polyline][PolylineOperations]")
{
    const Points path{Point(0, 0), Point(0, 0), Point(10, 0)};
    const auto result = foot_pt(path, Point(6, 3));
    CHECK(result.first == 1);
    CHECK(result.second == Point(6, 0));
}

// Known bug: remove_degenerate() moves survivors' points but leaves their fitting_result behind; fixed by the follow-up fix PR
TEST_CASE("Compacting polylines preserves each surviving path's arc metadata", "[Polyline][PolylineOperations][!shouldfail]")
{
    const Polyline expected = arc();
    Polylines paths{Polyline{Points{p(0, 0)}}, expected, Polyline{}, elbow()};
    REQUIRE(remove_degenerate(paths));
    REQUIRE(paths.size() == 2);
    CHECK(paths[0].points == expected.points);
    CHECK(paths[1].points == elbow().points);
    REQUIRE(paths[0].fitting_result.size() == 1);
    CHECK(paths[1].fitting_result.empty());
    const auto& fit = paths[0].fitting_result.front();
    CHECK(fit.path_type == EMovePathType::Arc_move_ccw);
    CHECK(fit.start_point_index == 0);
    CHECK(fit.end_point_index == 2);
    CHECK(fit.arc_data.start_point == expected.first_point());
    CHECK(fit.arc_data.end_point == expected.last_point());
    CHECK_THAT(fit.arc_data.length, WithinAbs(expected.fitting_result.front().arc_data.length, 1.));
}

// Known bug: equally_spaced_lines() measures from the last split instead of along the path; fixed by the follow-up fix PR
TEST_CASE("Unit spacing stays exact at large bed coordinates", "[Polyline][PolylineOperations][!shouldfail]")
{
    // One metre from the origin in either direction, beyond any real bed.
    const coord_t magnitude = scaled<coord_t>(1000.);
    for (const coord_t origin : {magnitude, -magnitude}) {
        const Polyline source{Points{Point(origin, 37), Point(origin + 4096, 37)}};
        const auto pieces = source.equally_spaced_lines(1.);
        REQUIRE(pieces.size() == 4096);
        for (size_t i = 0; i < pieces.size(); ++i) {
            CHECK(pieces[i].points == Points{Point(origin + coord_t(i), 37), Point(origin + coord_t(i + 1), 37)});
            if (i > 0)
                CHECK(pieces[i - 1].last_point() == pieces[i].first_point());
        }
        CHECK(pieces.front().first_point() == source.first_point());
        CHECK(pieces.back().last_point() == source.last_point());
    }
}

// Known bug: equally_spaced_lines() truncates split points in absolute coordinates, so they round differently
// on the negative side of the origin; fixed by the follow-up fix PR
TEST_CASE("Translating a bent polyline far from the origin preserves its partitions", "[Polyline][PolylineOperations][!shouldfail]")
{
    const Polyline local{Points{Point(0, 0), Point(2048, 0), Point(2048, 2048)}};
    const coord_t magnitude = scaled<coord_t>(1000.);
    for (const double distance : {1., 128., 513.}) {
        const auto expected = local.equally_spaced_lines(distance);
        for (const coord_t origin : {magnitude, -magnitude}) {
            const Point shift(origin, -origin);
            Polyline translated = local;
            for (Point& point : translated.points)
                point += shift;
            const auto actual = translated.equally_spaced_lines(distance);
            REQUIRE(actual.size() == expected.size());
            for (size_t piece = 0; piece < actual.size(); ++piece) {
                REQUIRE(actual[piece].points.size() == expected[piece].points.size());
                for (size_t vertex = 0; vertex < actual[piece].points.size(); ++vertex)
                    CHECK(actual[piece].points[vertex] - shift == expected[piece].points[vertex]);
            }
        }
    }
}

namespace {
Point3 p3(double x, double y, double z)
{
    return Point3::new_scale(x, y, z);
}

void check_point3(const Point3& actual, double x, double y, double z)
{
    CHECK_THAT(unscaled(actual.x()), WithinAbs(x, 0.002));
    CHECK_THAT(unscaled(actual.y()), WithinAbs(y, 0.002));
    CHECK_THAT(unscaled(actual.z()), WithinAbs(z, 0.002));
}

Polyline3 fitted_polyline3()
{
    Polyline3 path{Points3{p3(0., 0., 1.), p3(2., 0., 2.), p3(4., 0., 4.), p3(6., 0., 7.)}};
    path.fitting_result.push_back({0, path.points.size() - 1, EMovePathType::Linear_move, ArcSegment()});
    return path;
}
} // namespace

TEST_CASE("Polyline3 clipping preserves nonzero Z and terminal distance", "[Polyline3][PolylineOperations][Polyline3Operations]")
{
    const Polyline3 source{Points3{p3(0., 0., 1.), p3(10., 0., 3.), p3(20., 0., 9.)}};
    const Points3 original_points = source.points;
    const double clip_distance = 2.5;
    Polyline3 clipped = source;

    clipped.clip_end(scaled(clip_distance));

    const double last_segment = std::sqrt(10. * 10. + 6. * 6.);
    const double fraction = (last_segment - clip_distance) / last_segment;
    check_point3(clipped.last_point(), 10. + 10. * fraction, 0., 3. + 6. * fraction);
    CHECK_THAT(unscaled(source.length() - clipped.length()), WithinAbs(clip_distance, 0.002));
    CHECK(clipped.points.size() == 3);
    CHECK(source.points == original_points);
}

TEST_CASE("Polyline3 length splitting conserves a three dimensional endpoint", "[Polyline3][PolylineOperations][Polyline3Operations]")
{
    const Polyline3 source{Points3{p3(0., 0., 1.), p3(3., 4., 13.)}};
    const Points3 original_points = source.points;
    Polyline3 before, after;

    REQUIRE(source.split_at_length(scaled(5.), &before, &after));

    const double fraction = 5. / 13.;
    check_point3(before.last_point(), 3. * fraction, 4. * fraction, 1. + 12. * fraction);
    CHECK(before.last_point() == after.first_point());
    CHECK_THAT(unscaled(before.length()), WithinAbs(5., 0.002));
    CHECK_THAT(unscaled(after.length()), WithinAbs(8., 0.002));
    CHECK_THAT(unscaled(before.length() + after.length()), WithinAbs(unscaled(source.length()), 0.002));
    CHECK(source.points == original_points);
}

TEST_CASE("Polyline3 index splitting and reversal rebase linear fitting metadata", "[Polyline3][PolylineOperations][Polyline3Operations]")
{
    const Polyline3 source = fitted_polyline3();
    Polyline3 before, after;
    REQUIRE(source.split_at_index(2, &before, &after));

    CHECK(before.points == Points3{p3(0., 0., 1.), p3(2., 0., 2.), p3(4., 0., 4.)});
    CHECK(after.points == Points3{p3(4., 0., 4.), p3(6., 0., 7.)});
    REQUIRE(before.fitting_result.size() == 1);
    REQUIRE(after.fitting_result.size() == 1);
    CHECK(before.fitting_result[0].path_type == EMovePathType::Linear_move);
    CHECK(after.fitting_result[0].path_type == EMovePathType::Linear_move);
    CHECK(before.fitting_result[0].start_point_index == 0);
    CHECK(before.fitting_result[0].end_point_index == 2);
    CHECK(after.fitting_result[0].start_point_index == 0);
    CHECK(after.fitting_result[0].end_point_index == 1);
    CHECK(before.last_point() == after.first_point());
    CHECK(source.points == fitted_polyline3().points);

    Polyline3 reversed = source;
    // Unequal spans make the metadata ordering observable, even for linear moves.
    reversed.fitting_result.clear();
    reversed.fitting_result.push_back({0, 1, EMovePathType::Linear_move, ArcSegment()});
    reversed.fitting_result.push_back({1, 3, EMovePathType::Linear_move, ArcSegment()});
    reversed.reverse();
    CHECK(reversed.first_point() == source.last_point());
    CHECK(reversed.last_point() == source.first_point());
    REQUIRE(reversed.fitting_result.size() == 2);
    CHECK(reversed.fitting_result[0].start_point_index == 0);
    CHECK(reversed.fitting_result[0].end_point_index == 2);
    CHECK(reversed.fitting_result[1].start_point_index == 2);
    CHECK(reversed.fitting_result[1].end_point_index == 3);
    reversed.reverse();
    CHECK(reversed.points == source.points);
    CHECK(reversed.fitting_result[0].start_point_index == 0);
    CHECK(reversed.fitting_result[0].end_point_index == 1);
    CHECK(reversed.fitting_result[1].start_point_index == 1);
    CHECK(reversed.fitting_result[1].end_point_index == 3);
}

TEST_CASE("Polyline3 appending fitted paths preserves source Z and fitting offsets", "[Polyline3][PolylineOperations][Polyline3Operations]")
{
    Polyline3 destination{Points3{p3(0., 0., 1.), p3(2., 0., 2.)}};
    destination.fitting_result.push_back({0, 1, EMovePathType::Linear_move, ArcSegment()});
    Polyline3 source{Points3{p3(2., 0., 2.), p3(4., 0., 4.), p3(6., 0., 7.)}};
    source.fitting_result.push_back({0, 2, EMovePathType::Linear_move, ArcSegment()});
    const auto source_points = source.points;
    const auto source_fitting = source.fitting_result;
    const size_t destination_size = destination.points.size();

    destination.append(source);

    CHECK(destination.points == Points3{p3(0., 0., 1.), p3(2., 0., 2.), p3(4., 0., 4.), p3(6., 0., 7.)});
    REQUIRE(destination.fitting_result.size() == 2);
    CHECK(destination.fitting_result[0].start_point_index == 0);
    CHECK(destination.fitting_result[0].end_point_index == destination_size - 1);
    CHECK(destination.fitting_result[1].start_point_index == destination_size - 1);
    CHECK(destination.fitting_result[1].end_point_index == destination.points.size() - 1);
    CHECK(destination.fitting_result[0].path_type == EMovePathType::Linear_move);
    CHECK(destination.fitting_result[1].path_type == EMovePathType::Linear_move);
    CHECK(source.points == source_points);
    REQUIRE(source.fitting_result.size() == source_fitting.size());
    for (size_t i = 0; i < source.fitting_result.size(); ++i) {
        CHECK(source.fitting_result[i].start_point_index == source_fitting[i].start_point_index);
        CHECK(source.fitting_result[i].end_point_index == source_fitting[i].end_point_index);
        CHECK(source.fitting_result[i].path_type == source_fitting[i].path_type);
    }
}
