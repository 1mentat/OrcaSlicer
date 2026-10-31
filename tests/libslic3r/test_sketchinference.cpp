#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/catch_all.hpp>   // mainline OrcaSlicer ships Catch2 v3 (v2 was catch2/catch.hpp)
#include "libslic3r/CAD/SketchEngine.hpp"
#include "libslic3r/Point.hpp"
#include <cmath>
#include <math.h>
#include <vector>
using Catch::Approx;   // v3 scopes Approx into the Catch namespace; v2 had it at global scope

#include "libslic3r/CAD/SketchInference.hpp"

using namespace Slic3r;
using K = InferenceSnap::Kind;

static SketchEntity line(Vec2d a, Vec2d b)
{
    SketchEntity e; e.type = SketchEntity::Type::Line; e.p0 = a; e.p1 = b; return e;
}
static SketchEntity circle(Vec2d c, double r)
{
    SketchEntity e; e.type = SketchEntity::Type::Circle; e.center = c; e.p0 = c; e.radius = r; return e;
}

TEST_CASE("inference: cursor near a line endpoint snaps Coincident-able to it", "[inference]")
{
    std::vector<SketchEntity> ents = { line({0, 0}, {10, 0}) };
    auto s = infer_point_snap(ents, {10.3, 0.2}, 1.0);
    REQUIRE(s.kind == K::Endpoint);
    CHECK(s.entity == 0);
    CHECK(s.role == SketchPointRole::P1);
    CHECK((s.point - Vec2d(10, 0)).norm() == Approx(0.0).margin(1e-9));
}

TEST_CASE("inference: endpoint beats midpoint when both are in range", "[inference]")
{
    std::vector<SketchEntity> ents = { line({0, 0}, {2, 0}) };
    // Query equidistant-ish but closer to the endpoint: endpoint tier wins regardless.
    auto s = infer_point_snap(ents, {1.9, 0.0}, 5.0);
    CHECK(s.kind == K::Endpoint);
    CHECK(s.role == SketchPointRole::P1);
}

TEST_CASE("inference: midpoint of a line is detected", "[inference]")
{
    std::vector<SketchEntity> ents = { line({0, 0}, {10, 0}) };
    auto s = infer_point_snap(ents, {5.1, 0.1}, 0.5, /*include_origin=*/false);
    REQUIRE(s.kind == K::Midpoint);
    CHECK((s.point - Vec2d(5, 0)).norm() == Approx(0.0).margin(1e-9));
}

TEST_CASE("inference: circle centre and rim", "[inference]")
{
    std::vector<SketchEntity> ents = { circle({0, 0}, 5.0) };
    auto c = infer_point_snap(ents, {0.2, 0.1}, 1.0, false);
    CHECK(c.kind == K::Center);
    auto r = infer_point_snap(ents, {5.1, 0.0}, 1.0, false);
    REQUIRE(r.kind == K::OnEdge);
    CHECK((r.point - Vec2d(5, 0)).norm() == Approx(0.0).margin(1e-9));
}

TEST_CASE("inference: origin snap when nothing else is near", "[inference]")
{
    std::vector<SketchEntity> ents = { line({20, 20}, {30, 20}) };
    auto s = infer_point_snap(ents, {0.1, 0.1}, 1.0);
    REQUIRE(s.kind == K::Origin);
    CHECK(s.entity == -1);
    CHECK((s.point - Vec2d(0, 0)).norm() == Approx(0.0).margin(1e-9));
}

TEST_CASE("inference: nothing in range returns None and the raw query", "[inference]")
{
    std::vector<SketchEntity> ents = { line({0, 0}, {10, 0}) };
    auto s = infer_point_snap(ents, {50, 50}, 1.0, /*include_origin=*/false);
    CHECK(s.kind == K::None);
    CHECK((s.point - Vec2d(50, 50)).norm() == Approx(0.0).margin(1e-9));
}

TEST_CASE("inference: axis inference flags horizontal / vertical segments", "[inference]")
{
    CHECK(infer_axis_constraint({0, 0}, {10, 0.05}).value() == SketchConstraintType::Horizontal);
    CHECK(infer_axis_constraint({0, 0}, {0.05, 10}).value() == SketchConstraintType::Vertical);
    CHECK_FALSE(infer_axis_constraint({0, 0}, {10, 10}).has_value());   // 45 deg
    CHECK_FALSE(infer_axis_constraint({0, 0}, {0, 0}).has_value());     // degenerate
}

TEST_CASE("inference: perpendicular inferred for a connected square corner", "[inference]")
{
    std::vector<SketchEntity> ents = { line({0, 0}, {10, 0}), line({10, 0}, {10, 7}) };
    auto r = infer_relations(ents, 1);
    REQUIRE(r.size() == 1);
    CHECK(r[0].type == SketchConstraintType::Perpendicular);
    CHECK(r[0].ea == 0);
    CHECK(r[0].eb == 1);
}

TEST_CASE("inference: parallel inferred for connected collinear-ish lines", "[inference]")
{
    std::vector<SketchEntity> ents = { line({0, 0}, {10, 0}), line({10, 0}, {21, 0.1}) };
    auto r = infer_relations(ents, 1);
    REQUIRE(r.size() == 1);
    CHECK(r[0].type == SketchConstraintType::Parallel);
    CHECK(r[0].ea == 0);
    CHECK(r[0].eb == 1);
}

TEST_CASE("inference: two unconnected parallel lines infer nothing", "[inference]")
{
    std::vector<SketchEntity> ents = { line({0, 0}, {10, 0}), line({0, 5}, {10, 5}) };
    auto r = infer_relations(ents, 1);
    CHECK(r.empty());
}

TEST_CASE("inference: a corner outside tolerance infers nothing", "[inference]")
{
    std::vector<SketchEntity> ents = { line({0, 0}, {10, 0}), line({10, 0}, {15, 7}) };
    auto r = infer_relations(ents, 1);
    CHECK(r.empty());
}

TEST_CASE("inference: equal radius inferred for near-equal circles", "[inference]")
{
    auto r = infer_relations({ circle({0, 0}, 5.0), circle({30, 0}, 5.02) }, 1);
    REQUIRE(r.size() == 1);
    CHECK(r[0].type == SketchConstraintType::EqualRadius);
    CHECK(r[0].ea == 0);
    CHECK(r[0].eb == 1);

    auto r2 = infer_relations({ circle({0, 0}, 5.0), circle({30, 0}, 6.0) }, 1);
    CHECK(r2.empty());
}

TEST_CASE("inference: tangent inferred for a line meeting a circle tangentially", "[inference]")
{
    std::vector<SketchEntity> ents = { circle({0, 0}, 5.0), line({0, 5}, {10, 5}) };
    auto r = infer_relations(ents, 1);
    REQUIRE(r.size() == 1);
    CHECK(r[0].type == SketchConstraintType::Tangent);
    CHECK(r[0].ea == 0);
    CHECK(r[0].eb == 1);

    std::vector<SketchEntity> off = { circle({0, 0}, 5.0), line({0, 5}, {10, 9}) };
    CHECK(infer_relations(off, 1).empty());
}

TEST_CASE("inference: nothing inferred against a higher index", "[inference]")
{
    std::vector<SketchEntity> ents = { line({0, 0}, {10, 0}), line({10, 0}, {10, 7}) };
    auto r = infer_relations(ents, 0);
    CHECK(r.empty());
}

TEST_CASE("inference: degenerate entities are ignored", "[inference]")
{
    std::vector<SketchEntity> ents = { line({0, 0}, {10, 0}), line({10, 0}, {10, 0}) };
    auto r = infer_relations(ents, 1);
    CHECK(r.empty());
}

// The cap that keeps infer_relations linear rather than quadratic. Without it a drawing with
// many equal holes yields a constraint per PAIR: 200 equal circles produced ~20000 candidates,
// the batch was rejected as over-constrained, and the caller's one-at-a-time fallback then ran
// a solve per constraint -- which pinned the app at 95% of a core with the MCP socket
// unresponsive, and is what the corpus rung caught.
TEST_CASE("inference: at most one relation per rule per new entity", "[inference]")
{
    // 40 circles of the same radius; the 41st must not produce 40 EqualRadius constraints.
    std::vector<SketchEntity> ents;
    for (int i = 0; i < 41; ++i) {
        SketchEntity c;
        c.type = SketchEntity::Type::Circle;
        c.center = Vec2d(i * 20.0, 0.0);
        c.p0 = c.center;
        c.radius = 5.0;
        ents.push_back(c);
    }
    auto rels = infer_relations(ents, 40);
    CHECK(rels.size() == 1);
    CHECK(rels[0].type == SketchConstraintType::EqualRadius);
    CHECK(rels[0].eb == 40);
}

static SketchEntity arc(Vec2d c, double r, double start, double end)
{
    SketchEntity e;
    e.type = SketchEntity::Type::Arc;
    e.center = c;
    e.radius = r;
    e.start_angle = start;
    e.end_angle = end;
    e.p0 = c + r * Vec2d(std::cos(start), std::sin(start));
    e.p1 = c + r * Vec2d(std::cos(end), std::sin(end));
    return e;
}

TEST_CASE("Snapping projects the cursor onto wrapped and clockwise arcs", "[inference]")
{
    using Catch::Matchers::WithinAbs;
    const Vec2d center(8.0, -3.0);
    constexpr double radius = 5.0;
    auto rim = [&](double angle) -> Vec2d { return center + radius * Vec2d(std::cos(angle), std::sin(angle)); };
    auto near_rim = [&](double angle) -> Vec2d { return center + 5.3 * Vec2d(std::cos(angle), std::sin(angle)); };

    // atan2 reports 250 degrees as -110, below the stored 225..315 degree range; the projection
    // has to wrap the angle to see that the cursor is over the arc.
    const double wrapped_angle = 250.0 * M_PI / 180.0;
    const SketchEntity wrapped = arc(center, radius, 225.0 * M_PI / 180.0, 315.0 * M_PI / 180.0);
    const InferenceSnap wrapped_snap = infer_point_snap({wrapped}, near_rim(wrapped_angle), 0.5, false);
    REQUIRE(wrapped_snap.kind == K::OnEdge);
    CHECK(wrapped_snap.entity == 0);
    CHECK_THAT((wrapped_snap.point - rim(wrapped_angle)).norm(), WithinAbs(0.0, 1e-9));

    // The mid-arc snap of the same arc.
    const double mid_angle = 0.5 * (wrapped.start_angle + wrapped.end_angle);
    const InferenceSnap mid_snap = infer_point_snap({wrapped}, rim(mid_angle), 0.1, false);
    REQUIRE(mid_snap.kind == K::Midpoint);
    CHECK_THAT((mid_snap.point - rim(mid_angle)).norm(), WithinAbs(0.0, 1e-9));

    // A clockwise half arc (negative sweep) from 135 to -45 degrees passes through 90 degrees.
    const SketchEntity clockwise = arc(center, radius, 3.0 * M_PI / 4.0, -M_PI / 4.0);
    const InferenceSnap clockwise_snap = infer_point_snap({clockwise}, near_rim(M_PI / 2.0), 0.5, false);
    REQUIRE(clockwise_snap.kind == K::OnEdge);
    CHECK(clockwise_snap.entity == 0);
    CHECK_THAT((clockwise_snap.point - rim(M_PI / 2.0)).norm(), WithinAbs(0.0, 1e-9));
}

TEST_CASE("Snapping reaches point, ellipse and spline targets and ranks Center above OnEdge", "[inference]")
{
    using Catch::Matchers::WithinAbs;

    SketchEntity point;
    point.type = SketchEntity::Type::Point;
    point.p0 = Vec2d(2.0, 4.0);
    const InferenceSnap point_snap = infer_point_snap({point}, Vec2d(2.1, 4.1), 0.25, false);
    REQUIRE(point_snap.kind == K::Endpoint);
    CHECK(point_snap.entity == 0);
    CHECK(point_snap.role == SketchPointRole::P0);
    CHECK_THAT((point_snap.point - point.p0).norm(), WithinAbs(0.0, 1e-9));

    SketchEntity ellipse_arc;
    ellipse_arc.type = SketchEntity::Type::EllipseArc;
    ellipse_arc.center = Vec2d(20.0, 20.0);
    ellipse_arc.p0 = Vec2d(25.0, 21.0);
    ellipse_arc.p1 = Vec2d(22.0, 23.0);
    const InferenceSnap ellipse_arc_snap = infer_point_snap({ellipse_arc}, ellipse_arc.p1 + Vec2d(0.1, 0.0), 0.25, false);
    REQUIRE(ellipse_arc_snap.kind == K::Endpoint);
    CHECK(ellipse_arc_snap.entity == 0);
    CHECK(ellipse_arc_snap.role == SketchPointRole::P1);
    CHECK_THAT((ellipse_arc_snap.point - ellipse_arc.p1).norm(), WithinAbs(0.0, 1e-9));

    SketchEntity ellipse;
    ellipse.type = SketchEntity::Type::Ellipse;
    ellipse.center = Vec2d(30.0, 10.0);
    const InferenceSnap ellipse_snap = infer_point_snap({ellipse}, ellipse.center + Vec2d(0.1, 0.0), 0.25, false);
    REQUIRE(ellipse_snap.kind == K::Center);
    CHECK(ellipse_snap.entity == 0);
    CHECK(ellipse_snap.role == SketchPointRole::Center);
    CHECK_THAT((ellipse_snap.point - ellipse.center).norm(), WithinAbs(0.0, 1e-9));

    SketchEntity spline;
    spline.type = SketchEntity::Type::BSpline;
    spline.ctrl = {Vec2d(40.0, 0.0), Vec2d(42.0, 3.0), Vec2d(45.0, 0.0)};
    spline.p0 = spline.ctrl.front();
    spline.p1 = spline.ctrl.back();
    const InferenceSnap spline_snap = infer_point_snap({spline}, spline.p1 + Vec2d(-0.1, 0.0), 0.25, false);
    REQUIRE(spline_snap.kind == K::Endpoint);
    CHECK(spline_snap.entity == 0);
    CHECK(spline_snap.role == SketchPointRole::P1);
    CHECK_THAT((spline_snap.point - spline.p1).norm(), WithinAbs(0.0, 1e-9));

    // Priority outranks distance: the ellipse centre is farther from the cursor than the
    // circle-rim projection, but Center is preferred to OnEdge.
    ellipse.center = Vec2d(5.15, 0.0);
    const InferenceSnap priority_snap = infer_point_snap({circle(Vec2d(0.0, 0.0), 5.0), ellipse}, Vec2d(5.05, 0.0), 0.2, false);
    REQUIRE(priority_snap.kind == K::Center);
    CHECK(priority_snap.entity == 1);
    CHECK_THAT((priority_snap.point - ellipse.center).norm(), WithinAbs(0.0, 1e-9));
}

TEST_CASE("Tangency to an arc is inferred only at a shared endpoint", "[inference]")
{
    const SketchEntity quarter = arc(Vec2d(2.0, 3.0), 5.0, 0.0, M_PI / 2.0);

    // A horizontal line starting at the arc's top end continues it tangentially.
    const SketchEntity tangent = line(quarter.p1, quarter.p1 + Vec2d(10.0, 0.0));
    const auto inferred = infer_relations({quarter, tangent}, 1);
    REQUIRE(inferred.size() == 1);
    CHECK(inferred[0].type == SketchConstraintType::Tangent);
    CHECK(inferred[0].ea == 0);
    CHECK(inferred[0].eb == 1);

    // The same direction away from the arc is not a tangent relation.
    CHECK(infer_relations({quarter, line(Vec2d(12.0, 8.0), Vec2d(17.0, 8.0))}, 1).empty());
}
