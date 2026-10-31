#include <TopAbs_ShapeEnum.hxx>
#include <TopoDS_Wire.hxx>
#include <catch2/catch_all.hpp>   // mainline OrcaSlicer ships Catch2 v3 (v2 was catch2/catch.hpp)
#include <catch2/catch_message.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include "libslic3r/CAD/SketchEngine.hpp"
#include <cmath>
#include <algorithm>
#include <TopExp_Explorer.hxx>
#include <TopAbs.hxx>
#include <math.h>
#include <vector>
#include <cstddef>
#include "libslic3r/Point.hpp"

using namespace Slic3r;

using Catch::Matchers::WithinAbs;

// CONTRACT: mirror_entities hands the reflected half back REVERSED — the order of the entities
// and the direction of each — because a reflection reverses orientation and the result has to
// CONTINUE the chain it was made from. So a mirrored line's p0 is the reflection of the source's
// p1, not its p0. See [SketchProfile] "a mirrored half continues the original chain".
TEST_CASE("Mirror Line across Y axis (reversed: p0 is the reflection of the source p1)", "[SketchEdit]")
{
    SketchEntity e;
    e.type = SketchEntity::Type::Line;
    e.p0   = Vec2d(3, 2);
    e.p1   = Vec2d(5, 4);

    Vec2d a(0, -1);
    Vec2d b(0, 1);

    auto result = SketchEngine::mirror_entities({e}, a, b);
    REQUIRE(result.size() == 1);

    const auto& m = result[0];
    REQUIRE(m.type == SketchEntity::Type::Line);
    REQUIRE_THAT(m.p0.x(), WithinAbs(-5.0, 1e-9));   // reflection of the SOURCE p1
    REQUIRE_THAT(m.p0.y(), WithinAbs(4.0, 1e-9));
    REQUIRE_THAT(m.p1.x(), WithinAbs(-3.0, 1e-9));   // reflection of the SOURCE p0
    REQUIRE_THAT(m.p1.y(), WithinAbs(2.0, 1e-9));
}

TEST_CASE("Mirror Circle across Y axis", "[SketchEdit]")
{
    SketchEntity e;
    e.type   = SketchEntity::Type::Circle;
    e.center = Vec2d(5, 0);
    e.p0     = Vec2d(5, 0);
    e.radius = 3;

    Vec2d a(0, -1);
    Vec2d b(0, 1);

    auto result = SketchEngine::mirror_entities({e}, a, b);
    REQUIRE(result.size() == 1);

    const auto& m = result[0];
    REQUIRE(m.type == SketchEntity::Type::Circle);
    REQUIRE_THAT(m.center.x(), WithinAbs(-5.0, 1e-9));
    REQUIRE_THAT(m.center.y(), WithinAbs(0.0, 1e-9));
    REQUIRE_THAT(m.radius, WithinAbs(3.0, 1e-9));
    REQUIRE_THAT(m.p0.x(), WithinAbs(-5.0, 1e-9));
    REQUIRE_THAT(m.p0.y(), WithinAbs(0.0, 1e-9));
}

TEST_CASE("Mirror Arc across X axis", "[SketchEdit]")
{
    SketchEntity e;
    e.type        = SketchEntity::Type::Arc;
    e.center      = Vec2d(0, 0);
    e.radius      = 1.0;
    e.start_angle = 0.0;
    e.end_angle   = M_PI / 2.0;
    e.p0          = Vec2d(1, 0);
    e.p1          = Vec2d(0, 1);

    Vec2d a(-1, 0);
    Vec2d b(1, 0);

    auto result = SketchEngine::mirror_entities({e}, a, b);
    REQUIRE(result.size() == 1);

    const auto& m = result[0];
    REQUIRE(m.type == SketchEntity::Type::Arc);

    // Reversed with the rest of the half: the mirrored arc STARTS where the reflection of the
    // source's end is, and finishes at the reflection of its start.
    REQUIRE_THAT(m.p0.x(), WithinAbs(0.0, 1e-9));
    REQUIRE_THAT(m.p0.y(), WithinAbs(-1.0, 1e-9));
    REQUIRE_THAT(m.p1.x(), WithinAbs(1.0, 1e-9));
    REQUIRE_THAT(m.p1.y(), WithinAbs(0.0, 1e-9));

    // The reflection alone would negate the sweep; walking the arc the other way negates it
    // again, so a mirrored CCW arc is CCW once more and a mirrored CCW loop stays CCW.
    double sweep = m.end_angle - m.start_angle;
    double orig_sweep = e.end_angle - e.start_angle;
    REQUIRE(orig_sweep > 0.0);
    REQUIRE(sweep > 0.0);
}

TEST_CASE("Offset Line by positive d", "[SketchEdit]")
{
    SketchEntity e;
    e.type = SketchEntity::Type::Line;
    e.p0   = Vec2d(0, 0);
    e.p1   = Vec2d(10, 0);

    auto result = SketchEngine::offset_entities({e}, 2.0);
    REQUIRE(result.size() == 1);

    const auto& o = result[0];
    REQUIRE(o.type == SketchEntity::Type::Line);
    REQUIRE_THAT(o.p0.x(), WithinAbs(0.0, 1e-9));
    REQUIRE_THAT(o.p0.y(), WithinAbs(2.0, 1e-9));
    REQUIRE_THAT(o.p1.x(), WithinAbs(10.0, 1e-9));
    REQUIRE_THAT(o.p1.y(), WithinAbs(2.0, 1e-9));
}

// CONTRACT CHANGED: a circle follows the arc's "+d = left of travel" rule and counts as CCW, so
// +d shrinks it and -d grows it. It used to grow on +d, the opposite of the same outline drawn
// as a CCW chain of arcs.
TEST_CASE("Offset Circle: +d shrinks (a circle is CCW), -d grows, too far collapses", "[SketchEdit]")
{
    SketchEntity e;
    e.type   = SketchEntity::Type::Circle;
    e.center = Vec2d(0, 0);
    e.p0     = Vec2d(0, 0);
    e.radius = 5;

    auto inward = SketchEngine::offset_entities({e}, 2.0);
    REQUIRE(inward.size() == 1);
    REQUIRE_THAT(inward[0].radius, WithinAbs(3.0, 1e-9));

    auto outward = SketchEngine::offset_entities({e}, -2.0);
    REQUIRE(outward.size() == 1);
    REQUIRE_THAT(outward[0].radius, WithinAbs(7.0, 1e-9));

    auto collapsed = SketchEngine::offset_entities({e}, 5.0);
    REQUIRE(collapsed.empty());
}

TEST_CASE("Offset: a full circle and the same CCW outline drawn as arcs go the same way", "[SketchEdit]")
{
    SketchEntity c;
    c.type = SketchEntity::Type::Circle; c.center = Vec2d(0, 0); c.p0 = c.center; c.radius = 5;
    SketchEntity a0, a1;
    a0.type = a1.type = SketchEntity::Type::Arc;
    a0.center = a1.center = Vec2d(0, 0);
    a0.radius = a1.radius = 5;
    a0.start_angle = 0.0;  a0.end_angle = M_PI;       a0.p0 = Vec2d(5, 0);  a0.p1 = Vec2d(-5, 0);
    a1.start_angle = M_PI; a1.end_angle = 2.0 * M_PI; a1.p0 = Vec2d(-5, 0); a1.p1 = Vec2d(5, 0);
    const auto oc = SketchEngine::offset_entities({ c }, 1.0);
    const auto oa = SketchEngine::offset_entities({ a0, a1 }, 1.0);
    REQUIRE(oc.size() == 1);
    REQUIRE(oa.size() == 2);
    CHECK_THAT(oc[0].radius, WithinAbs(oa[0].radius, 1e-9));
    CHECK_THAT(oc[0].radius, WithinAbs(oa[1].radius, 1e-9));
}

// CONTRACT CHANGED: +d used to mean "radius + d" for every arc regardless of its sweep, while
// for a line it meant "left of the direction of travel". The two disagreed, so a profile made
// of lines AND arcs (any slot outline) offset with its straights going one way and its caps the
// other, and could never come back closed. The arc now follows the line's rule: +d is left of
// travel, which for this CCW quarter-arc is inward -> r = 3. See [SketchProfile].
TEST_CASE("Offset Arc by positive d (left of travel: a CCW arc shrinks)", "[SketchEdit]")
{
    SketchEntity e;
    e.type        = SketchEntity::Type::Arc;
    e.center      = Vec2d(0, 0);
    e.radius      = 4.0;
    e.start_angle = 0.0;
    e.end_angle   = M_PI / 2.0;
    e.p0          = Vec2d(4, 0);
    e.p1          = Vec2d(0, 4);

    auto result = SketchEngine::offset_entities({e}, 1.0);
    REQUIRE(result.size() == 1);

    const auto& o = result[0];
    REQUIRE(o.type == SketchEntity::Type::Arc);
    REQUIRE_THAT(o.radius, WithinAbs(3.0, 1e-9));
    REQUIRE_THAT(o.p0.x(), WithinAbs(3.0, 1e-9));
    REQUIRE_THAT(o.p0.y(), WithinAbs(0.0, 1e-9));
    REQUIRE_THAT(o.p1.x(), WithinAbs(0.0, 1e-9));
    REQUIRE_THAT(o.p1.y(), WithinAbs(3.0, 1e-9));
}

TEST_CASE("Fillet right-angle corner", "[SketchEdit]")
{
    SketchEntity a;
    a.type = SketchEntity::Type::Line;
    a.p0   = Vec2d(0, 0);
    a.p1   = Vec2d(10, 0);

    SketchEntity b;
    b.type = SketchEntity::Type::Line;
    b.p0   = Vec2d(10, 0);
    b.p1   = Vec2d(10, 10);

    SketchEntity a_out, b_out, arc_out;
    bool ok = SketchEngine::fillet_lines(a, b, 2.0, a_out, b_out, arc_out);
    REQUIRE(ok);

    REQUIRE_THAT(a_out.p0.x(), WithinAbs(0.0, 1e-9));
    REQUIRE_THAT(a_out.p0.y(), WithinAbs(0.0, 1e-9));
    REQUIRE_THAT(a_out.p1.x(), WithinAbs(8.0, 1e-9));
    REQUIRE_THAT(a_out.p1.y(), WithinAbs(0.0, 1e-9));

    REQUIRE_THAT(b_out.p0.x(), WithinAbs(10.0, 1e-9));
    REQUIRE_THAT(b_out.p0.y(), WithinAbs(2.0, 1e-9));
    REQUIRE_THAT(b_out.p1.x(), WithinAbs(10.0, 1e-9));
    REQUIRE_THAT(b_out.p1.y(), WithinAbs(10.0, 1e-9));

    REQUIRE(arc_out.type == SketchEntity::Type::Arc);
    REQUIRE_THAT(arc_out.radius, WithinAbs(2.0, 1e-9));
    REQUIRE_THAT(arc_out.center.x(), WithinAbs(8.0, 1e-9));
    REQUIRE_THAT(arc_out.center.y(), WithinAbs(2.0, 1e-9));
    REQUIRE_THAT((arc_out.p0 - arc_out.center).norm(), WithinAbs(2.0, 1e-9));
    REQUIRE_THAT((arc_out.p1 - arc_out.center).norm(), WithinAbs(2.0, 1e-9));
}

TEST_CASE("Fillet parallel lines returns false", "[SketchEdit]")
{
    SketchEntity a;
    a.type = SketchEntity::Type::Line;
    a.p0   = Vec2d(0, 0);
    a.p1   = Vec2d(10, 0);

    SketchEntity b;
    b.type = SketchEntity::Type::Line;
    b.p0   = Vec2d(0, 5);
    b.p1   = Vec2d(10, 5);

    SketchEntity a_out, b_out, arc_out;
    REQUIRE_FALSE(SketchEngine::fillet_lines(a, b, 1.0, a_out, b_out, arc_out));
}

TEST_CASE("Fillet arc too big returns false", "[SketchEdit]")
{
    SketchEntity a;
    a.type = SketchEntity::Type::Line;
    a.p0   = Vec2d(0, 0);
    a.p1   = Vec2d(1, 0);

    SketchEntity b;
    b.type = SketchEntity::Type::Line;
    b.p0   = Vec2d(1, 0);
    b.p1   = Vec2d(1, 1);

    SketchEntity a_out, b_out, arc_out;
    REQUIRE_FALSE(SketchEngine::fillet_lines(a, b, 5.0, a_out, b_out, arc_out));
}

TEST_CASE("Trim right arm", "[SketchEdit]")
{
    SketchEntity e;
    e.type = SketchEntity::Type::Line;
    e.p0   = Vec2d(-5, 0);
    e.p1   = Vec2d(5, 0);

    SketchEntity vc;
    vc.type = SketchEntity::Type::Line;
    vc.p0   = Vec2d(0, -5);
    vc.p1   = Vec2d(0, 5);

    bool ok = SketchEngine::trim_entity(e, {vc}, Vec2d(3, 0));
    REQUIRE(ok);
    REQUIRE_THAT(e.p0.x(), WithinAbs(-5.0, 1e-9));
    REQUIRE_THAT(e.p0.y(), WithinAbs(0.0, 1e-9));
    REQUIRE_THAT(e.p1.x(), WithinAbs(0.0, 1e-9));
    REQUIRE_THAT(e.p1.y(), WithinAbs(0.0, 1e-9));
}

TEST_CASE("Trim left arm", "[SketchEdit]")
{
    SketchEntity e;
    e.type = SketchEntity::Type::Line;
    e.p0   = Vec2d(-5, 0);
    e.p1   = Vec2d(5, 0);

    SketchEntity vc;
    vc.type = SketchEntity::Type::Line;
    vc.p0   = Vec2d(0, -5);
    vc.p1   = Vec2d(0, 5);

    bool ok = SketchEngine::trim_entity(e, {vc}, Vec2d(-3, 0));
    REQUIRE(ok);
    REQUIRE_THAT(e.p0.x(), WithinAbs(0.0, 1e-9));
    REQUIRE_THAT(e.p0.y(), WithinAbs(0.0, 1e-9));
    REQUIRE_THAT(e.p1.x(), WithinAbs(5.0, 1e-9));
    REQUIRE_THAT(e.p1.y(), WithinAbs(0.0, 1e-9));
}

TEST_CASE("Trim no cut (u out of range)", "[SketchEdit]")
{
    SketchEntity e;
    e.type = SketchEntity::Type::Line;
    e.p0   = Vec2d(-5, 0);
    e.p1   = Vec2d(5, 0);

    SketchEntity other;
    other.type = SketchEntity::Type::Line;
    other.p0   = Vec2d(0, 3);
    other.p1   = Vec2d(0, 8);

    REQUIRE_FALSE(SketchEngine::trim_entity(e, {other}, Vec2d(3, 0)));
}

TEST_CASE("Extend forward to line", "[SketchEdit]")
{
    SketchEntity e;
    e.type = SketchEntity::Type::Line;
    e.p0   = Vec2d(0, 0);
    e.p1   = Vec2d(2, 0);

    SketchEntity other;
    other.type = SketchEntity::Type::Line;
    other.p0   = Vec2d(5, -5);
    other.p1   = Vec2d(5, 5);

    bool ok = SketchEngine::extend_entity(e, {other}, Vec2d(2, 0));
    REQUIRE(ok);
    REQUIRE_THAT(e.p0.x(), WithinAbs(0.0, 1e-9));
    REQUIRE_THAT(e.p0.y(), WithinAbs(0.0, 1e-9));
    REQUIRE_THAT(e.p1.x(), WithinAbs(5.0, 1e-9));
    REQUIRE_THAT(e.p1.y(), WithinAbs(0.0, 1e-9));
}

TEST_CASE("Extend forward to circle", "[SketchEdit]")
{
    SketchEntity e;
    e.type = SketchEntity::Type::Line;
    e.p0   = Vec2d(0, 0);
    e.p1   = Vec2d(2, 0);

    SketchEntity other;
    other.type   = SketchEntity::Type::Circle;
    other.center = Vec2d(10, 0);
    other.p0     = Vec2d(10, 0);
    other.radius = 3;

    bool ok = SketchEngine::extend_entity(e, {other}, Vec2d(2, 0));
    REQUIRE(ok);
    REQUIRE_THAT(e.p0.x(), WithinAbs(0.0, 1e-9));
    REQUIRE_THAT(e.p0.y(), WithinAbs(0.0, 1e-9));
    REQUIRE_THAT(e.p1.x(), WithinAbs(7.0, 1e-9));
    REQUIRE_THAT(e.p1.y(), WithinAbs(0.0, 1e-9));
}

TEST_CASE("Extend backward", "[SketchEdit]")
{
    SketchEntity e;
    e.type = SketchEntity::Type::Line;
    e.p0   = Vec2d(0, 0);
    e.p1   = Vec2d(2, 0);

    SketchEntity other;
    other.type = SketchEntity::Type::Line;
    other.p0   = Vec2d(-3, -5);
    other.p1   = Vec2d(-3, 5);

    bool ok = SketchEngine::extend_entity(e, {other}, Vec2d(0, 0));
    REQUIRE(ok);
    REQUIRE_THAT(e.p0.x(), WithinAbs(-3.0, 1e-9));
    REQUIRE_THAT(e.p0.y(), WithinAbs(0.0, 1e-9));
    REQUIRE_THAT(e.p1.x(), WithinAbs(2.0, 1e-9));
    REQUIRE_THAT(e.p1.y(), WithinAbs(0.0, 1e-9));
}

TEST_CASE("Extend no target", "[SketchEdit]")
{
    SketchEntity e;
    e.type = SketchEntity::Type::Line;
    e.p0   = Vec2d(0, 0);
    e.p1   = Vec2d(2, 0);

    SketchEntity other;
    other.type = SketchEntity::Type::Line;
    other.p0   = Vec2d(5, -5);
    other.p1   = Vec2d(5, -1);

    REQUIRE_FALSE(SketchEngine::extend_entity(e, {other}, Vec2d(2, 0)));
}

// --- Arc/Circle subject trim & extend (Fase 4.5 kernel) -------------------

TEST_CASE("Trim arc drops the picked (start) side", "[SketchEdit]")
{
    // Upper semicircle r=5, ccw from (5,0) to (-5,0); cutter = vertical axis.
    SketchEntity e;
    e.type        = SketchEntity::Type::Arc;
    e.center      = Vec2d(0, 0);
    e.radius      = 5;
    e.start_angle = 0.0;
    e.end_angle   = M_PI;

    SketchEntity cut;
    cut.type = SketchEntity::Type::Line;
    cut.p0   = Vec2d(0, -10);
    cut.p1   = Vec2d(0, 10);

    // Pick the right quarter (phi=pi/4) -> it is removed, left quarter kept.
    bool ok = SketchEngine::trim_entity(e, {cut}, Vec2d(5 * std::cos(M_PI/4), 5 * std::sin(M_PI/4)));
    REQUIRE(ok);
    REQUIRE(e.type == SketchEntity::Type::Arc);
    REQUIRE_THAT(e.radius, WithinAbs(5.0, 1e-9));
    REQUIRE_THAT(e.start_angle, WithinAbs(M_PI / 2.0, 1e-9));
    REQUIRE_THAT(e.end_angle,   WithinAbs(M_PI, 1e-9));
    // The stored endpoints follow the angles: everything downstream reads p0/p1.
    CHECK_THAT(e.p0.x(), WithinAbs(0.0, 1e-9));  CHECK_THAT(e.p0.y(), WithinAbs(5.0, 1e-9));
    CHECK_THAT(e.p1.x(), WithinAbs(-5.0, 1e-9)); CHECK_THAT(e.p1.y(), WithinAbs(0.0, 1e-9));
}

TEST_CASE("Trim arc drops the picked (end) side", "[SketchEdit]")
{
    SketchEntity e;
    e.type        = SketchEntity::Type::Arc;
    e.center      = Vec2d(0, 0);
    e.radius      = 5;
    e.start_angle = 0.0;
    e.end_angle   = M_PI;

    SketchEntity cut;
    cut.type = SketchEntity::Type::Line;
    cut.p0   = Vec2d(0, -10);
    cut.p1   = Vec2d(0, 10);

    // Pick the left quarter (phi=3pi/4) -> removed, right quarter kept.
    bool ok = SketchEngine::trim_entity(e, {cut}, Vec2d(5 * std::cos(3*M_PI/4), 5 * std::sin(3*M_PI/4)));
    REQUIRE(ok);
    REQUIRE(e.type == SketchEntity::Type::Arc);
    REQUIRE_THAT(e.start_angle, WithinAbs(0.0, 1e-9));
    REQUIRE_THAT(e.end_angle,   WithinAbs(M_PI / 2.0, 1e-9));
    CHECK_THAT(e.p0.x(), WithinAbs(5.0, 1e-9)); CHECK_THAT(e.p0.y(), WithinAbs(0.0, 1e-9));
    CHECK_THAT(e.p1.x(), WithinAbs(0.0, 1e-9)); CHECK_THAT(e.p1.y(), WithinAbs(5.0, 1e-9));
}

TEST_CASE("Trim circle opens into an arc excluding the pick", "[SketchEdit]")
{
    // Full circle r=5; vertical axis cuts it at (0,+-5). Pick the right side
    // (5,0): the kept arc is the left half, sweeping pi and centred on (-5,0).
    SketchEntity e;
    e.type   = SketchEntity::Type::Circle;
    e.center = Vec2d(0, 0);
    e.p0     = e.center;          // circle convention: p0 mirrors the centre
    e.radius = 5;

    SketchEntity cut;
    cut.type = SketchEntity::Type::Line;
    cut.p0   = Vec2d(0, -10);
    cut.p1   = Vec2d(0, 10);

    bool ok = SketchEngine::trim_entity(e, {cut}, Vec2d(5, 0));
    REQUIRE(ok);
    REQUIRE(e.type == SketchEntity::Type::Arc);
    REQUIRE_THAT(e.radius, WithinAbs(5.0, 1e-9));
    REQUIRE_THAT(e.end_angle - e.start_angle, WithinAbs(M_PI, 1e-9));
    // Midpoint of the kept arc must point left (away from the pick).
    double mid = 0.5 * (e.start_angle + e.end_angle);
    REQUIRE_THAT(5 * std::cos(mid), WithinAbs(-5.0, 1e-9));
    REQUIRE_THAT(5 * std::sin(mid), WithinAbs(0.0, 1e-9));
    // p0 was the circle's centre; as an arc it must be the arc's start, on the rim.
    CHECK_THAT((e.p0 - e.center).norm(), WithinAbs(5.0, 1e-9));
    CHECK_THAT((e.p1 - e.center).norm(), WithinAbs(5.0, 1e-9));
    CHECK_THAT(e.p0.x(), WithinAbs(5.0 * std::cos(e.start_angle), 1e-9));
    CHECK_THAT(e.p1.x(), WithinAbs(5.0 * std::cos(e.end_angle), 1e-9));
}

TEST_CASE("Extend arc forward (end) to a crossing", "[SketchEdit]")
{
    // Quarter arc (5,0)->(0,5); cutter crosses the circle at (-5,0). Picking
    // near the end grows the sweep ccw to pi.
    SketchEntity e;
    e.type        = SketchEntity::Type::Arc;
    e.center      = Vec2d(0, 0);
    e.radius      = 5;
    e.start_angle = 0.0;
    e.end_angle   = M_PI / 2.0;

    SketchEntity cut;
    cut.type = SketchEntity::Type::Line;
    cut.p0   = Vec2d(-10, 0);
    cut.p1   = Vec2d(0, 0);

    bool ok = SketchEngine::extend_entity(e, {cut}, Vec2d(0, 5));
    REQUIRE(ok);
    REQUIRE(e.type == SketchEntity::Type::Arc);
    REQUIRE_THAT(e.start_angle, WithinAbs(0.0, 1e-9));
    REQUIRE_THAT(e.end_angle,   WithinAbs(M_PI, 1e-9));
    CHECK_THAT(e.p1.x(), WithinAbs(-5.0, 1e-9)); CHECK_THAT(e.p1.y(), WithinAbs(0.0, 1e-9));
}

TEST_CASE("Extend arc backward (start) to a crossing", "[SketchEdit]")
{
    // Quarter arc (0,5)->(-5,0); cutter crosses at (5,0). Picking near the
    // start grows the sweep cw to start_angle 0.
    SketchEntity e;
    e.type        = SketchEntity::Type::Arc;
    e.center      = Vec2d(0, 0);
    e.radius      = 5;
    e.start_angle = M_PI / 2.0;
    e.end_angle   = M_PI;

    SketchEntity cut;
    cut.type = SketchEntity::Type::Line;
    cut.p0   = Vec2d(10, 0);
    cut.p1   = Vec2d(0, 0);

    bool ok = SketchEngine::extend_entity(e, {cut}, Vec2d(0, 5));
    REQUIRE(ok);
    REQUIRE_THAT(e.start_angle, WithinAbs(0.0, 1e-9));
    REQUIRE_THAT(e.end_angle,   WithinAbs(M_PI, 1e-9));
}

TEST_CASE("Extend circle returns false (closed)", "[SketchEdit]")
{
    SketchEntity e;
    e.type   = SketchEntity::Type::Circle;
    e.center = Vec2d(0, 0);
    e.p0     = e.center;          // circle convention: p0 mirrors the centre
    e.radius = 5;

    SketchEntity cut;
    cut.type = SketchEntity::Type::Line;
    cut.p0   = Vec2d(0, -10);
    cut.p1   = Vec2d(0, 10);

    REQUIRE_FALSE(SketchEngine::extend_entity(e, {cut}, Vec2d(5, 0)));
}

TEST_CASE("Trim arc with no crossing returns false", "[SketchEdit]")
{
    SketchEntity e;
    e.type        = SketchEntity::Type::Arc;
    e.center      = Vec2d(0, 0);
    e.radius      = 5;
    e.start_angle = 0.0;
    e.end_angle   = M_PI / 2.0;

    SketchEntity cut;          // far away, never reaches the r=5 circle
    cut.type = SketchEntity::Type::Line;
    cut.p0   = Vec2d(20, -5);
    cut.p1   = Vec2d(20, 5);

    REQUIRE_FALSE(SketchEngine::trim_entity(e, {cut}, Vec2d(5 * std::cos(M_PI/4), 5 * std::sin(M_PI/4))));
}

// Regression guard: BEFORE the weld fix this test failed with 4 edges instead of 6.
// BRepLib_MakeWire::Add silently DROPS a disconnected edge (BRepLib_DisconnectedWire + NotDone)
// yet every successful Add ends with BRepLib_WireDone + Done(), so IsDone() reported only whether
// the LAST edge connected. This sketch is a real user loop (2 arcs + 4 lines) given in
// creation order, which is NOT traversal order, and its joint between the 3rd and 4th entity
// below is open by 2.28e-5 mm — larger than OCCT's default vertex tolerance.
TEST_CASE("entities_to_wires keeps every edge of a loop drawn out of order", "[SketchEngine]")
{
    std::vector<SketchEntity> ents(6);

    ents[0].type = SketchEntity::Type::Line;
    ents[0].p0   = Vec2d(-0.537697713190522, -0.0009077462579133498);
    ents[0].p1   = Vec2d(99.46230228680926, -0.0009141694814321626);

    ents[1].type        = SketchEntity::Type::Arc;
    ents[1].p0          = Vec2d(-0.537697713190522, -0.0009077462579133498);
    ents[1].p1          = Vec2d(-100.14602636660666, -0.27673132181233495);
    ents[1].center      = Vec2d(-50.313868583115394, -10.248112903179617);
    ents[1].radius      = 50.81999999999999;
    ents[1].start_angle = 0.20302922018398933;
    ents[1].end_angle   = 2.944101582158999;

    ents[2].type = SketchEntity::Type::Line;
    ents[2].p0   = Vec2d(99.46228668626469, -39.22091416947833);
    ents[2].p1   = Vec2d(-0.537864366432629, -39.22420589376945);

    ents[3].type        = SketchEntity::Type::Arc;
    ents[3].p0          = Vec2d(-100.14602636694521, -38.94673132181234);
    ents[3].p1          = Vec2d(-0.5378420354900413, -39.22421049164698);
    ents[3].center      = Vec2d(-50.31377078666179, -28.975499083790503);
    ents[3].radius      = 50.82006659345552;
    ents[3].start_angle = -2.944104841286737;
    ents[3].end_angle   = -0.20305921095748136;

    ents[4].type = SketchEntity::Type::Line;
    ents[4].p0   = Vec2d(99.46228668626469, -39.22091416947833);
    ents[4].p1   = Vec2d(99.46230228680926, -0.0009141694814321626);

    ents[5].type = SketchEntity::Type::Line;
    ents[5].p0   = Vec2d(-100.14602636694521, -38.94673132181234);
    ents[5].p1   = Vec2d(-100.14602636660666, -0.27673132181233495);

    auto wires = SketchEngine::entities_to_wires(ents, SketchPlane::XY());

    REQUIRE(wires.size() == 1);

    int edge_count = 0;
    for (TopExp_Explorer ex(wires[0], TopAbs_EDGE); ex.More(); ex.Next())
        ++edge_count;
    REQUIRE(edge_count == 6);

    REQUIRE(wires[0].Closed());
}

// Regression guard: this fails at 1e-4 (the wire builder refuses a joint the viewport had
// already shaded closed) and passes at kSketchJoinTol. A 20x10 quad with one joint left open
// by 9e-4 mm — just inside kSketchJoinTol, exactly the case the viewport shades closed — given
// in an order that is NOT traversal order, so the ordering path is covered too.
TEST_CASE("a loop the viewport shades closed is buildable by the kernel", "[SketchEngine]")
{
    std::vector<SketchEntity> ents(4);

    // (0,0) -> (20,0) -> (20,10) -> (0,10) -> (0.0009, 0): last endpoint misses (0,0) by 9e-4.
    ents[0].type = SketchEntity::Type::Line;
    ents[0].p0   = Vec2d(0, 0);
    ents[0].p1   = Vec2d(20, 0);

    // Index 1 is the FAR side, not the neighbour of index 0: creation order here is
    // deliberately not traversal order, so a partial wire would reject it without the
    // traversal walk.
    ents[1].type = SketchEntity::Type::Line;
    ents[1].p0   = Vec2d(20, 10);
    ents[1].p1   = Vec2d(0, 10);

    ents[2].type = SketchEntity::Type::Line;
    ents[2].p0   = Vec2d(20, 0);
    ents[2].p1   = Vec2d(20, 10);

    ents[3].type = SketchEntity::Type::Line;
    ents[3].p0   = Vec2d(0, 10);
    ents[3].p1   = Vec2d(0.0009, 0);

    auto wires = SketchEngine::entities_to_wires(ents, SketchPlane::XY());

    REQUIRE(wires.size() == 1);

    int edge_count = 0;
    for (TopExp_Explorer ex(wires[0], TopAbs_EDGE); ex.More(); ex.Next())
        ++edge_count;
    REQUIRE(edge_count == 4);

    REQUIRE(wires[0].Closed());
}

// Regression guard for the auto-close preference. Same 20x10 quad, one joint open by 9e-4 mm
// and given out of traversal order, as "a loop the viewport shades closed is buildable by the
// kernel". With auto-close ON the gap welds (one closed wire); with auto-close OFF it must not.
TEST_CASE("auto-close off makes the kernel demand an exact joint", "[SketchEngine]")
{
    std::vector<SketchEntity> ents(4);

    ents[0].type = SketchEntity::Type::Line;
    ents[0].p0   = Vec2d(0, 0);
    ents[0].p1   = Vec2d(20, 0);

    ents[1].type = SketchEntity::Type::Line;
    ents[1].p0   = Vec2d(20, 10);
    ents[1].p1   = Vec2d(0, 10);

    ents[2].type = SketchEntity::Type::Line;
    ents[2].p0   = Vec2d(20, 0);
    ents[2].p1   = Vec2d(20, 10);

    ents[3].type = SketchEntity::Type::Line;
    ents[3].p0   = Vec2d(0, 10);
    ents[3].p1   = Vec2d(0.0009, 0);

    auto edge_count = [](const TopoDS_Wire& w) {
        int n = 0;
        for (TopExp_Explorer ex(w, TopAbs_EDGE); ex.More(); ex.Next()) ++n;
        return n;
    };

    // ON: the 9e-4 mm gap is inside kSketchJoinTol, so the loop welds into one closed wire.
    Slic3r::set_sketch_auto_close(true);
    auto wires_on = SketchEngine::entities_to_wires(ents, SketchPlane::XY());
    REQUIRE(wires_on.size() == 1);
    REQUIRE(edge_count(wires_on[0]) == 4);
    REQUIRE(wires_on[0].Closed());

    // OFF: the joint is not exact, so the gap is NOT welded. entities_to_wires legitimately
    // returns open chains (a sweep path is open), so the observable is an OPEN wire — the
    // kernel no longer hands back the closed loop the viewport would have shaded.
    Slic3r::set_sketch_auto_close(false);
    auto wires_off = SketchEngine::entities_to_wires(ents, SketchPlane::XY());
    REQUIRE(wires_off.size() == 1);
    REQUIRE(edge_count(wires_off[0]) == 4);
    REQUIRE_FALSE(wires_off[0].Closed());

    // OFF + an EXACT joint (last endpoint exactly (0,0)): the quad still builds closed,
    // proving "off" means exact rather than broken.
    ents[3].p1 = Vec2d(0, 0);
    auto wires_exact = SketchEngine::entities_to_wires(ents, SketchPlane::XY());
    REQUIRE(wires_exact.size() == 1);
    REQUIRE(edge_count(wires_exact[0]) == 4);
    REQUIRE(wires_exact[0].Closed());

    // Restore the default so test order cannot leak OFF into the other cases.
    Slic3r::set_sketch_auto_close(true);
}

// A stray open segment touching nothing must not break a closed profile: the viewport
// discards open chains when it shades a region extrudable, so with closed_only the kernel
// must discard them too — otherwise Revolve/Extrude fail on a sketch that looks perfect.
TEST_CASE("a stray open segment does not break a closed profile", "[SketchEngine]")
{
    auto line = [](double x0, double y0, double x1, double y1) {
        SketchEntity e;
        e.type = SketchEntity::Type::Line;
        e.p0   = Vec2d(x0, y0);
        e.p1   = Vec2d(x1, y1);
        return e;
    };

    std::vector<SketchEntity> ents;
    ents.push_back(line(0, 0, 20, 0));     // 20x10 quad
    ents.push_back(line(20, 0, 20, 10));
    ents.push_back(line(20, 10, 0, 10));
    ents.push_back(line(0, 10, 0, 0));
    ents.push_back(line(5, 5, 6, 5.2));    // stray, touches nothing

    auto edge_count = [](const TopoDS_Wire& w) {
        int n = 0;
        for (TopExp_Explorer ex(w, TopAbs_EDGE); ex.More(); ex.Next()) ++n;
        return n;
    };

    // Unchanged behaviour: the stray line is its own open wire.
    auto wires_all = SketchEngine::entities_to_wires(ents, SketchPlane::XY(), /*closed_only=*/false);
    REQUIRE(wires_all.size() == 2);

    // closed_only drops the open chain: one closed quad survives.
    auto wires_closed = SketchEngine::entities_to_wires(ents, SketchPlane::XY(), /*closed_only=*/true);
    REQUIRE(wires_closed.size() == 1);
    REQUIRE(edge_count(wires_closed[0]) == 4);
    REQUIRE(wires_closed[0].Closed());

    // The Revolve path (entities_to_wire) finds the single closed loop.
    TopoDS_Wire w = SketchEngine::entities_to_wire(ents, SketchPlane::XY(), /*closed_only=*/true);
    REQUIRE_FALSE(w.IsNull());
    REQUIRE(edge_count(w) == 4);
}

// sketch_open_ends names the two free endpoints of an open chain, so the "does not form a
// single closed wire" failure can say WHERE the sketch is open.
TEST_CASE("sketch_open_ends names where a chain fails to close", "[SketchEngine]")
{
    auto line = [](double x0, double y0, double x1, double y1) {
        SketchEntity e;
        e.type = SketchEntity::Type::Line;
        e.p0   = Vec2d(x0, y0);
        e.p1   = Vec2d(x1, y1);
        return e;
    };

    // Open C shape: three lines, free endpoints at (0,0) and (0,10).
    std::vector<SketchEntity> ents;
    ents.push_back(line(0, 0, 10, 0));
    ents.push_back(line(10, 0, 10, 10));
    ents.push_back(line(10, 10, 0, 10));

    auto got = sketch_open_ends(ents, SketchPlane::XY());
    REQUIRE(got.size() == 2);

    std::sort(got.begin(), got.end(), [](const Vec2d& a, const Vec2d& b) {
        if (a.x() < b.x()) return true;
        if (a.x() > b.x()) return false;
        return a.y() < b.y();
    });

    REQUIRE_THAT(got[0].x(), WithinAbs(0.0, 1e-9));
    REQUIRE_THAT(got[0].y(), WithinAbs(0.0, 1e-9));
    REQUIRE_THAT(got[1].x(), WithinAbs(0.0, 1e-9));
    REQUIRE_THAT(got[1].y(), WithinAbs(10.0, 1e-9));
}

TEST_CASE("Mirror EllipseArc keeps the same arc, not its complement", "[SketchEdit]")
{
    // A quarter of an ellipse a=6 b=3 centred at (10,0), CCW from param 0 to pi/2, mirrored
    // across the Y axis: the image is a quarter again (the complement would be three quarters),
    // its start angle belongs to its p0, and its middle is still above the X axis.
    SketchEntity e;
    e.type = SketchEntity::Type::EllipseArc;
    e.center = Vec2d(10, 0); e.radius = 6; e.rminor = 3; e.rotation = 0.0;
    e.start_angle = 0.0; e.end_angle = M_PI / 2.0;
    e.p0 = Vec2d(16, 0); e.p1 = Vec2d(10, 3);
    const auto out = SketchEngine::mirror_entities({ e }, Vec2d(0, -1), Vec2d(0, 1));
    REQUIRE(out.size() == 1);
    const SketchEntity& m = out[0];
    REQUIRE(m.type == SketchEntity::Type::EllipseArc);
    CHECK_THAT(m.end_angle - m.start_angle, WithinAbs(M_PI / 2.0, 1e-9));
    const double cr = std::cos(m.rotation), sr = std::sin(m.rotation);
    auto at = [&](double t) {
        return Vec2d(m.center.x() + m.radius * std::cos(t) * cr - m.rminor * std::sin(t) * sr,
                     m.center.y() + m.radius * std::cos(t) * sr + m.rminor * std::sin(t) * cr);
    };
    CHECK((at(m.start_angle) - m.p0).norm() < 1e-6);
    CHECK((at(m.end_angle)   - m.p1).norm() < 1e-6);
    CHECK(at(0.5 * (m.start_angle + m.end_angle)).y() > 1.0);
}

TEST_CASE("Bridge between two collinear lines is straight and stays between them", "[SketchEdit]")
{
    SketchEntity a, b;
    a.type = b.type = SketchEntity::Type::Line;
    a.p0 = Vec2d(-10, 0); a.p1 = Vec2d(0, 0);    // ends at x=0 heading +X
    b.p0 = Vec2d(9, 0);   b.p1 = Vec2d(19, 0);   // starts at x=9 heading +X
    const SketchEntity br = SketchEngine::make_bridge(a, 1, b, 0);
    REQUIRE(br.ctrl.size() == 4);
    // G1 into b: the last inner pole sits BEFORE b's start, on the side the curve arrives from.
    CHECK(br.ctrl[1].x() > 0.0);
    CHECK(br.ctrl[2].x() < 9.0);
    for (const Vec2d& p : br.ctrl) CHECK_THAT(p.y(), WithinAbs(0.0, 1e-9));
}

TEST_CASE("Transform with a negative scale keeps an arc on its endpoints", "[SketchEdit]")
{
    SketchEntity e;
    e.type = SketchEntity::Type::Arc;
    e.center = Vec2d(3, 0); e.radius = 2; e.start_angle = 0.0; e.end_angle = M_PI / 2.0;
    e.p0 = Vec2d(5, 0); e.p1 = Vec2d(3, 2);
    const auto out = SketchEngine::transform_entities({ e }, Vec2d(0, 0), 0.0, -1.0, Vec2d(0, 0));
    REQUIRE(out.size() == 1);
    const SketchEntity& m = out[0];
    // A point reflection through the origin: centre (-3,0), start (-5,0), end (-3,-2).
    CHECK((m.center - Vec2d(-3, 0)).norm() < 1e-9);
    CHECK((m.p0 - Vec2d(-5, 0)).norm() < 1e-9);
    CHECK((m.p1 - Vec2d(-3, -2)).norm() < 1e-9);
}

TEST_CASE("A zero-radius circle does not build a wire (and does not throw)", "[SketchEdit]")
{
    SketchEntity c;
    c.type = SketchEntity::Type::Circle; c.center = Vec2d(0, 0); c.p0 = c.center; c.radius = 0.0;
    std::vector<TopoDS_Wire> w;
    REQUIRE_NOTHROW(w = SketchEngine::entities_to_wires({ c }, SketchPlane::XY(), true));
    CHECK(w.empty());
}

namespace sketch_array_chamfer_contracts {

Vec2d rotate_about(const Vec2d &point, const Vec2d &pivot, double angle)
{
    const Vec2d d = point - pivot;
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    return Vec2d(pivot.x() + c * d.x() - s * d.y(),
                 pivot.y() + s * d.x() + c * d.y());
}

Vec2d rigid_map(const Vec2d &point, const Vec2d &pivot, double angle, const Vec2d &translation)
{
    return rotate_about(point, pivot, angle) + translation;
}

Vec2d ellipse_point(const SketchEntity &entity, double parameter)
{
    const double c = std::cos(entity.rotation);
    const double s = std::sin(entity.rotation);
    return Vec2d(entity.center.x() + entity.radius * std::cos(parameter) * c -
                     entity.rminor * std::sin(parameter) * s,
                 entity.center.y() + entity.radius * std::cos(parameter) * s +
                     entity.rminor * std::sin(parameter) * c);
}

SketchEntity line_entity(const Vec2d &p0, const Vec2d &p1)
{
    SketchEntity entity;
    entity.type = SketchEntity::Type::Line;
    entity.p0 = p0;
    entity.p1 = p1;
    return entity;
}

SketchEntity arc_entity(const Vec2d &center, double radius, double start, double end)
{
    SketchEntity entity;
    entity.type = SketchEntity::Type::Arc;
    entity.center = center;
    entity.radius = radius;
    entity.start_angle = start;
    entity.end_angle = end;
    entity.p0 = center + radius * Vec2d(std::cos(start), std::sin(start));
    entity.p1 = center + radius * Vec2d(std::cos(end), std::sin(end));
    return entity;
}

SketchEntity ellipse_arc_entity(const Vec2d &center, double major, double minor,
                                double rotation, double start, double end)
{
    SketchEntity entity;
    entity.type = SketchEntity::Type::EllipseArc;
    entity.center = center;
    entity.radius = major;
    entity.rminor = minor;
    entity.rotation = rotation;
    entity.start_angle = start;
    entity.end_angle = end;
    entity.p0 = ellipse_point(entity, start);
    entity.p1 = ellipse_point(entity, end);
    return entity;
}

SketchEntity ellipse_entity(const Vec2d &center, double major, double minor, double rotation)
{
    SketchEntity entity;
    entity.type = SketchEntity::Type::Ellipse;
    entity.center = center;
    entity.p0 = center;
    entity.radius = major;
    entity.rminor = minor;
    entity.rotation = rotation;
    return entity;
}

SketchEntity bspline_entity()
{
    SketchEntity entity;
    entity.type = SketchEntity::Type::BSpline;
    entity.ctrl = {Vec2d(-2., 1.), Vec2d(0., 4.), Vec2d(3., 2.)};
    entity.p0 = entity.ctrl.front();
    entity.p1 = entity.ctrl.back();
    return entity;
}

void check_point(const Vec2d &actual, const Vec2d &expected)
{
    CHECK_THAT(actual.x(), WithinAbs(expected.x(), 1e-9));
    CHECK_THAT(actual.y(), WithinAbs(expected.y(), 1e-9));
}

void check_source_unchanged(const std::vector<SketchEntity> &actual,
                            const std::vector<SketchEntity> &expected)
{
    REQUIRE(actual.size() == expected.size());
    for (std::size_t i = 0; i < actual.size(); ++i) {
        const SketchEntity &a = actual[i];
        const SketchEntity &e = expected[i];
        CHECK(a.type == e.type);
        CHECK(a.construction == e.construction);
        check_point(a.p0, e.p0);
        check_point(a.p1, e.p1);
        check_point(a.center, e.center);
        CHECK_THAT(a.radius, WithinAbs(e.radius, 1e-9));
        CHECK_THAT(a.start_angle, WithinAbs(e.start_angle, 1e-9));
        CHECK_THAT(a.end_angle, WithinAbs(e.end_angle, 1e-9));
        CHECK_THAT(a.rminor, WithinAbs(e.rminor, 1e-9));
        CHECK_THAT(a.rotation, WithinAbs(e.rotation, 1e-9));
        REQUIRE(a.ctrl.size() == e.ctrl.size());
        for (std::size_t j = 0; j < a.ctrl.size(); ++j)
            check_point(a.ctrl[j], e.ctrl[j]);
    }
}

void check_translated_entity(const SketchEntity &actual, const SketchEntity &source,
                             const Vec2d &translation)
{
    CHECK(actual.type == source.type);
    switch (source.type) {
    case SketchEntity::Type::Line:
        check_point(actual.p0, source.p0 + translation);
        check_point(actual.p1, source.p1 + translation);
        break;
    case SketchEntity::Type::Point:
        check_point(actual.p0, source.p0 + translation);
        break;
    case SketchEntity::Type::Circle:
        check_point(actual.center, source.center + translation);
        check_point(actual.p0, source.center + translation);
        CHECK_THAT(actual.radius, WithinAbs(source.radius, 1e-9));
        break;
    case SketchEntity::Type::Arc:
        check_point(actual.center, source.center + translation);
        check_point(actual.p0, source.p0 + translation);
        check_point(actual.p1, source.p1 + translation);
        CHECK_THAT(actual.start_angle, WithinAbs(source.start_angle, 1e-9));
        CHECK_THAT(actual.end_angle, WithinAbs(source.end_angle, 1e-9));
        break;
    case SketchEntity::Type::Ellipse:
        check_point(actual.center, source.center + translation);
        check_point(actual.p0, source.center + translation);
        CHECK_THAT(actual.rotation, WithinAbs(source.rotation, 1e-9));
        break;
    case SketchEntity::Type::EllipseArc:
        check_point(actual.center, source.center + translation);
        check_point(actual.p0, source.p0 + translation);
        check_point(actual.p1, source.p1 + translation);
        CHECK_THAT(actual.start_angle, WithinAbs(source.start_angle, 1e-9));
        CHECK_THAT(actual.end_angle, WithinAbs(source.end_angle, 1e-9));
        CHECK_THAT(actual.rotation, WithinAbs(source.rotation, 1e-9));
        break;
    case SketchEntity::Type::BSpline:
        REQUIRE(actual.ctrl.size() == source.ctrl.size());
        for (std::size_t j = 0; j < source.ctrl.size(); ++j)
            check_point(actual.ctrl[j], source.ctrl[j] + translation);
        check_point(actual.p0, source.p0 + translation);
        check_point(actual.p1, source.p1 + translation);
        break;
    }
}

TEST_CASE("Linear sketch arrays translate every entity type and preserve the source", "[SketchEdit]")
{
    std::vector<SketchEntity> source = {
        line_entity(Vec2d(-3., 1.), Vec2d(4., 1.)),
        SketchEntity{},
        arc_entity(Vec2d(2., -1.), 2.5, 0.2, 1.4),
        ellipse_entity(Vec2d(-2., 3.), 5., 2., 0.35),
        ellipse_arc_entity(Vec2d(4., 2.), 6., 2.5, 0.25, 0.1, 1.2),
        bspline_entity(),
    };
    source[1].type = SketchEntity::Type::Point;
    source[1].p0 = Vec2d(1., -2.);
    SketchEntity circle;
    circle.type = SketchEntity::Type::Circle;
    circle.center = Vec2d(-4., -1.5);
    circle.p0 = circle.center;
    circle.radius = 1.75;
    source.push_back(circle);
    const std::vector<SketchEntity> original = source;
    const Vec2d step(4., -2.);

    const std::vector<SketchEntity> copies =
        SketchEngine::array_entities(source, 3, step, 0., Vec2d(17., 19.));
    REQUIRE(copies.size() == source.size() * 2);
    for (int copy = 1; copy < 3; ++copy) {
        const Vec2d translation = double(copy) * step;
        for (std::size_t j = 0; j < source.size(); ++j)
            check_translated_entity(copies[std::size_t(copy - 1) * source.size() + j], source[j], translation);
    }
    CHECK(SketchEngine::array_entities(source, 1, step, 0., Vec2d(0., 0.)).empty());
    check_source_unchanged(source, original);
}

TEST_CASE("Polar sketch arrays rotate every typed entity around a pivot and then translate", "[SketchEdit]")
{
    const Vec2d pivot(1., -2.);
    const double angle_step = M_PI / 2.;
    const int translation_case = GENERATE(0, 1);
    const Vec2d step = translation_case == 0 ? Vec2d(0., 0.) : Vec2d(3., -1.);
    CAPTURE(translation_case, step.x(), step.y());

    SketchEntity point;
    point.type = SketchEntity::Type::Point;
    point.p0 = Vec2d(-1., -3.);
    point.construction = true;
    std::vector<SketchEntity> source = {
        line_entity(Vec2d(3., -2.), Vec2d(3., 1.)),
        point,
        [&] {
            SketchEntity entity;
            entity.type = SketchEntity::Type::Circle;
            entity.center = Vec2d(4., -2.);
            entity.p0 = entity.center;
            entity.radius = 1.25;
            return entity;
        }(),
        arc_entity(Vec2d(1., -2.), 2., 0.2, 1.1),
        ellipse_entity(Vec2d(2., -1.), 3., 1., 0.3),
        ellipse_arc_entity(Vec2d(3., -1.), 2.5, 0.75, 0.25, 0.1, 1.2),
        bspline_entity(),
    };
    const std::vector<SketchEntity> original = source;

    const std::vector<SketchEntity> copies =
        SketchEngine::array_entities(source, 4, step, angle_step, pivot);
    REQUIRE(copies.size() == source.size() * 3);
    for (int copy = 1; copy < 4; ++copy) {
        const double angle = double(copy) * angle_step;
        const Vec2d translation = double(copy) * step;
        const std::size_t base = std::size_t(copy - 1) * source.size();

        const SketchEntity &line = copies[base];
        check_point(line.p0, rigid_map(source[0].p0, pivot, angle, translation));
        check_point(line.p1, rigid_map(source[0].p1, pivot, angle, translation));

        const SketchEntity &point_copy = copies[base + 1];
        check_point(point_copy.p0, rigid_map(source[1].p0, pivot, angle, translation));
        REQUIRE(source[1].construction);
        CHECK(point_copy.construction);

        const SketchEntity &circle = copies[base + 2];
        check_point(circle.center, rigid_map(source[2].center, pivot, angle, translation));
        check_point(circle.p0, circle.center);
        CHECK_THAT(circle.radius, WithinAbs(source[2].radius, 1e-9));

        const SketchEntity &arc = copies[base + 3];
        check_point(arc.center, rigid_map(source[3].center, pivot, angle, translation));
        CHECK_THAT(arc.start_angle, WithinAbs(source[3].start_angle + angle, 1e-9));
        CHECK_THAT(arc.end_angle, WithinAbs(source[3].end_angle + angle, 1e-9));
        CHECK_THAT(arc.radius, WithinAbs(source[3].radius, 1e-9));
        check_point(arc.p0, rigid_map(source[3].p0, pivot, angle, translation));
        check_point(arc.p1, rigid_map(source[3].p1, pivot, angle, translation));

        const SketchEntity &ellipse = copies[base + 4];
        check_point(ellipse.center, rigid_map(source[4].center, pivot, angle, translation));
        check_point(ellipse.p0, ellipse.center);
        CHECK_THAT(ellipse.radius, WithinAbs(source[4].radius, 1e-9));
        CHECK_THAT(ellipse.rminor, WithinAbs(source[4].rminor, 1e-9));
        CHECK_THAT(ellipse.rotation, WithinAbs(source[4].rotation + angle, 1e-9));

        const SketchEntity &ellipse_arc = copies[base + 5];
        check_point(ellipse_arc.center, rigid_map(source[5].center, pivot, angle, translation));
        check_point(ellipse_arc.p0, rigid_map(source[5].p0, pivot, angle, translation));
        check_point(ellipse_arc.p1, rigid_map(source[5].p1, pivot, angle, translation));
        CHECK_THAT(ellipse_arc.radius, WithinAbs(source[5].radius, 1e-9));
        CHECK_THAT(ellipse_arc.rminor, WithinAbs(source[5].rminor, 1e-9));
        CHECK_THAT(ellipse_arc.rotation, WithinAbs(source[5].rotation + angle, 1e-9));
        CHECK_THAT(ellipse_arc.start_angle, WithinAbs(source[5].start_angle, 1e-9));
        CHECK_THAT(ellipse_arc.end_angle, WithinAbs(source[5].end_angle, 1e-9));

        const SketchEntity &bspline = copies[base + 6];
        REQUIRE(bspline.ctrl.size() == source[6].ctrl.size());
        for (std::size_t j = 0; j < source[6].ctrl.size(); ++j)
            check_point(bspline.ctrl[j], rigid_map(source[6].ctrl[j], pivot, angle, translation));
        check_point(bspline.p0, rigid_map(source[6].p0, pivot, angle, translation));
        check_point(bspline.p1, rigid_map(source[6].p1, pivot, angle, translation));
    }
    check_source_unchanged(source, original);
}

TEST_CASE("Chamfers trim both legs by the requested setback", "[SketchEdit]")
{
    const SketchEntity a = line_entity(Vec2d(0., 0.), Vec2d(10., 0.));
    const SketchEntity b = line_entity(Vec2d(0., 10.), Vec2d(0., 0.));
    SketchEntity a_out, b_out, segment_out;
    REQUIRE(SketchEngine::chamfer_lines(a, b, 2., a_out, b_out, segment_out));

    REQUIRE(a_out.type == SketchEntity::Type::Line);
    REQUIRE(b_out.type == SketchEntity::Type::Line);
    REQUIRE(segment_out.type == SketchEntity::Type::Line);
    check_point(a_out.p0, Vec2d(2., 0.));
    check_point(a_out.p1, Vec2d(10., 0.));
    check_point(b_out.p0, Vec2d(0., 10.));
    check_point(b_out.p1, Vec2d(0., 2.));
    check_point(segment_out.p0, Vec2d(2., 0.));
    check_point(segment_out.p1, Vec2d(0., 2.));
    CHECK_THAT((a_out.p1 - a_out.p0).norm(), WithinAbs(8., 1e-9));
    CHECK_THAT((b_out.p1 - b_out.p0).norm(), WithinAbs(8., 1e-9));
    CHECK_THAT((segment_out.p1 - segment_out.p0).norm(), WithinAbs(2. * std::sqrt(2.), 1e-9));
    check_point(a.p0, Vec2d(0., 0.));
    check_point(a.p1, Vec2d(10., 0.));
    check_point(b.p0, Vec2d(0., 10.));
    check_point(b.p1, Vec2d(0., 0.));
}

TEST_CASE("Chamfer rejects parallel, invalid, and overrun legs without mutating outputs", "[SketchEdit]")
{
    const int scenario = GENERATE(0, 1, 2, 3, 4);
    SketchEntity a = line_entity(Vec2d(0., 0.), Vec2d(10., 0.));
    SketchEntity b = line_entity(Vec2d(0., 10.), Vec2d(0., 0.));
    double distance = 2.;
    if (scenario == 0) {
        b = line_entity(Vec2d(0., 5.), Vec2d(10., 5.));
    } else if (scenario == 1) {
        a = line_entity(Vec2d(0., 0.), Vec2d(1., 0.));
    } else if (scenario == 2) {
        distance = 0.;
    } else if (scenario == 3) {
        SketchEntity circle;
        circle.type = SketchEntity::Type::Circle;
        // Its p0/p1 would form a usable leg if read as a line, so only the type check rejects it.
        circle.center = Vec2d(2., -3.);
        circle.p0 = circle.center;
        circle.p1 = Vec2d(12., -3.);
        circle.radius = 2.;
        a = circle;
    } else {
        distance = 11.;
    }
    const SketchEntity original_a = a;
    const SketchEntity original_b = b;
    SketchEntity a_out = ellipse_entity(Vec2d(7., 8.), 3., 1., 0.2);
    SketchEntity b_out = ellipse_entity(Vec2d(-7., -8.), 4., 2., -0.3);
    SketchEntity segment_out = ellipse_entity(Vec2d(1., 9.), 2., 1., 0.4);
    const SketchEntity original_a_out = a_out;
    const SketchEntity original_b_out = b_out;
    const SketchEntity original_segment_out = segment_out;

    CAPTURE(scenario);
    CHECK_FALSE(SketchEngine::chamfer_lines(a, b, distance, a_out, b_out, segment_out));
    check_source_unchanged({a}, {original_a});
    check_source_unchanged({b}, {original_b});
    check_source_unchanged({a_out}, {original_a_out});
    check_source_unchanged({b_out}, {original_b_out});
    check_source_unchanged({segment_out}, {original_segment_out});
}

} // namespace sketch_array_chamfer_contracts

namespace sketch_circle_cutter_contracts {

SketchEntity circle_entity(const Vec2d& center, double radius)
{
    SketchEntity e;
    e.type = SketchEntity::Type::Circle;
    e.center = center;
    e.p0 = center;
    e.radius = radius;
    return e;
}

SketchEntity arc_entity(const Vec2d& center, double radius, double start, double end)
{
    SketchEntity e;
    e.type = SketchEntity::Type::Arc;
    e.center = center;
    e.radius = radius;
    e.start_angle = start;
    e.end_angle = end;
    e.p0 = center + radius * Vec2d(std::cos(start), std::sin(start));
    e.p1 = center + radius * Vec2d(std::cos(end), std::sin(end));
    return e;
}

} // namespace sketch_circle_cutter_contracts

TEST_CASE("Circle trim keeps the complement of the pick gap for full circular cutters",
          "[SketchEdit]")
{
    using namespace sketch_circle_cutter_contracts;

    const int cutter_kind = GENERATE(0, 1, 2);
    CAPTURE(cutter_kind);

    SketchEntity subject = circle_entity(Vec2d(0.0, 0.0), 5.0);
    SketchEntity cutter;
    if (cutter_kind == 0) {
        cutter = circle_entity(Vec2d(6.0, 0.0), 5.0);
    } else {
        const double cutter_start = GENERATE(-5.5, 0.35, 5.5);
        CAPTURE(cutter_start);
        const double signed_turn = cutter_kind == 1 ? 2.0 * M_PI : -2.0 * M_PI;
        cutter = arc_entity(Vec2d(6.0, 0.0), 5.0, cutter_start, cutter_start + signed_turn);
    }

    const double theta = std::atan2(4.0, 3.0);
    const Vec2d upper_crossing(3.0, 4.0);
    const Vec2d lower_crossing(3.0, -4.0);
    REQUIRE(SketchEngine::trim_entity(subject, {cutter}, Vec2d(5.0, 0.0)));

    REQUIRE(subject.type == SketchEntity::Type::Arc);
    CHECK_THAT(subject.radius, WithinAbs(5.0, 1e-9));
    CHECK_THAT(subject.end_angle - subject.start_angle,
               WithinAbs(2.0 * M_PI - 2.0 * theta, 1e-9));

    // The independently derived intersections bound the kept complement; endpoint order may vary.
    const double endpoint_error = std::min(
        (subject.p0 - upper_crossing).norm() + (subject.p1 - lower_crossing).norm(),
        (subject.p0 - lower_crossing).norm() + (subject.p1 - upper_crossing).norm());
    CHECK_THAT(endpoint_error, WithinAbs(0.0, 1e-9));

    const double middle = 0.5 * (subject.start_angle + subject.end_angle);
    const Vec2d retained_middle = subject.center + subject.radius *
        Vec2d(std::cos(middle), std::sin(middle));
    CHECK_THAT(retained_middle.x(), WithinAbs(-5.0, 1e-9));
    CHECK_THAT(retained_middle.y(), WithinAbs(0.0, 1e-9));
    CHECK_THAT((subject.p0 - (subject.center + subject.radius *
        Vec2d(std::cos(subject.start_angle), std::sin(subject.start_angle)))).norm(),
        WithinAbs(0.0, 1e-9));
    CHECK_THAT((subject.p1 - (subject.center + subject.radius *
        Vec2d(std::cos(subject.end_angle), std::sin(subject.end_angle)))).norm(),
        WithinAbs(0.0, 1e-9));
}

TEST_CASE("Circle trim ignores an arc cutter with only one crossing", "[SketchEdit]")
{
    using namespace sketch_circle_cutter_contracts;

    SketchEntity subject = circle_entity(Vec2d(0.0, 0.0), 5.0);
    const SketchEntity original = subject;
    const SketchEntity upper_semicircle = arc_entity(
        Vec2d(6.0, 0.0), 5.0, 0.0, M_PI);

    REQUIRE_FALSE(SketchEngine::trim_entity(subject, {upper_semicircle}, Vec2d(5.0, 0.0)));
    CHECK(subject.type == original.type);
    CHECK_THAT(subject.center.x(), WithinAbs(original.center.x(), 1e-9));
    CHECK_THAT(subject.center.y(), WithinAbs(original.center.y(), 1e-9));
    CHECK_THAT(subject.radius, WithinAbs(original.radius, 1e-9));
    CHECK_THAT((subject.p0 - original.p0).norm(), WithinAbs(0.0, 1e-9));
}

TEST_CASE("Circle trim leaves the subject unchanged for noncrossing circles",
          "[SketchEdit]")
{
    using namespace sketch_circle_cutter_contracts;

    const int relationship = GENERATE(0, 1, 2);
    CAPTURE(relationship);
    SketchEntity subject = circle_entity(Vec2d(0.0, 0.0), 5.0);
    SketchEntity cutter;
    if (relationship == 0) {
        cutter = circle_entity(Vec2d(0.0, 0.0), 2.0);
    } else if (relationship == 1) {
        cutter = circle_entity(Vec2d(20.0, 0.0), 2.0);
    } else {
        cutter = circle_entity(Vec2d(1.0, 0.0), 1.0);
    }
    const SketchEntity original = subject;

    REQUIRE_FALSE(SketchEngine::trim_entity(subject, {cutter}, Vec2d(5.0, 0.0)));
    CHECK(subject.type == original.type);
    CHECK_THAT((subject.center - original.center).norm(), WithinAbs(0.0, 1e-9));
    CHECK_THAT((subject.p0 - original.p0).norm(), WithinAbs(0.0, 1e-9));
    CHECK_THAT(subject.radius, WithinAbs(original.radius, 1e-9));
}

TEST_CASE("Arc extension selects the nearest forward circle intersection", "[SketchEdit]")
{
    using namespace sketch_circle_cutter_contracts;

    const int cutter_kind = GENERATE(0, 1, 2);
    CAPTURE(cutter_kind);
    const double theta = std::atan2(4.0, 3.0);
    SketchEntity subject = arc_entity(Vec2d(0.0, 0.0), 5.0, 0.0, 0.5);
    SketchEntity cutter;
    if (cutter_kind == 0) {
        cutter = circle_entity(Vec2d(6.0, 0.0), 5.0);
    } else {
        const double cutter_start = GENERATE(-5.5, 0.35, 5.5);
        CAPTURE(cutter_start);
        const double signed_turn = cutter_kind == 1 ? 2.0 * M_PI : -2.0 * M_PI;
        cutter = arc_entity(Vec2d(6.0, 0.0), 5.0,
                            cutter_start, cutter_start + signed_turn);
    }

    const Vec2d pick(5.0 * std::cos(0.4), 5.0 * std::sin(0.4));
    REQUIRE(SketchEngine::extend_entity(subject, {cutter}, pick));
    CHECK_THAT(subject.start_angle, WithinAbs(0.0, 1e-9));
    CHECK_THAT(subject.end_angle, WithinAbs(theta, 1e-9));
    CHECK_THAT((subject.p0 - Vec2d(5.0, 0.0)).norm(), WithinAbs(0.0, 1e-9));
    CHECK_THAT((subject.p1 - Vec2d(3.0, 4.0)).norm(), WithinAbs(0.0, 1e-9));
}

TEST_CASE("Line trim follows positive and negative arc-cutter sweeps", "[SketchEdit]")
{
    using namespace sketch_circle_cutter_contracts;

    const int sweep_direction = GENERATE(0, 1);
    CAPTURE(sweep_direction);
    SketchEntity subject;
    subject.type = SketchEntity::Type::Line;
    subject.p0 = Vec2d(-10.0, 0.0);
    subject.p1 = Vec2d(10.0, 0.0);

    const double start = M_PI / 2.0;
    const double end = sweep_direction == 0 ? 3.0 * M_PI / 2.0 : -M_PI / 2.0;
    const SketchEntity cutter = arc_entity(Vec2d(0.0, 0.0), 5.0, start, end);
    const Vec2d pick = sweep_direction == 0 ? Vec2d(-8.0, 0.0) : Vec2d(8.0, 0.0);

    REQUIRE(SketchEngine::trim_entity(subject, {cutter}, pick));
    if (sweep_direction == 0) {
        CHECK_THAT((subject.p0 - Vec2d(-5.0, 0.0)).norm(), WithinAbs(0.0, 1e-9));
        CHECK_THAT((subject.p1 - Vec2d(10.0, 0.0)).norm(), WithinAbs(0.0, 1e-9));
    } else {
        CHECK_THAT((subject.p0 - Vec2d(-10.0, 0.0)).norm(), WithinAbs(0.0, 1e-9));
        CHECK_THAT((subject.p1 - Vec2d(5.0, 0.0)).norm(), WithinAbs(0.0, 1e-9));
    }
}

