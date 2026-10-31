#include <catch2/catch_all.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_message.hpp>
#include <catch2/generators/catch_generators.hpp>
#include "libslic3r/TriangleSelector.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/Point.hpp"

#include <algorithm>
#include <array>
#include <memory>
#include "libslic3r/libslic3r.h"
#include <vector>
#include <cstddef>
#include <string>

using namespace Slic3r;

// A sphere gives well over ExtruderMax original facets, so every extruder state can be assigned
// to a facet of its own without any splitting getting in the way.
static TriangleMesh test_mesh() { return make_sphere(5., 2 * PI / 24); }

// Read the nibble_idx-th 4-bit group of a serialized bitstream, least significant bit first.
static int nibble_at(const std::vector<bool> &bitstream, size_t nibble_idx)
{
    int n = 0;
    for (size_t bit = 0; bit < 4; ++bit)
        n |= int(bitstream[nibble_idx * 4 + bit]) << bit;
    return n;
}

TEST_CASE("Every extruder state survives a serialize/deserialize round trip", "[TriangleSelector]")
{
    const TriangleMesh mesh      = test_mesh();
    const int          max_state = int(EnforcerBlockerType::ExtruderMax);
    REQUIRE(int(mesh.its.indices.size()) >= max_state);

    TriangleSelector selector(mesh);
    for (int state = 1; state <= max_state; ++state)
        selector.set_facet(state - 1, EnforcerBlockerType(state));

    TriangleSelector restored(mesh);
    restored.deserialize(selector.serialize());

    for (int state = 1; state <= max_state; ++state) {
        INFO("Extruder " << state);
        REQUIRE(restored.has_facets(EnforcerBlockerType(state)));
        REQUIRE(restored.num_facets(EnforcerBlockerType(state)) == 1);
    }
}

TEST_CASE("Serialized data reports the extruder states it uses", "[TriangleSelector]")
{
    const TriangleMesh mesh = test_mesh();
    TriangleSelector   selector(mesh);
    selector.set_facet(0, EnforcerBlockerType::Extruder16);
    selector.set_facet(1, EnforcerBlockerType::Extruder32);

    const TriangleSelector::TriangleSplittingData data = selector.serialize();

    REQUIRE(data.used_states.size() == size_t(EnforcerBlockerType::ExtruderMax) + 1);
    REQUIRE(data.used_states[size_t(EnforcerBlockerType::Extruder16)]);
    REQUIRE(data.used_states[size_t(EnforcerBlockerType::Extruder32)]);
    REQUIRE_FALSE(data.used_states[size_t(EnforcerBlockerType::Extruder17)]);

    SECTION("used_states recomputed from the bitstream agrees") {
        TriangleSelector::TriangleSplittingData recomputed = data;
        recomputed.reset_used_states();
        recomputed.update_used_states(0);
        REQUIRE(recomputed.used_states == data.used_states);
    }

    SECTION("has_facets on the raw data agrees") {
        REQUIRE(TriangleSelector::has_facets(data, EnforcerBlockerType::Extruder32));
        REQUIRE_FALSE(TriangleSelector::has_facets(data, EnforcerBlockerType::Extruder17));
    }
}

// States 3..17 must keep the pre-existing encoding ("11" prefix plus one nibble of state-3) so
// projects written by older builds stay readable and newly written ones stay readable by them.
TEST_CASE("Extruder states up to 17 keep the single-nibble encoding", "[TriangleSelector]")
{
    const int state = GENERATE(3, 8, 16, 17);

    TriangleSelector selector(test_mesh());
    selector.set_facet(0, EnforcerBlockerType(state));
    const std::vector<bool> bitstream = selector.serialize().bitstream;

    INFO("Extruder " << state);
    // Two nibbles: the "11"-prefixed leaf code, then the state itself.
    REQUIRE(bitstream.size() == 8);
    REQUIRE(nibble_at(bitstream, 0) == 0b1100);
    REQUIRE(nibble_at(bitstream, 1) == state - 3);
}

// States 18 and above set the state nibble to 0b1111 and carry (state-18) in one more nibble.
TEST_CASE("Extruder states above 17 are encoded in a second nibble", "[TriangleSelector]")
{
    const int state = GENERATE(18, 25, 32);

    TriangleSelector selector(test_mesh());
    selector.set_facet(0, EnforcerBlockerType(state));
    const std::vector<bool> bitstream = selector.serialize().bitstream;

    INFO("Extruder " << state);
    REQUIRE(bitstream.size() == 12);
    REQUIRE(nibble_at(bitstream, 0) == 0b1100);
    REQUIRE(nibble_at(bitstream, 1) == 0b1111);
    REQUIRE(nibble_at(bitstream, 2) == state - 18);
}

// Model.cpp writes these hex strings into the 3MF for colored mesh imports; the selector must
// decode exactly the states CONST_FILAMENTS assigns to them.
TEST_CASE("Extruder states match the CONST_FILAMENTS hex encoding", "[TriangleSelector]")
{
    struct Case { const char *hex; int state; };
    const auto c = GENERATE(values<Case>({
        {"8", 2}, {"0C", 3}, {"DC", 16}, {"EC", 17}, {"0FC", 18}, {"EFC", 32},
    }));

    // get_triangle_as_string emits the nibbles most significant first, so read the hex backwards.
    const std::string hex = c.hex;
    std::vector<bool> bitstream;
    for (auto it = hex.rbegin(); it != hex.rend(); ++it) {
        const int nibble = *it >= 'A' ? (*it - 'A' + 10) : (*it - '0');
        for (int bit = 0; bit < 4; ++bit)
            bitstream.push_back((nibble >> bit) & 1);
    }

    TriangleSelector::TriangleSplittingData data;
    data.triangles_to_split.emplace_back(0, 0);
    data.bitstream = bitstream;

    INFO("Hex " << c.hex << " -> extruder " << c.state);
    REQUIRE(TriangleSelector::has_facets(data, EnforcerBlockerType(c.state)));
}

// Pack 4-bit codes into a bitstream, least significant bit first, in the order the decoder reads them.
static std::vector<bool> pack_nibbles(const std::vector<int> &nibbles)
{
    std::vector<bool> bitstream;
    for (const int nibble : nibbles)
        for (int bit = 0; bit < 4; ++bit)
            bitstream.push_back((nibble >> bit) & 1);
    return bitstream;
}

TEST_CASE("A valid paint stream with nested splits round-trips bit for bit", "[TriangleSelector]")
{
    const TriangleMesh mesh = test_mesh();

    TriangleSelector::TriangleSplittingData data;
    data.triangles_to_split.emplace_back(0, 0);
    // A three-side split whose children, in stream order, are: a one-side split (side 2) into two
    // leaves, a two-side split (side 1) into leaves of states 20, 0 and 8, then two plain leaves.
    const std::vector<int> triangle_0 = {0b0011,
                                         0b1001, 0b1000, 0b0100,
                                         0b0110, 0b1100, 0b1111, 20 - 18, 0b0000, 0b1100, 8 - 3,
                                         0b1000,
                                         0b0100};
    data.bitstream = pack_nibbles(triangle_0);
    data.triangles_to_split.emplace_back(5, int(data.bitstream.size()));
    const std::vector<bool> triangle_5 = pack_nibbles({0b1100, 3 - 3});
    data.bitstream.insert(data.bitstream.end(), triangle_5.begin(), triangle_5.end());
    data.reset_used_states();
    REQUIRE(data.update_used_states(0));

    TriangleSelector restored(mesh);
    restored.deserialize(data);

    REQUIRE(restored.num_facets(EnforcerBlockerType::Extruder20) == 1);
    REQUIRE(restored.num_facets(EnforcerBlockerType::Extruder3) == 1);
    REQUIRE(restored.serialize() == data);
}

TEST_CASE("A truncated or malformed paint stream drops only the damaged triangle", "[TriangleSelector][Regression]")
{
    struct Case { const char *name; std::vector<int> nibbles; };
    const auto c = GENERATE(values<Case>({
        {"three-side split missing two children", {0b0011, 0b1000, 0b1000}},
        {"leaf missing its state nibble",         {0b1100}},
        {"leaf missing its second state nibble",  {0b1100, 0b1111}},
        {"splits nested past the end",            {0b0011, 0xF, 0xF, 0xF, 0xF, 0xF, 0xF, 0xF, 0xF,
                                                    0xF, 0xF, 0xF, 0xF, 0xF, 0xF, 0xF, 0xF}},
        {"one-side split of the nonexistent side 3", {0b1101, 0b1000, 0b1000}},
    }));
    INFO(c.name);

    const TriangleMesh mesh = test_mesh();
    TriangleSelector   intact(mesh);
    intact.set_facet(0, EnforcerBlockerType::Extruder2);

    // Triangle 0 stays intact, triangle 1 carries the damaged stream.
    TriangleSelector::TriangleSplittingData data = intact.serialize();
    data.triangles_to_split.emplace_back(1, int(data.bitstream.size()));
    const std::vector<bool> damaged = pack_nibbles(c.nibbles);
    data.bitstream.insert(data.bitstream.end(), damaged.begin(), damaged.end());

    TriangleSelector restored(mesh);
    REQUIRE_NOTHROW(restored.deserialize(data));
    // Triangle 1 unwinds completely, so the selector holds exactly the intact paint.
    REQUIRE(restored.serialize() == intact.serialize());

    REQUIRE_NOTHROW(TriangleSelector::has_facets(data, EnforcerBlockerType::Extruder3));

    TriangleSelector::TriangleSplittingData recomputed = data;
    recomputed.reset_used_states();
    REQUIRE_FALSE(recomputed.update_used_states(0));
    REQUIRE(std::none_of(recomputed.used_states.begin(), recomputed.used_states.end(), [](bool used) { return used; }));
}

namespace triangle_cursor_contracts {

namespace {
using Selector = TriangleSelector;
using Cursor = Selector::Cursor;
using Plane = Selector::ClippingPlane;

std::unique_ptr<Cursor> single(Selector::CursorType kind, float radius = 2.f,
                              const Transform3d &transform = Transform3d::Identity(),
                              const Plane &plane = Plane())
{
    return Selector::SinglePointCursor::cursor_factory(
        Vec3f::Zero(), Vec3f(0, 0, 10), radius, kind, transform, plane);
}

std::unique_ptr<Cursor> swept(Selector::CursorType kind,
                             const Transform3d &transform = Transform3d::Identity(),
                             const Plane &plane = Plane())
{
    return Selector::DoublePointCursor::cursor_factory(
        Vec3f(0, 0, 0), Vec3f(6, 0, 0), Vec3f(0, 0, 10), 2.f, kind, transform, plane);
}

// Cursor methods accept protected record types. Expose only construction of
// valid three-vertex inputs, without changing any selector implementation state.
struct CursorInput : Selector {
    static bool edge(const Cursor &cursor, const std::array<Vec3f, 3> &points)
    {
        const Triangle triangle(0, 1, 2, 0, EnforcerBlockerType::NONE);
        const std::vector<Vertex> vertices{Vertex(points[0]), Vertex(points[1]), Vertex(points[2])};
        return cursor.is_edge_inside_cursor(triangle, vertices);
    }
    static int count(const Cursor &cursor, const std::array<Vec3f, 3> &points)
    {
        const Triangle triangle(0, 1, 2, 0, EnforcerBlockerType::NONE);
        const std::vector<Vertex> vertices{Vertex(points[0]), Vertex(points[1]), Vertex(points[2])};
        return cursor.vertices_inside(triangle, vertices);
    }
};

Transform3d stretch()
{
    Transform3d transform = Transform3d::Identity();
    transform.scale(Vec3d(2, 3, 4));
    transform.translate(Vec3d(7, -2, 5));
    return transform;
}
}

TEST_CASE("A circular brush selects a projected disk at either depth", "[TriangleSelector][TriangleCursor]")
{
    const auto brush = single(Selector::CIRCLE);
    for (float depth : {-5.f, 0.f, 8.f}) {
        CHECK(brush->is_mesh_point_inside(Vec3f(1, 1, depth)));
        CHECK_FALSE(brush->is_mesh_point_inside(Vec3f(2.1f, 0, depth)));
        CHECK_FALSE(brush->is_mesh_point_inside(Vec3f(0, -2.1f, depth)));
    }
}

TEST_CASE("A spherical brush limits selection in depth as well as in the plane", "[TriangleSelector][TriangleCursor]")
{
    const auto brush = single(Selector::SPHERE);
    CHECK(brush->is_mesh_point_inside(Vec3f(1, 1, 1)));
    CHECK_FALSE(brush->is_mesh_point_inside(Vec3f(1, 1, 2)));
    CHECK_FALSE(brush->is_mesh_point_inside(Vec3f(0, 0, -3)));
}

TEST_CASE("Single-point brushes apply clipping in mesh coordinates", "[TriangleSelector][TriangleCursor]")
{
    const auto kind = GENERATE(Selector::CIRCLE, Selector::SPHERE);
    const auto brush = single(kind, 2.f, stretch(), Plane({1, 0, 0, 0}));
    CHECK(brush->is_mesh_point_inside(Vec3f(-0.2f, 0, 0)));
    CHECK_FALSE(brush->is_mesh_point_inside(Vec3f(0.2f, 0, 0)));
    CHECK(brush->is_mesh_point_inside(Vec3f(0, 0, 0)));
}

TEST_CASE("Nonuniform scaling keeps the brush radius in world units", "[TriangleSelector][TriangleCursor]")
{
    const auto kind = GENERATE(Selector::CIRCLE, Selector::SPHERE);
    const auto brush = single(kind, 2.f, stretch());
    CHECK(brush->is_mesh_point_inside(Vec3f(0.9f, 0, 0)));
    CHECK_FALSE(brush->is_mesh_point_inside(Vec3f(1.1f, 0, 0)));
    CHECK(brush->is_mesh_point_inside(Vec3f(0, 0.6f, 0)));
    CHECK_FALSE(brush->is_mesh_point_inside(Vec3f(0, 0.7f, 0)));
}

TEST_CASE("Uniform scaling converts a world brush radius to mesh units", "[TriangleSelector][TriangleCursor]")
{
    Transform3d transform = Transform3d::Identity();
    transform.scale(2.);
    const auto kind = GENERATE(Selector::CIRCLE, Selector::SPHERE);
    const auto brush = single(kind, 2.f, transform);
    CHECK(brush->is_mesh_point_inside(Vec3f(0.9f, 0, 0)));
    CHECK_FALSE(brush->is_mesh_point_inside(Vec3f(1.1f, 0, 0)));
}

TEST_CASE("Projected brushes reject back-facing and tangent facets", "[TriangleSelector][TriangleCursor]")
{
    const bool use_stretch = GENERATE(false, true);
    const auto brush = single(Selector::CIRCLE, 2.f, use_stretch ? stretch() : Transform3d::Identity());
    const std::vector<Vec3f> normals{Vec3f::UnitZ(), -Vec3f::UnitZ(), Vec3f::UnitX()};
    CHECK(brush->is_facet_visible(0, normals));
    CHECK_FALSE(brush->is_facet_visible(1, normals));
    CHECK_FALSE(brush->is_facet_visible(2, normals));
}

TEST_CASE("Volume brushes can paint either side of a facet", "[TriangleSelector][TriangleCursor]")
{
    const auto brush = swept(Selector::SPHERE);
    const std::vector<Vec3f> normals{Vec3f::UnitZ(), -Vec3f::UnitZ()};
    CHECK(brush->is_facet_visible(0, normals));
    CHECK(brush->is_facet_visible(1, normals));
}

TEST_CASE("A swept sphere selects both caps and the intervening cylinder", "[TriangleSelector][TriangleCursor]")
{
    const auto brush = swept(Selector::SPHERE);
    for (const Vec3f &point : {Vec3f(-1, 0, 0), Vec3f(7, 0, 0), Vec3f(3, 1, 1)})
        CHECK(brush->is_mesh_point_inside(point));
    for (const Vec3f &point : {Vec3f(-3, 0, 0), Vec3f(9, 0, 0), Vec3f(3, 2, 2)})
        CHECK_FALSE(brush->is_mesh_point_inside(point));
}

TEST_CASE("A swept circle selects a projected capsule at either depth", "[TriangleSelector][TriangleCursor]")
{
    const auto brush = swept(Selector::CIRCLE);
    for (float depth : {-10.f, 0.f, 10.f}) {
        for (float x : {-1.f, 3.f, 7.f})
            CHECK(brush->is_mesh_point_inside(Vec3f(x, 0.5f, depth)));
        CHECK_FALSE(brush->is_mesh_point_inside(Vec3f(3, 3, depth)));
        CHECK_FALSE(brush->is_mesh_point_inside(Vec3f(-3, 0, depth)));
        CHECK_FALSE(brush->is_mesh_point_inside(Vec3f(9, 0, depth)));
    }
}

TEST_CASE("Swept brushes clip their endcaps and middle independently", "[TriangleSelector][TriangleCursor]")
{
    const auto kind = GENERATE(Selector::CIRCLE, Selector::SPHERE);
    const auto brush = swept(kind, Transform3d::Identity(), Plane({0, 1, 0, 0}));
    for (float x : {-1.f, 3.f, 7.f}) {
        CHECK(brush->is_mesh_point_inside(Vec3f(x, -0.5f, 0)));
        CHECK_FALSE(brush->is_mesh_point_inside(Vec3f(x, 0.5f, 0)));
    }
}

TEST_CASE("Nonuniformly scaled swept brushes retain their world thickness", "[TriangleSelector][TriangleCursor]")
{
    const auto kind = GENERATE(Selector::CIRCLE, Selector::SPHERE);
    const auto brush = swept(kind, stretch());
    for (float x : {0.f, 3.f, 6.f}) {
        CHECK(brush->is_mesh_point_inside(Vec3f(x, 0.6f, 0)));
        CHECK_FALSE(brush->is_mesh_point_inside(Vec3f(x, 0.8f, 0)));
    }
}

TEST_CASE("A swept pointer hits a facet under either endpoint", "[TriangleSelector][TriangleCursor]")
{
    const auto kind = GENERATE(Selector::CIRCLE, Selector::SPHERE);
    const auto brush = swept(kind);
    for (float x : {0.f, 6.f})
        CHECK(brush->is_pointer_in_triangle(Vec3f(x-1, -1, 0), Vec3f(x+1, -1, 0), Vec3f(x, 1, 0)));
    CHECK_FALSE(brush->is_pointer_in_triangle(Vec3f(2, -1, 0), Vec3f(4, -1, 0), Vec3f(3, 1, 0)));
}

TEST_CASE("Brush vertex counts distinguish zero through three selected vertices", "[TriangleSelector][TriangleCursor]")
{
    const auto brush = single(Selector::CIRCLE);
    std::array<Vec3f, 3> points{Vec3f(5, 0, 0), Vec3f(0, 5, 0), Vec3f(-5, 0, 0)};
    CHECK(CursorInput::count(*brush, points) == 0);
    for (size_t i = 0; i < points.size(); ++i) {
        points[i] = Vec3f(float(i) * 0.5f, 0, 0);
        CHECK(CursorInput::count(*brush, points) == int(i + 1));
    }
}

TEST_CASE("Circular brushes detect crossing edges whose endpoints are outside", "[TriangleSelector][TriangleCursor]")
{
    const auto brush = single(Selector::CIRCLE);
    CHECK(CursorInput::edge(*brush, {Vec3f(-4, 0, 5), Vec3f(4, 0, 5), Vec3f(4, 5, 5)}));
    CHECK_FALSE(CursorInput::edge(*brush, {Vec3f(4, 3, 5), Vec3f(7, 3, 5), Vec3f(7, 5, 5)}));
}

TEST_CASE("Swept brushes detect edges across the middle of the stroke", "[TriangleSelector][TriangleCursor]")
{
    const auto kind = GENERATE(Selector::CIRCLE, Selector::SPHERE);
    const auto brush = swept(kind);
    CHECK(CursorInput::edge(*brush, {Vec3f(3, -4, 0), Vec3f(3, 4, 0), Vec3f(10, 4, 0)}));
    CHECK_FALSE(CursorInput::edge(*brush, {Vec3f(3, 4, 0), Vec3f(3, 8, 0), Vec3f(10, 8, 0)}));
}

TEST_CASE("Swept brushes detect edges across either round endcap", "[TriangleSelector][TriangleCursor]")
{
    const auto kind = GENERATE(Selector::CIRCLE, Selector::SPHERE);
    const auto brush = swept(kind);
    for (float x : {-1.f, 7.f})
        CHECK(CursorInput::edge(*brush, {Vec3f(x, -4, 0), Vec3f(x, 4, 0), Vec3f(x+0.2f, 4, 0)}));
}

TEST_CASE("Transformed brush edge tests agree with their world-space radius", "[TriangleSelector][TriangleCursor]")
{
    const auto kind = GENERATE(Selector::CIRCLE, Selector::SPHERE);
    const auto brush = swept(kind, stretch());
    CHECK(CursorInput::edge(*brush, {Vec3f(3, -2, 0), Vec3f(3, 2, 0), Vec3f(7, 2, 0)}));
    CHECK_FALSE(CursorInput::edge(*brush, {Vec3f(3, 1, 0), Vec3f(3, 2, 0), Vec3f(7, 2, 0)}));
}

TEST_CASE("Height brushes select a world-space band with a small boundary tolerance", "[TriangleSelector][TriangleCursor]")
{
    const auto brush = Selector::SinglePointCursor::cursor_factory(2.f, Vec3f(0, 0, 10), 3.f, Transform3d::Identity(), Plane());
    for (float z : {1.99f, 2.f, 3.f, 5.f, 5.01f})
        CHECK(brush->is_mesh_point_inside(Vec3f(100, -100, z)));
    for (float z : {1.9f, 5.1f})
        CHECK_FALSE(brush->is_mesh_point_inside(Vec3f(0, 0, z)));
    CHECK_FALSE(brush->is_pointer_in_triangle(Vec3f(-1, -1, 3), Vec3f(1, -1, 3), Vec3f(0, 1, 3)));
}

TEST_CASE("Height brushes apply the complete object transform", "[TriangleSelector][TriangleCursor]")
{
    Transform3d transform = Transform3d::Identity();
    transform.translate(Vec3d(0, 0, 10));
    transform.scale(Vec3d(2, 3, 4));
    const auto brush = Selector::SinglePointCursor::cursor_factory(14.f, Vec3f(0, 0, 30), 4.f, transform, Plane());
    CHECK(brush->is_mesh_point_inside(Vec3f(0, 0, 1.5f)));
    CHECK_FALSE(brush->is_mesh_point_inside(Vec3f(0, 0, 0.5f)));
    CHECK_FALSE(brush->is_mesh_point_inside(Vec3f(0, 0, 2.5f)));
}

TEST_CASE("Height brushes detect triangles spanning the band without an interior vertex", "[TriangleSelector][TriangleCursor]")
{
    const auto brush = Selector::SinglePointCursor::cursor_factory(2.f, Vec3f(0, 0, 10), 3.f, Transform3d::Identity(), Plane());
    CHECK(CursorInput::edge(*brush, {Vec3f(0, 0, 0), Vec3f(1, 0, 8), Vec3f(0, 1, 8)}));
    CHECK_FALSE(CursorInput::edge(*brush, {Vec3f(0, 0, 0), Vec3f(1, 0, 1), Vec3f(0, 1, 1)}));
    CHECK_FALSE(CursorInput::edge(*brush, {Vec3f(0, 0, 6), Vec3f(1, 0, 8), Vec3f(0, 1, 8)}));
    CHECK(CursorInput::edge(*brush, {Vec3f(0, 0, 5.00005f), Vec3f(1, 0, 5.00005f), Vec3f(0, 1, 5.00005f)}));
    CHECK_FALSE(CursorInput::edge(*brush, {Vec3f(0, 0, 5.0002f), Vec3f(1, 0, 5.0002f), Vec3f(0, 1, 5.0002f)}));
}

TEST_CASE("Cursor round ends exclude exact radius while capsule middle includes its boundary", "[TriangleSelector][TriangleCursor]")
{
    const auto single_kind = GENERATE(Selector::CIRCLE, Selector::SPHERE);
    const auto single_brush = single(single_kind);
    CHECK(single_brush->is_mesh_point_inside(Vec3f(1.999f, 0, 0)));
    CHECK_FALSE(single_brush->is_mesh_point_inside(Vec3f(2.f, 0, 0)));

    const auto swept_kind = GENERATE(Selector::CIRCLE, Selector::SPHERE);
    const auto swept_brush = swept(swept_kind);
    CHECK(swept_brush->is_mesh_point_inside(Vec3f(-1.999f, 0, 0)));
    CHECK_FALSE(swept_brush->is_mesh_point_inside(Vec3f(-2.f, 0, 0)));
    CHECK(swept_brush->is_mesh_point_inside(Vec3f(7.999f, 0, 0)));
    CHECK_FALSE(swept_brush->is_mesh_point_inside(Vec3f(8.f, 0, 0)));
    CHECK(swept_brush->is_mesh_point_inside(Vec3f(3.f, 2.f, 0)));
    CHECK_FALSE(swept_brush->is_mesh_point_inside(Vec3f(3.f, 2.001f, 0)));
}

} // namespace triangle_cursor_contracts
