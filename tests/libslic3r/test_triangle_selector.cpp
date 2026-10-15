#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "libslic3r/Geometry.hpp"
#include <array>
#include <cmath>
#include <functional>
#include <optional>
#include <memory>
#include "libslic3r/Point.hpp"
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/catch_all.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_message.hpp>
#include <catch2/generators/catch_generators.hpp>
#include "libslic3r/TriangleSelector.hpp"
#include "libslic3r/TriangleMesh.hpp"

#include <algorithm>
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

namespace triangle_selector_extended_contracts {

namespace {

stl_vertex vertex(float x, float y, float z)
{
    stl_vertex result;
    result << x, y, z;
    return result;
}

stl_triangle_vertex_indices face(int a, int b, int c)
{
    stl_triangle_vertex_indices result;
    result << a, b, c;
    return result;
}

// Two coplanar triangles make the adjacency, clipping, and partial-brush
// expectations analytic while still exercising the selector's split tree.
indexed_triangle_set square_its()
{
    indexed_triangle_set result;
    result.vertices = { vertex(0.f, 0.f, 0.f), vertex(10.f, 0.f, 0.f),
                        vertex(10.f, 10.f, 0.f), vertex(0.f, 10.f, 0.f) };
    result.indices = { face(0, 1, 2), face(0, 2, 3) };
    return result;
}

indexed_triangle_set translated_square_its(float x_offset)
{
    indexed_triangle_set result = square_its();
    for (stl_vertex &v : result.vertices)
        v.x() += x_offset;
    return result;
}

double area(const indexed_triangle_set &mesh)
{
    double result = 0.;
    for (const stl_triangle_vertex_indices &f : mesh.indices) {
        const stl_vertex &a = mesh.vertices[f(0)];
        const stl_vertex &b = mesh.vertices[f(1)];
        const stl_vertex &c = mesh.vertices[f(2)];
        result += .5 * double((b - a).cross(c - a).norm());
    }
    return result;
}

double painted_area(const TriangleSelector &selector, EnforcerBlockerType state)
{
    return area(selector.get_facets_strict(state));
}

TriangleMesh square_mesh()
{
    return TriangleMesh(square_its());
}

std::unique_ptr<TriangleSelector::Cursor> sphere_cursor(const Vec3f &center, float radius)
{
    return TriangleSelector::SinglePointCursor::cursor_factory(
        center, center + Vec3f(0.f, 0.f, 20.f), radius, TriangleSelector::CursorType::SPHERE,
        Transform3d::Identity(), TriangleSelector::ClippingPlane());
}

void select_patch(TriangleSelector &selector, const Vec3f &center, float radius, bool splitting)
{
    selector.select_patch(0, sphere_cursor(center, radius), EnforcerBlockerType::ENFORCER,
                          Transform3d::Identity(), splitting, 0.f, Vec3f::UnitZ(),
                          /* select_partially */ true);
}

TriangleSelector::TriangleSplittingData paint_first_face(const indexed_triangle_set &its,
                                                         EnforcerBlockerType state)
{
    const TriangleMesh mesh(its);
    TriangleSelector selector(mesh);
    selector.set_facet(0, state);
    return selector.serialize();
}

} // namespace

TEST_CASE("A large sphere brush selects every connected coplanar facet", "[TriangleSelector][TriangleSelectorExtended]")
{
    const TriangleMesh mesh = square_mesh();
    TriangleSelector selector(mesh);
    select_patch(selector, Vec3f(2.f, 2.f, 0.f), 20.f, true);

    CHECK(selector.num_facets(EnforcerBlockerType::ENFORCER) == 2);
    CHECK(selector.num_facets(EnforcerBlockerType::NONE) == 0);
    CHECK_THAT(painted_area(selector, EnforcerBlockerType::ENFORCER),
               Catch::Matchers::WithinAbs(100., 1e-4));
    CHECK(selector.select_unsplit_triangle(Vec3f(2.f, 2.f, 0.f), 0) >= 0);
}

TEST_CASE("A partial brush conserves area across strict painted facets", "[TriangleSelector][TriangleSelectorExtended]")
{
    const TriangleMesh mesh = square_mesh();
    TriangleSelector selector(mesh);
    select_patch(selector, Vec3f(2.f, 2.f, 0.f), 3.f, true);

    const double selected = painted_area(selector, EnforcerBlockerType::ENFORCER);
    const double remaining = painted_area(selector, EnforcerBlockerType::NONE);
    REQUIRE(selected > 0.);
    REQUIRE(remaining > 0.);
    CHECK_THAT(selected + remaining, Catch::Matchers::WithinAbs(100., 1e-4));

    std::vector<int> sources;
    const indexed_triangle_set strict = selector.get_facets_strict(EnforcerBlockerType::ENFORCER, &sources);
    REQUIRE(sources.size() == strict.indices.size());
    for (int source : sources) {
        CHECK(source >= 0);
        CHECK(source <= 1);
    }
}

TEST_CASE("Seed fill follows coplanar adjacency and applies its pending state", "[TriangleSelector][TriangleSelectorExtended]")
{
    const TriangleMesh mesh = square_mesh();
    TriangleSelector selector(mesh);
    selector.seed_fill_select_triangles(Vec3f(2.f, 2.f, 0.f), 0, Transform3d::Identity(),
                                        TriangleSelector::ClippingPlane(), 180.f);
    // Both coplanar facets are selected, so there is no internal contour.
    CHECK(selector.get_seed_fill_contour().empty());

    selector.seed_fill_apply_on_triangles(EnforcerBlockerType::Extruder4);
    CHECK(selector.num_facets(EnforcerBlockerType::Extruder4) == 2);
    CHECK(selector.num_facets(EnforcerBlockerType::NONE) == 0);
}

TEST_CASE("Seed fill clipping leaves the clipped neighbor untouched", "[TriangleSelector][TriangleSelectorExtended]")
{
    const TriangleMesh mesh = square_mesh();
    TriangleSelector selector(mesh);
    const TriangleSelector::ClippingPlane clip(std::array<float, 4>{ 0.f, 0.f, 1.f, -1.f });
    selector.seed_fill_select_triangles(Vec3f(2.f, 2.f, 0.f), 0, Transform3d::Identity(), clip, 180.f);
    const auto contour = selector.get_seed_fill_contour();
    REQUIRE_FALSE(contour.empty());

    selector.seed_fill_apply_on_triangles(EnforcerBlockerType::ENFORCER);
    CHECK(selector.num_facets(EnforcerBlockerType::ENFORCER) == 1);
    CHECK(selector.num_facets(EnforcerBlockerType::NONE) == 1);
}

TEST_CASE("Bucket fill distinguishes local selection from propagated selection", "[TriangleSelector][TriangleSelectorExtended]")
{
    SECTION("propagate marks both coplanar facets")
    {
        const TriangleMesh mesh = square_mesh();
        TriangleSelector selector(mesh);
        selector.bucket_fill_select_triangles(Vec3f(2.f, 2.f, 0.f), 0,
                                              TriangleSelector::ClippingPlane(), 180.f, true, true);
        selector.seed_fill_apply_on_triangles(EnforcerBlockerType::BLOCKER);
        CHECK(selector.num_facets(EnforcerBlockerType::BLOCKER) == 2);
    }

    SECTION("local mode marks only the hit facet")
    {
        const TriangleMesh mesh = square_mesh();
        TriangleSelector selector(mesh);
        selector.bucket_fill_select_triangles(Vec3f(2.f, 2.f, 0.f), 0,
                                              TriangleSelector::ClippingPlane(), 180.f, false, true);
        selector.seed_fill_apply_on_triangles(EnforcerBlockerType::BLOCKER);
        CHECK(selector.num_facets(EnforcerBlockerType::BLOCKER) == 1);
        CHECK(selector.num_facets(EnforcerBlockerType::NONE) == 1);
    }
}

TEST_CASE("State shifts and remapping preserve threshold semantics", "[TriangleSelector][TriangleSelectorExtended]")
{
    const TriangleMesh mesh = square_mesh();
    TriangleSelector selector(mesh);
    selector.set_facet(0, EnforcerBlockerType::Extruder2);
    selector.set_facet(1, EnforcerBlockerType::Extruder5);

    selector.shift_states_above(EnforcerBlockerType::Extruder5, 2);
    CHECK(selector.num_facets(EnforcerBlockerType::Extruder2) == 1);
    CHECK(selector.num_facets(EnforcerBlockerType::Extruder7) == 1);

    EnforcerBlockerStateMap state_map{};
    for (size_t i = 0; i < state_map.size(); ++i)
        state_map[i] = EnforcerBlockerType(i);
    state_map[size_t(EnforcerBlockerType::Extruder2)] = EnforcerBlockerType::Extruder8;
    state_map[size_t(EnforcerBlockerType::Extruder7)] = EnforcerBlockerType::Extruder9;
    selector.remap_triangle_state(state_map);
    CHECK(selector.num_facets(EnforcerBlockerType::Extruder8) == 1);
    CHECK(selector.num_facets(EnforcerBlockerType::Extruder9) == 1);
}

TEST_CASE("Grouped facet extraction keeps each whole-face state", "[TriangleSelector][TriangleSelectorExtended]")
{
    const TriangleMesh mesh = square_mesh();
    TriangleSelector selector(mesh);
    selector.set_facet(0, EnforcerBlockerType::ENFORCER);
    selector.set_facet(1, EnforcerBlockerType::BLOCKER);

    std::vector<indexed_triangle_set> grouped;
    selector.get_facets(grouped);
    REQUIRE(grouped.size() > size_t(EnforcerBlockerType::BLOCKER));
    CHECK(grouped[size_t(EnforcerBlockerType::NONE)].indices.empty());
    CHECK(grouped[size_t(EnforcerBlockerType::ENFORCER)].indices.size() == 1);
    CHECK(grouped[size_t(EnforcerBlockerType::BLOCKER)].indices.size() == 1);
    CHECK_THAT(area(grouped[size_t(EnforcerBlockerType::ENFORCER)]), Catch::Matchers::WithinAbs(50., 1e-4));
    CHECK_THAT(area(grouped[size_t(EnforcerBlockerType::BLOCKER)]), Catch::Matchers::WithinAbs(50., 1e-4));
}

TEST_CASE("Remapping painting across a translated mesh preserves existing states", "[TriangleSelector][TriangleSelectorExtended]")
{
    const indexed_triangle_set source_its = square_its();
    const indexed_triangle_set target_its = translated_square_its(20.f);
    const TriangleSelector::TriangleSplittingData source_painting =
        paint_first_face(source_its, EnforcerBlockerType::ENFORCER);
    const TriangleSelector::TriangleSplittingData existing_painting =
        [&target_its] {
            const TriangleMesh mesh(target_its);
            TriangleSelector selector(mesh);
            selector.set_facet(1, EnforcerBlockerType::BLOCKER);
            return selector.serialize();
        }();
    const auto existing = std::optional<std::reference_wrapper<const TriangleSelector::TriangleSplittingData>>(
        std::cref(existing_painting));

    const TriangleSelector::TriangleSplittingData result = TriangleSelector::remap_painting(
        source_its, source_painting, target_its, Geometry::translation_transform(Vec3d(-20., 0., 0.)), existing);
    const TriangleMesh target_mesh(target_its);
    TriangleSelector restored{target_mesh};
    restored.deserialize(result);

    const double restored_enforcer = painted_area(restored, EnforcerBlockerType::ENFORCER);
    const double restored_blocker = painted_area(restored, EnforcerBlockerType::BLOCKER);
    // Remapping rebuilds split triangles, so the partition boundary can move by a
    // sub-millimeter amount.  The independent oracle is class preservation plus
    // conservation of the complete translated square.
    CHECK_THAT(restored_enforcer + restored_blocker, Catch::Matchers::WithinAbs(100., 1e-3));
    CHECK(restored_enforcer > 49.);
    CHECK(restored_blocker > 49.);
}

TEST_CASE("Remapping empty painting does not invent target facets", "[TriangleSelector][TriangleSelectorExtended]")
{
    const indexed_triangle_set source_its = square_its();
    const indexed_triangle_set target_its = translated_square_its(20.f);
    const TriangleSelector::TriangleSplittingData empty_painting =
        TriangleSelector(TriangleMesh(source_its)).serialize();

    const TriangleSelector::TriangleSplittingData result = TriangleSelector::remap_painting(
        source_its, empty_painting, target_its, Geometry::translation_transform(Vec3d(-20., 0., 0.)), std::nullopt);
    CHECK(result.bitstream.empty());
    CHECK(result.triangles_to_split.empty());
}

} // namespace triangle_selector_extended_contracts
