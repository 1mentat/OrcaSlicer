#include <catch2/catch_all.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_message.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/generators/catch_generators.hpp>
#include "libslic3r/TriangleSelector.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/Geometry.hpp"
#include "libslic3r/Point.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <memory>
#include <optional>
#include <utility>
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

namespace triangle_selector_painting {

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

// The second facet is lifted along the shared diagonal, so the two facet normals meet at
// acos(1 / sqrt(3)), about 54.7 degrees.
indexed_triangle_set folded_its()
{
    indexed_triangle_set result = square_its();
    result.vertices[3] = vertex(0.f, 10.f, 10.f);
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

// Reads selector internals the same way the painting gizmo subclass does: seed fill contour
// coordinates and the triangle storage that garbage collection compacts.
struct InspectableSelector : TriangleSelector
{
    using TriangleSelector::TriangleSelector;

    size_t triangle_count() const { return m_triangles.size(); }
    size_t invalid_triangle_count() const
    {
        return size_t(std::count_if(m_triangles.begin(), m_triangles.end(), [](const Triangle &tr) { return !tr.valid(); }));
    }

    std::vector<Vec3f> contour_points() const
    {
        std::vector<Vec3f> points;
        for (const Vec2i32 &edge : get_seed_fill_contour()) {
            points.push_back(m_vertices[edge(0)].v);
            points.push_back(m_vertices[edge(1)].v);
        }
        return points;
    }
};

} // namespace

TEST_CASE("A sphere brush covering the whole mesh paints every facet whole", "[TriangleSelector]")
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

TEST_CASE("A partial brush paints the part of the square inside its radius", "[TriangleSelector]")
{
    const TriangleMesh mesh = square_mesh();
    TriangleSelector selector(mesh);
    select_patch(selector, Vec3f(2.f, 2.f, 0.f), 3.f, true);

    const double selected = painted_area(selector, EnforcerBlockerType::ENFORCER);
    const double remaining = painted_area(selector, EnforcerBlockerType::NONE);
    // The radius-3 disk centred at (2, 2) loses two circular segments past x = 0 and y = 0
    // (9 acos(2/3) - 2 sqrt(5) each) and gets back their small overlap below the origin.
    const double disk_in_square = 9. * PI - 2. * (9. * std::acos(2. / 3.) - 2. * std::sqrt(5.)) + 0.03;
    CHECK_THAT(selected, Catch::Matchers::WithinAbs(disk_in_square, 0.5));
    CHECK_THAT(selected + remaining, Catch::Matchers::WithinAbs(100., 1e-4));

    std::vector<int> sources;
    const indexed_triangle_set strict = selector.get_facets_strict(EnforcerBlockerType::ENFORCER, &sources);
    REQUIRE(sources.size() == strict.indices.size());
    for (int source : sources) {
        CHECK(source >= 0);
        CHECK(source <= 1);
    }
}

TEST_CASE("Seed fill follows coplanar adjacency and applies its pending state", "[TriangleSelector]")
{
    const TriangleMesh mesh = square_mesh();
    TriangleSelector selector(mesh);
    selector.seed_fill_select_triangles(Vec3f(7.f, 2.f, 0.f), 0, Transform3d::Identity(),
                                        TriangleSelector::ClippingPlane(), 180.f);
    // Both coplanar facets are selected, so there is no internal contour.
    CHECK(selector.get_seed_fill_contour().empty());

    selector.seed_fill_apply_on_triangles(EnforcerBlockerType::Extruder4);
    CHECK(selector.num_facets(EnforcerBlockerType::Extruder4) == 2);
    CHECK(selector.num_facets(EnforcerBlockerType::NONE) == 0);
}

TEST_CASE("Seed fill clipping leaves the clipped neighbor untouched", "[TriangleSelector]")
{
    const TriangleMesh mesh = square_mesh();
    TriangleSelector selector(mesh);
    // Clips points with y - x > 5: only the far corner (0, 10) of facet 1, none of facet 0.
    const TriangleSelector::ClippingPlane clip(std::array<float, 4>{ -1.f, 1.f, 0.f, 5.f });
    selector.seed_fill_select_triangles(Vec3f(7.f, 2.f, 0.f), 0, Transform3d::Identity(), clip, 180.f);
    const auto contour = selector.get_seed_fill_contour();
    REQUIRE_FALSE(contour.empty());

    selector.seed_fill_apply_on_triangles(EnforcerBlockerType::ENFORCER);
    CHECK(selector.num_facets(EnforcerBlockerType::ENFORCER) == 1);
    CHECK(selector.num_facets(EnforcerBlockerType::NONE) == 1);
}

TEST_CASE("Bucket fill distinguishes local selection from propagated selection", "[TriangleSelector]")
{
    SECTION("propagate marks both coplanar facets")
    {
        const TriangleMesh mesh = square_mesh();
        TriangleSelector selector(mesh);
        selector.bucket_fill_select_triangles(Vec3f(7.f, 2.f, 0.f), 0,
                                              TriangleSelector::ClippingPlane(), 180.f, true, true);
        selector.seed_fill_apply_on_triangles(EnforcerBlockerType::BLOCKER);
        CHECK(selector.num_facets(EnforcerBlockerType::BLOCKER) == 2);
    }

    SECTION("local mode marks only the hit facet")
    {
        const TriangleMesh mesh = square_mesh();
        TriangleSelector selector(mesh);
        selector.bucket_fill_select_triangles(Vec3f(7.f, 2.f, 0.f), 0,
                                              TriangleSelector::ClippingPlane(), 180.f, false, true);
        selector.seed_fill_apply_on_triangles(EnforcerBlockerType::BLOCKER);
        CHECK(selector.num_facets(EnforcerBlockerType::BLOCKER) == 1);
        CHECK(selector.num_facets(EnforcerBlockerType::NONE) == 1);
    }
}

TEST_CASE("Seed and bucket fill stop at a fold sharper than the angle limit", "[TriangleSelector]")
{
    const TriangleMesh mesh(folded_its());
    for (const auto &[angle, expected] : {std::pair<float, int>{45.f, 1}, std::pair<float, int>{65.f, 2}}) {
        CAPTURE(angle);
        TriangleSelector seed(mesh);
        seed.seed_fill_select_triangles(Vec3f(7.f, 2.f, 0.f), 0, Transform3d::Identity(),
                                        TriangleSelector::ClippingPlane(), angle);
        seed.seed_fill_apply_on_triangles(EnforcerBlockerType::ENFORCER);
        CHECK(seed.num_facets(EnforcerBlockerType::ENFORCER) == expected);

        TriangleSelector bucket(mesh);
        bucket.bucket_fill_select_triangles(Vec3f(7.f, 2.f, 0.f), 0,
                                            TriangleSelector::ClippingPlane(), angle, true, true);
        bucket.seed_fill_apply_on_triangles(EnforcerBlockerType::ENFORCER);
        CHECK(bucket.num_facets(EnforcerBlockerType::ENFORCER) == expected);
    }
}

TEST_CASE("State shifts and remapping preserve threshold semantics", "[TriangleSelector]")
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

TEST_CASE("Grouped facet extraction keeps each whole-face state", "[TriangleSelector]")
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

TEST_CASE("Remapping painting across a translated mesh preserves existing states", "[TriangleSelector]")
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
    // Remapping repaints the source facet through a cursor that may overlap the shared diagonal
    // slightly, so each half may move by a fraction of a square millimetre.
    CHECK_THAT(restored_enforcer + restored_blocker, Catch::Matchers::WithinAbs(100., 1e-3));
    CHECK_THAT(restored_enforcer, Catch::Matchers::WithinAbs(50., 0.5));
    CHECK_THAT(restored_blocker, Catch::Matchers::WithinAbs(50., 0.5));
}

TEST_CASE("Remapping empty painting does not invent target facets", "[TriangleSelector]")
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

TEST_CASE("Recursive bucket fill propagates through split same-state leaves", "[TriangleSelector][TriangleSelectorRecursive]")
{
    const TriangleMesh mesh = square_mesh();
    TriangleSelector selector(mesh);
    select_patch(selector, Vec3f(2.f, 2.f, 0.f), 3.f, true);

    const double enforcer_before = painted_area(selector, EnforcerBlockerType::ENFORCER);
    const double none_before = painted_area(selector, EnforcerBlockerType::NONE);
    REQUIRE(enforcer_before > 0.);
    REQUIRE(none_before > 0.);
    CHECK_THAT(enforcer_before + none_before, Catch::Matchers::WithinAbs(100., 1e-3));

    TriangleSelector local(mesh);
    local.deserialize(selector.serialize());
    local.bucket_fill_select_triangles(Vec3f(8.f, 1.f, 0.f), 0,
                                       TriangleSelector::ClippingPlane(), 180.f, false, true);
    local.seed_fill_apply_on_triangles(EnforcerBlockerType::BLOCKER);
    const double local_area = painted_area(local, EnforcerBlockerType::BLOCKER);
    REQUIRE(local_area > 0.);
    REQUIRE(local_area < none_before);
    CHECK_THAT(painted_area(local, EnforcerBlockerType::ENFORCER),
               Catch::Matchers::WithinAbs(enforcer_before, 1e-3));

    selector.bucket_fill_select_triangles(Vec3f(8.f, 1.f, 0.f), 0,
                                          TriangleSelector::ClippingPlane(), 180.f, true, true);
    selector.seed_fill_apply_on_triangles(EnforcerBlockerType::BLOCKER);

    CHECK_THAT(painted_area(selector, EnforcerBlockerType::ENFORCER),
               Catch::Matchers::WithinAbs(enforcer_before, 1e-3));
    CHECK_THAT(painted_area(selector, EnforcerBlockerType::BLOCKER),
               Catch::Matchers::WithinAbs(none_before, 1e-3));
    CHECK_THAT(painted_area(selector, EnforcerBlockerType::NONE),
               Catch::Matchers::WithinAbs(0., 1e-3));
    CHECK_THAT(painted_area(selector, EnforcerBlockerType::ENFORCER) +
                   painted_area(selector, EnforcerBlockerType::BLOCKER),
               Catch::Matchers::WithinAbs(100., 1e-3));
}

TEST_CASE("Recursive bucket fill stops at a different-state split-tree boundary", "[TriangleSelector][TriangleSelectorRecursive]")
{
    const TriangleMesh mesh = square_mesh();
    TriangleSelector selector(mesh);
    select_patch(selector, Vec3f(2.f, 2.f, 0.f), 3.f, true);
    selector.set_facet(1, EnforcerBlockerType::BLOCKER);

    const double enforcer_before = painted_area(selector, EnforcerBlockerType::ENFORCER);
    const double blocker_before = painted_area(selector, EnforcerBlockerType::BLOCKER);
    const double none_before = painted_area(selector, EnforcerBlockerType::NONE);
    REQUIRE(enforcer_before > 0.);
    REQUIRE_THAT(blocker_before, Catch::Matchers::WithinAbs(50., 1e-3));
    REQUIRE(none_before > 0.);

    selector.bucket_fill_select_triangles(Vec3f(8.f, 1.f, 0.f), 0,
                                          TriangleSelector::ClippingPlane(), 180.f, true, true);
    selector.seed_fill_apply_on_triangles(EnforcerBlockerType::Extruder4);

    const double filled_area = painted_area(selector, EnforcerBlockerType::Extruder4);
    CHECK_THAT(painted_area(selector, EnforcerBlockerType::ENFORCER),
               Catch::Matchers::WithinAbs(enforcer_before, 1e-3));
    CHECK_THAT(painted_area(selector, EnforcerBlockerType::BLOCKER),
               Catch::Matchers::WithinAbs(blocker_before, 1e-3));
    CHECK_THAT(filled_area, Catch::Matchers::WithinAbs(none_before, 1e-3));
    CHECK_THAT(painted_area(selector, EnforcerBlockerType::NONE),
               Catch::Matchers::WithinAbs(0., 1e-3));
    CHECK_THAT(painted_area(selector, EnforcerBlockerType::ENFORCER) + blocker_before + filled_area,
               Catch::Matchers::WithinAbs(100., 1e-3));
}

TEST_CASE("Bucket fill contours follow a boundary through recursively split facets", "[TriangleSelector][TriangleSelectorRecursive]")
{
    const TriangleMesh mesh = square_mesh();
    InspectableSelector selector(mesh);
    select_patch(selector, Vec3f(2.f, 2.f, 0.f), 3.f, true);
    const double enforcer_before = painted_area(selector, EnforcerBlockerType::ENFORCER);
    REQUIRE(enforcer_before > 0.);

    selector.bucket_fill_select_triangles(Vec3f(8.f, 1.f, 0.f), 0,
                                          TriangleSelector::ClippingPlane(), 180.f, true, true);
    const std::vector<Vec2i32> contour = selector.get_seed_fill_contour();
    REQUIRE_FALSE(contour.empty());
    for (const Vec2i32 &edge : contour) {
        CHECK(edge(0) >= 0);
        CHECK(edge(1) >= 0);
        CHECK(edge(0) != edge(1));
    }
    // The fill boundary is the edge of the radius-3 patch, not the square's outline or the
    // shared diagonal; the slack covers the split triangles straddling the brush edge.
    const std::vector<Vec3f> points = selector.contour_points();
    CHECK(std::all_of(points.begin(), points.end(), [](const Vec3f &point) {
        const float distance = (point - Vec3f(2.f, 2.f, 0.f)).norm();
        return distance > 2.5f && distance < 3.5f;
    }));

    selector.seed_fill_apply_on_triangles(EnforcerBlockerType::BLOCKER);
    CHECK_THAT(painted_area(selector, EnforcerBlockerType::ENFORCER),
               Catch::Matchers::WithinAbs(enforcer_before, 1e-3));
    CHECK_THAT(painted_area(selector, EnforcerBlockerType::NONE),
               Catch::Matchers::WithinAbs(0., 1e-3));
    CHECK_THAT(painted_area(selector, EnforcerBlockerType::ENFORCER) +
                   painted_area(selector, EnforcerBlockerType::BLOCKER),
               Catch::Matchers::WithinAbs(100., 1e-3));
}

TEST_CASE("Repainting split facets whole discards their split trees", "[TriangleSelector]")
{
    const TriangleMesh mesh = square_mesh();
    TriangleSelector selector(mesh);
    select_patch(selector, Vec3f::Zero(), 2.f, true);
    REQUIRE_FALSE(selector.serialize().triangles_to_split.empty());
    REQUIRE(selector.num_facets(EnforcerBlockerType::ENFORCER) > 2);

    selector.set_facet(0, EnforcerBlockerType::BLOCKER);
    selector.set_facet(1, EnforcerBlockerType::BLOCKER);
    TriangleSelector fresh(mesh);
    fresh.set_facet(0, EnforcerBlockerType::BLOCKER);
    fresh.set_facet(1, EnforcerBlockerType::BLOCKER);
    CHECK(selector.serialize() == fresh.serialize());
    CHECK(selector.num_facets(EnforcerBlockerType::BLOCKER) == 2);
    CHECK(selector.num_facets(EnforcerBlockerType::ENFORCER) == 0);
    CHECK(selector.num_facets(EnforcerBlockerType::NONE) == 0);
}

TEST_CASE("Garbage collection compacts discarded split children without changing the painting", "[TriangleSelector]")
{
    const TriangleMesh mesh = square_mesh();
    InspectableSelector selector(mesh);
    select_patch(selector, Vec3f::Zero(), 2.f, true);
    // Collapsing facet 1 leaves its split children behind as garbage while facet 0 keeps a live
    // split tree whose child indices garbage collection has to renumber.
    selector.set_facet(1, EnforcerBlockerType::BLOCKER);
    const auto before = selector.serialize();
    REQUIRE_FALSE(before.triangles_to_split.empty());
    const double enforcer_before = painted_area(selector, EnforcerBlockerType::ENFORCER);
    const double none_before     = painted_area(selector, EnforcerBlockerType::NONE);
    REQUIRE(enforcer_before > 0.);
    REQUIRE(none_before > 0.);
    const size_t triangles_before = selector.triangle_count();
    const size_t garbage_before   = selector.invalid_triangle_count();
    REQUIRE(garbage_before > 0);

    selector.garbage_collect();
    CHECK(selector.invalid_triangle_count() == 0);
    CHECK(selector.triangle_count() == triangles_before - garbage_before);
    // The surviving split tree of facet 0 serializes identically only if its child indices were
    // renumbered to the compacted positions.
    CHECK(selector.serialize() == before);
    CHECK_THAT(painted_area(selector, EnforcerBlockerType::ENFORCER), Catch::Matchers::WithinAbs(enforcer_before, 1e-6));
    CHECK_THAT(painted_area(selector, EnforcerBlockerType::NONE), Catch::Matchers::WithinAbs(none_before, 1e-6));
    CHECK_THAT(painted_area(selector, EnforcerBlockerType::BLOCKER), Catch::Matchers::WithinAbs(50., 1e-4));
}

TEST_CASE("Used paint states are extracted in state order without duplicates", "[TriangleSelector]")
{
    TriangleSelector::TriangleSplittingData data;
    data.used_states.assign(size_t(EnforcerBlockerType::ExtruderMax) + 1, false);
    CHECK(TriangleSelector::extract_used_facet_states(data).empty());
    data.used_states[32] = true;
    data.used_states[1] = true;
    data.used_states[17] = true;
    const std::vector<EnforcerBlockerType> expected{
        EnforcerBlockerType::Extruder1, EnforcerBlockerType::Extruder17, EnforcerBlockerType::Extruder32};
    CHECK(TriangleSelector::extract_used_facet_states(data) == expected);
}

} // namespace triangle_selector_painting

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
    // Only the x = 3 edge comes within the radius, and it stays more than a radius away from
    // both centres (0,0,0) and (6,0,0), so only the cylinder/rectangle test can accept it.
    CHECK(CursorInput::edge(*brush, {Vec3f(3, -4, 0), Vec3f(3, 4, 0), Vec3f(3.1f, 4, 0)}));
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
    // In world space the stroke runs from (14,-6,20) to (26,-6,20). The positive triangle maps to
    // x ~ 20, at least 4 from either centre, so it can only hit the middle of the stroke.
    CHECK(CursorInput::edge(*brush, {Vec3f(3, -2, 0), Vec3f(3, 2, 0), Vec3f(3.1f, 2, 0)}));
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
    // The edge test widens the [2, 5] band by EPSILON on each side.
    const float just_inside  = float(5. + EPSILON / 2.);
    const float just_outside = float(5. + EPSILON * 2.);
    CHECK(CursorInput::edge(*brush, {Vec3f(0, 0, just_inside), Vec3f(1, 0, just_inside), Vec3f(0, 1, just_inside)}));
    CHECK_FALSE(CursorInput::edge(*brush, {Vec3f(0, 0, just_outside), Vec3f(1, 0, just_outside), Vec3f(0, 1, just_outside)}));
}

TEST_CASE("Cursor round ends exclude exact radius while capsule middle includes its boundary", "[TriangleSelector][TriangleCursor]")
{
    // Pins current behavior: the round ends compare with `<` and the capsule body with `<=`.
    // Nothing relies on the difference; update this test if the comparisons are unified.
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
