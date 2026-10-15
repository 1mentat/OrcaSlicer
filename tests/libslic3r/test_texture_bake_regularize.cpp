#include "catch2/matchers/catch_matchers.hpp"
#include "libslic3r/TextureBake/TextureBakeIndex.hpp"
#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <array>
#include <cstddef>
#include <vector>

#include "libslic3r/Point.hpp"
#include "libslic3r/TextureBake/TextureBakeRegularize.hpp"

using namespace Slic3r;
using namespace Slic3r::TextureBake;

namespace {

void add_triangle(TriSoup &soup, const Vec3f &a, const Vec3f &b, const Vec3f &c,
                  bool include_excluded = false, float excluded = 0.f)
{
    soup.pos.insert(soup.pos.end(), { a, b, c });
    if (include_excluded)
        soup.exclude_weight.insert(soup.exclude_weight.end(), { excluded, excluded, excluded });
}

// Four triangles contain one extremely short interior edge u-v. Its two wing
// faces are slivers; T2/T3 remain as valid faces after the midpoint collapse.
TriSoup sliver_chain(bool shared_link = false, bool excluded_wings = false)
{
    const Vec3f u { -0.1f, 0.f, 0.f };
    const Vec3f v {  0.1f, 0.f, 0.f };
    const Vec3f a {  0.f, 10.f, 0.f };
    const Vec3f b {  0.f, -10.f, 0.f };
    const Vec3f c { -10.f, 0.f, 0.f };
    const Vec3f d = shared_link ? c : Vec3f { 10.f, 0.f, 0.f };

    TriSoup soup;
    if (excluded_wings)
        soup.exclude_weight.reserve(12);
    add_triangle(soup, u, v, a, excluded_wings, excluded_wings ? 1.f : 0.f);
    add_triangle(soup, v, u, b, excluded_wings, excluded_wings ? 1.f : 0.f);
    if (shared_link) {
        // Keep c as a common one-ring neighbor of u and v so u-v fails the
        // link condition, while making the other edges of the two sliver
        // wings boundaries.  Otherwise u-a or v-b is a legal interior edge
        // and the regularizer can collapse that edge after rejecting u-v.
        add_triangle(soup, u, c, { -10.f, 10.f, 0.f }, excluded_wings);
        add_triangle(soup, v, c, { -10.f, -10.f, 0.f }, excluded_wings);
    } else {
        add_triangle(soup, u, a, c, excluded_wings);
        add_triangle(soup, v, b, d, excluded_wings);
    }
    return soup;
}

TriSoup sharp_edge_pair()
{
    TriSoup soup;
    add_triangle(soup, { 0.f, 0.f, 0.f }, { .1f, 0.f, 0.f }, { 0.f, 1.f, 0.f });
    add_triangle(soup, { .1f, 0.f, 0.f }, { 0.f, 0.f, 0.f }, { 0.f, 0.f, 1.f });
    return soup;
}

void check_same_positions(const TriSoup &a, const TriSoup &b)
{
    REQUIRE(a.pos.size() == b.pos.size());
    REQUIRE(a.exclude_weight.size() == b.exclude_weight.size());
    for (std::size_t i = 0; i < a.pos.size(); ++i)
        CHECK_THAT((a.pos[i] - b.pos[i]).norm(), Catch::Matchers::WithinAbs(0., 1e-7));
    for (std::size_t i = 0; i < a.exclude_weight.size(); ++i)
        CHECK_THAT(a.exclude_weight[i], Catch::Matchers::WithinAbs(b.exclude_weight[i], 1e-7));
}

} // namespace

TEST_CASE("Texture bake regularization collapses a sliver edge and preserves surviving parents", "[TextureBakeRegularize]")
{
    const TriSoup input = sliver_chain();
    const std::vector<int> parents { 100, 101, 102, 103 };
    RegularizeOptions options;
    options.maxrounds = 1;

    const RegularizeResult result = regularize_mesh(input, parents, 20., options);
    REQUIRE(result.collapse_count == 1);
    REQUIRE(result.geometry.triangle_count() == 2);
    REQUIRE(result.face_parent_id == std::vector<int> { 102, 103 });
    REQUIRE(result.geometry.pos.size() == 6);
    CHECK_THAT(result.geometry.pos[0].x(), Catch::Matchers::WithinAbs(0., 1e-6));
    CHECK_THAT(result.geometry.pos[0].y(), Catch::Matchers::WithinAbs(0., 1e-6));
    REQUIRE(result.geometry.nrm.size() == result.geometry.pos.size());
    for (const Vec3f &normal : result.geometry.nrm)
        CHECK(normal.norm() > 0.f);
}

TEST_CASE("Texture bake regularization leaves invalid requests unchanged", "[TextureBakeRegularize]")
{
    const TriSoup input = sliver_chain();
    const std::vector<int> parents { 100, 101, 102, 103 };

    const RegularizeResult empty = regularize_mesh({}, parents, 20.);
    CHECK(empty.geometry.empty());
    CHECK(empty.face_parent_id == parents);

    const RegularizeResult disabled = regularize_mesh(input, parents, 0.);
    CHECK(disabled.collapse_count == 0);
    check_same_positions(disabled.geometry, input);
    CHECK(disabled.face_parent_id == parents);
}

TEST_CASE("Texture bake regularization preserves excluded faces and their weights", "[TextureBakeRegularize]")
{
    const TriSoup input = sliver_chain(false, true);
    const std::vector<int> parents { 100, 101, 102, 103 };
    RegularizeOptions options;
    options.maxrounds = 1;
    options.preserve_excluded = true;

    const RegularizeResult result = regularize_mesh(input, parents, 20., options);
    CHECK(result.collapse_count == 0);
    CHECK(result.geometry.triangle_count() == input.triangle_count());
    CHECK(result.face_parent_id == parents);
    CHECK(result.geometry.exclude_weight.size() == input.exclude_weight.size());
    for (std::size_t i = 0; i < result.geometry.exclude_weight.size(); ++i)
        CHECK_THAT(result.geometry.exclude_weight[i], Catch::Matchers::WithinAbs(i < 6 ? 1.f : 0.f, 1e-6));
    CHECK(result.reject_stats.frozen > 0);
}

TEST_CASE("Texture bake regularization rejects a collapse that would violate the link condition", "[TextureBakeRegularize]")
{
    const TriSoup input = sliver_chain(true);
    RegularizeOptions options;
    options.maxrounds = 1;

    const RegularizeResult result = regularize_mesh(input, { 0, 1, 2, 3 }, 20., options);
    CHECK(result.collapse_count == 0);
    CHECK(result.geometry.triangle_count() == input.triangle_count());
    CHECK(result.reject_stats.link_condition > 0);
}

TEST_CASE("Texture bake regularization freezes a sharp non-sliver edge", "[TextureBakeRegularize]")
{
    const TriSoup input = sharp_edge_pair();
    RegularizeOptions options;
    options.aspect_threshold = 1.;
    options.extreme_sliver_aspect = 1000.;
    options.sharp_edge_cos = 0.99;
    options.maxrounds = 1;

    const RegularizeResult result = regularize_mesh(input, { 0, 1 }, 20., options);
    CHECK(result.collapse_count == 0);
    CHECK(result.geometry.triangle_count() == input.triangle_count());
    CHECK(result.reject_stats.frozen > 0);
}
