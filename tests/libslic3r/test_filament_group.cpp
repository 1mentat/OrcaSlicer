#include <catch2/catch_test_macros.hpp>
#include "libslic3r/PrintConfig.hpp"
#include <cstddef>
#include <stdexcept>

#include <algorithm>
#include <initializer_list>
#include <map>
#include <set>
#include <vector>

#include "libslic3r/FilamentGroup.hpp"
#include "libslic3r/FilamentGroupUtils.hpp"
#include "libslic3r/MultiNozzleUtils.hpp"

using namespace Slic3r;

namespace {

FilamentGroupContext context(const int filament_count = 6, const int nozzle_count = 2)
{
    FilamentGroupContext c{};
    c.model_info.filament_info.resize(filament_count);
    c.model_info.flush_matrix.assign(nozzle_count, FlushMatrix(filament_count, std::vector<float>(filament_count, 10.f)));
    for (auto &matrix : c.model_info.flush_matrix)
        for (int i = 0; i < filament_count; ++i)
            matrix[i][i] = 0.f;
    c.nozzle_info.nozzle_list.reserve(nozzle_count);
    for (int i = 0; i < nozzle_count; ++i)
        c.nozzle_info.nozzle_list.push_back({"0.4", nvtStandard, i, i});
    for (int i = 0; i < nozzle_count; ++i)
        c.nozzle_info.extruder_nozzle_list[i] = {i};
    return c;
}

template <typename T>
void require_layer_membership(const FilamentPlanRes &result, const std::vector<T> &requested, const int filament_count, const int nozzle_count)
{
    std::vector<int> actual = result.fil_order;
    std::sort(actual.begin(), actual.end());
    std::vector<int> expected;
    expected.reserve(requested.size());
    for (T filament : requested)
        expected.push_back(static_cast<int>(filament));
    std::sort(expected.begin(), expected.end());
    expected.erase(std::unique(expected.begin(), expected.end()), expected.end());
    REQUIRE(actual == expected);
    REQUIRE(result.fil_nozzle_match.size() == static_cast<size_t>(filament_count));
    for (int filament : expected) {
        REQUIRE(filament >= 0);
        REQUIRE(filament < filament_count);
        REQUIRE(result.fil_nozzle_match.at(static_cast<size_t>(filament)) >= 0);
        REQUIRE(result.fil_nozzle_match.at(static_cast<size_t>(filament)) < nozzle_count);
    }
}

void require_layer_membership(const FilamentPlanRes &result, std::initializer_list<int> requested, const int filament_count, const int nozzle_count)
{
    require_layer_membership(result, std::vector<int>(requested), filament_count, nozzle_count);
}

} // namespace

TEST_CASE("Filament planning preserves empty layer boundaries", "[FilamentGroup][FilamentPlanContracts]")
{
    auto c = context();
    c.model_info.layer_filaments = {{}, {}};
    const auto result = plan_filament_nozzle_mapping_and_order(c);
    REQUIRE(result.size() == 2);
    CHECK(result[0].fil_order.empty());
    CHECK(result[1].fil_order.empty());
    CHECK(result[0].fil_nozzle_match.empty());
    CHECK(result[1].fil_nozzle_match.empty());
}

TEST_CASE("Filament planning assigns every requested filament to unloaded nozzles", "[FilamentGroup][FilamentPlanContracts]")
{
    auto c = context(4, 2);
    c.model_info.layer_filaments = {{0, 1}};
    const auto result = plan_filament_nozzle_mapping_and_order(c);
    REQUIRE(result.size() == 1);
    require_layer_membership(result.front(), {0, 1}, 4, 2);
}

TEST_CASE("Filament planning matches loaded filament identifiers rather than nozzle indices", "[FilamentGroup][FilamentPlanContracts]")
{
    auto c = context(4, 2);
    c.nozzle_info.nozzle_status = {{0, 2}, {1, 3}};
    c.model_info.layer_filaments = {{0, 1}};
    const auto result = plan_filament_nozzle_mapping_and_order(c);
    REQUIRE(result.size() == 1);
    require_layer_membership(result.front(), {0, 1}, 4, 2);
}

TEST_CASE("Filament planning reuses loaded requested filaments without duplication", "[FilamentGroup][FilamentPlanContracts]")
{
    auto c = context(4, 2);
    c.nozzle_info.nozzle_status = {{0, 2}, {1, 3}};
    c.model_info.layer_filaments = {{3, 2, 3}};
    const auto result = plan_filament_nozzle_mapping_and_order(c);
    REQUIRE(result.size() == 1);
    require_layer_membership(result.front(), {2, 3}, 4, 2);
    CHECK(result.front().fil_nozzle_match.at(2) == 0);
    CHECK(result.front().fil_nozzle_match.at(3) == 1);
}

TEST_CASE("Filament planning keeps a loaded filament on its physical nozzle", "[FilamentGroup][FilamentPlanContracts]")
{
    auto c = context(5, 2);
    c.nozzle_info.nozzle_status = {{1, 4}};
    c.model_info.layer_filaments = {{4}};
    const auto result = plan_filament_nozzle_mapping_and_order(c);
    REQUIRE(result.size() == 1);
    require_layer_membership(result.front(), {4}, 5, 2);
    CHECK(result.front().fil_nozzle_match.at(4) == 1);
}

TEST_CASE("Filament planning carries nozzle state across layers", "[FilamentGroup][FilamentPlanContracts]")
{
    auto c = context(4, 2);
    c.model_info.layer_filaments = {{0}, {1}, {0, 1}};
    const auto result = plan_filament_nozzle_mapping_and_order(c);
    REQUIRE(result.size() == 3);
    for (size_t i = 0; i < result.size(); ++i)
        require_layer_membership(result[i], c.model_info.layer_filaments[i], 4, 2);
    CHECK(result[2].fil_nozzle_match.at(0) >= 0);
    CHECK(result[2].fil_nozzle_match.at(1) >= 0);
}

TEST_CASE("Filament planning assigns more filaments than one epoch across two nozzles", "[FilamentGroup][FilamentPlanContracts]")
{
    auto c = context(6, 2);
    c.model_info.layer_filaments = {{0, 1, 2, 3, 4}};
    const auto result = plan_filament_nozzle_mapping_and_order(c);
    REQUIRE(result.size() == 1);
    require_layer_membership(result.front(), {0, 1, 2, 3, 4}, 6, 2);
}

TEST_CASE("Filament planning is invariant to duplicate layer requests", "[FilamentGroup][FilamentPlanContracts]")
{
    auto unique = context(4, 2);
    unique.model_info.layer_filaments = {{0, 1}};
    auto duplicate = context(4, 2);
    duplicate.model_info.layer_filaments = {{1, 0, 1, 0}};
    const auto a = plan_filament_nozzle_mapping_and_order(unique);
    const auto b = plan_filament_nozzle_mapping_and_order(duplicate);
    REQUIRE(a.size() == 1);
    REQUIRE(b.size() == 1);
    CHECK(a.front().fil_order == b.front().fil_order);
    CHECK(a.front().fil_nozzle_match == b.front().fil_nozzle_match);
}

TEST_CASE("Filament planning uses all physical nozzles across extruder groups", "[FilamentGroup][FilamentPlanContracts]")
{
    auto c = context(3, 3);
    c.nozzle_info.nozzle_list[0].group_id = 0;
    c.nozzle_info.nozzle_list[1].group_id = 1;
    c.nozzle_info.nozzle_list[2].group_id = 2;
    c.nozzle_info.nozzle_list[2].extruder_id = 1;
    c.nozzle_info.extruder_nozzle_list = {{0, {0}}, {1, {1, 2}}};
    c.model_info.flush_matrix.assign(2, FlushMatrix(3, std::vector<float>(3, 50.f)));
    c.model_info.flush_matrix[0][0][0] = 1.f;
    c.model_info.flush_matrix[0][1][1] = 1.f;
    c.model_info.flush_matrix[0][2][2] = 1.f;
    c.model_info.flush_matrix[1][0][1] = 40.f;
    c.model_info.flush_matrix[1][0][2] = 3.f;
    c.model_info.layer_filaments = {{0, 1, 2}};
    const auto result = plan_filament_nozzle_mapping_and_order(c);
    REQUIRE(result.size() == 1);
    require_layer_membership(result.front(), {0, 1, 2}, 3, 3);
    const auto &matches = result.front().fil_nozzle_match;
    CHECK(std::set<int>(matches.begin(), matches.end()).size() == 3);
}

TEST_CASE("Filament planning handles a partially specified nozzle status", "[FilamentGroup][FilamentPlanContracts]")
{
    auto c = context(5, 3);
    c.nozzle_info.nozzle_status = {{2, 4}};
    c.model_info.layer_filaments = {{0, 1, 2}};
    const auto result = plan_filament_nozzle_mapping_and_order(c);
    REQUIRE(result.size() == 1);
    require_layer_membership(result.front(), {0, 1, 2}, 5, 3);
}

TEST_CASE("Filament planning preserves every requested ID across sequential layers", "[FilamentGroup][FilamentPlanContracts]")
{
    auto c = context(7, 3);
    c.nozzle_info.nozzle_status = {{0, 5}, {2, 6}};
    c.model_info.layer_filaments = {{0, 1}, {}, {2, 3, 4}, {5, 6, 1}};
    const auto result = plan_filament_nozzle_mapping_and_order(c);
    REQUIRE(result.size() == 4);
    require_layer_membership(result[0], {0, 1}, 7, 3);
    CHECK(result[1].fil_order.empty());
    require_layer_membership(result[2], {2, 3, 4}, 7, 3);
    require_layer_membership(result[3], {1, 5, 6}, 7, 3);
}

TEST_CASE("Filament planning reports a stable nozzle match for every used filament", "[FilamentGroup][FilamentPlanContracts]")
{
    auto c = context(5, 2);
    c.model_info.layer_filaments = {{0, 2}, {1, 3}, {0, 1, 2, 3}};
    const auto result = plan_filament_nozzle_mapping_and_order(c);
    REQUIRE(result.size() == 3);
    for (const auto &layer : result) {
        for (int filament : layer.fil_order) {
            REQUIRE(filament >= 0);
            REQUIRE(filament < 5);
            CHECK(layer.fil_nozzle_match.at(static_cast<size_t>(filament)) >= 0);
            CHECK(layer.fil_nozzle_match.at(static_cast<size_t>(filament)) < 2);
        }
    }
}

TEST_CASE("Filament planning uses each fresh physical nozzle once", "[FilamentGroup][FilamentPlanContracts]")
{
    auto c = context(2, 2);
    c.model_info.layer_filaments = {{0, 1}};
    const auto result = plan_filament_nozzle_mapping_and_order(c);
    REQUIRE(result.size() == 1);
    REQUIRE(result.front().fil_nozzle_match.at(0) != result.front().fil_nozzle_match.at(1));
    CHECK(result.front().fil_nozzle_match.at(0) >= 0);
    CHECK(result.front().fil_nozzle_match.at(0) < 2);
    CHECK(result.front().fil_nozzle_match.at(1) >= 0);
    CHECK(result.front().fil_nozzle_match.at(1) < 2);
}

TEST_CASE("Filament planning maps a new filament to the remaining physical nozzle", "[FilamentGroup][FilamentPlanContracts]")
{
    auto c = context(4, 2);
    c.nozzle_info.nozzle_status = {{0, 2}};
    c.model_info.layer_filaments = {{2, 0}};
    const auto result = plan_filament_nozzle_mapping_and_order(c);
    REQUIRE(result.size() == 1);
    require_layer_membership(result.front(), {0, 2}, 4, 2);
    CHECK(result.front().fil_nozzle_match.at(2) == 0);
    CHECK(result.front().fil_nozzle_match.at(0) != result.front().fil_nozzle_match.at(2));
}

TEST_CASE("Filament planning carries a newly loaded filament into the next layer", "[FilamentGroup][FilamentPlanContracts]")
{
    auto c = context(4, 2);
    c.nozzle_info.nozzle_status = {{0, 2}};
    c.model_info.flush_matrix.assign(2, FlushMatrix(4, std::vector<float>(4, 50.f)));
    for (auto &matrix : c.model_info.flush_matrix) {
        for (int i = 0; i < 4; ++i) matrix[i][i] = 0.f;
    }
    c.model_info.flush_matrix[0][0][0] = 0.f;
    c.model_info.flush_matrix[0][0][2] = 80.f;
    c.model_info.flush_matrix[0][0][3] = 80.f;
    c.model_info.flush_matrix[0][1][0] = 1.f;
    c.model_info.flush_matrix[0][1][2] = 1.f;
    c.model_info.layer_filaments = {{2, 0}, {0, 1}};
    const auto result = plan_filament_nozzle_mapping_and_order(c);
    REQUIRE(result.size() == 2);
    REQUIRE(result[0].fil_nozzle_match.at(0) == 1);
    REQUIRE(result[1].fil_nozzle_match.at(0) == 1);
    REQUIRE(result[1].fil_nozzle_match.at(1) == 0);
}

TEST_CASE("Filament planning filters consumed nozzle groups before matching", "[FilamentGroup][FilamentPlanContracts]")
{
    auto c = context(5, 3);
    for (int i = 0; i < 3; ++i) {
        c.nozzle_info.nozzle_list[i].group_id = i;
        c.nozzle_info.nozzle_list[i].extruder_id = i;
    }
    c.nozzle_info.nozzle_status = {{0, 2}, {1, 3}, {2, 4}};
    c.model_info.flush_matrix.assign(3, FlushMatrix(5, std::vector<float>(5, 100.f)));
    // With the filtered right vector, group 1 serves physical nozzle 1 and
    // group 2 serves physical nozzle 2. The old unfiltered vector would use
    // groups 0 and 1 after nozzle 0 is consumed and choose the opposite map.
    c.model_info.flush_matrix[0][0][3] = 100.f;
    c.model_info.flush_matrix[0][1][3] = 1.f;
    c.model_info.flush_matrix[0][0][4] = 100.f;
    c.model_info.flush_matrix[0][1][4] = 100.f;
    c.model_info.flush_matrix[1][0][3] = 1.f;
    c.model_info.flush_matrix[1][1][3] = 100.f;
    c.model_info.flush_matrix[1][0][4] = 100.f;
    c.model_info.flush_matrix[1][1][4] = 100.f;
    c.model_info.flush_matrix[2][0][3] = 100.f;
    c.model_info.flush_matrix[2][1][3] = 100.f;
    c.model_info.flush_matrix[2][0][4] = 100.f;
    c.model_info.flush_matrix[2][1][4] = 1.f;
    c.model_info.layer_filaments = {{2, 0, 1}};
    const auto result = plan_filament_nozzle_mapping_and_order(c);
    REQUIRE(result.size() == 1);
    REQUIRE(result.front().fil_nozzle_match.at(2) == 0);
    CHECK(result.front().fil_nozzle_match.at(0) == 1);
    CHECK(result.front().fil_nozzle_match.at(1) == 2);
}

TEST_CASE("Filament planning reopens every nozzle when all are retained", "[FilamentGroup][FilamentPlanContracts]")
{
    auto c = context(4, 2);
    c.nozzle_info.nozzle_status = {{0, 2}, {1, 3}};
    c.model_info.flush_matrix.assign(2, FlushMatrix(4, std::vector<float>(4, 100.f)));
    c.model_info.flush_matrix[0][2][2] = 0.f;
    c.model_info.flush_matrix[0][2][3] = 100.f;
    c.model_info.flush_matrix[0][2][0] = 100.f;
    c.model_info.flush_matrix[1][2][0] = 1.f;
    c.model_info.layer_filaments = {{2, 3, 0, 1}};
    const auto result = plan_filament_nozzle_mapping_and_order(c);
    REQUIRE(result.size() == 1);
    require_layer_membership(result.front(), {0, 1, 2, 3}, 4, 2);
    CHECK(result.front().fil_nozzle_match.at(2) == 0);
    CHECK(result.front().fil_nozzle_match.at(3) == 1);
    CHECK(result.front().fil_nozzle_match.at(0) != result.front().fil_nozzle_match.at(1));
}


TEST_CASE("Filament planning preserves duplicate loaded right-node identity", "[FilamentGroup][FilamentPlanContracts]")
{
    auto c = context(4, 2);
    c.nozzle_info.nozzle_status = {{0, 2}, {1, 2}};
    c.model_info.layer_filaments = {{0, 1}};
    const auto result = plan_filament_nozzle_mapping_and_order(c);
    REQUIRE(result.size() == 1);
    require_layer_membership(result.front(), {0, 1}, 4, 2);
    CHECK(result.front().fil_nozzle_match.at(0) != result.front().fil_nozzle_match.at(1));
}

TEST_CASE("Filament planning preserves a mixed loaded and dummy solver choice", "[FilamentGroup][FilamentPlanContracts]")
{
    auto c = context(4, 3);
    c.nozzle_info.nozzle_status = {{0, 2}, {1, 2}};
    c.model_info.flush_matrix.assign(3, FlushMatrix(4, std::vector<float>(4, 100.f)));
    c.model_info.flush_matrix[0][0][2] = 100.f;
    c.model_info.flush_matrix[0][1][2] = 100.f;
    c.model_info.flush_matrix[1][0][2] = 1.f;
    c.model_info.flush_matrix[1][1][2] = 1.f;
    c.model_info.layer_filaments = {{0, 1}};
    const auto result = plan_filament_nozzle_mapping_and_order(c);
    REQUIRE(result.size() == 1);
    require_layer_membership(result.front(), {0, 1}, 4, 3);
    std::set<int> matched = {result.front().fil_nozzle_match.at(0), result.front().fil_nozzle_match.at(1)};
    REQUIRE(matched.size() == 2);
    CHECK(matched.count(0) == 0);
    CHECK(matched.count(1) == 1);
    CHECK(matched.count(2) == 1);
}

TEST_CASE("Filament planning preserves empty layers without physical nozzles", "[FilamentGroup][FilamentPlanContracts]")
{
    auto c = context(3, 0);
    c.model_info.layer_filaments = {{}, {}, {}};
    const auto result = plan_filament_nozzle_mapping_and_order(c);
    REQUIRE(result.size() == 3);
    for (const auto &layer : result) {
        CHECK(layer.fil_order.empty());
        CHECK(layer.fil_nozzle_match.empty());
    }
}

TEST_CASE("Filament planning rejects requested layers without physical nozzles", "[FilamentGroup][FilamentPlanContracts]")
{
    auto c = context(3, 0);
    c.model_info.layer_filaments = {{}, {0}};
    CHECK_THROWS_AS(plan_filament_nozzle_mapping_and_order(c), std::invalid_argument);
}


TEST_CASE("Filament planning rebuilds duplicate reverse maps across layers", "[FilamentGroup][FilamentPlanContracts]")
{
    auto c = context(5, 3);
    c.nozzle_info.nozzle_status = {{0, 2}, {1, 2}, {2, 4}};
    c.model_info.layer_filaments = {{0, 1}, {0, 1}, {3}, {0, 1}};
    const auto result = plan_filament_nozzle_mapping_and_order(c);
    REQUIRE(result.size() == 4);
    require_layer_membership(result[0], {0, 1}, 5, 3);
    require_layer_membership(result[1], {0, 1}, 5, 3);
    require_layer_membership(result[2], {3}, 5, 3);
    require_layer_membership(result[3], {0, 1}, 5, 3);
    CHECK(result[0].fil_nozzle_match.at(0) != result[0].fil_nozzle_match.at(1));
    CHECK(result[1].fil_nozzle_match.at(0) == result[0].fil_nozzle_match.at(0));
    CHECK(result[1].fil_nozzle_match.at(1) == result[0].fil_nozzle_match.at(1));
    CHECK(result[3].fil_nozzle_match.at(0) == result[0].fil_nozzle_match.at(0));
    CHECK(result[3].fil_nozzle_match.at(1) == result[0].fil_nozzle_match.at(1));
}


TEST_CASE("TPU grouping applies its nozzle preference without changing unused filament assignments",
          "[FilamentGroup][FilamentTPUGrouping]")
{
    auto c = context(5, 2);
    c.group_info.total_filament_num = 5;
    c.machine_info.master_extruder_id = 1;
    c.model_info.layer_filaments = {{3, 1, 3}, {}, {1}};
    c.speed_info.filament_print_time = {{1, {{0, 100.0}, {1, 105.0}}},
                                      {3, {{0, 200.0}, {1, 400.0}}}};
    // With two standard nozzles, the first is strictly faster for both used
    // filaments. The TPU preference discounts nozzle 1's 105 to 94.5 for
    // filament 1, while filament 3 still prefers its 200 to the discounted 360.
    FilamentGroup standard(c);
    CHECK(standard.calc_filament_group_for_tpu() == std::vector<int>{1, 0, 1, 0, 1});
    c.nozzle_info.nozzle_list[1].volume_type = nvtTPUHighFlow;
    FilamentGroup preferred(c);
    CHECK(preferred.calc_filament_group_for_tpu() == std::vector<int>{1, 1, 1, 0, 1});
}

TEST_CASE("TPU grouping honors physical extruder and nozzle-volume restrictions",
          "[FilamentGroup][FilamentTPUGrouping]")
{
    auto c = context(5, 2);
    c.group_info.total_filament_num = 5;
    c.machine_info.master_extruder_id = 1;
    c.model_info.layer_filaments = {{3, 1}};
    c.speed_info.filament_print_time = {{1, {{0, 100.0}, {1, 105.0}}},
                                      {3, {{0, 200.0}, {1, 400.0}}}};
    c.nozzle_info.nozzle_list[1].volume_type = nvtTPUHighFlow;
    // The restrictions oppose both unconstrained choices. They use original
    // filament IDs 1 and 3, not the compact indices in the solver's matrix.
    c.model_info.unprintable_filaments.resize(2);
    c.model_info.unprintable_filaments[1].insert(1);
    c.model_info.unprintable_volumes[3].insert(nvtStandard);
    FilamentGroup restricted(c);
    CHECK(restricted.calc_filament_group_for_tpu() == std::vector<int>{1, 0, 1, 1, 1});
}
