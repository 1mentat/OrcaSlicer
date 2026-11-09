#include <array>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <algorithm>
#include <cstddef>
#include <limits>
#include <map>
#include <numeric>
#include <optional>
#include <set>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

#include "libslic3r/GCode/ToolOrderUtils.hpp"

using namespace Slic3r;
using Catch::Matchers::WithinAbs;

namespace {

template<class Solver, class = void>
struct HasPhysicalNozzleIndices : std::false_type {};

template<class Solver>
struct HasPhysicalNozzleIndices<Solver, std::void_t<decltype(std::declval<Solver &>().solve_indices())>> : std::true_type {};

template<class Solver>
std::vector<int> solve_physical_indices(Solver &solver)
{
    if constexpr (HasPhysicalNozzleIndices<Solver>::value) {
        return solver.solve_indices();
    } else {
        // Test-only overlays also run on PR heads that predate this API. Keep
        // the missing contract visible without preventing unrelated tests from building.
        FAIL("This production tree does not expose physical nozzle indices through solve_indices()");
        return {};
    }
}

std::map<int, int> counts(const std::vector<int> &values)
{
    std::map<int, int> result;
    for (int value : values)
        ++result[value];
    return result;
}

int count_value(const std::vector<int> &values, int wanted)
{
    return static_cast<int>(std::count(values.begin(), values.end(), wanted));
}

std::map<unsigned int, int> unsigned_counts(const std::vector<unsigned int> &values)
{
    std::map<unsigned int, int> result;
    for (unsigned int value : values)
        ++result[value];
    return result;
}

float path_cost(const std::vector<std::vector<float>> &matrix,
                const std::vector<unsigned int> &path,
                const std::optional<unsigned int> &start = std::nullopt)
{
    float result = 0.f;
    std::optional<unsigned int> previous = start;
    for (unsigned int value : path) {
        if (previous)
            result += matrix.at(*previous).at(value);
        previous = value;
    }
    return result;
}

bool is_permutation_of(const std::vector<unsigned int> &values,
                       const std::vector<unsigned int> &expected)
{
    return unsigned_counts(values) == unsigned_counts(expected);
}

} // namespace

TEST_CASE("Flow matching respects link, unlink, node, and group capacities", "[ToolOrderUtils]")
{
    const std::vector<int> left{10, 11, 12};
    const std::vector<int> right{20, 21};
    const std::unordered_map<int, std::vector<int>> links{{1, {1}}};
    const std::unordered_map<int, std::vector<int>> unlinks{{0, {1}}};
    MaxFlowSolver solver(left, right, links, unlinks, {1, 1, 1}, {2, 1});
    const auto matching = solver.solve();

    REQUIRE(matching.size() == left.size());
    CHECK(matching[0] == 20);
    CHECK(matching[1] == 21);
    CHECK(matching[2] == 20);
    CHECK(counts(matching) == std::map<int, int>{{20, 2}, {21, 1}});
}

TEST_CASE("Flow matching routes grouped right nodes through group capacity", "[ToolOrderUtils]")
{
    MaxFlowSolver solver({0, 1}, {10, 11, 12}, {}, {}, {}, {},
                         {{{0, 1}, 1}});
    const auto matching = solver.solve();

    REQUIRE(matching.size() == 2);
    CHECK(std::all_of(matching.begin(), matching.end(), [](int value) { return value != MaxFlowGraph::INVALID_ID; }));
    CHECK(count_value(matching, 12) == 1);
    CHECK(count_value(matching, 10) + count_value(matching, 11) == 1);
}

TEST_CASE("Minimum-cost matching chooses the independently cheapest perfect matching", "[ToolOrderUtils]")
{
    const std::vector<std::vector<float>> matrix{{9.f, 1.f}, {2.f, 8.f}};
    GeneralMinCostSolver solver(matrix, {0, 1}, {0, 1});
    const auto matching = solver.solve();

    REQUIRE(matching == std::vector<int>{1, 0});
    CHECK_THAT(matrix[0][matching[0]] + matrix[1][matching[1]], WithinAbs(3.f, 1e-6f));
}

TEST_CASE("Minimum-cost matching treats a dummy left node as neutral", "[ToolOrderUtils]")
{
    const std::vector<std::vector<float>> matrix{{4.f, 7.f}};
    GeneralMinCostSolver solver(matrix, {-1, 0}, {0, 1});
    const auto matching = solver.solve();

    // The dummy row costs zero for either column, so the real row must take
    // the independently cheaper column 0 while the dummy takes column 1.
    REQUIRE(matching == std::vector<int>{1, 0});
    CHECK(std::set<int>(matching.begin(), matching.end()) == std::set<int>{0, 1});
}

TEST_CASE("Minimum-cost matching satisfies a group lower bound and restrictions", "[ToolOrderUtils]")
{
    const std::vector<FlushMatrix> matrix{
        {{1.f, 8.f}, {7.f, 2.f}},
        {{0.f}, {0.f}}
    };
    const std::vector<int> left{0, 1};
    const std::vector<int> right{0, 1, -1};
    const std::vector<int> groups{0, 0, 1};
    const std::unordered_map<int, std::vector<int>> unlinks{{0, {1}}};
    GeneralMinCostLowerBoundsSolver solver(matrix, left, right, groups, {}, unlinks);
    const auto matching = solver.solve();

    REQUIRE(matching.size() == left.size());
    CHECK(std::find(matching.begin(), matching.end(), -1) != matching.end());
    CHECK(std::find(matching.begin(), matching.end(), 0) != matching.end());
    CHECK(std::set<int>(matching.begin(), matching.end()) == std::set<int>{-1, 0});
    REQUIRE(std::all_of(matching.begin(), matching.end(), [](int value) { return value == -1 || value == 0; }));
    float observed_cost = 0.f;
    for (size_t i = 0; i < matching.size(); ++i)
        if (matching[i] >= 0)
            observed_cost += matrix.at(static_cast<size_t>(groups.at(static_cast<size_t>(matching[i]))))
                                     .at(static_cast<size_t>(i))
                                     .at(static_cast<size_t>(matching[i]));
    CHECK_THAT(observed_cost, WithinAbs(1.f, 1e-6f));
}

TEST_CASE("Group flow covers every group and honors link limits", "[ToolOrderUtils]")
{
    const std::vector<FlushMatrix> matrix{
        {{9.f, 1.f}, {2.f, 8.f}},
        {{5.f, 6.f}, {7.f, 3.f}}
    };
    GroupMinCostFlowSolver solver(matrix, {0, 1}, {0, 1}, {0, 1}, {{0, {0}}}, {});
    const auto matching = solver.solve();

    REQUIRE(matching == std::vector<int>{0, 1});
}

TEST_CASE("Group flow uses dummy right nodes and excludes forbidden edges", "[ToolOrderUtils]")
{
    const std::vector<FlushMatrix> matrix{
        {{1.f, 2.f}, {3.f, 4.f}, {5.f, 6.f}},
        {{0.f, 0.f}, {0.f, 0.f}, {0.f, 0.f}}
    };
    GroupMinCostFlowSolver solver(matrix, {0, 1, 2}, {0, 1, -1}, {0, 0, 1}, {}, {{0, {0}}});
    const auto matching = solver.solve();

    REQUIRE(matching.size() == 3);
    CHECK(matching[2] == -1);
    CHECK(std::set<int>(matching.begin(), matching.end()) == std::set<int>{-1, 0, 1});
}

TEST_CASE("Group flow pays the group-coverage bonus before local flush cost", "[ToolOrderUtils]")
{
    const std::vector<FlushMatrix> matrix{
        {{50.f, 50.f, 50.f}, {50.f, 50.f, 50.f}},
        {{0.f, 0.f, 0.f}, {0.f, 0.f, 0.f}}
    };
    GroupMinCostFlowSolver solver(matrix, {0, 1}, {0, 1, 2}, {0, 1, 1});
    const auto matching = solver.solve();

    REQUIRE(matching.size() == 2);
    CHECK(std::find(matching.begin(), matching.end(), 0) != matching.end());
    CHECK(std::all_of(matching.begin(), matching.end(), [](int value) { return value >= 0 && value <= 2; }));
}

TEST_CASE("Flush matching satisfies right capacities and minimizes total matrix cost", "[ToolOrderUtils]")
{
    const std::vector<std::vector<float>> matrix{
        {7.f, 2.f},
        {1.f, 8.f},
        {4.f, 6.f}
    };
    MinFlushFlowSolver solver(matrix, {0, 1, 2}, {0, 1}, {}, {}, {}, {2, 1});
    const auto matching = solver.solve();

    REQUIRE(matching.size() == 3);
    REQUIRE(std::all_of(matching.begin(), matching.end(), [](int value) { return value == 0 || value == 1; }));
    const auto observed = counts(matching);
    REQUIRE(observed.count(0) == 1);
    REQUIRE(observed.count(1) == 1);
    CHECK(observed.at(0) <= 2);
    CHECK(observed.at(1) <= 1);
    float cost = 0.f;
    for (size_t i = 0; i < matching.size(); ++i)
        cost += matrix.at(i).at(static_cast<size_t>(matching[i]));
    CHECK_THAT(cost, WithinAbs(7.f, 1e-6f));
}

TEST_CASE("Asymmetric capacities distribute matching across groups", "[ToolOrderUtils]")
{
    const std::vector<std::vector<float>> matrix{
        {1.f, 9.f},
        {2.f, 8.f},
        {7.f, 3.f}
    };
    MatchModeGroupSolver solver(matrix, {0, 1, 2}, {0, 1}, {1, 2}, {{1, {1}}});
    const auto matching = solver.solve();

    REQUIRE(matching.size() == 3);
    CHECK(matching[1] == 0);
    CHECK(matching[0] == 1);
    CHECK(matching[2] == 1);
    CHECK(counts(matching) == std::map<int, int>{{0, 1}, {1, 2}});
}

TEST_CASE("Extruder ordering handles empty and singleton layers", "[ToolOrderUtils]")
{
    const std::vector<std::vector<float>> matrix{{0.f, 2.f}, {3.f, 0.f}};
    float empty_cost = -1.f;
    CHECK(get_extruders_order(matrix, {}, {}, std::nullopt, false, &empty_cost).empty());
    CHECK_THAT(empty_cost, WithinAbs(0.f, 1e-6f));

    float singleton_cost = -1.f;
    CHECK(get_extruders_order(matrix, {1}, {}, 0u, false, &singleton_cost) == std::vector<unsigned int>{1});
    CHECK_THAT(singleton_cost, WithinAbs(2.f, 1e-6f));
}

TEST_CASE("Extruder ordering finds the exact minimum Hamilton path", "[ToolOrderUtils]")
{
    const std::vector<std::vector<float>> matrix{
        {0.f, 8.f, 2.f},
        {3.f, 0.f, 7.f},
        {5.f, 1.f, 0.f}
    };
    const std::vector<unsigned int> values{0, 1, 2};
    float observed_cost = 0.f;
    const auto sequence = get_extruders_order(matrix, values, {}, std::nullopt, false, &observed_cost);

    float best_cost = std::numeric_limits<float>::max();
    auto permutation = values;
    do {
        best_cost = std::min(best_cost, path_cost(matrix, permutation));
    } while (std::next_permutation(permutation.begin(), permutation.end()));
    REQUIRE(is_permutation_of(sequence, values));
    CHECK_THAT(observed_cost, WithinAbs(best_cost, 1e-6f));
    CHECK_THAT(path_cost(matrix, sequence), WithinAbs(best_cost, 1e-6f));
}

TEST_CASE("Extruder ordering includes a start filament absent from the current layer", "[ToolOrderUtils]")
{
    const std::vector<std::vector<float>> matrix{
        {0.f, 9.f, 4.f, 1.f},
        {3.f, 0.f, 8.f, 6.f},
        {7.f, 2.f, 0.f, 5.f},
        {6.f, 3.f, 1.f, 0.f}
    };
    const std::vector<unsigned int> values{0, 1, 2};
    float observed_cost = 0.f;
    const auto sequence = get_extruders_order(matrix, values, {}, 3u, false, &observed_cost);

    float best_cost = std::numeric_limits<float>::max();
    auto permutation = values;
    do {
        best_cost = std::min(best_cost, path_cost(matrix, permutation, 3u));
    } while (std::next_permutation(permutation.begin(), permutation.end()));
    REQUIRE(sequence.size() == values.size());
    REQUIRE(is_permutation_of(sequence, values));
    CHECK_THAT(observed_cost, WithinAbs(best_cost, 1e-6f));
    CHECK_THAT(path_cost(matrix, sequence, 3u), WithinAbs(best_cost, 1e-6f));
}

TEST_CASE("Extruder ordering minimizes two-layer transition cost", "[ToolOrderUtils]")
{
    const std::vector<std::vector<float>> matrix{
        {0.f, 9.f, 1.f, 8.f},
        {2.f, 0.f, 7.f, 1.f},
        {5.f, 3.f, 0.f, 6.f},
        {4.f, 2.f, 8.f, 0.f}
    };
    const std::vector<unsigned int> current{0, 1};
    const std::vector<unsigned int> next{2, 3};
    float observed_cost = 0.f;
    const auto sequence = get_extruders_order(matrix, current, next, std::nullopt, true, &observed_cost);

    float best_total = std::numeric_limits<float>::max();
    float best_current = 0.f;
    auto current_permutation = current;
    do {
        auto next_permutation = next;
        do {
            const float total = path_cost(matrix, current_permutation) +
                                matrix[current_permutation.back()][next_permutation.front()] +
                                matrix[next_permutation.front()][next_permutation.back()];
            if (total < best_total) {
                best_total = total;
                best_current = path_cost(matrix, current_permutation);
            }
        } while (std::next_permutation(next_permutation.begin(), next_permutation.end()));
    } while (std::next_permutation(current_permutation.begin(), current_permutation.end()));

    REQUIRE(is_permutation_of(sequence, current));
    CHECK_THAT(observed_cost, WithinAbs(best_current, 1e-6f));
    CHECK_THAT(path_cost(matrix, sequence), WithinAbs(best_current, 1e-6f));
    float chosen_total = std::numeric_limits<float>::max();
    auto next_permutation = next;
    do {
        chosen_total = std::min(chosen_total, path_cost(matrix, sequence) +
                               path_cost(matrix, next_permutation, sequence.back()));
    } while (std::next_permutation(next_permutation.begin(), next_permutation.end()));
    CHECK_THAT(chosen_total, WithinAbs(best_total, 1e-6f));
}

TEST_CASE("Extruder ordering uses bounded greedy ordering for large layers", "[ToolOrderUtils]")
{
    constexpr unsigned int count = 21;
    std::vector<std::vector<float>> matrix(count, std::vector<float>(count, 5.f));
    for (unsigned int i = 0; i < count; ++i)
        matrix[i][i] = 0.f;
    std::vector<unsigned int> values(count);
    std::iota(values.begin(), values.end(), 0u);
    float observed_cost = 0.f;
    const auto sequence = get_extruders_order(matrix, values, {}, 0u, false, &observed_cost);

    REQUIRE(is_permutation_of(sequence, values));
    CHECK_THAT(observed_cost, WithinAbs(path_cost(matrix, sequence, 0u), 1e-6f));
    CHECK(observed_cost >= 0.f);
}

TEST_CASE("Filament reordering honors a custom layer order", "[ToolOrderUtils]")
{
    const std::vector<unsigned int> filament_lists{0, 1, 2};
    const std::vector<int> filament_maps{0, 0, 1};
    const std::vector<std::vector<unsigned int>> layers{{0, 2}, {1, 2}};
    const std::vector<FlushMatrix> matrix{
        {{0.f, 4.f, 10.f}, {3.f, 0.f, 8.f}, {9.f, 2.f, 0.f}},
        {{0.f, 4.f, 10.f}, {3.f, 0.f, 8.f}, {9.f, 2.f, 0.f}}
    };
    const auto custom = [](int layer, std::vector<int> &sequence) {
        sequence = (layer == 0) ? std::vector<int>{3, 1} : std::vector<int>{2, 3};
        return true;
    };
    std::vector<std::vector<unsigned int>> output;
    const int cost = reorder_filaments_for_minimum_flush_volume(
        filament_lists, filament_maps, layers, matrix, custom, &output);

    REQUIRE(output == std::vector<std::vector<unsigned int>>{{2, 0}, {1, 2}});
    CHECK(cost == 4);
    for (size_t layer = 0; layer < layers.size(); ++layer)
        CHECK(unsigned_counts(output[layer]) == unsigned_counts(layers[layer]));
}

TEST_CASE("Filament reordering preserves each layer's filament multiset", "[ToolOrderUtils]")
{
    const std::vector<unsigned int> filament_lists{0, 1, 2, 3};
    const std::vector<int> filament_maps{0, 0, 1, 1};
    const std::vector<std::vector<unsigned int>> layers{{0, 2}, {1, 3}, {2, 0, 3, 1}};
    const FlushMatrix flush{{0.f, 6.f, 2.f, 8.f}, {3.f, 0.f, 7.f, 1.f},
                            {5.f, 4.f, 0.f, 9.f}, {2.f, 1.f, 6.f, 0.f}};
    const std::vector<FlushMatrix> matrix{flush, flush};
    std::vector<std::vector<unsigned int>> output;
    const int cost = reorder_filaments_for_minimum_flush_volume(
        filament_lists, filament_maps, layers, matrix, std::nullopt, &output, {{0, 1}, {1, 2}});

    REQUIRE(output.size() == layers.size());
    // Group 0 pays 1->0, 0->1, then 1->0; group 1 pays 2->3, then 3->2.
    const int minimum_cost = 3 + 6 + 3 + 9 + 6;
    CHECK(cost == minimum_cost);
    for (size_t layer = 0; layer < layers.size(); ++layer) {
        CHECK(unsigned_counts(output[layer]) == unsigned_counts(layers[layer]));
        CHECK(output[layer].size() == layers[layer].size());
    }
}

TEST_CASE("Filament reordering charges the initial loaded filament", "[ToolOrderUtils]")
{
    const std::vector<unsigned int> filament_lists{0, 1};
    const std::vector<int> filament_maps{0, 0};
    const std::vector<std::vector<unsigned int>> layers{{0}};
    const std::vector<FlushMatrix> matrix{
        {{0.f, 9.f}, {3.f, 0.f}},
        {{0.f, 0.f}, {0.f, 0.f}}
    };
    std::vector<std::vector<unsigned int>> output;
    const int cost = reorder_filaments_for_minimum_flush_volume(
        filament_lists, filament_maps, layers, matrix, std::nullopt, &output, {{0, 1}});

    REQUIRE(output == std::vector<std::vector<unsigned int>>{{0}});
    CHECK(cost == 3);
}

TEST_CASE("Group matching chooses the independently enumerated minimum cost assignment", "[ToolOrderUtils][FilamentPlanContracts]")
{
    std::vector<FlushMatrix> matrix(2, FlushMatrix(4, std::vector<float>(4, 100.f)));
    matrix[0][0][2] = 7.f;
    matrix[0][1][2] = 41.f;
    matrix[1][0][3] = 53.f;
    matrix[1][1][3] = 11.f;
    const std::array<int, 2> left_nodes{0, 1};
    const std::array<int, 2> right_nodes{2, 3};
    const std::array<int, 2> right_groups{0, 1};
    const std::array<std::array<int, 2>, 4> legal_assignments{{{{0, 0}}, {{0, 1}}, {{1, 0}}, {{1, 1}}}};
    const auto score = [&](const std::array<int, 2> &assignment) {
        const std::set<int> groups = {right_groups[assignment[0]], right_groups[assignment[1]]};
        const std::set<int> nozzles = {assignment[0], assignment[1]};
        const int cost = static_cast<int>(matrix[right_groups[assignment[0]]][left_nodes[0]][right_nodes[assignment[0]]]) +
                         static_cast<int>(matrix[right_groups[assignment[1]]][left_nodes[1]][right_nodes[assignment[1]]]);
        return std::array<int, 3>{static_cast<int>(groups.size()), static_cast<int>(nozzles.size()), -cost};
    };
    auto better = [](const std::array<int, 3> &a, const std::array<int, 3> &b) {
        return a[0] != b[0] ? a[0] > b[0] : (a[1] != b[1] ? a[1] > b[1] : a[2] > b[2]);
    };
    std::array<int, 3> optimum = score(legal_assignments.front());
    for (const auto &assignment : legal_assignments)
        if (better(score(assignment), optimum)) optimum = score(assignment);

    GroupMinCostFlowSolver solver(matrix, {0, 1}, {2, 3}, {0, 1});
    const auto indices = solve_physical_indices(solver);
    REQUIRE(indices.size() == 2);
    REQUIRE(indices.at(0) >= 0);
    REQUIRE(indices.at(0) < 2);
    REQUIRE(indices.at(1) >= 0);
    REQUIRE(indices.at(1) < 2);
    const std::array<int, 2> selected{indices.at(0), indices.at(1)};
    const auto selected_score = score(selected);
    CHECK(selected_score == optimum);
    CHECK(selected_score[0] == 2); // both group IDs are covered
    CHECK(selected_score[1] == 2); // both physical right nodes are used
    CHECK(-selected_score[2] == 18); // the independently enumerated minimum cost
}


TEST_CASE("Group matching exposes repeated values without changing the legacy value result", "[ToolOrderUtils][FilamentPlanContracts]")
{
    std::vector<FlushMatrix> matrix(2, FlushMatrix(3, std::vector<float>(3, 100.f)));
    matrix[0][0][2] = 1.f;
    matrix[0][1][2] = 100.f;
    matrix[1][0][2] = 100.f;
    matrix[1][1][2] = 1.f;
    GroupMinCostFlowSolver by_index(matrix, {0, 1}, {2, 2}, {0, 1});
    const auto indices = solve_physical_indices(by_index);
    REQUIRE(indices.size() == 2);
    CHECK(indices.at(0) == 0);
    CHECK(indices.at(1) == 1);
    GroupMinCostFlowSolver by_value(matrix, {0, 1}, {2, 2}, {0, 1});
    const auto values = by_value.solve();
    REQUIRE(values.size() == 2);
    CHECK(values.at(0) == 2);
    CHECK(values.at(1) == 2);
}
