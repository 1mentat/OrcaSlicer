#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "libslic3r/ParameterUtils.hpp"

using namespace Slic3r;

TEST_CASE("Layer print sequences preserve interval endpoints and tool order", "[ParameterUtils][ParameterSequences]")
{
    const std::vector<int> flat = {2, 8, 3, 1, 2, 9, 15, 2, 3, 1};
    const std::vector<LayerPrintSequence> expected = {
        {{2, 8}, {3, 1, 2}},
        {{9, 15}, {2, 3, 1}},
    };
    CHECK(get_other_layers_print_sequence(2, flat) == expected);

    int count = 99;
    std::vector<int> encoded = {111, 222};
    get_other_layers_print_sequence(expected, count, encoded);
    CHECK(count == 2);
    CHECK(encoded == flat);
}

TEST_CASE("Empty layer print sequences clear previously encoded state", "[ParameterUtils][ParameterSequences]")
{
    CHECK(get_other_layers_print_sequence(0, {2, 8, 3, 1, 2}).empty());
    CHECK(get_other_layers_print_sequence(2, {}).empty());

    int count = 7;
    std::vector<int> encoded = {2, 8, 3, 1, 2};
    get_other_layers_print_sequence(std::vector<LayerPrintSequence>{}, count, encoded);
    CHECK(count == 0);
    CHECK(encoded.empty());
}
