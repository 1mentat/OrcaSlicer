#include <array>
#include <string>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "libslic3r/Config.hpp"
#include "libslic3r/ParameterUtils.hpp"
#include "libslic3r/PrintConfig.hpp"

using namespace Slic3r;

TEST_CASE("Extruder parameter lookup uses each family's variant columns", "[ParameterUtils][ParameterVariants]")
{
    DynamicPrintConfig config;
    config.option<ConfigOptionInts>("printer_extruder_id", true)->values = {1, 1, 2, 2};
    config.option<ConfigOptionStrings>("printer_extruder_variant", true)->values = {
        "Direct Drive Standard", "Direct Drive High Flow", "Direct Drive Standard", "Direct Drive High Flow"};
    // Deliberately use different column orders for process and filament settings.
    config.option<ConfigOptionInts>("print_extruder_id", true)->values = {2, 1, 2, 1};
    config.option<ConfigOptionStrings>("print_extruder_variant", true)->values = {
        "Direct Drive High Flow", "Direct Drive High Flow", "Direct Drive Standard", "Direct Drive Standard"};
    config.option<ConfigOptionStrings>("filament_extruder_variant", true)->values = {
        "Direct Drive High Flow", "Direct Drive Standard"};

    struct Row {
        int physical_extruder;
        NozzleVolumeType volume;
        int printer_column;
        int motion_slot;
        int process_column;
        int filament_column;
    };
    const std::array<Row, 6> rows = {{
        {0, nvtStandard, 0, 0, 3, 1},
        {0, nvtHighFlow, 1, 2, 1, 0},
        {0, nvtHybrid,   0, 0, 3, 1},
        {1, nvtStandard, 2, 4, 2, 1},
        {1, nvtHighFlow, 3, 6, 0, 0},
        {1, nvtHybrid,   2, 4, 2, 1},
    }};
    for (const Row &row : rows) {
        DYNAMIC_SECTION("physical extruder " << row.physical_extruder << " volume " << int(row.volume)) {
            const auto lookup = [&](const std::string &key) {
                return get_index_for_extruder_parameter(config, key, row.physical_extruder, etDirectDrive, row.volume);
            };
            CHECK(lookup("retraction_length") == row.printer_column);
            CHECK(lookup("machine_max_speed_x") == row.motion_slot);
            CHECK(lookup("outer_wall_speed") == row.process_column);
            CHECK(lookup("filament_max_volumetric_speed") == row.filament_column);
            CHECK(lookup("layer_height") == 0);
        }
    }
}

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

TEST_CASE("A print sequence splits into layer ranges with their extruder orders", "[ParameterUtils]")
{
    // Two groups of equal length: layers 1-5 print extruders 2 then 1, layers 6-9 print 1 then 2.
    const std::vector<int> sequence{1, 5, 2, 1, 6, 9, 1, 2};
    const auto             seqs = get_other_layers_print_sequence(2, sequence);
    REQUIRE(seqs.size() == 2);
    CHECK(seqs[0].first.first == 1);
    CHECK(seqs[0].first.second == 5);
    CHECK(seqs[0].second == std::vector<int>{2, 1});
    CHECK(seqs[1].first.first == 6);
    CHECK(seqs[1].first.second == 9);
    CHECK(seqs[1].second == std::vector<int>{1, 2});
}

TEST_CASE("A print sequence round-trips through its flat form", "[ParameterUtils]")
{
    std::vector<LayerPrintSequence> seqs{{{1, 3}, {2, 1}}, {{4, 8}, {1, 2}}};
    int              nums = 0;
    std::vector<int> flat;
    get_other_layers_print_sequence(seqs, nums, flat);
    CHECK(nums == 2);
    CHECK(flat == std::vector<int>{1, 3, 2, 1, 4, 8, 1, 2});
    CHECK(get_other_layers_print_sequence(nums, flat) == seqs);
}

TEST_CASE("A sequence count without matching values gives no custom sequence", "[ParameterUtils]")
{
    // The stored default: one placeholder value and a count of zero.
    const std::vector<int> placeholder{0};
    CHECK(get_other_layers_print_sequence(0, placeholder).empty());
    // The count raised on its own, as a command line override does.
    CHECK(get_other_layers_print_sequence(1, placeholder).empty());
    // A group with a layer range but no extruder, alone and as two groups that divide evenly.
    CHECK(get_other_layers_print_sequence(1, std::vector<int>{1, 5}).empty());
    CHECK(get_other_layers_print_sequence(2, std::vector<int>{1, 5, 6, 9}).empty());
    // A count that does not divide the values.
    CHECK(get_other_layers_print_sequence(2, std::vector<int>{1, 5, 2, 1, 6}).empty());
    // A negative count, and no values at all.
    CHECK(get_other_layers_print_sequence(-1, std::vector<int>{1, 5, 2}).empty());
    CHECK(get_other_layers_print_sequence(1, {}).empty());
}
