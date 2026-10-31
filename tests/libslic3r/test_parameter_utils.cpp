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
