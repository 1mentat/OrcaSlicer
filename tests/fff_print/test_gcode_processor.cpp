#include <Eigen/Core>
#include <cmath>
#include "libslic3r/Config.hpp"
#include "libslic3r/MultiNozzleUtils.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/Utils.hpp"
#include <catch2/catch_all.hpp>

#include "libslic3r/libslic3r.h"
#include "libslic3r/GCode/GCodeProcessor.hpp"
#include "libslic3r/PrintConfig.hpp"

#include "test_utils.hpp"

#include <cstddef>
#include <fstream>
#include <string>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

using namespace Slic3r;
using Catch::Matchers::WithinAbs;

namespace {

float processed_belt_tilt(const std::string &gcode)
{
    ScopedTemporaryFile temp(".gcode");
    {
        std::ofstream os(temp.string());
        os << gcode;
    }
    GCodeProcessor proc;
    proc.apply_config(FullPrintConfig{});
    proc.process_file(temp.string());
    return proc.get_result().belt_tilt_angle;
}

constexpr const char *body = "G1 X10 Y10 Z0.2 F3000\nG1 X20 Y10 E1 F1200\n";

} // namespace

TEST_CASE("The config block's belt angle does not mark G-code as belt G-code", "[GCodeProcessor][belt]")
{
    // Every printer's config block lists belt_slice_rotation_angle (default 45), belt or not.
    const std::string gcode = std::string("; CONFIG_BLOCK_START\n; belt_printer = 0\n; belt_slice_rotation_angle = 45\n; CONFIG_BLOCK_END\n") + body;
    CHECK_THAT(processed_belt_tilt(gcode), WithinAbs(0., 1e-6));
}

TEST_CASE("The belt header's angle marks G-code as belt G-code", "[GCodeProcessor][belt]")
{
    const std::string gcode = std::string("; belt_slice_rotation_angle = -45.0\n") + body +
                              "; CONFIG_BLOCK_START\n; belt_printer = 1\n; belt_slice_rotation_angle = -45\n; CONFIG_BLOCK_END\n";
    CHECK_THAT(processed_belt_tilt(gcode), WithinAbs(45., 1e-6));
}

TEST_CASE("Non-belt start G-code moves keep the first-layer Z in the processor", "[GCodeProcessor]")
{
    // The belt path (GCodeWriter tests: "start-gcode prepare-stage moves keep their real Z")
    // stores the real Z of a move inside the start G-code. Every other printer must keep
    // the historical behaviour: a prepare-stage move is pinned to the first-layer height
    // so the preview does not draw the start sequence's travel. The gate is the belt
    // header, so a file without one, whatever its config block says, takes this path.
    struct BBLPrinterGuard {
        bool prev = GCodeProcessor::s_IsBBLPrinter;
        BBLPrinterGuard()  { GCodeProcessor::s_IsBBLPrinter = false; }
        ~BBLPrinterGuard() { GCodeProcessor::s_IsBBLPrinter = prev; }
    } bbl_guard;

    const std::string gcode =
        "G90\n"
        "G21\n"
        "M83\n"
        ";TYPE:Custom\n"
        "G1 E-1.5 F2100\n"
        "G1 X45 Y0.3 Z50 F12000\n"   // prepare-stage travel to a high Z
        "G1 E1.5 F1800\n"
        ";TYPE:Outer wall\n"
        "G1 X46 Y0.3 Z50 E0.05\n"
        "; CONFIG_BLOCK_START\n; belt_printer = 0\n; belt_slice_rotation_angle = 45\n; CONFIG_BLOCK_END\n";

    GCodeProcessor processor;
    FullPrintConfig config;
    config.initial_layer_print_height.value = 0.3;
    processor.apply_config(config);
    processor.process_buffer(gcode);
    const GCodeProcessorResult &result = processor.get_result();
    REQUIRE_THAT(result.belt_tilt_angle, WithinAbs(0., 1e-6));

    size_t first_extrude = result.moves.size();
    for (size_t i = 0; i < result.moves.size(); ++i)
        if (result.moves[i].type == EMoveType::Extrude) { first_extrude = i; break; }
    REQUIRE(first_extrude < result.moves.size());
    REQUIRE(first_extrude > 0);

    // The extrusion keeps its real Z; the prepare-stage move before it is pinned to the
    // first-layer height.
    CHECK_THAT(result.moves[first_extrude].position.z(),     WithinAbs(50., 1e-3));
    CHECK_THAT(result.moves[first_extrude - 1].position.z(), WithinAbs(0.3, 1e-3));
}


TEST_CASE("Static processor fills a short filament map before selecting each filament",
          "[GCodeProcessor][GCodeProcessorPublic][StaticFilamentMapFallback]")
{
    const bool previous_bbl = GCodeProcessor::s_IsBBLPrinter;
    const ScopeGuard restore([previous_bbl] { GCodeProcessor::s_IsBBLPrinter = previous_bbl; });
    GCodeProcessor::s_IsBBLPrinter = true;
    FullPrintConfig config;
    config.gcode_flavor.value = gcfMarlinFirmware;
    config.nozzle_hrc.value = 40;
    config.filament_diameter.values = {1.75, 2.0};
    config.filament_density.values = {1.24, 1.10};
    config.filament_cost.values = {100.0, 200.0};
    config.filament_map.values = {1};
    config.master_extruder_id.value = 1;
    GCodeProcessor processor;
    processor.apply_config(config);
    processor.initialize("static_filament_map_contract.gcode");
    processor.initialize_result_moves();
    processor.process_buffer("M83\nT0\nG1 X10 Y0 Z0.2 E1 F600\nT1\nG1 X20 Y0 E2 F600\n");
    processor.finalize(false);
    const auto &volumes = processor.get_result().print_statistics.total_volumes_per_extruder;
    REQUIRE(volumes.size() == 2);
    const auto first = volumes.find(0);
    const auto second = volumes.find(1);
    REQUIRE(first != volumes.end());
    REQUIRE(second != volumes.end());
    const double pi = std::acos(-1.0);
    CHECK_THAT(first->second, WithinAbs(pi * 0.25 * 1.75 * 1.75, 1e-5));
    CHECK_THAT(second->second, WithinAbs(pi * 0.25 * 2.0 * 2.0 * 2.0, 1e-5));
}


namespace processor_dynamic_filament_map_contracts {
namespace {

FullPrintConfig full_fixture_config()
{
    FullPrintConfig config;
    config.gcode_flavor.value = gcfMarlinFirmware;
    config.nozzle_hrc.value = 40;
    config.filament_diameter.values = {1.75, 2.0};
    config.filament_density.values = {1.24, 1.10};
    config.filament_cost.values = {100.0, 200.0};
    config.filament_colour.values = {"#112233"};
    config.filament_map.values = {1};
    config.master_extruder_id.value = 2;
    config.single_extruder_multi_material.value = false;
    config.enable_pre_heating.value = false;
    config.printer_extruder_id.values = {1, 2};
    config.printer_extruder_variant.values = {"Direct Drive Standard", "Direct Drive Standard"};
    config.nozzle_diameter.values = {0.4, 0.4};
    config.nozzle_volume.values = {0.0, 0.0};
    config.physical_extruder_map.values = {0, 1};
    config.extruder_max_nozzle_count.values = {1, 1};
    config.extruder_type.values = {etDirectDrive, etDirectDrive};
    config.extruder_offset.values = {Vec2d::Zero(), Vec2d::Zero()};
    return config;
}

void import_dynamic_fixture(GCodeProcessor &processor, const DynamicPrintConfig &config)
{
    // The public file loader creates the full DynamicPrintConfig used internally.
    std::string input = "; generated by OrcaSlicer\n; CONFIG_BLOCK_START\n";
    for (const auto &key : config.keys()) {
        const auto *option = config.option(key);
        REQUIRE(option != nullptr);
        input += "; " + key + " = " + option->serialize() + "\n";
    }
    input += "; CONFIG_BLOCK_END\nM83\nT0\nG1 X10 Y0 Z0.2 E1 F600\nT1\nG1 X20 Y0 E2 F600\n";
    REQUIRE(input.size() <= 512 * 1024);
    ScopedTemporaryFile file(".gcode");
    {
        std::ofstream output(file.string());
        REQUIRE(output.good());
        output << input;
        output.flush();
        REQUIRE(output.good());
    }
    processor.process_file(file.string());
}

void check_usage_and_group(const GCodeProcessorResult &result, int second_extruder)
{
    REQUIRE(result.filaments_count == 2);
    const auto &volumes = result.print_statistics.total_volumes_per_extruder;
    REQUIRE(volumes.size() == 2);
    const auto first = volumes.find(0);
    const auto second = volumes.find(1);
    REQUIRE(first != volumes.end());
    REQUIRE(second != volumes.end());
    REQUIRE(std::isfinite(first->second));
    REQUIRE(std::isfinite(second->second));
    const double pi = std::acos(-1.0);
    CHECK_THAT(first->second, WithinAbs(pi * 0.25 * 1.75 * 1.75, 1e-5));
    CHECK_THAT(second->second, WithinAbs(pi * 0.25 * 2.0 * 2.0 * 2.0, 1e-5));
    REQUIRE(result.nozzle_group_result != nullptr);
    const auto first_nozzle = result.nozzle_group_result->get_first_nozzle_for_filament(0);
    const auto second_nozzle = result.nozzle_group_result->get_first_nozzle_for_filament(1);
    REQUIRE(first_nozzle.has_value());
    REQUIRE(second_nozzle.has_value());
    CHECK(first_nozzle->extruder_id == 0);
    CHECK(second_nozzle->extruder_id == second_extruder);
}

} // namespace

TEST_CASE("Orca file import fills a short filament map from the configured master extruder",
          "[GCodeProcessor][GCodeProcessorPublic][DynamicFilamentMapFallback]")
{
    const bool previous_bbl = GCodeProcessor::s_IsBBLPrinter;
    const ScopeGuard restore([previous_bbl] { GCodeProcessor::s_IsBBLPrinter = previous_bbl; });
    GCodeProcessor::s_IsBBLPrinter = true;
    DynamicPrintConfig config;
    config.apply(full_fixture_config());
    GCodeProcessor processor;
    import_dynamic_fixture(processor, config);
    check_usage_and_group(processor.get_result(), 1);
}

TEST_CASE("Reused Orca file import resolves omitted map and master headers to fresh defaults",
          "[GCodeProcessor][GCodeProcessorPublic][DynamicFilamentMapFallback]")
{
    const bool previous_bbl = GCodeProcessor::s_IsBBLPrinter;
    const ScopeGuard restore([previous_bbl] { GCodeProcessor::s_IsBBLPrinter = previous_bbl; });
    GCodeProcessor::s_IsBBLPrinter = true;
    DynamicPrintConfig config;
    config.apply(full_fixture_config());
    GCodeProcessor processor;
    import_dynamic_fixture(processor, config);
    check_usage_and_group(processor.get_result(), 1);

    REQUIRE(config.erase("filament_map"));
    REQUIRE(config.erase("master_extruder_id"));
    processor.reset();
    import_dynamic_fixture(processor, config);
    check_usage_and_group(processor.get_result(), 0);
}

} // namespace processor_dynamic_filament_map_contracts
