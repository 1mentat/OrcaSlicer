#include "libslic3r/CustomGCode.hpp"
#include <cmath>
#include <map>
#include "libslic3r/ExtrusionEntity.hpp"
#include <catch2/catch_all.hpp>

#include <catch2/catch_message.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "libslic3r/libslic3r.h"
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/Config.hpp"
#include "libslic3r/GCode/GCodeProcessor.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/Utils.hpp"

#include "test_helpers.hpp"
#include "test_utils.hpp"

#include <algorithm>
#include <cstddef>
#include <fstream>
#include <initializer_list>
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/Point.hpp"
#include <sstream>
#include <string>
#include <utility>
#include <vector>

using namespace Slic3r;

namespace elegoo_command_contracts {

using Catch::Matchers::WithinAbs;

namespace {

void process_gcode(const std::string &gcode, const std::string &printer_model,
                   GCodeProcessorResult &result)
{
    FullPrintConfig config;
    config.gcode_flavor.value = gcfMarlinFirmware;
    config.printer_model.value = printer_model;

    const bool previous_bbl = GCodeProcessor::s_IsBBLPrinter;
    const ScopeGuard restore_bbl([previous_bbl] { GCodeProcessor::s_IsBBLPrinter = previous_bbl; });
    GCodeProcessor::s_IsBBLPrinter = false;

    GCodeProcessor processor;
    processor.apply_config(config);
    processor.initialize("elegoo-command-contract.gcode");
    processor.initialize_result_moves();
    processor.process_buffer(gcode);
    processor.finalize(false);
    result = std::move(processor.extract_result());
}

void run_m6211(const std::string &printer_model, const std::string &command,
               GCodeProcessorResult &result)
{
    process_gcode("M83\nG90\n"
                  "G1 X10 Y20 Z0.2 F600\n"
                  + command + "\n"
                  "G1 X20 Y20 E1 F600\n",
                  printer_model, result);
}

const GCodeProcessorResult::MoveVertex *last_extrusion(const GCodeProcessorResult &result)
{
    for (auto it = result.moves.rbegin(); it != result.moves.rend(); ++it)
        if (!it->internal_only && it->type == EMoveType::Extrude)
            return &*it;
    return nullptr;
}

} // namespace

TEST_CASE("Elegoo M6211 is ignored for other printer models", "[GCodeProcessor][Elegoo]")
{
    GCodeProcessorResult result;
    run_m6211("Generic Marlin", "M6211 L80 T0 P20000", result);

    CHECK_THAT(result.print_statistics.total_filament_load_time, WithinAbs(0.0, 1e-5));
    CHECK(result.print_statistics.total_filament_changes == 0);
    CHECK(last_extrusion(result) != nullptr);
}

TEST_CASE("Elegoo Carbon M6211 records clamped flush timing and cooling parameter", "[GCodeProcessor][Elegoo]")
{
    GCodeProcessorResult short_flush, minimum_flush, maximum_flush, over_maximum, cooled;
    run_m6211("Elegoo Centauri Carbon", "M6211 L1 T0 P0 S210", short_flush);
    run_m6211("elegoo centauri carbon", "M6211 L10 T0 P0 S210", minimum_flush);
    run_m6211("Elegoo Centauri Carbon", "M6211 L1000 T0 P0 S210", maximum_flush);
    run_m6211("Elegoo Centauri Carbon", "M6211 L1200 T0 P0 S210", over_maximum);
    run_m6211("Elegoo Centauri Carbon", "M6211 L10 T0 P20000 S210", cooled);

    REQUIRE(short_flush.print_statistics.total_filament_load_time > 0.0f);
    CHECK_THAT(short_flush.print_statistics.total_filament_load_time,
               WithinAbs(minimum_flush.print_statistics.total_filament_load_time, 1e-4));
    CHECK_THAT(maximum_flush.print_statistics.total_filament_load_time,
               WithinAbs(over_maximum.print_statistics.total_filament_load_time, 1e-4));
    CHECK_THAT(cooled.print_statistics.total_filament_load_time - minimum_flush.print_statistics.total_filament_load_time,
               WithinAbs(20.0, 1e-3));

    const auto *move = last_extrusion(cooled);
    REQUIRE(move != nullptr);
    CHECK_THAT(move->temperature, WithinAbs(210.0, 1e-4));
}

TEST_CASE("Elegoo Centauri Carbon 2 uses its distinct flush and cooling model", "[GCodeProcessor][Elegoo]")
{
    GCodeProcessorResult base, alias, extra_cooling, more_flush;
    run_m6211("Elegoo Centauri Carbon 2", "M6211 L10 T0 P0 S250 R270", base);
    run_m6211("Elegoo Centauri 2", "M6211 L10 T0 P0 S250 R270", alias);
    run_m6211("Elegoo Centauri 2", "M6211 L10 T0 P20000 S250 R270", extra_cooling);
    run_m6211("Elegoo Centauri Carbon 2", "M6211 L160 T0 K20 M100 N100 P0 S250 R270", more_flush);

    REQUIRE(base.print_statistics.total_filament_load_time > 0.0f);
    CHECK_THAT(alias.print_statistics.total_filament_load_time,
               WithinAbs(base.print_statistics.total_filament_load_time, 1e-4));
    CHECK(extra_cooling.print_statistics.total_filament_load_time > base.print_statistics.total_filament_load_time);
    CHECK(more_flush.print_statistics.total_filament_load_time > base.print_statistics.total_filament_load_time);
    REQUIRE(last_extrusion(extra_cooling) != nullptr);
    CHECK_THAT(last_extrusion(extra_cooling)->temperature, WithinAbs(250.0, 1e-4));
}

TEST_CASE("Elegoo M6211 rejects incomplete and out-of-range filament commands", "[GCodeProcessor][Elegoo]")
{
    GCodeProcessorResult no_length, no_filament, invalid_filament;
    run_m6211("Elegoo Centauri Carbon", "M6211 T0 P20000", no_length);
    run_m6211("Elegoo Centauri Carbon", "M6211 L40", no_filament);
    // The fixture has one filament: both a negative slot and the first slot
    // beyond the configured count must be rejected.
    const int invalid_slot = GENERATE(-1, 1, 99);
    run_m6211("Elegoo Centauri Carbon", "M6211 L40 T" + std::to_string(invalid_slot), invalid_filament);

    CHECK_THAT(no_length.print_statistics.total_filament_load_time, WithinAbs(0.0, 1e-5));
    CHECK_THAT(no_filament.print_statistics.total_filament_load_time, WithinAbs(0.0, 1e-5));
    CHECK_THAT(invalid_filament.print_statistics.total_filament_load_time, WithinAbs(0.0, 1e-5));
    CHECK(no_length.print_statistics.total_filament_changes == 0);
    CHECK(no_filament.print_statistics.total_filament_changes == 0);
    CHECK(invalid_filament.print_statistics.total_filament_changes == 0);
}

} // namespace elegoo_command_contracts

namespace extended_command_contracts {

using Catch::Matchers::WithinAbs;

namespace {

void process_buffer(const std::string &gcode, GCodeProcessorResult &result,
                    GCodeFlavor flavor = gcfMarlinFirmware, bool bbl_printer = false,
                    float retract_length = 0.f)
{
    FullPrintConfig config;
    config.gcode_flavor.value = flavor;
    if (retract_length > 0.f)
        config.retraction_length.values = {retract_length};
    const bool previous_bbl = GCodeProcessor::s_IsBBLPrinter;
    const ScopeGuard restore_bbl([previous_bbl] { GCodeProcessor::s_IsBBLPrinter = previous_bbl; });
    GCodeProcessor::s_IsBBLPrinter = bbl_printer;

    GCodeProcessor processor;
    processor.apply_config(config);
    processor.initialize("gcode-processor-contract.gcode");
    processor.initialize_result_moves();
    processor.process_buffer(gcode);
    processor.finalize(false);
    result = std::move(processor.extract_result());
}

void process_file(const std::string &gcode, GCodeProcessorResult &result,
                  GCodeFlavor flavor = gcfMarlinFirmware, bool bbl_printer = false)
{
    ScopedTemporaryFile temp(".gcode");
    {
        std::ofstream output(temp.string());
        REQUIRE(output.good());
        output << gcode;
        output.flush();
        REQUIRE(output.good());
    }

    FullPrintConfig config;
    config.gcode_flavor.value = flavor;
    const bool previous_bbl = GCodeProcessor::s_IsBBLPrinter;
    const ScopeGuard restore_bbl([previous_bbl] { GCodeProcessor::s_IsBBLPrinter = previous_bbl; });
    GCodeProcessor::s_IsBBLPrinter = bbl_printer;

    GCodeProcessor processor;
    processor.apply_config(config);
    processor.process_file(temp.string());
    result = std::move(processor.extract_result());
}

std::string orca_header_with_defaults()
{
    const FullPrintConfig config;
    std::string header = "; generated by OrcaSlicer\n; BambuStudio\n; CONFIG_BLOCK_START\n";
    for (const std::string &key : config.keys()) {
        const ConfigOption *option = config.option(key);
        if (option != nullptr)
            header += "; " + key + " = " + option->serialize() + "\n";
    }
    header += "; CONFIG_BLOCK_END\n";
    return header;
}

std::vector<const GCodeProcessorResult::MoveVertex *> moves_of(const GCodeProcessorResult &result, EMoveType type)
{
    std::vector<const GCodeProcessorResult::MoveVertex *> moves;
    for (const auto &move : result.moves)
        if (!move.internal_only && move.type == type)
            moves.push_back(&move);
    return moves;
}

} // namespace

TEST_CASE("G-code line helpers select the last complete motion and tool records", "[GCodeProcessor][GCodeProcessorExtended][Parser]")
{
    const std::string gcode =
        "T1\n"
        "G1 X1 Y2 Z0.2 E0.4\n"
        "T4\n"
        "G2 X3 Y4 I1 J0\n"
        "G1 X5 Y6\n";

    CHECK(GCodeProcessor::get_gcode_last_filament(gcode) == 4);

    double z = -1.;
    REQUIRE(GCodeProcessor::get_last_z_from_gcode(gcode, z));
    CHECK_THAT(z, WithinAbs(0.2, 1e-6));

    Vec3f position = Vec3f::Zero();
    REQUIRE(GCodeProcessor::get_last_position_from_gcode(gcode, position));
    CHECK_THAT(position.x(), WithinAbs(5., 1e-6));
    CHECK_THAT(position.y(), WithinAbs(6., 1e-6));
    CHECK_THAT(position.z(), WithinAbs(0.2, 1e-6));

    double unchanged_z = 17.;
    CHECK_FALSE(GCodeProcessor::get_last_z_from_gcode("; only a comment\n", unchanged_z));
    CHECK_THAT(unchanged_z, WithinAbs(17., 1e-6));
}

TEST_CASE("Absolute XYZ and relative E preserve analytic endpoints and extrusion deltas", "[GCodeProcessor][GCodeProcessorExtended][Modal]")
{
    GCodeProcessorResult result;
    process_buffer(
        "G90\nM83\n"
        "G1 X1 Y2 Z0.2 F600\n"
        "G1 X11 Y2 E2 F600\n"
        "G91\n"
        "G1 X1 Y-2 E0.5\n"
        "G90\n"
        "G1 X20 Y0 E3\n", result);
    const auto extrusions = moves_of(result, EMoveType::Extrude);
    REQUIRE(extrusions.size() == 3);

    CHECK_THAT(extrusions[0]->position.x(), WithinAbs(11., 1e-4));
    CHECK_THAT(extrusions[0]->position.y(), WithinAbs(2., 1e-4));
    CHECK_THAT(extrusions[0]->delta_extruder, WithinAbs(2., 1e-4));
    CHECK_THAT(extrusions[1]->position.x(), WithinAbs(12., 1e-4));
    CHECK_THAT(extrusions[1]->position.y(), WithinAbs(0., 1e-4));
    CHECK_THAT(extrusions[1]->delta_extruder, WithinAbs(0.5, 1e-4));
    CHECK_THAT(extrusions[2]->position.x(), WithinAbs(20., 1e-4));
    CHECK_THAT(extrusions[2]->position.y(), WithinAbs(0., 1e-4));
    CHECK_THAT(extrusions[2]->delta_extruder, WithinAbs(3., 1e-4));
}

TEST_CASE("G92 and unit changes reset coordinates without changing physical millimeters", "[GCodeProcessor][GCodeProcessorExtended][Modal]")
{
    GCodeProcessorResult result;
    process_buffer(
        "G90\nM82\nG20\n"
        "G1 X1 Y1 Z0.1 E1 F60\n"
        "G92 X0 Y0 Z0 E0\n"
        "G21\n"
        "G1 X10 Y0 Z0.2 E1 F600\n", result);
    const auto extrusions = moves_of(result, EMoveType::Extrude);
    REQUIRE(extrusions.size() == 2);
    CHECK_THAT(extrusions[0]->position.x(), WithinAbs(25.4, 1e-4));
    CHECK_THAT(extrusions[0]->position.y(), WithinAbs(25.4, 1e-4));
    CHECK_THAT(extrusions[0]->delta_extruder, WithinAbs(25.4, 1e-4));
    CHECK_THAT(extrusions[1]->position.x(), WithinAbs(35.4, 1e-4));
    CHECK_THAT(extrusions[1]->position.y(), WithinAbs(25.4, 1e-4));
    CHECK_THAT(extrusions[1]->position.z(), WithinAbs(2.74, 1e-4));
    CHECK_THAT(extrusions[1]->delta_extruder, WithinAbs(1., 1e-4));
}

TEST_CASE("Thermal, fan, and pressure commands are stamped onto following moves", "[GCodeProcessor][GCodeProcessorExtended][State]")
{
    GCodeProcessorResult result;
    process_buffer(
        "M83\nG90\nM104 S205\nM106 S128\nM900 K0.42\n"
        "G1 X10 Y0 Z0.2 E1 F600\n"
        "M107\nM572 S0.18\nM109 S210\n"
        "G1 X20 Y0 E1 F600\n", result);
    const auto extrusions = moves_of(result, EMoveType::Extrude);
    REQUIRE(extrusions.size() == 2);
    CHECK_THAT(extrusions[0]->temperature, WithinAbs(205., 1e-4));
    CHECK_THAT(extrusions[0]->fan_speed, WithinAbs(100. * 128. / 255., 1e-4));
    CHECK_THAT(extrusions[0]->pressure_advance, WithinAbs(0.42, 1e-4));
    CHECK_THAT(extrusions[1]->temperature, WithinAbs(210., 1e-4));
    CHECK_THAT(extrusions[1]->fan_speed, WithinAbs(0., 1e-4));
    CHECK_THAT(extrusions[1]->pressure_advance, WithinAbs(0.18, 1e-4));
}

TEST_CASE("Klipper pressure and acceleration commands update the next move's metadata", "[GCodeProcessor][GCodeProcessorExtended][Firmware]")
{
    GCodeProcessorResult result;
    process_buffer(
        "M83\nG90\nSET_PRESSURE_ADVANCE ADVANCE=0.31\n"
        "SET_VELOCITY_LIMIT ACCEL=500\n"
        "G1 X10 Y0 Z0.2 E1 F600\n", result, gcfKlipper);
    const auto extrusions = moves_of(result, EMoveType::Extrude);
    REQUIRE(extrusions.size() == 1);
    CHECK_THAT(extrusions.front()->pressure_advance, WithinAbs(0.31, 1e-4));
    CHECK_THAT(extrusions.front()->acceleration, WithinAbs(500., 1e-3));
}

TEST_CASE("Role geometry and layer tags are carried to extrusion vertices", "[GCodeProcessor][GCodeProcessorExtended][Tags]")
{
    GCodeProcessorResult result;
    process_buffer(
        "M83\nG90\n"
        ";LAYER_CHANGE\n"
        "; Z_HEIGHT: 0.2\n"
        ";TYPE:Outer wall\n"
        ";WIDTH:0.80\n"
        ";HEIGHT:0.30\n"
        "G1 X10 Y0 Z0.2 E1 F600\n"
        ";LAYER_CHANGE\n"
        ";TYPE:Inner wall\n"
        "G1 X20 Y0 E1\n", result);
    const auto extrusions = moves_of(result, EMoveType::Extrude);
    REQUIRE(extrusions.size() == 2);
    CHECK(extrusions[0]->extrusion_role == erExternalPerimeter);
    CHECK_THAT(extrusions[0]->width, WithinAbs(0.80, 1e-4));
    CHECK_THAT(extrusions[0]->height, WithinAbs(0.30, 1e-4));
    CHECK(extrusions[0]->layer_id == 0);
    CHECK_THAT(extrusions[0]->print_z, WithinAbs(0.2, 1e-4));
    CHECK(extrusions[1]->extrusion_role == erPerimeter);
    CHECK(extrusions[1]->layer_id == 1);
}

TEST_CASE("Arc extrusion ends at its declared endpoint and remains an extrusion", "[GCodeProcessor][GCodeProcessorExtended][Arcs]")
{
    GCodeProcessorResult result;
    process_buffer(
        "M83\nG90\n"
        "G1 X0 Y0 Z0.2 F600\n"
        "G1 X10 Y0 E1 F600\n"
        "G2 X0 Y10 I-5 J5 E1 F600\n", result);
    const auto extrusions = moves_of(result, EMoveType::Extrude);
    REQUIRE(extrusions.size() > 2);
    CHECK_THAT(extrusions.back()->position.x(), WithinAbs(0., 1e-3));
    CHECK_THAT(extrusions.back()->position.y(), WithinAbs(10., 1e-3));
    float arc_extrusion = 0.f;
    for (size_t i = 1; i < extrusions.size(); ++i)
        arc_extrusion += extrusions[i]->delta_extruder;
    CHECK_THAT(arc_extrusion, WithinAbs(1., 1e-3));
}

TEST_CASE("Firmware retract, homing, and explicit retract markers classify moves", "[GCodeProcessor][GCodeProcessorExtended][Commands]")
{
    GCodeProcessorResult result;
    process_buffer(
        "M83\nG90\n"
        "G1 X10 Y0 Z0.2 E1 F600\n"
        "G10\nG11\nG22\nG23\nG28 X Y\n"
        "G1 X20 Y0 E1 F600\n", result, gcfMarlinFirmware, false, 1.0f);
    const auto retracts = moves_of(result, EMoveType::Retract);
    const auto unretracts = moves_of(result, EMoveType::Unretract);
    REQUIRE(retracts.size() >= 2);
    REQUIRE(unretracts.size() >= 2);
    CHECK(retracts.front()->delta_extruder < 0.f);
    CHECK(unretracts.front()->delta_extruder > 0.f);

    const auto extrusions = moves_of(result, EMoveType::Extrude);
    REQUIRE(extrusions.size() >= 2);
    CHECK_THAT(extrusions.back()->position.x(), WithinAbs(20., 1e-3));
    CHECK_THAT(extrusions.back()->position.y(), WithinAbs(0., 1e-3));
}

TEST_CASE("Bambu producer comments retain standard roles during file processing", "[GCodeProcessor][GCodeProcessorExtended][Producer]")
{
    GCodeProcessorResult result;
    process_file(
        orca_header_with_defaults() +
        "M83\nG90\n"
        "; FEATURE: Outer wall\n"
        "G1 X10 Y0 Z0.2 E1 F600\n"
        "; CHANGE_LAYER\n"
        "; FEATURE: Inner wall\n"
        "G1 X20 Y0 E1 F600\n", result, gcfMarlinFirmware, true);
    const auto extrusions = moves_of(result, EMoveType::Extrude);
    REQUIRE(extrusions.size() == 2);
    CHECK(extrusions[0]->extrusion_role == erExternalPerimeter);
    CHECK(extrusions[1]->extrusion_role == erPerimeter);
    CHECK(extrusions[0]->layer_id == 0);
    CHECK(extrusions[1]->layer_id == 0);
}

TEST_CASE("Skippable tags contribute only time inside the requested region", "[GCodeProcessor][GCodeProcessorExtended][Tags]")
{
    GCodeProcessorResult result;
    process_buffer(
        "M83\nG90\n"
        ";TYPE:Outer wall\n"
        "; SKIPPABLE_START\n"
        "; SKIPTYPE: timelapse\n"
        "G1 X10 Y0 Z0.2 E1 F600\n"
        "; SKIPPABLE_END\n"
        "G1 X20 Y0 E1 F600\n", result);
    REQUIRE(result.print_statistics.modes[0].time > 0.f);
    const auto extrusions = moves_of(result, EMoveType::Extrude);
    REQUIRE(extrusions.size() == 2);
    CHECK(extrusions[0]->time[0] > 0.f);
    CHECK(extrusions[1]->time[0] > 0.f);

    // The result exposes the estimator's partition explicitly.  Check that the
    // marked move contributes to the timelapse bucket and the following move does
    // not, rather than merely checking that both moves took nonzero time.
    const auto skipped = result.skippable_part_time.find(SkipType::stTimelapse);
    REQUIRE(skipped != result.skippable_part_time.end());
    CHECK(skipped->second > 0.f);
    CHECK_THAT(skipped->second, WithinAbs(extrusions[0]->time[0], 1e-3));
    CHECK(skipped->second < result.print_statistics.modes[0].time);
}

} // namespace extended_command_contracts

namespace gcode_processor_deep_contracts {
using namespace Slic3r;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {

struct RunOptions {
    GCodeFlavor flavor = gcfMarlinFirmware;
    bool bbl_printer = false;
    double retract_length = 0.0;
    double unload_time = 0.0;
    double max_speed_e = 0.0;
    std::vector<double> filament_diameters;
    std::vector<int> filament_map;
};

struct TestResult : GCodeProcessorResult {
    TestResult() = default;
    TestResult(TestResult&& other) noexcept { GCodeProcessorResult::operator=(std::move(other)); }
    TestResult& operator=(TestResult&& other) noexcept
    {
        GCodeProcessorResult::operator=(std::move(other));
        return *this;
    }
    TestResult& operator=(GCodeProcessorResult&& other) noexcept
    {
        GCodeProcessorResult::operator=(std::move(other));
        return *this;
    }
};

TestResult process_buffer(const std::string& gcode, RunOptions options = {})
{
    FullPrintConfig config;
    config.gcode_flavor.value = options.flavor;
    if (options.retract_length > 0.0)
        config.retraction_length.values = {options.retract_length};
    if (options.unload_time > 0.0)
        config.machine_unload_filament_time.value = options.unload_time;
    if (!options.filament_diameters.empty())
        config.filament_diameter.values = options.filament_diameters;
    if (!options.filament_map.empty())
        config.filament_map.values = options.filament_map;
    if (options.max_speed_e > 0.0)
        config.machine_max_speed_e.values = {options.max_speed_e, options.max_speed_e,
                                              options.max_speed_e, options.max_speed_e};

    const bool previous_bbl = GCodeProcessor::s_IsBBLPrinter;
    const ScopeGuard restore_bbl([previous_bbl] { GCodeProcessor::s_IsBBLPrinter = previous_bbl; });
    GCodeProcessor::s_IsBBLPrinter = options.bbl_printer;

    GCodeProcessor processor;
    processor.apply_config(config);
    processor.initialize("gcode_processor_contract.gcode");
    processor.initialize_result_moves();
    processor.process_buffer(gcode);
    processor.finalize(false);
    TestResult result;
    result = std::move(processor.extract_result());
    return result;
}

TestResult process_file(const std::string& gcode, RunOptions options = {})
{
    ScopedTemporaryFile temp(".gcode");
    {
        std::ofstream output(temp.string());
        REQUIRE(output.good());
        output << gcode;
        output.flush();
        REQUIRE(output.good());
    }

    FullPrintConfig config;
    config.gcode_flavor.value = options.flavor;
    if (options.filament_diameters.empty() == false)
        config.filament_diameter.values = options.filament_diameters;
    if (options.filament_map.empty() == false)
        config.filament_map.values = options.filament_map;
    const bool previous_bbl = GCodeProcessor::s_IsBBLPrinter;
    const ScopeGuard restore_bbl([previous_bbl] { GCodeProcessor::s_IsBBLPrinter = previous_bbl; });
    GCodeProcessor::s_IsBBLPrinter = options.bbl_printer;

    GCodeProcessor processor;
    processor.apply_config(config);
    processor.process_file(temp.string());
    TestResult result;
    result = std::move(processor.extract_result());
    return result;
}

std::vector<const GCodeProcessorResult::MoveVertex *> moves_of(const GCodeProcessorResult& result, EMoveType type)
{
    std::vector<const GCodeProcessorResult::MoveVertex *> out;
    for (const auto& move : result.moves)
        if (!move.internal_only && move.type == type)
            out.push_back(&move);
    return out;
}

const GCodeProcessorResult::MoveVertex *last_move_of(const GCodeProcessorResult& result, EMoveType type)
{
    for (auto it = result.moves.rbegin(); it != result.moves.rend(); ++it)
        if (!it->internal_only && it->type == type)
            return &*it;
    return nullptr;
}

double total_extruded_mm(const GCodeProcessorResult& result)
{
    double total = 0.0;
    for (const auto& move : result.moves)
        if (!move.internal_only && move.type == EMoveType::Extrude)
            total += move.delta_extruder;
    return total;
}

} // namespace

TEST_CASE("Custom markers create typed events and preserve per-Z metadata", "[GCodeProcessor][GCodeProcessorDeep][Metadata]")
{
    const auto result = process_buffer(
        "M83\nG90\nG1 X5 Y0 Z0.2 E0.5 F600\nG1 X10 E0.5\n"
        "; COLOR_CHANGE,T0,#112233\n"
        "G1 X15 E0.5\nG1 X20 E0.5\n"
        "; PAUSE_PRINTING\n"
        "G1 X25 E0.5\nG1 X30 E0.5\n"
        "; CUSTOM_GCODE\n"
        "G1 X35 E0.5\nG1 X40 E0.5\n", RunOptions{gcfMarlinFirmware, true});

    const auto color = moves_of(result, EMoveType::Color_change);
    const auto pause = moves_of(result, EMoveType::Pause_Print);
    const auto custom = moves_of(result, EMoveType::Custom_GCode);
    REQUIRE(color.size() == 1);
    REQUIRE(pause.size() == 1);
    REQUIRE(custom.size() == 1);
    REQUIRE(result.custom_gcode_per_print_z.size() == 3);
    CHECK(result.custom_gcode_per_print_z[0].type == CustomGCode::ColorChange);
    CHECK(result.custom_gcode_per_print_z[0].extruder == 1);
    CHECK(result.custom_gcode_per_print_z[0].color == "#112233");
    CHECK(result.custom_gcode_per_print_z[1].type == CustomGCode::PausePrint);
    CHECK(result.custom_gcode_per_print_z[2].type == CustomGCode::Custom);
    CHECK_THAT(result.custom_gcode_per_print_z[0].print_z, WithinAbs(0.2, 1e-4));
    CHECK_THAT(result.custom_gcode_per_print_z[1].print_z, WithinAbs(0.2, 1e-4));
    CHECK_THAT(result.custom_gcode_per_print_z[2].print_z, WithinAbs(0.2, 1e-4));
    // Two planned moves precede each timed marker so the estimator flushes a
    // nonempty partition. A single move would leave only the synthetic final
    // ColorChange bucket and could hide a misclassified marker.
    const auto &times = result.print_statistics.modes[0].custom_gcode_times;
    REQUIRE(times.size() == 3);
    CHECK(times[0].first == CustomGCode::ColorChange);
    CHECK(times[1].first == CustomGCode::PausePrint);
    CHECK(times[2].first == CustomGCode::ColorChange);
    CHECK(times[0].second.first >= 1.f); // 10 mm at 600 mm/min, plus acceleration.
    CHECK(times[1].second.first >= 1.f);
    CHECK(times[2].second.first >= 2.f);
    const double partition_sum = times[0].second.first + times[1].second.first + times[2].second.first;
    CHECK_THAT(partition_sum, WithinAbs(result.print_statistics.modes[0].time, 1e-5));
    CHECK_THAT(times[1].second.second, WithinAbs(times[1].second.first + times[2].second.first, 1e-5));
    REQUIRE(result.print_statistics.volumes_per_color_change.size() == 2);
    CHECK(result.print_statistics.volumes_per_color_change.front() > 0.0);
    CHECK(result.print_statistics.volumes_per_color_change.back() > 0.0);
}

TEST_CASE("Wipe, wipe tower, and flush regions partition move and filament statistics", "[GCodeProcessor][GCodeProcessorDeep][Metadata]")
{
    const auto result = process_buffer(
        "M83\nG90\nG1 X0 Y0 Z0.2 F600\n"
        "; FEATURE: Outer wall\nG1 X10 Y0 E1 F600\n"
        "; WIPE_START\nG1 X20 Y0 E1\n; WIPE_END\n"
        "; FEATURE: Prime tower\n; WIPE_TOWER_START\nG1 X30 Y0 E1\n; WIPE_TOWER_END\n"
        "; FLUSH_START\nG10\nG11\n; FLUSH_END\n"
        "; VFLUSH_START\nG10\nG11\n; VFLUSH_END\n",
        RunOptions{gcfMarlinFirmware, true, 1.0});

    const auto wipes = moves_of(result, EMoveType::Wipe);
    REQUIRE(wipes.size() == 1);
    CHECK_THAT(wipes.front()->width, WithinAbs(GCodeProcessor::Wipe_Width, 1e-6));
    CHECK_THAT(wipes.front()->height, WithinAbs(GCodeProcessor::Wipe_Height, 1e-6));

    const auto extrusions = moves_of(result, EMoveType::Extrude);
    REQUIRE(extrusions.size() == 2);
    REQUIRE(result.print_statistics.wipe_tower_volumes_per_extruder.count(0) == 1);
    REQUIRE(result.print_statistics.flush_per_filament.count(0) == 1);
    CHECK(result.print_statistics.wipe_tower_volumes_per_extruder.at(0) > 0.0);
    CHECK(result.print_statistics.flush_per_filament.at(0) > 0.0);
    CHECK(result.print_statistics.total_volumes_per_extruder.at(0) >
          result.print_statistics.model_volumes_per_extruder.at(0));
}

TEST_CASE("External purge contributes to flush filament without creating a motion", "[GCodeProcessor][GCodeProcessorDeep][Metadata]")
{
    const auto result = process_buffer(
        "M83\nG90\nG1 X10 Y0 Z0.2 E1 F600\n"
        "; EXTERNAL_PURGE 2.5\nG1 X20 Y0 E1 F600\n");
    REQUIRE(result.print_statistics.flush_per_filament.count(0) == 1);
    const double filament_area = PI * 0.25 * 1.75 * 1.75;
    CHECK_THAT(result.print_statistics.flush_per_filament.at(0),
               WithinAbs(2.5 * filament_area, 1e-3));
    CHECK(moves_of(result, EMoveType::Extrude).size() == 2);
}

TEST_CASE("M221 scales planned extrusion feed without changing geometric E", "[GCodeProcessor][GCodeProcessorDeep][Flow]")
{
    const std::string prefix = "M83\nG90\nG1 X0 Y0 Z0.2 F600\n";
    RunOptions constrained;
    constrained.max_speed_e = 0.75;
    const auto full = process_buffer(prefix + "M221 S100\nG1 X20 Y0 E2 F600\n", constrained);
    const auto half = process_buffer(prefix + "M221 S50\nG1 X20 Y0 E2 F600\n", constrained);
    const auto full_move = last_move_of(full, EMoveType::Extrude);
    const auto half_move = last_move_of(half, EMoveType::Extrude);
    REQUIRE(full_move != nullptr);
    REQUIRE(half_move != nullptr);
    CHECK_THAT(full_move->delta_extruder, WithinAbs(2.0, 1e-5));
    CHECK_THAT(half_move->delta_extruder, WithinAbs(2.0, 1e-5));
    CHECK(full_move->time[0] > half_move->time[0]);
    CHECK(full.print_statistics.modes[0].time > half.print_statistics.modes[0].time);
}

TEST_CASE("M204, M205, and M566 stamp acceleration and jerk limits", "[GCodeProcessor][GCodeProcessorDeep][Limits]")
{
    const auto result = process_buffer(
        "M83\nG90\nM204 P420 R310 T280\nM205 X7 Y8 Z9 E10 J0.03\nM566 X1 Y2 Z3 E4\n"
        "G1 X10 Y0 Z0.2 E1 F600\nG1 X20 Y0 E1 F600\n", RunOptions{gcfRepRapSprinter});
    const auto extrusion = last_move_of(result, EMoveType::Extrude);
    REQUIRE(extrusion != nullptr);
    CHECK_THAT(extrusion->acceleration, WithinAbs(420.0, 1e-3));
    CHECK_THAT(extrusion->jerk, WithinAbs(1.0 / 60.0, 1e-4));
    CHECK(extrusion->time[0] > 0.0f);
}

TEST_CASE("Legacy M204 S/T forms set extrusion, travel, and retract acceleration", "[GCodeProcessor][GCodeProcessorDeep][Limits]")
{
    const auto result = process_buffer(
        "M83\nG90\nM204 S330 T220\nG1 X10 Y0 Z0.2 E1 F600\nG1 X20 Y0 F600\n"
        "G10\nG11\n", RunOptions{gcfMarlinLegacy, false, 1.0});
    const auto extrusion = moves_of(result, EMoveType::Extrude);
    const auto travels = moves_of(result, EMoveType::Travel);
    const auto retracts = moves_of(result, EMoveType::Retract);
    REQUIRE(!extrusion.empty());
    REQUIRE(!travels.empty());
    REQUIRE(!retracts.empty());
    CHECK_THAT(extrusion.back()->acceleration, WithinAbs(330.0, 1e-3));
    CHECK_THAT(travels.back()->acceleration, WithinAbs(330.0, 1e-3));
    CHECK_THAT(retracts.back()->acceleration, WithinAbs(220.0, 1e-3));
}

TEST_CASE("Klipper velocity limit accepts acceleration, velocity, and corner speed independently", "[GCodeProcessor][GCodeProcessorDeep][Klipper]")
{
    const auto baseline = process_buffer(
        "M83\nG90\nG1 X10 Y0 Z0.2 E1 F2400\nG1 X20 Y0 E1 F2400\n", RunOptions{gcfKlipper});
    const auto result = process_buffer(
        "M83\nG90\nSET_VELOCITY_LIMIT VELOCITY=12 ACCEL=480 SQUARE_CORNER_VELOCITY=3\n"
        "G1 X10 Y0 Z0.2 E1 F2400\nG1 X20 Y0 E1 F2400\n", RunOptions{gcfKlipper});
    const auto extrusion = last_move_of(result, EMoveType::Extrude);
    REQUIRE(extrusion != nullptr);
    CHECK_THAT(extrusion->acceleration, WithinAbs(480.0, 1e-3));
    CHECK(extrusion->jerk > 0.0f);
    CHECK(result.print_statistics.modes[0].time > baseline.print_statistics.modes[0].time);
}

TEST_CASE("G29 timing is gated by BBL mode and M622/M623 measurement state", "[GCodeProcessor][GCodeProcessorDeep][Timing]")
{
    const std::string prefix = "M83\nG90\nG1 X1 Y0 Z0.2 E0.1 F600\n";
    const auto generic = process_buffer(prefix + "G29\nG1 X2 Y0 E0.1 F600\n", RunOptions{gcfMarlinFirmware, false});
    const auto bbl_off = process_buffer(prefix + "G29\nG1 X2 Y0 E0.1 F600\n", RunOptions{gcfMarlinFirmware, true});
    const auto bbl_on = process_buffer(prefix + "M622 J1\nG29\nM623\nG1 X2 Y0 E0.1 F600\n", RunOptions{gcfMarlinFirmware, true});
    CHECK(generic.print_statistics.modes[0].time > 200.0f);
    CHECK(bbl_off.print_statistics.modes[0].time < 1.0f);
    CHECK(bbl_on.print_statistics.modes[0].time > bbl_off.print_statistics.modes[0].time + 200.0f);
}

TEST_CASE("M1020 selects valid mapped filaments and rejects out-of-range IDs", "[GCodeProcessor][GCodeProcessorDeep][Firmware]")
{
    RunOptions options;
    options.filament_diameters = {1.75, 1.75};
    options.filament_map = {1, 2};
    const auto result = process_buffer(
        "M83\nG90\nM1020 S0 H0\nG1 X10 Y0 Z0.2 E1 F600\n"
        "M1020 S1 H1\nG1 X20 Y0 E1 F600\nM1020 S99 H0\nG1 X30 Y0 E1 F600\n", options);
    const auto extrusions = moves_of(result, EMoveType::Extrude);
    const auto changes = moves_of(result, EMoveType::Tool_change);
    REQUIRE(extrusions.size() == 3);
    REQUIRE(changes.size() >= 2);
    CHECK(extrusions[1]->extruder_id != extrusions[0]->extruder_id);
    CHECK(extrusions[2]->extruder_id == extrusions[1]->extruder_id);
}

TEST_CASE("M132 resets only the selected absolute axis origin", "[GCodeProcessor][GCodeProcessorDeep][Modal]")
{
    const auto reset_x = process_buffer(
        "M83\nG90\nG1 X5 Y2 Z0.2 E1 F600\nG92 X0\nM132 X\nG1 X10 Y2 E1 F600\n");
    const auto ordinary = process_buffer(
        "M83\nG90\nG1 X5 Y2 Z0.2 E1 F600\nG92 X0\nG1 X10 Y2 E1 F600\n");
    const auto reset_move = last_move_of(reset_x, EMoveType::Extrude);
    const auto ordinary_move = last_move_of(ordinary, EMoveType::Extrude);
    REQUIRE(reset_move != nullptr);
    REQUIRE(ordinary_move != nullptr);
    CHECK_THAT(reset_move->position.x(), WithinAbs(10.0, 1e-4));
    CHECK_THAT(ordinary_move->position.x(), WithinAbs(15.0, 1e-4));
    CHECK_THAT(reset_move->position.y(), WithinAbs(2.0, 1e-4));
}

TEST_CASE("M400, M191, and SYNC add exactly their modeled delays", "[GCodeProcessor][GCodeProcessorDeep][Timing]")
{
    const std::string base = "M83\nG90\nG1 X10 Y0 Z0.2 E1 F600\nG1 X20 Y0 E1 F600\n";
    const auto no_wait = process_buffer(base);
    const auto waits = process_buffer(base + "M400 S2 P500\nM191 S41\nSYNC T3\n");
    const float delta = waits.print_statistics.modes[0].time - no_wait.print_statistics.modes[0].time;
    CHECK_THAT(delta, WithinAbs(2.0 + 720.0 + 3.0, 1e-2));
    CHECK(waits.print_statistics.modes[0].time > no_wait.print_statistics.modes[0].time);
}

TEST_CASE("Object labels attach to moves until the matching stop tag", "[GCodeProcessor][GCodeProcessorDeep][Metadata]")
{
    const auto result = process_buffer(
        "M83\nG90\n; start printing object: 7\n"
        "G1 X10 Y0 Z0.2 E1 F600\n; stop printing object\n"
        "G1 X20 Y0 E1 F600\n");
    const auto extrusions = moves_of(result, EMoveType::Extrude);
    REQUIRE(extrusions.size() == 2);
    CHECK(extrusions[0]->object_label_id == 7);
    CHECK(extrusions[1]->object_label_id == -1);
}

TEST_CASE("Unknown skippable types are isolated from timelapse accounting", "[GCodeProcessor][GCodeProcessorDeep][Timing]")
{
    const auto result = process_buffer(
        "M83\nG90\n; SKIPPABLE_START\n; SKIPTYPE: vendor_extra\n"
        "G1 X10 Y0 Z0.2 E1 F600\n; SKIPPABLE_END\nG1 X20 Y0 E1 F600\n");
    REQUIRE(result.print_statistics.modes[0].time > 0.0f);
    const auto other = result.skippable_part_time.find(SkipType::stOther);
    REQUIRE(other != result.skippable_part_time.end());
    CHECK(other->second > 0.0f);
    CHECK(result.skippable_part_time.find(SkipType::stTimelapse) == result.skippable_part_time.end());
}

TEST_CASE("M108 and M135 tool aliases are flavor-gated", "[GCodeProcessor][GCodeProcessorDeep][Firmware]")
{
    const std::string body = "M83\nG90\nT0\nG1 X10 Y0 Z0.2 E1 F600\nM108 T1\nM135 T1\nG1 X20 Y0 E1 F600\n";
    RunOptions mapped;
    mapped.filament_diameters = {1.75, 1.75};
    mapped.filament_map = {1, 2};
    mapped.flavor = gcfSailfish;
    const auto sailfish = process_buffer(body, mapped);
    mapped.flavor = gcfMakerWare;
    const auto makerware = process_buffer(body, mapped);
    mapped.flavor = gcfMarlinFirmware;
    const auto marlin = process_buffer(body, mapped);
    CHECK(sailfish.print_statistics.total_extruder_changes >= 1);
    CHECK(makerware.print_statistics.total_extruder_changes >= 1);
    CHECK(marlin.print_statistics.total_extruder_changes == 0);
}

TEST_CASE("Malformed and unknown commands do not corrupt later valid motion", "[GCodeProcessor][GCodeProcessorDeep][Recovery]")
{
    const auto result = process_buffer(
        "M83\nG90\nG1 X10 Y0 Z0.2 E1 F600\n"
        "G1 Xbad Y?? Eoops\nUNKNOWN FOO BAR\n; malformed tag WIDTH:not-a-number\n"
        ";VUNKNOWN FOO\nG1 X20 Y0 E1 F600\n");
    const auto extrusions = moves_of(result, EMoveType::Extrude);
    REQUIRE(extrusions.size() == 2);
    CHECK_THAT(extrusions.back()->position.x(), WithinAbs(20.0, 1e-4));
    CHECK_THAT(extrusions.back()->delta_extruder, WithinAbs(1.0, 1e-4));
    CHECK(std::isfinite(result.print_statistics.modes[0].time));
}

TEST_CASE("File import keeps a valid Orca config block and ignores malformed body lines", "[GCodeProcessor][GCodeProcessorDeep][Import]")
{
    const FullPrintConfig defaults;
    std::string gcode = "; generated by OrcaSlicer\n; CONFIG_BLOCK_START\n";
    for (const std::string& key : defaults.keys()) {
        const ConfigOption* option = defaults.option(key);
        if (option != nullptr)
            gcode += "; " + key + " = " + option->serialize() + "\n";
    }
    gcode += "; CONFIG_BLOCK_END\n";
    gcode += "M83\nG90\nG1 X10 Y0 Z0.2 E1 F600\nG1 Xbad Eoops\nG1 X20 Y0 E1 F600\n";
    const auto result = process_file(gcode, RunOptions{gcfMarlinFirmware, true});
    REQUIRE(moves_of(result, EMoveType::Extrude).size() == 2);
    CHECK_THAT(total_extruded_mm(result), WithinAbs(2.0, 1e-4));
    CHECK(result.filament_diameters.size() >= defaults.filament_diameter.values.size());
}

TEST_CASE("M702 C accounts for unloading a loaded filament", "[GCodeProcessor][GCodeProcessorUnloadRegression]")
{
    const std::string loaded_gcode = "M83\nG90\nT0\nG1 X10 Y0 Z0.2 E1 F600\n";
    const auto loaded = process_buffer(loaded_gcode, RunOptions{gcfMarlinFirmware, false, 0.0, 12.0});
    const auto result = process_buffer(
        loaded_gcode + "M702 C\nM702 C\n",
        RunOptions{gcfMarlinFirmware, false, 0.0, 12.0});
    CHECK_THAT(result.print_statistics.total_filament_unload_time, WithinAbs(12.0, 1e-4));
    CHECK_THAT(result.print_statistics.modes[0].time - loaded.print_statistics.modes[0].time,
               WithinAbs(12.0, 1e-3));
}

TEST_CASE("M702 without C has no unloading side effect", "[GCodeProcessor][GCodeProcessorUnloadRegression]")
{
    const auto result = process_buffer(
        "M83\nG90\nT0\nG1 X10 Y0 Z0.2 E1 F600\nM702\n",
        RunOptions{gcfMarlinFirmware, false, 0.0, 12.0});
    CHECK_THAT(result.print_statistics.total_filament_unload_time, WithinAbs(0.0, 1e-4));
}

TEST_CASE("Repetier saved positions become the origin of the next move", "[GCodeProcessor][GCodeProcessorRepetierRegression]")
{
    const auto result = process_buffer(
        "M83\nG90\nG1 X10 Y20 Z0.2 F600\nM401\nG1 X30 Y40 Z1.2 F1200\nM402\nG1 X15 Y25 Z0.2 E1\n",
        RunOptions{gcfRepetier});
    const auto extrusions = moves_of(result, EMoveType::Extrude);
    REQUIRE(extrusions.size() == 1);
    CHECK_THAT(extrusions.front()->position.x(), WithinAbs(15.0, 1e-4));
    CHECK_THAT(extrusions.front()->position.y(), WithinAbs(25.0, 1e-4));
    CHECK_THAT(extrusions.front()->travel_dist, WithinAbs(std::sqrt(50.0), 1e-4));
    CHECK_THAT(extrusions.front()->feedrate, WithinAbs(10.0, 1e-4));
}

TEST_CASE("Unknown-axis G92 preserves XYZ origins while bare G92 resets them", "[GCodeProcessor][GCodeProcessorDeep][GCodeProcessorUnknownAxis]")
{
    const std::string prefix = "M83\nG90\nG1 X10 Y20 Z0.2 E1 F600\n";
    const auto unknown_axis = process_buffer(prefix + "G92 A0 B0\nG1 X11 Y21 E2 F600\n");
    const auto bare = process_buffer(prefix + "G92\nG1 X11 Y21 E2 F600\n");
    const auto unknown_extrusions = moves_of(unknown_axis, EMoveType::Extrude);
    const auto bare_extrusions = moves_of(bare, EMoveType::Extrude);
    REQUIRE(unknown_extrusions.size() == 2);
    REQUIRE(bare_extrusions.size() == 2);

    CHECK_THAT(unknown_extrusions.front()->position.x(), WithinAbs(10.0, 1e-4));
    CHECK_THAT(unknown_extrusions.front()->position.y(), WithinAbs(20.0, 1e-4));
    CHECK_THAT(unknown_extrusions.back()->position.x(), WithinAbs(11.0, 1e-4));
    CHECK_THAT(unknown_extrusions.back()->position.y(), WithinAbs(21.0, 1e-4));
    CHECK_THAT(unknown_extrusions.back()->position.z(), WithinAbs(0.2, 1e-4));
    CHECK_THAT(unknown_extrusions.back()->delta_extruder, WithinAbs(2.0, 1e-4));

    // A bare G92 is the contrasting, documented reset-to-machine-position form.
    CHECK_THAT(bare_extrusions.back()->position.x(), WithinAbs(21.0, 1e-4));
    CHECK_THAT(bare_extrusions.back()->position.y(), WithinAbs(41.0, 1e-4));
}

TEST_CASE("R-form clockwise arcs preserve the declared radius and endpoint", "[GCodeProcessor][GCodeProcessorDeep][GCodeProcessorArcR]")
{
    const auto result = process_buffer(
        "M83\nG90\n"
        "G1 X9 Y0 Z0.2 F600\n"
        "G1 X10 Y0 E1 F600\n"
        "G2 X0 Y10 R10 E1 F600\n");
    const auto extrusions = moves_of(result, EMoveType::Extrude);
    REQUIRE(extrusions.size() > 2);

    CHECK_THAT(extrusions.back()->position.x(), WithinAbs(0.0, 1e-3));
    CHECK_THAT(extrusions.back()->position.y(), WithinAbs(10.0, 1e-3));
    // The first extrusion ends at the arc start; all later ones are arc segments.
    CHECK_THAT(extrusions.front()->position.x(), WithinAbs(10.0, 1e-4));
    CHECK_THAT(extrusions.front()->position.y(), WithinAbs(0.0, 1e-4));
    float arc_extrusion = 0.f;
    for (size_t i = 1; i < extrusions.size(); ++i) {
        CHECK(extrusions[i]->delta_extruder > 0.f);
        arc_extrusion += extrusions[i]->delta_extruder;
        const double radius = std::hypot(double(extrusions[i]->position.x()) - 10.0,
                                         double(extrusions[i]->position.y()) - 10.0);
        CHECK_THAT(radius, WithinAbs(10.0, 0.05));
    }
    CHECK_THAT(arc_extrusion, WithinAbs(1.0, 1e-3));
}

} // namespace gcode_processor_deep_contracts

// Bambu firmware uses the " FEATURE: " style reserved tags, everything else the Slic3r-compatible
// "TYPE:" style, so which list applies depends on the printer kind passed in.
TEST_CASE("Reserved keyword detection follows the printer kind it is given", "[GCodeProcessor]")
{
    struct Case
    {
        const char* name;
        std::string gcode;
        bool        reserved_on_bbl;
        bool        reserved_on_non_bbl;
    };

    const auto test_case = GENERATE(values<Case>({
        {"compatible feature tag", ";TYPE:Prime tower", false, true},
        {"compatible layer tag", ";LAYER_CHANGE", false, true},
        {"bbl feature tag", "; FEATURE: Outer wall", true, false},
        {"tag shared by both lists", ";_GP_FIRST_LINE_M73_PLACEHOLDER", true, true},
        {"bbl spells this one with a leading space", ";COLOR_CHANGE", false, true},
        {"ordinary comment", "; heat the bed", false, false},
        {"not a comment at all", "G1 X10 Y10 F3000", false, false},
        // A tag counts only as the whole comment's prefix, so neither a tag mentioned mid-comment
        // nor one trailing a real command is a reserved use.
        {"tag text later in the comment", "; the TYPE:Prime tower marker", false, false},
        {"tag trailing a command", "G1 X10 ;TYPE:Prime tower", false, false},
    }));

    DYNAMIC_SECTION(test_case.name)
    {
        std::vector<std::string> tags;
        REQUIRE(GCodeProcessor::contains_reserved_tags(test_case.gcode, 5, tags, true) == test_case.reserved_on_bbl);

        tags.clear();
        REQUIRE(GCodeProcessor::contains_reserved_tags(test_case.gcode, 5, tags, false) == test_case.reserved_on_non_bbl);
    }
}

TEST_CASE("Reserved keyword detection reports every offending line", "[GCodeProcessor]")
{
    const std::string gcode = ";TYPE:Prime tower\nG1 X10\n;LAYER_CHANGE\n";

    std::vector<std::string> tags;
    REQUIRE(GCodeProcessor::contains_reserved_tags(gcode, 5, tags, false));
    REQUIRE(tags.size() == 2);
    // Reported in the order they appear, which is what makes the max_count cut-off meaningful.
    CHECK(tags[0] == "TYPE:Prime tower");
    CHECK(tags[1] == "LAYER_CHANGE");

    SECTION("the reported count is capped at max_count")
    {
        tags.clear();
        REQUIRE(GCodeProcessor::contains_reserved_tags(gcode, 1, tags, false));
        CHECK(tags.size() == 1);
        CHECK(tags[0] == "TYPE:Prime tower");
    }

    SECTION("a max_count of zero still reports the first tag")
    {
        tags.clear();
        REQUIRE(GCodeProcessor::contains_reserved_tags(gcode, 0, tags, false));
        CHECK(tags.size() == 1);
    }

    SECTION("g-code with nothing reserved in it reports nothing")
    {
        tags.clear();
        CHECK_FALSE(GCodeProcessor::contains_reserved_tags("G28\n; home all axes\n", 5, tags, false));
        CHECK(tags.empty());
    }
}

namespace {

void process_gcode(const std::string &gcode, GCodeProcessorResult &result)
{
    FullPrintConfig config;
    config.gcode_flavor.value = gcfMarlinFirmware;
    // s_IsBBLPrinter selects the "; FEATURE: " role tags the G-code uses.
    const bool       was_bbl_printer = GCodeProcessor::s_IsBBLPrinter;
    const ScopeGuard restore_bbl_printer([was_bbl_printer] { GCodeProcessor::s_IsBBLPrinter = was_bbl_printer; });
    GCodeProcessor::s_IsBBLPrinter = true;
    ScopedTemporaryFile temp(".gcode");
    std::ofstream(temp.string()) << gcode;
    GCodeProcessor processor;
    processor.apply_config(config);
    processor.process_file(temp.string());
    result = std::move(processor.extract_result());
}

// Closed outer-wall squares, each after a fast travel and before an inner-wall move, so the processor
// records seams and inserts actual speed moves. virtual_moves adds a VG1 move after each square.
void process_squares(int squares, GCodeProcessorResult &result, bool virtual_moves = false)
{
    std::ostringstream gcode;
    gcode << "M83\nG90\n";
    for (int i = 0; i < squares; ++i) {
        gcode << "G1 X10 Y10 Z" << 0.2 * (i + 1) << " F12000\n"
              << "; FEATURE: Outer wall\n"
              << "G1 X50 Y10 E2 F3000\nG1 X50 Y50 E2\nG1 X10 Y50 E2\nG1 X10 Y10 E2\n"
              << "; FEATURE: Inner wall\n"
              << "G1 X12 Y12 E0.1\nG1 X30 Y12 E1\n";
        if (virtual_moves)
            gcode << "VG1 X20 Y30 F12000\n";
    }
    process_gcode(gcode.str(), result);
}

// Objects A and B on the first layer and A again on the second, with A's brim and support. The skirt and the
// prime tower belong to neither.
void process_two_objects(GCodeProcessorResult &result)
{
    std::ostringstream gcode;
    gcode << "M83\nG90\n"
          << "; CHANGE_LAYER\n; LAYER_HEIGHT: 0.2\nG1 Z0.2 F12000\n"
          << "; FEATURE: Skirt\nG1 X0 Y100 E5 F3000\n"
          << "; FEATURE: Brim\nG1 X8 Y8 F12000\nG1 X12 Y8 E1 F3000\n"
          << "; FEATURE: Support\nG1 X10 Y20 F12000\nG1 X10 Y30 E1 F3000\n"
          << "; FEATURE: Outer wall\nG1 X10 Y10 F12000\nG1 X20 Y10 E1 F3000\n"
          << "; FEATURE: Outer wall\nG1 X50 Y50 F12000\nG1 X60 Y50 E2 F3000\n"
          << "; FEATURE: Prime tower\nG1 X80 Y80 F12000\nG1 X90 Y80 E1 F3000\n"
          << "; CHANGE_LAYER\n; LAYER_HEIGHT: 0.2\nG1 Z0.4 F12000\n"
          << "; FEATURE: Outer wall\nG1 X10 Y10 F12000\nG1 X20 Y10 E1 F3000\n";
    process_gcode(gcode.str(), result);
}

// Bead centers of process_two_objects(), half the 0.2 mm layer below the nozzle.
const Vec3d a_brim(10., 8., 0.1), a_support(10., 25., 0.1), a_wall_0(15., 10., 0.1), a_wall_1(15., 10., 0.3), b_wall(55., 50., 0.1);

Vec3d center_of(const GCodeProcessorResult::ObjectMass::Sum &sum) { return sum.moment / sum.mass; }

// One filament, so each bead weighs as much as the E it was extruded with.
Vec3d weighted_center(std::initializer_list<std::pair<double, Vec3d>> beads)
{
    double mass = 0.;
    Vec3d  moment = Vec3d::Zero();
    for (const auto &[e, center] : beads) {
        mass += e;
        moment += e * center;
    }
    return moment / mass;
}

bool is_block_move(const GCodeProcessorResult::MoveVertex &move)
{
    return !move.internal_only && (move.type == EMoveType::Extrude || move.type == EMoveType::Travel);
}

} // namespace

TEST_CASE("Actual speed moves are inserted on their block's segment just before its move", "[GCodeProcessor]")
{
    // 60 squares take several planner passes, which remap the blocks kept between passes.
    const int                  squares       = GENERATE(10, 60);
    const bool                 virtual_moves = GENERATE(false, true);
    GCodeProcessorResult       result;
    process_squares(squares, result, virtual_moves);
    const auto                &moves   = result.moves;
    constexpr size_t           normal  = size_t(PrintEstimatedStatistics::ETimeMode::Normal);

    size_t inserted = 0;
    for (size_t i = 1; i < moves.size(); ++i) {
        if (!moves[i].internal_only)
            continue;
        ++inserted;
        // Inserted moves have zero time, but a VG1 block's time is written to whatever move its move_id names.
        if (!virtual_moves)
            CHECK(moves[i].time[normal] == 0.f);
        size_t block = i + 1;
        while (block < moves.size() && moves[block].internal_only)
            ++block;
        size_t previous = i - 1;
        while (previous > 0 && moves[previous].internal_only)
            --previous;
        REQUIRE(block < moves.size());
        CHECK(moves[block].gcode_id == moves[i].gcode_id);
        const Vec3f segment = moves[block].position - moves[previous].position;
        const Vec3f offset  = moves[i].position - moves[previous].position;
        CHECK(segment.cross(offset).norm() / segment.norm() < 1e-3f);
    }
    REQUIRE(inserted > 0);
}

TEST_CASE("A seam takes the actual speed of the move it follows", "[GCodeProcessor]")
{
    GCodeProcessorResult result;
    // 10 squares fit in one planner pass, so the seam's move and the block after it are timed together.
    process_squares(10, result);
    const auto                &moves  = result.moves;

    size_t seams = 0;
    for (size_t i = 1; i < moves.size(); ++i)
        if (moves[i].type == EMoveType::Seam && is_block_move(moves[i - 1])) {
            ++seams;
            CHECK_THAT(moves[i].actual_feedrate, Catch::Matchers::WithinAbs(moves[i - 1].actual_feedrate, 1e-4));
        }
    REQUIRE(seams > 0);
}

TEST_CASE("Line ends of the exported G-code mark every newline in the file", "[GCodeProcessor]")
{
    struct Case
    {
        const char* name;
        bool        preheat_backtrace;
        bool        pre_heating;
    };
    const auto test_case = GENERATE(values<Case>({
        { "written by size", false, false },
        { "written by time for the preheat backtrace", true, false },
        { "rewritten by the pre-heating pass", false, true },
    }));
    INFO(test_case.name);
    DynamicPrintConfig config = Test::multifilament_config(2, {
        { "single_extruder_multi_material", 0 },
        { "ooze_prevention",                test_case.preheat_backtrace },
        { "preheat_time",                   30 },
        { "enable_pre_heating",             test_case.pre_heating },
    });
    Print print;
    Model model;
    const std::vector<std::vector<ConfigBase::SetDeserializeItem>> overrides{ { { "extruder", 1 } }, { { "extruder", 2 } } };
    Test::init_print({ Test::cube(20), Test::cube(20) }, print, model, config, &overrides);
    GCodeProcessorResult result;
    const std::string    gcode = Test::gcode(print, &result);
    REQUIRE((gcode.find("preheat T") != std::string::npos) == test_case.preheat_backtrace);
    REQUIRE((gcode.find(GCodeProcessor::Machine_Start_GCode_End_Tag) != std::string::npos) == test_case.pre_heating);
    REQUIRE(gcode.size() > GCodeProcessor::Output_Block_Size);

    std::vector<size_t> newline_ends;
    for (size_t i = gcode.find('\n'); i != std::string::npos; i = gcode.find('\n', i + 1))
        newline_ends.push_back(i + 1);
    REQUIRE(result.lines_ends.size() == newline_ends.size());
    const auto difference = std::mismatch(result.lines_ends.begin(), result.lines_ends.end(), newline_ends.begin());
    INFO("first difference at line " << difference.first - result.lines_ends.begin() + 1);
    CHECK(difference.first == result.lines_ends.end());
}

TEST_CASE("Reloaded moves name their lines in G-code a script rewrote in place", "[GCodeProcessor]")
{
    Print print;
    Model model;
    Test::init_print({ Test::cube(20) }, print, model);
    GCodeProcessorResult result;
    const std::string    gcode          = Test::gcode(print, &result);
    const auto           exported_moves = result.moves;

    // A script that prepends one comment and, writing in text mode on Windows, turns every LF into CRLF.
    const std::string prepended = ";EDITED\r\n";
    std::string       edited    = prepended;
    for (const char c : gcode) {
        if (c == '\n')
            edited += '\r';
        edited += c;
    }
    ScopedTemporaryFile temp(".gcode");
    save_string_file(temp.path(), edited);
    result.filename = temp.string();
    print.reload_gcode_moves(&result);

    std::vector<size_t> newline_ends;
    for (size_t i = edited.find('\n'); i != std::string::npos; i = edited.find('\n', i + 1))
        newline_ends.push_back(i + 1);
    CHECK(result.lines_ends == newline_ends);

    // Every move that came from a line now names the same line one further down.
    REQUIRE(result.moves.size() == exported_moves.size());
    const auto difference = std::mismatch(exported_moves.begin(), exported_moves.end(), result.moves.begin(),
                                          [](const auto &exported, const auto &reloaded) {
                                              return reloaded.gcode_id == (exported.gcode_id == 0 ? 0 : exported.gcode_id + 1);
                                          });
    INFO("first difference at move " << difference.first - exported_moves.begin());
    CHECK(difference.first == exported_moves.end());
}

TEST_CASE("Rewritten G-code that cannot be re-read keeps the moves and hides the G-code window", "[GCodeProcessor]")
{
    Print print;
    Model model;
    Test::init_print({ Test::cube(20) }, print, model);
    GCodeProcessorResult result;
    const std::string    gcode          = Test::gcode(print, &result);
    const auto           exported_moves = result.moves;

    // A script that strips the trailing config block, which the G-code reader needs.
    const size_t config_block = gcode.find("; CONFIG_BLOCK_START");
    REQUIRE(config_block != std::string::npos);
    ScopedTemporaryFile temp(".gcode");
    save_string_file(temp.path(), gcode.substr(0, config_block));
    result.filename = temp.string();
    print.reload_gcode_moves(&result);

    CHECK(result.lines_ends.empty());
    REQUIRE(result.moves.size() == exported_moves.size());
    CHECK(result.moves.back().gcode_id == exported_moves.back().gcode_id);
}

TEST_CASE("The plate's center of mass takes every extrusion of G-code without a print behind it", "[GCodeProcessor]")
{
    GCodeProcessorResult result;
    process_two_objects(result);

    CHECK(result.object_masses.empty());
    CHECK(result.body_masses.empty());
    const GCodeProcessorResult::ObjectMass &plate = result.plate_mass;
    REQUIRE(plate.printed_up_to_layer.size() == 2);
    CHECK_THAT((center_of(plate.printed_up_to_layer.front()) -
                weighted_center({ { 1., a_brim }, { 1., a_support }, { 1., a_wall_0 }, { 2., b_wall } })).norm(),
               Catch::Matchers::WithinAbs(0., 1e-5));
    CHECK_THAT((center_of(plate.printed_up_to_layer.back()) -
                weighted_center({ { 1., a_brim }, { 1., a_support }, { 1., a_wall_0 }, { 2., b_wall }, { 1., a_wall_1 } })).norm(),
               Catch::Matchers::WithinAbs(0., 1e-5));

    // Each bead weighs its volume at the default density and spreads along its move, (a^2 + ab + b^2) / 3 for one from
    // a to b: the brim from x 8 to 12 at y 8, the support at x 10 from y 20 to 30, A's walls from x 10 to 20 at y 10 and
    // B's from x 50 to 60 at y 50 with twice the filament, all at z 0.1 but A's second wall at 0.3.
    const GCodeProcessorResult::ObjectMass::Sum total = plate.total();
    CHECK_THAT(total.mass / total.volume, Catch::Matchers::WithinRel(double(DEFAULT_FILAMENT_DENSITY), 1e-6));
    const Vec3d second = total.second / total.mass;
    CHECK_THAT(second.x(), Catch::Matchers::WithinRel((304. / 3. + 100. + 2. * 700. / 3. + 2. * 9100. / 3.) / 6., 1e-6));
    CHECK_THAT(second.y(), Catch::Matchers::WithinRel((64. + 1900. / 3. + 2. * 100. + 2. * 2500.) / 6., 1e-6));
    CHECK_THAT(second.z(), Catch::Matchers::WithinRel((5. * 0.01 + 0.09) / 6., 1e-5));
    // The beads' center lines, brim and support included, from the first layer's bottom to the second's top.
    CHECK_THAT((plate.box.min - Vec3d(8., 8., 0.)).norm(), Catch::Matchers::WithinAbs(0., 1e-5));
    CHECK_THAT((plate.box.max - Vec3d(60., 50., 0.4)).norm(), Catch::Matchers::WithinAbs(0., 1e-5));
}

TEST_CASE("Each sliced cube's center of mass is its center, and the brim lowers the plate's printed one", "[GCodeProcessor]")
{
    const bool copies = GENERATE(false, true);
    INFO((copies ? "two copies of one cube" : "two cubes"));
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({ { "skirt_loops", 0 }, { "brim_type", "outer_only" }, { "brim_width", 5 }, { "combine_brims", 0 } });
    std::vector<TriangleMesh> cubes{ Test::cube(20) };
    if (!copies)
        cubes.emplace_back(Test::cube(20));
    Print print;
    Model model;
    Test::init_print(std::move(cubes), print, model, config, nullptr, true, copies ? 2 : 1);
    GCodeProcessorResult result;
    Test::gcode(print, &result);

    CHECK(result.body_masses.empty());
    REQUIRE(result.object_masses.size() == 2);
    for (const ModelObject *object : model.objects)
        for (size_t instance = 0; instance < object->instances.size(); ++instance) {
            const Vec3d center = object->instance_bounding_box(instance).center();
            const auto  mass   = std::min_element(result.object_masses.begin(), result.object_masses.end(), [&center](const auto &l, const auto &r) {
                return (center_of(l.total()) - center).squaredNorm() < (center_of(r.total()) - center).squaredNorm();
            });
            // Off the center only by the infill's alignment and the top and bottom shells.
            const Vec3d part = center_of(mass->total());
            CHECK_THAT(part.x(), Catch::Matchers::WithinAbs(center.x(), 0.5));
            CHECK_THAT(part.y(), Catch::Matchers::WithinAbs(center.y(), 0.5));
            CHECK_THAT(part.z(), Catch::Matchers::WithinAbs(center.z(), 1.));
            // The outer walls' center lines run half a line inside the cube's sides, of copies touching each other too.
            const BoundingBoxf3 box = object->instance_bounding_box(instance);
            for (int axis = 0; axis < 3; ++axis) {
                CHECK_THAT(mass->box.min[axis], Catch::Matchers::WithinAbs(box.min[axis], 0.3));
                CHECK_THAT(mass->box.max[axis], Catch::Matchers::WithinAbs(box.max[axis], 0.3));
            }
        }
    GCodeProcessorResult::ObjectMass::Sum objects;
    for (const GCodeProcessorResult::ObjectMass &object : result.object_masses)
        objects.add(object.total());
    const GCodeProcessorResult::ObjectMass::Sum plate = result.plate_mass.total();
    CHECK(plate.mass > objects.mass);
    CHECK(center_of(plate).z() < center_of(objects).z());
}

TEST_CASE("Each cube's raft is its support, centered below it", "[GCodeProcessor]")
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({ { "skirt_loops", 0 }, { "brim_type", "no_brim" }, { "raft_layers", 3 } });
    Print print;
    Model model;
    Test::init_print({ Test::cube(20), Test::cube(20) }, print, model, config);
    GCodeProcessorResult result;
    Test::gcode(print, &result);

    REQUIRE(result.support_masses.size() == 2);
    for (size_t i = 0; i < 2; ++i) {
        const GCodeProcessorResult::ObjectMass::Sum support = result.support_masses[i].total();
        const Vec3d                                 object  = center_of(result.object_masses[i].total());
        REQUIRE(support.mass > 0.);
        CHECK_THAT(center_of(support).x(), Catch::Matchers::WithinAbs(object.x(), 1.));
        CHECK_THAT(center_of(support).y(), Catch::Matchers::WithinAbs(object.y(), 1.));
        CHECK(center_of(support).z() < 1.);
    }
}

TEST_CASE("A spiral vase cube counts all its extrusions, rising through each layer", "[GCodeProcessor]")
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({ { "skirt_loops", 0 }, { "brim_type", "no_brim" }, { "spiral_mode", 1 }, { "wall_loops", 1 },
                                    { "top_shell_layers", 0 }, { "sparse_infill_density", 0 } });
    Print print;
    Model model;
    Test::init_print({ Test::cube(20) }, print, model, config);
    GCodeProcessorResult result;
    Test::gcode(print, &result);

    REQUIRE(result.object_masses.size() == 1);
    CHECK_THAT(result.object_masses.front().total().mass, Catch::Matchers::WithinRel(result.plate_mass.total().mass, 1e-6));
}

TEST_CASE("Each separate part of an assembly gets its center of mass, overlapping parts one", "[GCodeProcessor]")
{
    const bool overlapping = GENERATE(false, true);
    // Separated infills finds the bodies first, which the G-code export then takes.
    const bool separated = GENERATE(false, true);
    INFO((overlapping ? "overlapping parts" : "separate parts") << (separated ? ", separated infills" : ""));
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({ { "skirt_loops", 0 }, { "brim_type", "no_brim" }, { "separated_infills", separated ? 1 : 0 } });
    TriangleMesh first = make_cube(20, 20, 20);
    TriangleMesh second = make_cube(20, 20, 20);
    first.translate(50, 50, 0);
    second.translate(overlapping ? 60 : 90, 50, 0);
    Print print;
    Model model;
    Test::init_print({ first }, print, model, config, nullptr, false);
    model.objects.front()->add_volume(std::move(second), ModelVolumeType::MODEL_PART, false);
    print.apply(model, config);
    GCodeProcessorResult result;
    Test::gcode(print, &result);

    REQUIRE(result.object_masses.size() == 1);
    CHECK(result.object_masses.front().assembly);
    // One body is the object itself.
    if (overlapping) {
        CHECK(result.body_masses.empty());
        return;
    }
    REQUIRE(result.body_masses.size() == 2);
    CHECK(print.objects().front()->separated_body_bboxes().size() == (separated ? 2 : 0));
    const ModelObject &object = *model.objects.front();
    for (const ModelVolume *volume : object.volumes) {
        const Vec3d center = volume->mesh().transformed_bounding_box(object.instances.front()->get_matrix() * volume->get_matrix()).center();
        const auto  body   = std::min_element(result.body_masses.begin(), result.body_masses.end(), [&center](const auto &l, const auto &r) {
            return (center_of(l.total()) - center).squaredNorm() < (center_of(r.total()) - center).squaredNorm();
        });
        const Vec3d part = center_of(body->total());
        CHECK_THAT(part.x(), Catch::Matchers::WithinAbs(center.x(), 0.5));
        CHECK_THAT(part.y(), Catch::Matchers::WithinAbs(center.y(), 0.5));
        CHECK_THAT(part.z(), Catch::Matchers::WithinAbs(center.z(), 1.));
    }
}

TEST_CASE("Each extrusion weighs its filament's density", "[GCodeProcessor]")
{
    // Two like cubes, the second's filament three times as dense.
    DynamicPrintConfig config = Test::multifilament_config(2, { { "filament_density", "1,3" }, { "skirt_loops", 0 }, { "brim_type", "no_brim" } });
    const std::vector<std::vector<ConfigBase::SetDeserializeItem>> overrides{ { { "extruder", 1 } }, { { "extruder", 2 } } };
    Print print;
    Model model;
    Test::init_print({ Test::cube(20), Test::cube(20) }, print, model, config, &overrides);
    GCodeProcessorResult result;
    Test::gcode(print, &result);

    REQUIRE(result.object_masses.size() == 2);
    std::vector<const GCodeProcessorResult::ObjectMass *> masses;
    for (const ModelObject *object : model.objects) {
        const Vec3d center = object->instance_bounding_box(0).center();
        masses.emplace_back(&*std::min_element(result.object_masses.begin(), result.object_masses.end(), [&center](const auto &l, const auto &r) {
            return (center_of(l.total()) - center).squaredNorm() < (center_of(r.total()) - center).squaredNorm();
        }));
    }
    CHECK_THAT(masses[1]->total().mass / masses[0]->total().mass, Catch::Matchers::WithinRel(3., 0.02));
    // The plate's center lies three quarters of the way to the dense cube.
    const Vec3d plate = center_of(result.plate_mass.total());
    const Vec3d light = center_of(masses[0]->total());
    const Vec3d dense = center_of(masses[1]->total());
    CHECK_THAT((plate - light).dot(dense - light) / (dense - light).squaredNorm(), Catch::Matchers::WithinAbs(0.75, 0.01));
}
