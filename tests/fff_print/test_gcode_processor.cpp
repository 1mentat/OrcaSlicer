#include "libslic3r/MultiNozzleUtils.hpp"
#include "libslic3r/Config.hpp"
#include "libslic3r/Utils.hpp"
#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>
#include <boost/nowide/fstream.hpp>
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


namespace public_processor_contracts {

using namespace Slic3r;
using Catch::Matchers::WithinAbs;

namespace {

struct TestResult : GCodeProcessorResult {
    TestResult() = default;
    TestResult(TestResult &&other) noexcept { GCodeProcessorResult::operator=(std::move(other)); }
    TestResult &operator=(GCodeProcessorResult &&other) noexcept
    {
        GCodeProcessorResult::operator=(std::move(other));
        return *this;
    }
    TestResult &operator=(TestResult &&other) noexcept
    {
        GCodeProcessorResult::operator=(std::move(other));
        return *this;
    }
};

std::vector<const GCodeProcessorResult::MoveVertex *> moves_of(const GCodeProcessorResult &result, EMoveType type)
{
    std::vector<const GCodeProcessorResult::MoveVertex *> ret;
    for (const auto &move : result.moves)
        if (!move.internal_only && move.type == type)
            ret.push_back(&move);
    return ret;
}

const GCodeProcessorResult::MoveVertex *last_of(const GCodeProcessorResult &result, EMoveType type)
{
    for (auto it = result.moves.rbegin(); it != result.moves.rend(); ++it)
        if (!it->internal_only && it->type == type)
            return &*it;
    return nullptr;
}

TestResult process_buffer(const std::string &gcode, FullPrintConfig config = {})
{
    GCodeProcessor processor;
    processor.apply_config(config);
    processor.initialize("gcode_processor_public_contract.gcode");
    processor.initialize_result_moves();
    processor.process_buffer(gcode);
    processor.finalize(false);
    TestResult result;
    result = std::move(processor.extract_result());
    return result;
}

std::string orca_header(const FullPrintConfig &config)
{
    std::string out = "; generated by OrcaSlicer\n; CONFIG_BLOCK_START\n";
    for (const std::string &key : config.keys()) {
        const ConfigOption *option = config.option(key);
        if (option != nullptr)
            out += "; " + key + " = " + option->serialize() + "\n";
    }
    out += "; CONFIG_BLOCK_END\n";
    return out;
}

TestResult process_file(const std::string &gcode)
{
    ScopedTemporaryFile file(".gcode");
    {
        boost::nowide::ofstream output(file.string());
        REQUIRE(output.good());
        output << gcode;
        output.flush();
        REQUIRE(output.good());
    }
    GCodeProcessor processor;
    processor.process_file(file.string());
    TestResult result;
    result = std::move(processor.extract_result());
    return result;
}

float normal_time(const GCodeProcessorResult &result)
{
    return result.print_statistics.modes[static_cast<size_t>(PrintEstimatedStatistics::ETimeMode::Normal)].time;
}

} // namespace

TEST_CASE("Orca file import preserves parsed filament dimensions and density", "[GCodeProcessor][GCodeProcessorPublic]")
{
    FullPrintConfig config;
    config.filament_colour.values = {"#112233", "#ABCDEF"};
    config.filament_is_support.values = {false, true};
    config.filament_type.values = {"PLA", "PETG"};
    config.filament_map.values = {1, 2};
    config.filament_diameter.values = {1.75, 1.80};
    config.filament_density.values = {1.24, 1.27};

    const auto result = process_file(orca_header(config) + "M83\nG90\nG1 X10 Y0 Z0.2 E1 F600\n");
    // The public file-import path owns the parsed dimensions and density in its result.
    REQUIRE(result.filaments_count == 2);
    REQUIRE(result.filament_diameters.size() == 2);
    REQUIRE(result.filament_densities.size() == 2);
    CHECK_THAT(result.filament_diameters[0], WithinAbs(1.75, 1e-5));
    CHECK_THAT(result.filament_diameters[1], WithinAbs(1.80, 1e-5));
    CHECK_THAT(result.filament_densities[0], WithinAbs(1.24, 1e-5));
    CHECK_THAT(result.filament_densities[1], WithinAbs(1.27, 1e-5));
}

TEST_CASE("Orca file import preserves filament count and trailing density fallback", "[GCodeProcessor][GCodeProcessorPublic]")
{
    FullPrintConfig config;
    config.filament_colour.values = {"#010203", "#040506", "#070809"};
    config.filament_is_support.values = {false, false, true};
    config.filament_type.values = {"PLA", "ABS", "TPU"};
    config.filament_map.values = {2};
    config.filament_diameter.values = {1.70, 1.75, 2.85};
    const auto result = process_file(orca_header(config) + "G90\nG1 X2 Y3 Z0.2 E1 F600\n");
    REQUIRE(result.filaments_count == 3);
    REQUIRE(result.filament_diameters.size() == 3);
    REQUIRE(result.filament_densities.size() == 3);
    CHECK_THAT(result.filament_diameters[0], WithinAbs(1.70, 1e-5));
    CHECK_THAT(result.filament_diameters[1], WithinAbs(1.75, 1e-5));
    CHECK_THAT(result.filament_diameters[2], WithinAbs(2.85, 1e-5));
    CHECK_THAT(result.filament_densities[2], WithinAbs(DEFAULT_FILAMENT_DENSITY, 1e-5));
}

TEST_CASE("Applied config exports independent render settings copies", "[GCodeProcessor][GCodeProcessorPublic]")
{
    FullPrintConfig config;
    config.filament_colour.values = {"#123456", "#654321"};
    config.filament_is_support.values = {false, true};
    config.filament_type.values = {"PLA", "PETG"};
    config.filament_map.values = {1, 2};
    config.filament_diameter.values = {1.75, 1.80};

    GCodeProcessor processor;
    processor.apply_config(config);
    DynamicConfig exported = processor.export_config_for_render();
    const auto *colours = exported.opt<ConfigOptionStrings>("filament_colour");
    const auto *support = exported.opt<ConfigOptionBools>("filament_is_support");
    const auto *types = exported.opt<ConfigOptionStrings>("filament_type");
    const auto *mapping = exported.opt<ConfigOptionInts>("filament_map");
    REQUIRE(colours != nullptr);
    REQUIRE(support != nullptr);
    REQUIRE(types != nullptr);
    REQUIRE(mapping != nullptr);
    CHECK(colours->values == config.filament_colour.values);
    CHECK(support->values == config.filament_is_support.values);
    CHECK(types->values == config.filament_type.values);
    CHECK(mapping->values == config.filament_map.values);

    exported.set_key_value("filament_colour", new ConfigOptionStrings(std::vector<std::string>{"#FFFFFF", "#654321"}));
    const DynamicConfig fresh = processor.export_config_for_render();
    REQUIRE(fresh.opt<ConfigOptionStrings>("filament_colour") != nullptr);
    CHECK(fresh.opt<ConfigOptionStrings>("filament_colour")->values[0] == "#123456");
}

TEST_CASE("Import-only initialization exposes default and configured nozzle maps", "[GCodeProcessor][GCodeProcessorPublic]")
{
    GCodeProcessor processor;
    processor.init_filament_maps_and_nozzle_type_when_import_only_gcode();
    processor.ensure_nozzle_group_result(2);
    const auto *default_group = dynamic_cast<const MultiNozzleUtils::LayeredNozzleGroupResult *>(
        processor.get_result().nozzle_group_result.get());
    REQUIRE(default_group != nullptr);
    REQUIRE(default_group->get_extruder_map().size() >= 2);
    CHECK(default_group->get_extruder_map()[0] == 0);
    CHECK(default_group->get_extruder_map()[1] == 0);
    CHECK(default_group->get_extruder_count() == 1);

    GCodeProcessor second;
    FullPrintConfig config;
    config.filament_map.values = {2, 1};
    second.apply_config(config);
    second.init_filament_maps_and_nozzle_type_when_import_only_gcode();
    second.ensure_nozzle_group_result(2);
    const auto *configured_group = dynamic_cast<const MultiNozzleUtils::LayeredNozzleGroupResult *>(
        second.get_result().nozzle_group_result.get());
    REQUIRE(configured_group != nullptr);
    REQUIRE(configured_group->get_extruder_map().size() >= 2);
    CHECK(configured_group->get_extruder_map()[0] == 1);
    CHECK(configured_group->get_extruder_map()[1] == 0);
    CHECK(configured_group->get_extruder_count() == 2);
    CHECK(configured_group->get_used_filaments().size() >= 2);
}

TEST_CASE("G4 adds its requested wait to normal print time", "[GCodeProcessor][GCodeProcessorPublic]")
{
    const auto no_wait = process_buffer("G4 S0\n");
    const auto waited = process_buffer("G4 S2\n");
    CHECK_THAT(normal_time(waited) - normal_time(no_wait), WithinAbs(2.0f, 0.1f));
}

TEST_CASE("G0 travel is retained as a travel vertex and does not extrude", "[GCodeProcessor][GCodeProcessorPublic]")
{
    const auto result = process_buffer("G90\nG0 X12 Y8 Z0.2 F1200\n");
    const auto travels = moves_of(result, EMoveType::Travel);
    REQUIRE(!travels.empty());
    CHECK_THAT(travels.back()->position.x(), WithinAbs(12., 1e-4));
    CHECK_THAT(travels.back()->position.y(), WithinAbs(8., 1e-4));
    CHECK(moves_of(result, EMoveType::Extrude).empty());
}

TEST_CASE("G28 without axes homes all three cartesian axes", "[GCodeProcessor][GCodeProcessorPublic]")
{
    const auto result = process_buffer("G90\nG1 X20 Y30 Z4 F600\nG28\n");
    const auto travels = moves_of(result, EMoveType::Travel);
    REQUIRE(!travels.empty());
    CHECK_THAT(travels.back()->position.x(), WithinAbs(0., 1e-4));
    CHECK_THAT(travels.back()->position.y(), WithinAbs(0., 1e-4));
    CHECK_THAT(travels.back()->position.z(), WithinAbs(0., 1e-4));
}

TEST_CASE("G10 and G11 use configured firmware retract lengths", "[GCodeProcessor][GCodeProcessorPublic]")
{
    FullPrintConfig config;
    config.retraction_length.values = {2.25};
    const auto result = process_buffer("M83\nG1 X10 Y0 Z0.2 E1 F600\nG10\nG11\n", config);
    const auto retracts = moves_of(result, EMoveType::Retract);
    const auto unretracts = moves_of(result, EMoveType::Unretract);
    REQUIRE(retracts.size() == 1);
    REQUIRE(unretracts.size() == 1);
    CHECK_THAT(retracts.front()->delta_extruder, WithinAbs(-2.25, 1e-4));
    CHECK_THAT(unretracts.front()->delta_extruder, WithinAbs(2.25, 1e-4));
}

TEST_CASE("M221 applies flow override only to planned extrusion volume", "[GCodeProcessor][GCodeProcessorPublic]")
{
    FullPrintConfig constrained;
    // Keep the X/E path identical in both runs; E=1.2 is the only active cap so the
    // S100 versus S50 override remains externally visible in the timed move.
    constrained.machine_max_speed_e.values = {1.2, 1.2, 1.2, 1.2};
    const auto control = process_buffer("M83\nG1 X0 Y0 Z0.2 F600\nG1 X10 Y0 E2 F600\n", constrained);
    const auto normal = process_buffer("M83\nM221 S100\nG1 X0 Y0 Z0.2 F600\nG1 X10 Y0 E2 F600\n", constrained);
    const auto half = process_buffer("M83\nM221 S50\nG1 X0 Y0 Z0.2 F600\nG1 X10 Y0 E2 F600\n", constrained);
    const auto *a = last_of(normal, EMoveType::Extrude);
    const auto *b = last_of(half, EMoveType::Extrude);
    REQUIRE(a != nullptr);
    REQUIRE(b != nullptr);
    CHECK_THAT(a->delta_extruder, WithinAbs(2., 1e-4));
    CHECK_THAT(b->delta_extruder, WithinAbs(2., 1e-4));
    CHECK_THAT(normal_time(normal), WithinAbs(normal_time(control), 1e-4f));
    REQUIRE(a->time[0] > 0.0f);
    CHECK(b->time[0] < a->time[0] * 0.9f);
}

TEST_CASE("M109 R updates a selected extruder and S updates the active one", "[GCodeProcessor][GCodeProcessorPublic]")
{
    FullPrintConfig config;
    config.filament_diameter.values = {1.75, 1.75};
    config.filament_map.values = {1, 2};
    const auto result = process_buffer(
        "M83\nT0\nM109 R190 T1\nT1\nG1 X10 Y0 Z0.2 E1 F600\n"
        "T0\nM109 S210\nG1 X20 Y0 E1 F600\n", config);
    const auto extrusions = moves_of(result, EMoveType::Extrude);
    REQUIRE(extrusions.size() == 2);
    CHECK_THAT(extrusions[0]->temperature, WithinAbs(190., 1e-4));
    CHECK_THAT(extrusions[1]->temperature, WithinAbs(210., 1e-4));
}

TEST_CASE("M106 P0 is ignored while the print fan and P1 are accepted", "[GCodeProcessor][GCodeProcessorPublic]")
{
    const auto result = process_buffer("M83\nM106 S100\nM106 P0 S255\nG1 X10 Y0 Z0.2 E1 F600\nM106 P1 S64\nG1 X20 Y0 E1 F600\n");
    const auto extrusions = moves_of(result, EMoveType::Extrude);
    REQUIRE(extrusions.size() == 2);
    CHECK_THAT(extrusions[0]->fan_speed, WithinAbs(100. * 100. / 255., 1e-4));
    CHECK_THAT(extrusions[1]->fan_speed, WithinAbs(100. * 64. / 255., 1e-4));
}

TEST_CASE("M203 limits the normal feedrate but is ignored for Repetier", "[GCodeProcessor][GCodeProcessorPublic]")
{
    const auto limited = process_buffer("M83\nM203 X5\nG1 X0 Y0 Z0.2 F1200\nG1 X20 Y0 E1 F1200\n");
    const auto ignored = process_buffer("M83\nM203 X5\nG1 X0 Y0 Z0.2 F1200\nG1 X20 Y0 E1 F1200\n", [] {
        FullPrintConfig c;
        c.gcode_flavor.value = gcfRepetier;
        return c;
    }());
    const auto *a = last_of(limited, EMoveType::Extrude);
    const auto *b = last_of(ignored, EMoveType::Extrude);
    REQUIRE(a != nullptr);
    REQUIRE(b != nullptr);
    CHECK(normal_time(limited) > normal_time(ignored));
    CHECK_THAT(a->feedrate, WithinAbs(b->feedrate, 1e-4));
}

TEST_CASE("M201 limits acceleration and M205 exposes the configured jerk", "[GCodeProcessor][GCodeProcessorPublic]")
{
    const auto baseline = process_buffer("M83\nG1 X20 Y0 Z0.2 E1 F1200\nG1 X40 Y0 E1 F1200\n");
    const auto configured = process_buffer(
        "M83\nM201 X321\nM205 X7 J0.04\nG1 X20 Y0 Z0.2 E1 F1200\nG1 X40 Y0 E1 F1200\n");
    const auto *base_move = last_of(baseline, EMoveType::Extrude);
    const auto *configured_move = last_of(configured, EMoveType::Extrude);
    REQUIRE(base_move != nullptr);
    REQUIRE(configured_move != nullptr);
    CHECK(normal_time(configured) > normal_time(baseline));
    CHECK_THAT(configured_move->jerk, WithinAbs(7., 1e-4));
}

TEST_CASE("M204 P/R/T uses the three modern acceleration channels", "[GCodeProcessor][GCodeProcessorPublic]")
{
    const auto result = process_buffer(
        "M204 P400 R250 T180\nG1 X20 Y0 Z0.2 E1 F1200\nG10\nG0 X30 Y0 F1200\n");
    const auto *extrusion = last_of(result, EMoveType::Extrude);
    const auto *retract = last_of(result, EMoveType::Retract);
    const auto *travel = last_of(result, EMoveType::Travel);
    REQUIRE(extrusion != nullptr);
    REQUIRE(retract != nullptr);
    REQUIRE(travel != nullptr);
    CHECK_THAT(extrusion->acceleration, WithinAbs(400., 1e-2));
    CHECK_THAT(retract->acceleration, WithinAbs(250., 1e-2));
    CHECK_THAT(travel->acceleration, WithinAbs(180., 1e-2));
}

TEST_CASE("M204 legacy S sets the extrusion acceleration channel", "[GCodeProcessor][GCodeProcessorPublic]")
{
    const auto result = process_buffer("M204 S321\nG1 X20 Y0 Z0.2 E1 F1200\n");
    const auto *extrusion = last_of(result, EMoveType::Extrude);
    REQUIRE(extrusion != nullptr);
    CHECK_THAT(extrusion->acceleration, WithinAbs(321., 1e-2));
}

TEST_CASE("M400 and SYNC add their expressed waits to a move baseline", "[GCodeProcessor][GCodeProcessorPublic]")
{
    const auto no_wait = process_buffer("M400 S0\nSYNC T0\n");
    const auto waited = process_buffer("M400 S1\nSYNC T2\n");
    CHECK_THAT(normal_time(waited) - normal_time(no_wait), WithinAbs(3.0f, 0.1f));
}

TEST_CASE("VM104 and VM109 stamp temperatures exactly like their M-code forms", "[GCodeProcessor][GCodeProcessorPublic]")
{
    const auto result = process_buffer("M83\nVM104 S195\nG1 X10 Y0 Z0.2 E1 F600\nVM109 S205\nG1 X20 Y0 E1 F600\n");
    const auto extrusions = moves_of(result, EMoveType::Extrude);
    REQUIRE(extrusions.size() == 2);
    CHECK_THAT(extrusions[0]->temperature, WithinAbs(195., 1e-4));
    CHECK_THAT(extrusions[1]->temperature, WithinAbs(205., 1e-4));
}

TEST_CASE("M900 clamps negative pressure advance and preserves positive values", "[GCodeProcessor][GCodeProcessorPublic]")
{
    const auto result = process_buffer("M83\nM900 K-1\nG1 X10 Y0 Z0.2 E1 F600\nM900 K0.27\nG1 X20 Y0 E1 F600\n");
    const auto extrusions = moves_of(result, EMoveType::Extrude);
    REQUIRE(extrusions.size() == 2);
    CHECK_THAT(extrusions[0]->pressure_advance, WithinAbs(0., 1e-4));
    CHECK_THAT(extrusions[1]->pressure_advance, WithinAbs(0.27, 1e-4));
}

TEST_CASE("Dynamic config selects spiral mode and propagates settings identifiers", "[GCodeProcessor][GCodeProcessorPublic]")
{
    FullPrintConfig config;
    config.spiral_mode.value = true;
    std::string header = orca_header(config);
    const size_t end = header.find("; CONFIG_BLOCK_END");
    REQUIRE(end != std::string::npos);
    header.insert(end, "; print_settings_id = print-42\n; printer_settings_id = printer-42\n; filament_settings_id = filament-42\n");
    const auto result = process_file(header + "G1 X10 Y0 Z0.2 E1 F600\n");
    CHECK(result.spiral_vase_mode);
    CHECK(result.settings_ids.print == "print-42");
    CHECK(result.settings_ids.printer == "printer-42");
    REQUIRE(result.settings_ids.filament.size() == 1);
    CHECK(result.settings_ids.filament[0] == "filament-42");
}

TEST_CASE("Stealth estimator flag can be toggled through the public API", "[GCodeProcessor][GCodeProcessorPublic]")
{
    GCodeProcessor processor;
    processor.apply_config(FullPrintConfig{});
    CHECK_FALSE(processor.is_stealth_time_estimator_enabled());
    processor.enable_stealth_time_estimator(true);
    CHECK(processor.is_stealth_time_estimator_enabled());
    processor.enable_stealth_time_estimator(false);
    CHECK_FALSE(processor.is_stealth_time_estimator_enabled());
}

TEST_CASE("Invalid time mode reports N/A and valid mode reports a formatted duration", "[GCodeProcessor][GCodeProcessorPublic]")
{
    GCodeProcessor processor;
    processor.apply_config(FullPrintConfig{});
    processor.initialize("time.gcode");
    processor.initialize_result_moves();
    processor.process_buffer("G4 S2\n");
    processor.finalize(false);
    CHECK(processor.get_time_dhm(static_cast<PrintEstimatedStatistics::ETimeMode>(255)) == "N/A");
    CHECK(processor.get_time_dhm(PrintEstimatedStatistics::ETimeMode::Normal) == "2s");
}

TEST_CASE("Filament dimensions retain filament order when nozzle mapping is reversed", "[GCodeProcessor][GCodeProcessorPublic]")
{
    FullPrintConfig config;
    config.filament_diameter.values = {1.75, 2.85};
    config.filament_density.values = {1.24, 1.10};
    config.filament_map.values = {2, 1};
    const auto result = process_file(orca_header(config) + "G1 X10 Y0 Z0.2 E1 F600\n");
    REQUIRE(result.filaments_count == 2);
    REQUIRE(result.filament_diameters.size() == 2);
    REQUIRE(result.filament_densities.size() == 2);
    CHECK_THAT(result.filament_diameters[0], WithinAbs(1.75, 1e-5));
    CHECK_THAT(result.filament_diameters[1], WithinAbs(2.85, 1e-5));
    CHECK_THAT(result.filament_densities[0], WithinAbs(1.24, 1e-5));
    CHECK_THAT(result.filament_densities[1], WithinAbs(1.10, 1e-5));
}

TEST_CASE("Malformed optional commands preserve a later valid move", "[GCodeProcessor][GCodeProcessorPublic]")
{
    const auto result = process_buffer("M104 Sbad\nM221 Sbad\nG1 X10 Y0 Z0.2 E1 F600\n");
    const auto *move = last_of(result, EMoveType::Extrude);
    REQUIRE(move != nullptr);
    CHECK_THAT(move->position.x(), WithinAbs(10., 1e-4));
    CHECK_THAT(move->position.y(), WithinAbs(0., 1e-4));
    CHECK_THAT(move->position.z(), WithinAbs(0.2, 1e-4));
    CHECK_THAT(move->delta_extruder, WithinAbs(1., 1e-4));
    CHECK_THAT(move->fan_speed, WithinAbs(0., 1e-4));
    CHECK_THAT(move->temperature, WithinAbs(0., 1e-4));
    CHECK(std::isfinite(normal_time(result)));
}

} // namespace public_processor_contracts
