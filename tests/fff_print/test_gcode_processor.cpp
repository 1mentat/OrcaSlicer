#include <array>
#include <cctype>
#include <cstdint>
#include <filesystem>
#include <system_error>
#include <catch2/generators/catch_generators.hpp>

#include "libslic3r/MultiNozzleUtils.hpp"
#include "libslic3r/Config.hpp"
#include "libslic3r/CommonDefs.hpp"
#include "libslic3r/Utils.hpp"
#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>
#include <set>
#include <boost/nowide/fstream.hpp>
#include <catch2/catch_all.hpp>

#include "libslic3r/libslic3r.h"
#include "libslic3r/GCode/GCodeProcessor.hpp"
#include "libslic3r/GCodeReader.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/PrintConfig.hpp"

#include "test_utils.hpp"

#include <cstddef>
#include <fstream>
#include <string>
#include <string_view>
#include <sstream>
#include "libslic3r/Print.hpp"
#include "libslic3r/CustomGCode.hpp"
#include <catch2/catch_message.hpp>
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

float stealth_time(const GCodeProcessorResult &result)
{
    return result.print_statistics.modes[static_cast<size_t>(PrintEstimatedStatistics::ETimeMode::Stealth)].time;
}

FullPrintConfig stealth_config()
{
    FullPrintConfig config;
    config.gcode_flavor.value = gcfMarlinFirmware;
    // Pin the source defaults consumed by the live M203/M204 paths.
    config.machine_max_speed_x.values = {500., 200.};
    config.machine_max_speed_y.values = {500., 200.};
    config.machine_max_speed_z.values = {12., 12.};
    config.machine_max_speed_e.values = {120., 120.};
    config.machine_max_acceleration_x.values = {1000., 1000.};
    config.machine_max_acceleration_y.values = {1000., 1000.};
    config.machine_max_acceleration_z.values = {500., 200.};
    config.machine_max_acceleration_e.values = {5000., 5000.};
    config.machine_max_acceleration_extruding.values = {1500., 1250.};
    config.machine_max_acceleration_retracting.values = {1500., 1250.};
    config.machine_max_acceleration_travel.values = {0., 0.};
    return config;
}

TestResult process_buffer_with_stealth(const std::string &gcode)
{
    GCodeProcessor processor;
    processor.apply_config(stealth_config());
    processor.enable_stealth_time_estimator(true);
    processor.initialize("gcode_processor_stealth_contract.gcode");
    processor.initialize_result_moves();
    processor.process_buffer(gcode);
    processor.finalize(false);
    TestResult result;
    result = std::move(processor.extract_result());
    return result;
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

TEST_CASE("G28 homes only the selected Z axis", "[GCodeProcessor][GCodeProcessorPublic][GCodeProcessorModal]")
{
    const auto result = process_buffer("G90\nG1 X20 Y30 Z4 F600\nG28 Z\n");
    const auto travels = moves_of(result, EMoveType::Travel);
    REQUIRE(!travels.empty());
    // G28 Z must synthesize a Z-only zero move while preserving the current XY position.
    CHECK_THAT(travels.back()->position.x(), WithinAbs(20., 1e-4));
    CHECK_THAT(travels.back()->position.y(), WithinAbs(30., 1e-4));
    CHECK_THAT(travels.back()->position.z(), WithinAbs(0., 1e-4));
}

TEST_CASE("M106 without S selects full print fan while auxiliary channels stay isolated",
          "[GCodeProcessor][GCodeProcessorPublic][GCodeProcessorFan]")
{
    const auto result = process_buffer(
        "M83\nM106 S32\nM106 P2 S255\nG1 X10 Y0 Z0.2 E1 F600\n"
        "M106\nG1 X20 Y0 E1 F600\n"
        "M106 P1 S64\nG1 X30 Y0 E1 F600\n");
    const auto extrusions = moves_of(result, EMoveType::Extrude);
    REQUIRE(extrusions.size() == 3);
    // P2 must not alter the print fan; a missing S means 100%, and P1 is the accepted
    // Bambu print-fan channel.
    CHECK_THAT(extrusions[0]->fan_speed, WithinAbs(100. * 32. / 255., 1e-4));
    CHECK_THAT(extrusions[1]->fan_speed, WithinAbs(100., 1e-4));
    CHECK_THAT(extrusions[2]->fan_speed, WithinAbs(100. * 64. / 255., 1e-4));
}

TEST_CASE("Processor reports bed and nozzle compatibility warnings from public motion",
          "[GCodeProcessor][GCodeProcessorPublic][GCodeProcessorWarnings]")
{
    FullPrintConfig conflict;
    conflict.filament_diameter.values = {1.75};
    conflict.filament_map.values = {1};
    conflict.temperature_vitrification.values = {180};
    conflict.required_nozzle_HRC.values = {5};
    conflict.nozzle_hrc.value = 2;
    const std::string gcode =
        "M83\nG90\nM140 S170\nM190 S180\nG1 X10 Y0 Z0.2 E1 F600\n";
    const auto conflict_result = process_buffer(gcode, conflict);

    const auto has_warning = [](const GCodeProcessorResult &result, const std::string &code) {
        return std::any_of(result.warnings.begin(), result.warnings.end(),
                           [&](const GCodeProcessorResult::SliceWarning &warning) { return warning.error_code == code; });
    };
    REQUIRE(has_warning(conflict_result, "1000C001"));
    REQUIRE(has_warning(conflict_result, "1000C002"));
    const auto bed_warning = std::find_if(conflict_result.warnings.begin(), conflict_result.warnings.end(),
                                          [](const GCodeProcessorResult::SliceWarning &warning) { return warning.error_code == "1000C001"; });
    const auto hrc_warning = std::find_if(conflict_result.warnings.begin(), conflict_result.warnings.end(),
                                          [](const GCodeProcessorResult::SliceWarning &warning) { return warning.error_code == "1000C002"; });
    REQUIRE(bed_warning != conflict_result.warnings.end());
    REQUIRE(hrc_warning != conflict_result.warnings.end());
    CHECK(bed_warning->params == std::vector<std::string>{"0"});
    CHECK(hrc_warning->params == std::vector<std::string>{"0"});

    FullPrintConfig control = conflict;
    control.temperature_vitrification.values = {180};
    control.required_nozzle_HRC.values = {2};
    const auto safe_result = process_buffer(
        "M83\nG90\nM190 S179\nG1 X10 Y0 Z0.2 E1 F600\n", control);
    CHECK_FALSE(has_warning(safe_result, "1000C001"));
    CHECK_FALSE(has_warning(safe_result, "1000C002"));
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

namespace {

TestResult process_continuous_rising_outer_wall(bool detect_layer_from_tags)
{
    // TYPE: is the compatible/Marlin role dialect. The process-global BBL flag
    // otherwise selects FEATURE: and leaves these role comments as erNone.
    const bool previous_bbl = GCodeProcessor::s_IsBBLPrinter;
    const ScopeGuard restore_bbl([previous_bbl] { GCodeProcessor::s_IsBBLPrinter = previous_bbl; });
    GCodeProcessor::s_IsBBLPrinter = false;

    // Keep the seam detector active through the rising external extrusion. The
    // following inner-wall move ends the continuous outer path, allowing the
    // two tracking modes to select different seam origins.
    const std::string gcode =
        "M83\nG90\n"
        "G1 X0 Y0 Z0.2 F600\n"
        ";TYPE:Outer wall\n"
        "G1 X10 Y0 E1 F600\n"
        "G1 X10 Y10 Z0.4 E1\n"
        "G1 X0 Y10 E1\n"
        "G1 X10.05 Y10 E1\n"
        ";TYPE:Inner wall\n"
        "G1 X10.10 Y10 E1\n";

    GCodeProcessor processor;
    FullPrintConfig config;
    config.gcode_flavor.value = gcfMarlinFirmware;
    processor.apply_config(config);
    processor.initialize("gcode_processor_layer_tag_contract.gcode");
    processor.initialize_result_moves();
    processor.detect_layer_based_on_tag(detect_layer_from_tags);
    processor.process_buffer(gcode);
    processor.finalize(false);

    TestResult result;
    result = std::move(processor.extract_result());
    return result;
}

} // namespace

TEST_CASE("Tag-based seam tracking follows a rising continuous outer path", "[GCodeProcessor][GCodeProcessorLayerTags]")
{
    const auto z_only = process_continuous_rising_outer_wall(false);
    const auto tag_based = process_continuous_rising_outer_wall(true);
    const auto z_only_seams = moves_of(z_only, EMoveType::Seam);
    const auto tag_based_seams = moves_of(tag_based, EMoveType::Seam);

    const auto z_only_extrusions = moves_of(z_only, EMoveType::Extrude);
    const auto tag_based_extrusions = moves_of(tag_based, EMoveType::Extrude);
    REQUIRE(z_only_extrusions.size() == 5);
    REQUIRE(tag_based_extrusions.size() == 5);
    for (size_t i = 0; i < z_only_extrusions.size(); ++i) {
        CAPTURE(i);
        CHECK_THAT(tag_based_extrusions[i]->position.x(), WithinAbs(z_only_extrusions[i]->position.x(), 1e-4f));
        CHECK_THAT(tag_based_extrusions[i]->position.y(), WithinAbs(z_only_extrusions[i]->position.y(), 1e-4f));
        CHECK_THAT(tag_based_extrusions[i]->position.z(), WithinAbs(z_only_extrusions[i]->position.z(), 1e-4f));
    }

    // The enabled mode starts a new seam at the rising outer endpoint
    // (10, 10, 0.4). Its midpoint with the final outer endpoint (10.05, 10, 0.4)
    // is (10.025, 10, 0.4); the disabled mode retains a distant earlier origin.
    REQUIRE(z_only_seams.empty());
    REQUIRE(tag_based_seams.size() == 1);
    CHECK_THAT(tag_based_seams.front()->position.x(), WithinAbs(10.025f, 1e-4f));
    CHECK_THAT(tag_based_seams.front()->position.y(), WithinAbs(10.f, 1e-4f));
    CHECK_THAT(tag_based_seams.front()->position.z(), WithinAbs(0.4f, 1e-4f));
}


TEST_CASE("Default color changes cycle and wrap public color metadata", "[GCodeProcessor][GCodeProcessorColorMetadata]")
{
    const auto result = process_buffer(
        "M83\nG90\nG1 X0 Y0 Z0.2 F600\n"
        "; COLOR_CHANGE,T0\nG1 X1 Y0 E0.1 F600\n"
        "; COLOR_CHANGE,T0\nG1 X2 Y0 E0.1 F600\n"
        "; COLOR_CHANGE,T0\nG1 X3 Y0 E0.1 F600\n"
        "; COLOR_CHANGE,T0\nG1 X4 Y0 E0.1 F600\n"
        "; COLOR_CHANGE,T0\nG1 X5 Y0 E0.1 F600\n"
        "; COLOR_CHANGE,T0\nG1 X6 Y0 E0.1 F600\n"
        "; COLOR_CHANGE,T0\nG1 X7 Y0 E0.1 F600\n");

    const auto valid_hex_color = [](const std::string &value) {
        if (value.size() != 7)
            return false;
        if (value.front() != '#')
            return false;
        return std::all_of(value.begin() + 1, value.end(), [](char c) {
            return std::isxdigit(static_cast<unsigned char>(c)) != 0;
        });
    };

    const auto color_moves = moves_of(result, EMoveType::Color_change);
    REQUIRE(color_moves.size() == 7);
    REQUIRE(result.custom_gcode_per_print_z.size() == 7);
    std::set<std::string> first_six_colors;
    for (size_t i = 0; i < result.custom_gcode_per_print_z.size(); ++i) {
        CAPTURE(i);
        const auto &item = result.custom_gcode_per_print_z[i];
        CHECK(color_moves[i]->type == EMoveType::Color_change);
        CHECK(item.type == CustomGCode::ColorChange);
        CHECK(item.extruder == 1);
        CHECK(valid_hex_color(item.color));
        CHECK_THAT(item.print_z, WithinAbs(0.2, 1e-4));
        if (i < 6)
            first_six_colors.insert(item.color);
    }
    REQUIRE(first_six_colors.size() == 6);
    CHECK(result.custom_gcode_per_print_z[6].color == result.custom_gcode_per_print_z[0].color);
}

TEST_CASE("Malformed color changes match an independent fallback and preserve valid colors", "[GCodeProcessor][GCodeProcessorColorMetadata]")
{
    const std::string prefix = "M83\nG90\nG1 X0 Y0 Z0.2 F600\n";
    const auto make_fixture = [&prefix](const std::string &suffix) {
        return prefix + "; COLOR_CHANGE,T0" + suffix + "\nG1 X5 Y0 E0.4 F600\n";
    };
    const std::string malformed = GENERATE(std::string("#12GGZZ"), std::string("0x12ABCD"),
                                            std::string("#12345"), std::string("#1234567"));

    const auto fallback = process_buffer(make_fixture(""));
    const auto malformed_result = process_buffer(make_fixture(std::string(",") + malformed));
    const auto explicit_result = process_buffer(make_fixture(",#aBc123"));
    REQUIRE(fallback.custom_gcode_per_print_z.size() == 1);
    REQUIRE(malformed_result.custom_gcode_per_print_z.size() == 1);
    REQUIRE(explicit_result.custom_gcode_per_print_z.size() == 1);

    const auto &fallback_item = fallback.custom_gcode_per_print_z.front();
    const auto &malformed_item = malformed_result.custom_gcode_per_print_z.front();
    const auto &explicit_item = explicit_result.custom_gcode_per_print_z.front();
    CHECK(fallback_item.type == CustomGCode::ColorChange);
    CHECK(malformed_item.type == CustomGCode::ColorChange);
    CHECK(explicit_item.type == CustomGCode::ColorChange);
    CHECK(malformed_item.color == fallback_item.color);
    CHECK(malformed_item.color != malformed);
    CHECK(explicit_item.color == "#aBc123");
    CHECK_THAT(malformed_item.print_z, WithinAbs(0.2, 1e-4));
    CHECK_THAT(explicit_item.print_z, WithinAbs(0.2, 1e-4));
}

TEST_CASE("Nozzle type values reach processor metadata and hardness selection", "[GCodeProcessor][NozzleTypeSettings]")
{
    struct Row {
        NozzleType type;
        int        expected_hrc;
    };
    const std::vector<Row> rows = {
        {ntUndefine, 0},
        {ntHardenedSteel, 55},
        {ntStainlessSteel, 20},
        {ntTungstenCarbide, 85},
        {ntBrass, 2},
    };

    for (const Row &row : rows) {
        INFO("nozzle_type=" << static_cast<int>(row.type));
        FullPrintConfig config;
        config.nozzle_type.values = {static_cast<int>(row.type)};
        config.nozzle_hrc.value = 61;
        const auto result = process_file(orca_header(config) + "M83\nG90\nG1 X10 Y0 Z0.2 E1 F600\n");
        REQUIRE(result.nozzle_type.size() == 1);
        CHECK(result.nozzle_type[0] == row.type);
        CHECK(result.nozzle_hrc == 61);
        CHECK(Print::get_hrc_by_nozzle_type(row.type) == row.expected_hrc);
    }
}


TEST_CASE("Public result copy and move preserve imported nozzle metadata",
          "[GCodeProcessor][NozzleTypeSettings][GCodeProcessorPublic]")
{
    FullPrintConfig config;
    const std::vector<NozzleType> expected = {ntHardenedSteel, ntBrass};
    config.nozzle_type.values = {static_cast<int>(expected[0]), static_cast<int>(expected[1])};
    config.nozzle_hrc.value = 61;

    // This is the public file-import path: the CONFIG_BLOCK is parsed into a
    // DynamicPrintConfig before GCodeProcessorResult is extracted.
    TestResult source = process_file(orca_header(config) +
        "M83\nG90\nG1 X10 Y0 Z0.2 E1 F600\n");
    REQUIRE(source.nozzle_type.size() == expected.size());
    CHECK(source.nozzle_type[0] == expected[0]);
    CHECK(source.nozzle_type[1] == expected[1]);
    CHECK(source.nozzle_hrc == 61);

    GCodeProcessorResult copied;
    copied = source;
    REQUIRE(copied.nozzle_type.size() == expected.size());
    CHECK(copied.nozzle_type[0] == expected[0]);
    CHECK(copied.nozzle_type[1] == expected[1]);
    CHECK(copied.nozzle_hrc == 61);
    CHECK(source.nozzle_type == expected);
    CHECK(source.nozzle_hrc == 61);

    GCodeProcessorResult moved;
    moved = std::move(copied);
    REQUIRE(moved.nozzle_type.size() == expected.size());
    CHECK(moved.nozzle_type[0] == expected[0]);
    CHECK(moved.nozzle_type[1] == expected[1]);
    CHECK(moved.nozzle_hrc == 61);

    GCodeProcessorResult fresh;
    CHECK(fresh.nozzle_type.empty());
    CHECK(fresh.nozzle_hrc == 0);

    // Reset a processor after importing metadata, then reuse it for a
    // headerless buffer. Neither the old nozzle types nor HRC may leak.
    GCodeProcessor reused;
    reused.apply_config(config);
    REQUIRE(reused.get_result().nozzle_type.size() == expected.size());
    CHECK(reused.get_result().nozzle_hrc == 61);
    reused.reset();
    CHECK(reused.get_result().nozzle_type.empty());
    CHECK(reused.get_result().nozzle_hrc == 0);
    reused.initialize("gcode_processor_headerless_reuse.gcode");
    reused.initialize_result_moves();
    reused.process_buffer("M83\nG90\nG1 X2 Y0 Z0.2 E0.2 F600\n");
    reused.finalize(false);
    CHECK(reused.get_result().nozzle_type.empty());
    CHECK(reused.get_result().nozzle_hrc == 0);
}

TEST_CASE("Counterclockwise IJ arcs preserve direction, endpoint, and extrusion",
          "[GCodeProcessor][GCodeProcessorPublic][GCodeProcessorArcCCW]")
{
    const auto result = process_buffer(
        "M83\nG90\nG1 X0 Y0 Z0.2 F600\n"
        "G3 X10 Y0 I5 J0 E1 F600\n");
    const auto extrusions = moves_of(result, EMoveType::Extrude);
    REQUIRE(extrusions.size() > 2);

    float minimum_y = extrusions.front()->position.y();
    float total_extrusion = 0.f;
    for (const auto *move : extrusions) {
        REQUIRE(std::isfinite(move->position.x()));
        REQUIRE(std::isfinite(move->position.y()));
        minimum_y = std::min(minimum_y, move->position.y());
        total_extrusion += move->delta_extruder;
    }

    // In the processor's XY convention a CCW half-circle from (0,0) to (10,0)
    // with I5/J0 passes below the chord.  This checks orientation independently
    // of the internal segment count.
    CHECK(minimum_y < -4.5f);
    CHECK_THAT(extrusions.back()->position.x(), WithinAbs(10.f, 1e-4f));
    CHECK_THAT(extrusions.back()->position.y(), WithinAbs(0.f, 1e-4f));
    CHECK_THAT(total_extrusion, WithinAbs(1.f, 1e-3f));
}

TEST_CASE("Counterclockwise radius arcs preserve endpoint and physical extrusion",
          "[GCodeProcessor][GCodeProcessorPublic][GCodeProcessorArcRadius]")
{
    FullPrintConfig config;
    config.gcode_flavor.value = gcfRepRapFirmware;
    const auto result = process_buffer(
        "M83\nG90\nG1 X0 Y0 Z0.2 F600\n"
        "G3 X10 Y0 R8 E1 F600\n", config);
    const auto extrusions = moves_of(result, EMoveType::Extrude);
    REQUIRE(extrusions.size() > 2);

    float minimum_y = extrusions.front()->position.y();
    float total_extrusion = 0.f;
    for (const auto *move : extrusions) {
        REQUIRE(std::isfinite(move->position.x()));
        REQUIRE(std::isfinite(move->position.y()));
        minimum_y = std::min(minimum_y, move->position.y());
        total_extrusion += move->delta_extruder;
    }

    CHECK(minimum_y < -1.f);
    CHECK_THAT(extrusions.back()->position.x(), WithinAbs(10.f, 1e-4f));
    CHECK_THAT(extrusions.back()->position.y(), WithinAbs(0.f, 1e-4f));
    CHECK_THAT(total_extrusion, WithinAbs(1.f, 1e-3f));
}

TEST_CASE("M201 applies a Y acceleration limit to a physical extrusion move",
          "[GCodeProcessor][GCodeProcessorPublic][GCodeProcessorAxisLimits]")
{
    const std::string body =
        "M83\nG90\nG1 X0 Y0 Z0.2 F600\nG1 X0 Y20 E1 F1200\n";
    FullPrintConfig config;
    config.gcode_flavor.value = gcfMarlinFirmware;
    config.machine_max_acceleration_extruding.values = {2000., 2000.};
    const std::string limits = "M201 Y2000\nM204 P2000\n";
    const auto baseline = process_buffer(limits + body, config);
    const auto limited = process_buffer(limits + "M201 Y321\n" + body, config);
    const auto *baseline_move = last_of(baseline, EMoveType::Extrude);
    const auto *limited_move = last_of(limited, EMoveType::Extrude);
    REQUIRE(baseline_move != nullptr);
    REQUIRE(limited_move != nullptr);

    CHECK(normal_time(limited) > normal_time(baseline));
    CHECK_THAT(limited_move->position.y(), WithinAbs(20.f, 1e-4f));
    CHECK_THAT(limited_move->delta_extruder, WithinAbs(1.f, 1e-4f));
}

TEST_CASE("M203 applies independent Y and E axis speed limits",
          "[GCodeProcessor][GCodeProcessorPublic][GCodeProcessorAxisLimits]")
{
    const std::string body =
        "M83\nG90\nG1 X0 Y0 Z0.2 F600\n"
        "G1 X0 Y20 E1 F1200\nG1 E1 F1200\n";
    FullPrintConfig config;
    config.gcode_flavor.value = gcfMarlinFirmware;
    const std::string limits = "M203 Y100 E100\n";
    const auto baseline = process_buffer(limits + body, config);
    const auto y_limited = process_buffer(limits + "M203 Y5\n" + body, config);
    const auto e_limited = process_buffer(limits + "M203 E2\n" + body, config);
    const auto both_limited = process_buffer(limits + "M203 Y5 E2\n" + body, config);

    CHECK(normal_time(y_limited) > normal_time(baseline));
    CHECK(normal_time(e_limited) > normal_time(baseline));
    CHECK(normal_time(both_limited) > normal_time(y_limited));
    CHECK(normal_time(both_limited) > normal_time(e_limited));
    const auto *move = last_of(both_limited, EMoveType::Unretract);
    REQUIRE(move != nullptr);
    CHECK_THAT(move->delta_extruder, WithinAbs(1.f, 1e-4f));
}

TEST_CASE("M203 converts non-Marlin maximum feedrate units",
          "[GCodeProcessor][GCodeProcessorPublic][GCodeProcessorFirmwareLimits]")
{
    const std::string body =
        "M83\nG90\nM203 X60\nG1 X0 Y0 Z0.2 F600\nG1 X20 Y0 E1 F1200\n";
    FullPrintConfig marlin_config;
    marlin_config.gcode_flavor.value = gcfMarlinFirmware;
    FullPrintConfig reprap_config;
    reprap_config.gcode_flavor.value = gcfRepRapFirmware;

    const auto marlin = process_buffer(body, marlin_config);
    const auto reprap = process_buffer(body, reprap_config);
    const auto *marlin_move = last_of(marlin, EMoveType::Extrude);
    const auto *reprap_move = last_of(reprap, EMoveType::Extrude);
    REQUIRE(marlin_move != nullptr);
    REQUIRE(reprap_move != nullptr);

    // The processor interprets Marlin M203 values in mm/s and converts the
    // RepRapFirmware values from mm/min. Both streams retain the same endpoint.
    CHECK(normal_time(reprap) > normal_time(marlin));
    CHECK_THAT(marlin_move->position.x(), WithinAbs(20.f, 1e-4f));
    CHECK_THAT(reprap_move->position.x(), WithinAbs(20.f, 1e-4f));
    CHECK_THAT(marlin_move->delta_extruder, WithinAbs(1.f, 1e-4f));
    CHECK_THAT(reprap_move->delta_extruder, WithinAbs(1.f, 1e-4f));
}

TEST_CASE("Live M203 updates the stealth estimator after motion starts",
          "[GCodeProcessor][GCodeProcessorPublic][GCodeProcessorStealthLimits]")
{
    const std::string prefix = "M83\nG1 X0 Y0 Z0.2 F600\n";
    const auto baseline = process_buffer_with_stealth(prefix + "G1 X40 Y0 E1 F1200\n");
    const auto limited = process_buffer_with_stealth(prefix + "M203 X5\nG1 X40 Y0 E1 F1200\n");

    REQUIRE(normal_time(baseline) > 0.f);
    REQUIRE(stealth_time(baseline) > 0.f);
    CHECK(normal_time(limited) > normal_time(baseline) + 0.5f);
    CHECK(stealth_time(limited) > stealth_time(baseline) + 0.5f);
}

TEST_CASE("Live M204 updates the stealth acceleration estimate after motion starts",
          "[GCodeProcessor][GCodeProcessorPublic][GCodeProcessorStealthLimits]")
{
    const std::string prefix = "M83\nG1 X0 Y0 Z0.2 F600\n";
    const auto baseline = process_buffer_with_stealth(prefix + "G1 X40 Y0 E1 F1200\n");
    const auto limited = process_buffer_with_stealth(prefix + "M204 P20\nG1 X40 Y0 E1 F1200\n");

    REQUIRE(normal_time(baseline) > 0.f);
    REQUIRE(stealth_time(baseline) > 0.f);
    CHECK(normal_time(limited) > normal_time(baseline) + 0.5f);
    CHECK(stealth_time(limited) > stealth_time(baseline) + 0.5f);
}

TEST_CASE("Lowercase standard commands retain public motion state semantics",
          "[GCodeProcessor][GCodeProcessorPublic][GCodeProcessorCase]")
{
    const std::string upper_stream =
        "M83\nG90\nM106 S128\nG1 X10 Y0 Z0.2 E1 F600\n"
        "G91\nG1 X5 Y0 E0.5 F600\nG90\nG1 X20 Y0 E1 F600\n";
    const std::string lower_stream =
        "m83\ng90\nm106 S128\ng1 X10 Y0 Z0.2 E1 F600\n"
        "g91\ng1 X5 Y0 E0.5 F600\ng90\ng1 X20 Y0 E1 F600\n";
    const auto upper = process_buffer(upper_stream);
    const auto lower = process_buffer(lower_stream);
    const auto upper_extrusions = moves_of(upper, EMoveType::Extrude);
    const auto lower_extrusions = moves_of(lower, EMoveType::Extrude);
    REQUIRE(upper_extrusions.size() == 3);
    REQUIRE(lower_extrusions.size() == upper_extrusions.size());
    REQUIRE(upper_extrusions.front()->fan_speed > 0.f);
    REQUIRE(lower_extrusions.front()->fan_speed > 0.f);
    for (size_t i = 0; i < upper_extrusions.size(); ++i) {
        CHECK_THAT(lower_extrusions[i]->position.x(), WithinAbs(upper_extrusions[i]->position.x(), 1e-4f));
        CHECK_THAT(lower_extrusions[i]->position.y(), WithinAbs(upper_extrusions[i]->position.y(), 1e-4f));
        CHECK_THAT(lower_extrusions[i]->position.z(), WithinAbs(upper_extrusions[i]->position.z(), 1e-4f));
        CHECK_THAT(lower_extrusions[i]->delta_extruder, WithinAbs(upper_extrusions[i]->delta_extruder, 1e-4f));
        CHECK_THAT(lower_extrusions[i]->fan_speed, WithinAbs(upper_extrusions[i]->fan_speed, 1e-4f));
    }
    CHECK_THAT(upper_extrusions[0]->position.x(), WithinAbs(10.f, 1e-4f));
    CHECK_THAT(upper_extrusions[1]->position.x(), WithinAbs(15.f, 1e-4f));
    CHECK_THAT(upper_extrusions[2]->position.x(), WithinAbs(20.f, 1e-4f));
    CHECK_THAT(upper_extrusions[0]->delta_extruder, WithinAbs(1.f, 1e-4f));
    CHECK_THAT(upper_extrusions[1]->delta_extruder, WithinAbs(0.5f, 1e-4f));
    CHECK_THAT(upper_extrusions[2]->delta_extruder, WithinAbs(1.f, 1e-4f));
}

TEST_CASE("Unregistered producer headers leave dialect tags inactive while compatible tags still work",
          "[GCodeProcessor][GCodeProcessorPublic][Producer]")
{
    const bool previous_bbl = GCodeProcessor::s_IsBBLPrinter;
    const ScopeGuard restore_bbl([previous_bbl] { GCodeProcessor::s_IsBBLPrinter = previous_bbl; });
    GCodeProcessor::s_IsBBLPrinter = false;

    struct UnsupportedProducer {
        const char *header;
        const char *dialect_role_tag;
    };
    const std::array<UnsupportedProducer, 8> unsupported = {{
        {"; generated by Slic3r Bambu Edition\n", ";TYPE:Perimeter\n"},
        {"; generated by Slic3r\n", ";TYPE:Perimeter\n"},
        {"; generated by SuperSlicer\n", ";TYPE:External perimeter\n"},
        {"; Cura_SteamEngine\n", ";TYPE:WALL-OUTER\n"},
        {"; G-Code generated by Simplify3D(R)\n", "; feature outer perimeter\n"},
        {"; CraftWare\n", ";segType:Perimeter\n"},
        {"; ideaMaker\n", ";TYPE:WALL-OUTER\n"},
        {"; generated by KISSlicer\n", "; 'Perimeter Path'\n"},
    }};

    // These common legacy spellings are not part of the current TYPE vocabulary;
    // the active generic spelling below is "Outer wall".
    CHECK(ExtrusionEntity::string_to_role("Perimeter") == erNone);
    CHECK(ExtrusionEntity::string_to_role("External perimeter") == erNone);
    CHECK(ExtrusionEntity::string_to_role("WALL-OUTER") == erNone);

    for (const auto &[header, dialect_role_tag] : unsupported) {
        CAPTURE(header, dialect_role_tag);
        const auto result = process_file(
            std::string(header) + "M83\nG90\n"
            ";TYPE:Perimeter\n" + dialect_role_tag +
            "G1 X10 Y0 Z0.2 E1 F600\n"
            ";TYPE:Outer wall\n"
            "G1 X20 Y0 E1 F600\n");
        const auto extrusions = moves_of(result, EMoveType::Extrude);
        REQUIRE(extrusions.size() == 2);
        CHECK(extrusions[0]->extrusion_role == erNone);
        CHECK(extrusions[1]->extrusion_role == erExternalPerimeter);
        CHECK_THAT(extrusions[0]->position.x(), WithinAbs(10., 1e-4));
        CHECK_THAT(extrusions[1]->position.x(), WithinAbs(20., 1e-4));
        CHECK_THAT(extrusions[0]->delta_extruder, WithinAbs(1., 1e-4));
        CHECK_THAT(extrusions[1]->delta_extruder, WithinAbs(1., 1e-4));
        CHECK(std::isfinite(normal_time(result)));
        CHECK(normal_time(result) > 0.f);
    }
}

TEST_CASE("Unregistered Simplify3D headers do not import legacy filament or bed metadata",
          "[GCodeProcessor][GCodeProcessorPublic][Producer]")
{
    const bool previous_bbl = GCodeProcessor::s_IsBBLPrinter;
    const ScopeGuard restore_bbl([previous_bbl] { GCodeProcessor::s_IsBBLPrinter = previous_bbl; });
    GCodeProcessor::s_IsBBLPrinter = false;

    FullPrintConfig defaults;
    const auto baseline = process_file(
        "M83\nG90\nG1 X10 Y0 Z0.2 E1 F600\nG1 X20 Y0 E1 F600\n");
    const auto result = process_file(
        "; G-Code generated by Simplify3D(R)\n"
        "; strokeXoverride,220\n"
        "; strokeYoverride,210\n"
        "; filamentDiameters,2.85|1.80\n"
        "; filamentDensities,1.30|1.10\n"
        "; extruderDiameter,2.85|1.80\n"
        "M83\nG90\nG1 X10 Y0 Z0.2 E1 F600\nG1 X20 Y0 E1 F600\n");

    REQUIRE(!defaults.filament_diameter.values.empty());
    CHECK(result.filaments_count == baseline.filaments_count);
    CHECK(result.filaments_count == 0);
    REQUIRE(result.filament_diameters.size() == baseline.filament_diameters.size());
    REQUIRE(result.filament_densities.size() == baseline.filament_densities.size());
    REQUIRE(!result.filament_diameters.empty());
    REQUIRE(!result.filament_densities.empty());
    CHECK_THAT(result.filament_diameters.front(), WithinAbs(defaults.filament_diameter.values.front(), 1e-5));
    CHECK_THAT(result.filament_densities.front(), WithinAbs(DEFAULT_FILAMENT_DENSITY, 1e-5));
    for (size_t i = 0; i < baseline.filament_diameters.size(); ++i)
        CHECK_THAT(result.filament_diameters[i], WithinAbs(baseline.filament_diameters[i], 1e-5));
    for (size_t i = 0; i < baseline.filament_densities.size(); ++i)
        CHECK_THAT(result.filament_densities[i], WithinAbs(baseline.filament_densities[i], 1e-5));
    CHECK(baseline.printable_area.empty());
    CHECK(result.printable_area.empty());
    CHECK(std::isfinite(normal_time(result)));
    CHECK(normal_time(result) > 0.f);
}

TEST_CASE("Orca file import keeps its config footer and Bambu feature roles",
          "[GCodeProcessor][GCodeProcessorPublic][Producer]")
{
    const bool previous_bbl = GCodeProcessor::s_IsBBLPrinter;
    const ScopeGuard restore_bbl([previous_bbl] { GCodeProcessor::s_IsBBLPrinter = previous_bbl; });
    GCodeProcessor::s_IsBBLPrinter = false;

    FullPrintConfig config;
    config.printer_model.value = "Bambu Lab X1 Carbon";
    config.filament_diameter.values = {1.75, 2.85};
    config.filament_density.values = {1.24, 1.10};
    const auto result = process_file(
        orca_header(config) + "M83\nG90\n; FEATURE: Outer wall\nG1 X10 Y0 Z0.2 E1 F600\nG1 X20 Y0 E1 F600\n");
    const auto extrusions = moves_of(result, EMoveType::Extrude);
    REQUIRE(extrusions.size() == 2);
    CHECK(extrusions[0]->extrusion_role == erExternalPerimeter);
    CHECK(extrusions[1]->extrusion_role == erExternalPerimeter);
    CHECK_THAT(extrusions[0]->position.x(), WithinAbs(10., 1e-4));
    CHECK_THAT(extrusions[1]->position.x(), WithinAbs(20., 1e-4));
    CHECK_THAT(extrusions[0]->delta_extruder, WithinAbs(1., 1e-4));
    CHECK_THAT(extrusions[1]->delta_extruder, WithinAbs(1., 1e-4));
    REQUIRE(result.filaments_count == 2);
    REQUIRE(result.filament_diameters.size() == 2);
    REQUIRE(result.filament_densities.size() == 2);
    CHECK_THAT(result.filament_diameters[0], WithinAbs(1.75, 1e-5));
    CHECK_THAT(result.filament_diameters[1], WithinAbs(2.85, 1e-5));
    CHECK_THAT(result.filament_densities[0], WithinAbs(1.24, 1e-5));
    CHECK_THAT(result.filament_densities[1], WithinAbs(1.10, 1e-5));
    CHECK(std::isfinite(normal_time(result)));
    CHECK(normal_time(result) > 0.f);
}

} // namespace public_processor_contracts

namespace post_processor_contracts {

namespace {

struct Processed
{
    std::string output;
    GCodeProcessorResult result;
    Processed() = default;
    Processed(Processed&& other) noexcept : output(std::move(other.output)) { result = std::move(other.result); }
    Processed& operator=(Processed&& other) noexcept
    {
        output = std::move(other.output);
        result = std::move(other.result);
        return *this;
    }
    Processed(const Processed&)            = delete;
    Processed& operator=(const Processed&) = delete;
};

Processed process_post(const std::string& gcode, FullPrintConfig config = {})
{
    ScopedTemporaryFile file(".gcode");
    {
        std::ofstream out(file.string());
        REQUIRE(out.good());
        out << gcode;
    }

    Print print;
    GCodeProcessor processor;
    const bool old_bbl             = GCodeProcessor::s_IsBBLPrinter;
    GCodeProcessor::s_IsBBLPrinter = true;
    const ScopeGuard restore_bbl([old_bbl] { GCodeProcessor::s_IsBBLPrinter = old_bbl; });
    processor.apply_config(config);
    processor.set_print(&print);
    processor.initialize(file.string());
    processor.initialize_result_moves();
    processor.process_buffer(gcode);
    processor.finalize(true);

    // finalize(true) atomically renames the temporary postprocess file over the input path.
    std::ifstream rewritten(file.string());
    REQUIRE(rewritten.good());
    std::ostringstream contents;
    contents << rewritten.rdbuf();

    Processed ret;
    ret.output = contents.str();
    ret.result = std::move(processor.extract_result());
    return ret;
}

const std::string& tag(GCodeProcessor::ETags which) { return GCodeProcessor::reserved_tag(which); }

} // namespace

TEST_CASE("Post process replaces print time and filament inline placeholders", "[GCodeProcessor][GcpPost][Placeholders]")
{
    const std::string gcode = ";" + tag(GCodeProcessor::ETags::Print_Time_Total_Sec_Placeholder) + " " +
                              tag(GCodeProcessor::ETags::Used_Filament_Length_Placeholder) + "\nG4 S2\nM83\nG1 X20 Y0 Z0.2 E1000 F600\n";
    const auto processed = process_post(gcode);
    CHECK(processed.output.find("@PRINT_TIME_TOTAL_SEC@") == std::string::npos);
    CHECK(processed.output.find("@USED_FILAMENT_LENGTH@") == std::string::npos);
    CHECK(processed.output.find("1.00") != std::string::npos);
}

TEST_CASE("Post process expands all clock components consistently", "[GCodeProcessor][GcpPost][Placeholders]")
{
    const std::string line = ";" + tag(GCodeProcessor::ETags::Print_Time_Day_Placeholder) + " " +
                             tag(GCodeProcessor::ETags::Print_Time_Hour_Placeholder) + " " +
                             tag(GCodeProcessor::ETags::Print_Time_Minute_Placeholder) + " " +
                             tag(GCodeProcessor::ETags::Print_Time_Sec_Placeholder) + "\n";
    const auto processed = process_post(line + "G4 S90061\n");
    CHECK(processed.output.find(";1 1 1 1\n") != std::string::npos);
    CHECK(processed.output.find("@PRINT_TIME_DAY@") == std::string::npos);
    CHECK(processed.output.find("@PRINT_TIME_HOUR@") == std::string::npos);
    CHECK(processed.output.find("@PRINT_TIME_MINUTE@") == std::string::npos);
    CHECK(processed.output.find("@PRINT_TIME_SEC@") == std::string::npos);
}

TEST_CASE("Post process rewrites filament length and mass summary lines", "[GCodeProcessor][GcpPost][Filament]")
{
    FullPrintConfig config;
    config.filament_density.values  = {1.24};
    config.filament_cost.values     = {100.0};
    config.filament_diameter.values = {1.75};
    const auto processed            = process_post("; filament used [mm] = 0\n"
                                                              "; filament used [g] = 0\n"
                                                              "; total filament used [g] = 0\n"
                                                              "; filament used [cm3] = 0\n"
                                                              "; filament cost = 0\n"
                                                              "; total filament cost = 0\n"
                                                              "M83\nG1 X20 Y0 Z0.2 E1000 F600\n",
                                                   config);
    CHECK(processed.output.find("; filament used [mm] = 0\n") == std::string::npos);
    CHECK(processed.output.find("; filament used [g] = 0\n") == std::string::npos);
    CHECK(processed.output.find("; total filament used [g] = 0\n") == std::string::npos);
    CHECK(processed.output.find("; filament used [mm] = 1000.00\n") != std::string::npos);
    CHECK(processed.output.find("; filament used [g] = 2.98\n") != std::string::npos);
    CHECK(processed.output.find("; total filament used [g] = 2.98\n") != std::string::npos);
    CHECK(processed.output.find("; filament used [cm3] = 2.41\n") != std::string::npos);
    CHECK(processed.output.find("; filament cost = 0.30\n") != std::string::npos);
    CHECK(processed.output.find("; total filament cost = 0.30\n") != std::string::npos);
}

TEST_CASE("Post process emits first and last M73 placeholders", "[GCodeProcessor][GcpPost][M73]")
{
    const auto processed = process_post(";_GP_FIRST_LINE_M73_PLACEHOLDER\n"
                                        "G4 S120\nM83\nG1 X20 Y0 Z0.2 E1 F600\n"
                                        ";_GP_LAST_LINE_M73_PLACEHOLDER\n");
    CHECK(processed.output.find("_GP_FIRST_LINE_M73_PLACEHOLDER") == std::string::npos);
    CHECK(processed.output.find("_GP_LAST_LINE_M73_PLACEHOLDER") == std::string::npos);
    CHECK(processed.output.find("M73 P0 R2") != std::string::npos);
    CHECK(processed.output.find("M73 P100 R0") != std::string::npos);
}

namespace {

std::vector<std::string> stop_progress_lines(const std::string &output)
{
    std::vector<std::string> result;
    std::istringstream input(output);
    std::string line;
    while (std::getline(input, line)) {
        if (line.rfind("M73 C", 0) == 0)
            result.push_back(line);
    }
    return result;
}

} // namespace

TEST_CASE("Post process exports remaining time across subminute stop transitions",
          "[GCodeProcessor][GcpPost][M73][GcpPostStop]")
{
    const std::string stop_fixture =
        ";_GP_FIRST_LINE_M73_PLACEHOLDER\n"
        "M83\n"
        "G1 X10 Y0 Z0.2 E0.5 F600\n"
        "G4 S80\n"
        "G1 X10.05 Y0 E0.01 F600\n"
        "G1 X10.10 Y0 E0.01 F600\n"
        "; PAUSE_PRINTING\n"
        "G1 X20 Y0 E0.5 F600\n"
        "G4 S80\n"
        "G1 X20.05 Y0 E0.01 F600\n"
        "G1 X20.10 Y0 E0.01 F600\n"
        "; COLOR_CHANGE,T0,#112233\n"
        "G1 X30 Y0 E0.5 F600\n"
        ";_GP_LAST_LINE_M73_PLACEHOLDER\n";

    FullPrintConfig enabled_config;
    enabled_config.disable_m73.value = false;
    const auto processed = process_post(stop_fixture, enabled_config);

    const auto stop_lines = stop_progress_lines(processed.output);
    REQUIRE_FALSE(stop_lines.empty());
    CHECK(std::find(stop_lines.begin(), stop_lines.end(), "M73 C1") != stop_lines.end());
    CHECK(std::find(stop_lines.begin(), stop_lines.end(), "M73 C0.00") != stop_lines.end());
    CHECK(std::find(stop_lines.begin(), stop_lines.end(), "M73 C0") != stop_lines.end());
    CHECK(processed.output.find("M73 P0 R") != std::string::npos);

    size_t positive_extrusions = 0;
    size_t pause_moves = 0;
    size_t color_change_moves = 0;
    for (const auto &move : processed.result.moves) {
        if (move.type == EMoveType::Extrude && move.delta_extruder > 0.0f && move.feedrate > 0.0f)
            ++positive_extrusions;
        if (move.type == EMoveType::Pause_Print)
            ++pause_moves;
        if (move.type == EMoveType::Color_change)
            ++color_change_moves;
    }
    REQUIRE(positive_extrusions >= 7);
    REQUIRE(pause_moves == 1);
    REQUIRE(color_change_moves == 1);

    const size_t first_stop = processed.output.find("M73 C0.00\n");
    const size_t first_short = processed.output.find("G1 X10.10 Y0 E0.01 F600");
    const size_t first_marker = processed.output.find("; PAUSE_PRINTING");
    const size_t final_stop = processed.output.find("M73 C0\n");
    const size_t final_short = processed.output.find("G1 X20.10 Y0 E0.01 F600");
    const size_t final_marker = processed.output.find("; COLOR_CHANGE,T0,#112233");
    const size_t positive_stop = processed.output.find("M73 C1\n", first_marker);
    REQUIRE(first_stop != std::string::npos);
    REQUIRE(first_short != std::string::npos);
    REQUIRE(first_marker != std::string::npos);
    REQUIRE(final_stop != std::string::npos);
    REQUIRE(final_short != std::string::npos);
    REQUIRE(final_marker != std::string::npos);
    REQUIRE(positive_stop != std::string::npos);
    CHECK(first_short < first_stop);
    CHECK(first_stop < first_marker);
    CHECK(first_marker < positive_stop);
    CHECK(positive_stop < final_marker);
    CHECK(final_short < final_stop);
    CHECK(final_stop < final_marker);

    FullPrintConfig disabled_config = enabled_config;
    disabled_config.disable_m73.value = true;
    const auto disabled = process_post(stop_fixture, disabled_config);
    CHECK(disabled.output.find("M73 ") == std::string::npos);

    size_t disabled_positive_extrusions = 0;
    size_t disabled_pause_moves = 0;
    size_t disabled_color_change_moves = 0;
    for (const auto &move : disabled.result.moves) {
        if (move.type == EMoveType::Extrude && move.delta_extruder > 0.0f && move.feedrate > 0.0f)
            ++disabled_positive_extrusions;
        if (move.type == EMoveType::Pause_Print)
            ++disabled_pause_moves;
        if (move.type == EMoveType::Color_change)
            ++disabled_color_change_moves;
    }
    REQUIRE(disabled_positive_extrusions == positive_extrusions);
    REQUIRE(disabled_pause_moves == 1);
    REQUIRE(disabled_color_change_moves == 1);
}

TEST_CASE("Post process expands estimated time and layer placeholders", "[GCodeProcessor][GcpPost][Placeholders]")
{
    const auto processed = process_post(";_GP_ESTIMATED_PRINTING_TIME_PLACEHOLDER\n"
                                        ";_GP_TOTAL_LAYER_NUMBER_PLACEHOLDER\n"
                                        "; CHANGE_LAYER\n"
                                        "M83\nG1 X10 Y0 Z0.2 E1 F600\n");
    CHECK(processed.output.find("_GP_ESTIMATED_PRINTING_TIME_PLACEHOLDER") == std::string::npos);
    CHECK(processed.output.find("_GP_TOTAL_LAYER_NUMBER_PLACEHOLDER") == std::string::npos);
    CHECK(processed.output.find("model printing time") != std::string::npos);
    CHECK(processed.output.find("total layer number: 1") != std::string::npos);
}

TEST_CASE("Disabling M73 removes generated progress commands but keeps motion", "[GCodeProcessor][GcpPost][M73]")
{
    FullPrintConfig config;
    config.disable_m73.value = true;
    const auto processed = process_post(";_GP_FIRST_LINE_M73_PLACEHOLDER\nM83\nG1 X10 Y0 Z0.2 E1 F600\n;_GP_LAST_LINE_M73_PLACEHOLDER\n",
                                        config);
    CHECK(processed.output.find("M73 P") == std::string::npos);
    CHECK(processed.output.find("G1 X10 Y0 Z0.2 E1 F600") != std::string::npos);
}

TEST_CASE("Post process preserves ordinary comments and command order", "[GCodeProcessor][GcpPost][Rewrite]")
{
    const auto processed = process_post("; ordinary marker\nG90\nG1 X1 Y2 Z0.2 F600\nM400 S1\n");
    const size_t marker  = processed.output.find("; ordinary marker");
    const size_t motion  = processed.output.find("G1 X1 Y2 Z0.2 F600");
    const size_t sync    = processed.output.find("M400 S1");
    REQUIRE(marker != std::string::npos);
    REQUIRE(motion != std::string::npos);
    REQUIRE(sync != std::string::npos);
    CHECK(marker < motion);
    CHECK(motion < sync);
}

TEST_CASE("Post process replaces repeated inline placeholders", "[GCodeProcessor][GcpPost][Placeholders]")
{
    const auto processed = process_post("; " + tag(GCodeProcessor::ETags::Print_Time_Sec_Placeholder) + " " +
                                        tag(GCodeProcessor::ETags::Print_Time_Sec_Placeholder) + "\nG4 S1\n");
    CHECK(processed.output.find("@PRINT_TIME_SEC@") == std::string::npos);
    CHECK(processed.output.find("; 1 1") != std::string::npos);
}

TEST_CASE("Inline total-time placeholder reports a known dwell", "[GCodeProcessor][GcpPost][Timing]")
{
    const auto processed = process_post("; " + tag(GCodeProcessor::ETags::Print_Time_Total_Sec_Placeholder) + "\nG4 S2\n");
    CHECK(processed.output.find("; 2.00\n") != std::string::npos);
}

TEST_CASE("Post process leaves unknown inline placeholders untouched", "[GCodeProcessor][GcpPost][Placeholders]")
{
    const auto processed = process_post("; @NOT_A_PROCESSOR_PLACEHOLDER@\nG1 X1 Y0 Z0.2 F600\n");
    CHECK(processed.output.find("@NOT_A_PROCESSOR_PLACEHOLDER@") != std::string::npos);
}

TEST_CASE("Post process uses the configured filament diameter for length", "[GCodeProcessor][GcpPost][Filament]")
{
    FullPrintConfig config;
    config.filament_diameter.values = {2.0};
    const auto processed            = process_post("; filament used [mm] = 0\nM83\nG1 X10 Y0 Z0.2 E2 F600\n", config);
    const size_t pos                = processed.output.find("; filament used [mm] = ");
    REQUIRE(pos != std::string::npos);
    CHECK(processed.output.substr(pos, 40).find("2.00") != std::string::npos);
    CHECK(processed.output.substr(pos, 40).find("0.00") == std::string::npos);
}

TEST_CASE("End marker remains in output while raw dwell statistics stay explicit", "[GCodeProcessor][GcpPost][Timing]")
{
    const auto without_end = process_post("G4 S2\n");
    const auto with_end    = process_post("G4 S2\n; MACHINE_END_GCODE_START\nG4 S20\n");
    CHECK_THAT(without_end.result.print_statistics.modes[0].time, WithinAbs(2.f, 1e-4));
    CHECK_THAT(with_end.result.print_statistics.modes[0].time, WithinAbs(22.f, 1e-4));
    CHECK(with_end.output.find("MACHINE_END_GCODE_START") != std::string::npos);
}

TEST_CASE("Start marker is retained while the rewritten file remains parseable", "[GCodeProcessor][GcpPost][Rewrite]")
{
    const auto processed = process_post("; MACHINE_START_GCODE_END\nG90\nG1 X2 Y3 Z0.2 F600\n");
    CHECK(processed.output.find("MACHINE_START_GCODE_END") != std::string::npos);
    CHECK(processed.output.find("G1 X2 Y3 Z0.2 F600") != std::string::npos);
}

TEST_CASE("Layer change tracking is independent of ordinary text", "[GCodeProcessor][GcpPost][Layers]")
{
    const auto processed = process_post("; CHANGE_LAYER\nG1 X1 Y0 Z0.2 F600\n; CHANGE_LAYER\nG1 X2 Y0 Z0.4 F600\n"
                                        ";_GP_TOTAL_LAYER_NUMBER_PLACEHOLDER\n");
    CHECK(processed.output.find("total layer number: 2") != std::string::npos);
}

TEST_CASE("Wipe moves receive the fixed preview dimensions", "[GCodeProcessor][GcpPost][Metadata]")
{
    const auto processed = process_post("M83\n; WIPE_START\nG1 X3 Y0 Z0.2 E-0.2 F600\n; WIPE_END\n");
    bool found           = false;
    for (const auto& move : processed.result.moves)
        if (move.type == EMoveType::Wipe) {
            found = true;
            CHECK_THAT(move.width, WithinAbs(GCodeProcessor::Wipe_Width, 1e-6));
            CHECK_THAT(move.height, WithinAbs(GCodeProcessor::Wipe_Height, 1e-6));
        }
    CHECK(found);
}

TEST_CASE("Color change metadata survives rewriting", "[GCodeProcessor][GcpPost][Metadata]")
{
    const auto processed = process_post("M83\nG1 X2 Y0 Z0.2 E1 F600\n; COLOR_CHANGE,T0,#112233\n");
    CHECK(processed.output.find("COLOR_CHANGE,T0,#112233") != std::string::npos);
    REQUIRE(!processed.result.custom_gcode_per_print_z.empty());
    CHECK(processed.result.custom_gcode_per_print_z.back().type == CustomGCode::ColorChange);
}

TEST_CASE("Pause and custom-code metadata share their print height", "[GCodeProcessor][GcpPost][Metadata]")
{
    const auto processed = process_post("M83\nG1 X2 Y0 Z0.4 E1 F600\n; PAUSE_PRINTING\n; CUSTOM_GCODE\n");
    REQUIRE(processed.result.custom_gcode_per_print_z.size() == 2);
    CHECK_THAT(processed.result.custom_gcode_per_print_z[0].print_z, WithinAbs(0.4, 1e-4));
    CHECK_THAT(processed.result.custom_gcode_per_print_z[1].print_z, WithinAbs(0.4, 1e-4));
}

TEST_CASE("Flush tags remain in output while flush volume is partitioned", "[GCodeProcessor][GcpPost][Filament]")
{
    const auto processed = process_post("M83\n; FLUSH_START\n; EXTERNAL_PURGE 3.00\n; FLUSH_END\n");
    CHECK(processed.output.find("FLUSH_START") != std::string::npos);
    CHECK(processed.output.find("FLUSH_END") != std::string::npos);
    const double expected_volume = 3. * PI * .25 * 1.75 * 1.75;
    REQUIRE(processed.result.print_statistics.flush_per_filament.count(0) == 1);
    CHECK_THAT(processed.result.print_statistics.flush_per_filament.at(0), WithinAbs(expected_volume, 1e-3));
    REQUIRE(processed.result.print_statistics.total_volumes_per_extruder.count(0) == 1);
    CHECK_THAT(processed.result.print_statistics.total_volumes_per_extruder.at(0), WithinAbs(expected_volume, 1e-3));
}

TEST_CASE("Virtual flush tags do not remove their motion lines", "[GCodeProcessor][GcpPost][Filament]")
{
    const auto processed = process_post("M83\n; VFLUSH_START\nG1 X10 Y0 Z0.2 E1 F600\n; VFLUSH_END\n");
    CHECK(processed.output.find("G1 X10 Y0 Z0.2 E1 F600") != std::string::npos);
    CHECK(processed.result.print_statistics.total_volumes_per_extruder.size() == 1);
}

TEST_CASE("External purge is represented in flush statistics without a move", "[GCodeProcessor][GcpPost][Filament]")
{
    const auto processed = process_post("; EXTERNAL_PURGE 3.00\n");
    CHECK(processed.output.find("EXTERNAL_PURGE 3.00") != std::string::npos);
    const double expected_volume = 3. * PI * .25 * 1.75 * 1.75;
    REQUIRE(processed.result.print_statistics.flush_per_filament.count(0) == 1);
    CHECK_THAT(processed.result.print_statistics.flush_per_filament.at(0), WithinAbs(expected_volume, 1e-3));
    CHECK(processed.result.moves.size() == 1);
    bool has_extrusion_move = false;
    for (const auto& move : processed.result.moves)
        if (move.type == EMoveType::Extrude)
            has_extrusion_move = true;
    CHECK_FALSE(has_extrusion_move);
}

TEST_CASE("Skippable region boundaries survive post processing", "[GCodeProcessor][GcpPost][Skippable]")
{
    const auto processed = process_post("; SKIPPABLE_START\n; SKIPTYPE: timelapse\nG1 X5 Y0 Z0.2 F600\n; SKIPPABLE_END\n");
    CHECK(processed.output.find("SKIPPABLE_START") != std::string::npos);
    CHECK(processed.output.find("SKIPTYPE: timelapse") != std::string::npos);
    CHECK(processed.output.find("SKIPPABLE_END") != std::string::npos);
}

TEST_CASE("Finalization reports first layer time for first-layer motion", "[GCodeProcessor][GcpPost][Timing]")
{
    const auto processed = process_post("G1 X10 Y0 Z0.2 F600\n; CHANGE_LAYER\nG1 X30 Y0 Z0.4 F600\n");
    CHECK(processed.result.initial_layer_time > 0.f);
}

TEST_CASE("Post process preserves arc source lines and endpoint state", "[GCodeProcessor][GcpPost][Arcs]")
{
    const auto processed = process_post("M83\nG1 X10 Y0 Z0.2 E1 F600\nG2 X0 Y10 I-5 J5 E1 F600\n");
    CHECK(processed.output.find("G2 X0 Y10 I-5 J5 E1 F600") != std::string::npos);
    CHECK_THAT(processed.result.moves.back().position.x(), WithinAbs(0., 1e-3));
    CHECK_THAT(processed.result.moves.back().position.y(), WithinAbs(10., 1e-3));
}

TEST_CASE("Post process handles an empty input without inventing motion", "[GCodeProcessor][GcpPost][Empty]")
{
    const auto processed = process_post("");
    CHECK(processed.result.moves.size() == 1);
    CHECK(processed.output.empty());
}

TEST_CASE("Post process keeps line endings and a final unterminated line", "[GCodeProcessor][GcpPost][Rewrite]")
{
    const auto processed = process_post("G90\r\nG1 X4 Y0 Z0.2 F600");
    CHECK(processed.output.find("G90\r\n") != std::string::npos);
    CHECK(processed.output.find("G1 X4 Y0 Z0.2 F600") != std::string::npos);
}

} // namespace post_processor_contracts

namespace gcode_processor_reachable_contracts {

namespace {

// Bound fixture input and rewritten output before reading them into memory.
constexpr std::size_t kMaxFixtureBytes = 64 * 1024;
constexpr std::uintmax_t kMaxOutputBytes = 512 * 1024;

struct Processed {
    std::string output;
    GCodeProcessorResult result;
    Processed() = default;
    Processed(Processed &&other) noexcept : output(std::move(other.output)) { result = std::move(other.result); }
    Processed &operator=(Processed &&other) noexcept
    {
        output = std::move(other.output);
        result = std::move(other.result);
        return *this;
    }
    Processed(const Processed &) = delete;
    Processed &operator=(const Processed &) = delete;
};

FullPrintConfig preheat_config()
{
    FullPrintConfig config;
    config.gcode_flavor.value = gcfMarlinFirmware;
    config.filament_diameter.values = {1.75, 1.75};
    config.filament_map.values = {1, 2};
    config.filament_type.values = {"PLA", "PETG"};
    config.physical_extruder_map.values = {0, 1};
    config.extruder_max_nozzle_count.values = {1, 1};
    config.nozzle_diameter.values = {0.4, 0.4};
    config.nozzle_volume.values = {0., 0.};
    config.extruder_type.values = {etDirectDrive, etDirectDrive};
    config.nozzle_temperature_initial_layer.values = {205, 225};
    config.nozzle_temperature.values = {210, 230};
    config.filament_pre_cooling_temperature.values = {150, 160};
    config.filament_preheat_temperature_delta.values = {10., 10.};
    config.hotend_cooling_rate.values = {4., 4.};
    config.hotend_heating_rate.values = {4., 4.};
    config.enable_pre_heating.value = true;
    config.preheat_time.value = 30;
    config.preheat_steps.value = 2;
    config.single_extruder_multi_material.value = false;
    return config;
}

void check_group_mapping(const GCodeProcessorResult &result);

Processed process_post(const std::string &gcode, FullPrintConfig config = {}, bool group = false)
{
    REQUIRE(gcode.size() <= kMaxFixtureBytes);
    if (group) {
        REQUIRE(config.filament_map.values.size() >= 2);
        REQUIRE(config.filament_diameter.values.size() >= 2);
        REQUIRE(config.filament_type.values.size() >= 2);
        REQUIRE(config.physical_extruder_map.values.size() >= 2);
        REQUIRE(config.extruder_max_nozzle_count.values.size() >= 2);
        REQUIRE(config.nozzle_diameter.values.size() >= 2);
        REQUIRE(config.nozzle_volume.values.size() >= 2);
        REQUIRE(config.extruder_type.values.size() >= 2);
        REQUIRE(config.hotend_cooling_rate.values.size() >= 2);
        REQUIRE(config.hotend_heating_rate.values.size() >= 2);
        REQUIRE(config.nozzle_temperature.values.size() >= 2);
        REQUIRE(config.nozzle_temperature_initial_layer.values.size() >= 2);
        REQUIRE(config.filament_pre_cooling_temperature.values.size() >= 2);
        REQUIRE(config.filament_preheat_temperature_delta.values.size() >= 2);
    }
    ScopedTemporaryFile file(".gcode");
    {
        std::ofstream out(file.string());
        REQUIRE(out.good());
        out << gcode;
    }

    Print print;
    GCodeProcessor processor;
    const bool old_bbl = GCodeProcessor::s_IsBBLPrinter;
    GCodeProcessor::s_IsBBLPrinter = true;
    const ScopeGuard restore_bbl([old_bbl] { GCodeProcessor::s_IsBBLPrinter = old_bbl; });
    processor.apply_config(config);
    // A fresh processor's public static-config path retains its initialized
    // handle_hotend_as_extruder=false state; the dynamic overload is private.
    if (group) {
        processor.ensure_nozzle_group_result(2);
        check_group_mapping(processor.get_result());
    }
    processor.set_print(&print);
    processor.initialize(file.string());
    processor.initialize_result_moves();
    processor.process_buffer(gcode);
    processor.finalize(true);

    std::error_code output_size_error;
    const std::uintmax_t output_size = std::filesystem::file_size(file.string(), output_size_error);
    REQUIRE_FALSE(output_size_error);
    REQUIRE(output_size <= kMaxOutputBytes);

    std::ifstream in(file.string());
    REQUIRE(in.good());
    std::ostringstream output;
    output << in.rdbuf();
    Processed result;
    result.output = output.str();
    REQUIRE(result.output.size() <= kMaxOutputBytes);
    result.result = std::move(processor.extract_result());
    if (group)
        check_group_mapping(result.result);
    return result;
}

void check_group_mapping(const GCodeProcessorResult &result)
{
    const auto *group = dynamic_cast<const MultiNozzleUtils::LayeredNozzleGroupResult *>(
        result.nozzle_group_result.get());
    REQUIRE(group != nullptr);
    const auto filament0 = group->get_first_nozzle_for_filament(0);
    const auto filament1 = group->get_first_nozzle_for_filament(1);
    REQUIRE(filament0.has_value());
    REQUIRE(filament1.has_value());
    CHECK(filament0->extruder_id == 0);
    CHECK(filament1->extruder_id == 1);
}

} // namespace

TEST_CASE("The singular reserved-tag query returns the first compatible tag and stops", "[GCodeProcessorReachable][GCodeProcessor]")
{
    const bool old_bbl = GCodeProcessor::s_IsBBLPrinter;
    const ScopeGuard restore([old_bbl] { GCodeProcessor::s_IsBBLPrinter = old_bbl; });
    GCodeProcessor::s_IsBBLPrinter = false;

    std::string found;
    REQUIRE(GCodeProcessor::contains_reserved_tag("; ordinary\n;TYPE:Outer wall\n;LAYER_CHANGE\n", found));
    CHECK(found == "TYPE:Outer wall");
    CHECK_FALSE(GCodeProcessor::contains_reserved_tag("G1 X1 Y1\n; ordinary\n", found));
}

TEST_CASE("The singular reserved-tag query follows BBL feature spelling", "[GCodeProcessorReachable][GCodeProcessor]")
{
    const bool old_bbl = GCodeProcessor::s_IsBBLPrinter;
    const ScopeGuard restore([old_bbl] { GCodeProcessor::s_IsBBLPrinter = old_bbl; });
    GCodeProcessor::s_IsBBLPrinter = true;

    std::string found;
    REQUIRE(GCodeProcessor::contains_reserved_tag("; FEATURE: Outer wall\n;TYPE:ignored\n", found));
    CHECK(found == " FEATURE: Outer wall");
    CHECK_FALSE(GCodeProcessor::contains_reserved_tag("; the FEATURE: text is not a prefix\n", found));
}

TEST_CASE("Preheating-enabled post processing records machine and skippable boundaries", "[GCodeProcessor][GCodeProcessorReachable][GcpPreheat]")
{
    const std::string gcode =
        "; MACHINE_START_GCODE_END\n"
        "M83\nT0\nG1 X20 Y0 Z0.2 E1 F600\n"
        "; SKIPPABLE_START\n; SKIPTYPE: timelapse\nG4 S3\n; SKIPPABLE_END\n"
        "; CP_TOOLCHANGE_WIPE CT1 FL0\n"
        "; NOZZLE_CHANGE_START OF0 NF1 ON0 NN1\n"
        "T1 H1\nG1 X40 Y0 E1 F600\n"
        "; NOZZLE_CHANGE_END OF0 NF1 ON0 NN1\n"
        "; MACHINE_END_GCODE_START\nG4 S1\n";
    const auto processed = process_post(gcode, preheat_config(), true);
    INFO("preheat output: " << processed.output);

    REQUIRE(processed.result.nozzle_group_result != nullptr);
    CHECK(processed.output.find("MACHINE_START_GCODE_END") != std::string::npos);
    CHECK(processed.output.find("MACHINE_END_GCODE_START") != std::string::npos);
    CHECK(processed.output.find("CP_TOOLCHANGE_WIPE CT1 FL0") != std::string::npos);
    CHECK(processed.output.find("NOZZLE_CHANGE_START OF0 NF1 ON0 NN1") != std::string::npos);
    CHECK(processed.output.find("NOZZLE_CHANGE_END OF0 NF1 ON0 NN1") != std::string::npos);
    CHECK(processed.result.moves.size() > 1);
}

TEST_CASE("Preheating usage parsing accepts VT and M1020 filament changes", "[GCodeProcessor][GCodeProcessorReachable][GcpPreheat]")
{
    const std::string gcode =
        "; MACHINE_START_GCODE_END\nM83\n;VT0 H0\nG1 X4 Y0 Z0.2 E1 F600\n"
        "M1020 S1 H1\nG1 X8 Y0 E1 F600\n; MACHINE_END_GCODE_START\n";
    const auto processed = process_post(gcode, preheat_config(), true);
    CHECK(processed.output.find("M1020 S1 H1") != std::string::npos);
    CHECK(processed.output.find(";VT0 H0") != std::string::npos);
    REQUIRE(processed.result.moves.size() > 2);
}

TEST_CASE("Preheating mode leaves a valid file unchanged when no idle injection is possible", "[GCodeProcessor][GCodeProcessorReachable][GcpPreheat]")
{
    const std::string gcode = "M83\nG1 X2 Y0 Z0.2 E1 F600\n";
    const auto processed = process_post(gcode, preheat_config(), true);
    CHECK(processed.output == gcode);
    REQUIRE(processed.result.moves.size() > 1);
}



TEST_CASE("Post processing rewrites all filament summary dimensions independently", "[GCodeProcessor][GCodeProcessorReachable][GcpPost]")
{
    FullPrintConfig config;
    config.filament_diameter.values = {1.75, 2.0};
    config.filament_density.values = {1.24, 1.10};
    config.filament_cost.values = {100.0, 200.0};
    const std::string summaries = "; filament used [mm] = 0, 0\n; filament used [g] = 0, 0\n"
                                   "; total filament used [g] = 0\n; filament used [cm3] = 0, 0\n"
                                   "; filament cost = 0, 0\n; total filament cost = 0\n";
    const auto processed = process_post(summaries + "M83\nT1\nG1 X10 Y0 Z0.2 E2 F600\n", config);
    CHECK(processed.output.find("filament used [mm] = 0.00, 2.00") != std::string::npos);
    CHECK(processed.output.find("filament used [g] = 0.00, 0.01") != std::string::npos);
    CHECK(processed.output.find("filament used [cm3] = 0.00, 0.01") != std::string::npos);
    CHECK(processed.output.find("filament cost = 0.00, 0.00") != std::string::npos);
    CHECK(processed.output.find("total filament used [g] = 0.01") != std::string::npos);
}



TEST_CASE("Preheating multi-extruder usage emits a temperature transition in a measured idle window", "[GCodeProcessor][GCodeProcessorReachable][GcpPreheat]")
{
    const std::string gcode =
        "; MACHINE_START_GCODE_END\nM83\nT0\nG1 X100 Y0 Z0.2 E1 F60\n"
        "; NOZZLE_CHANGE_START OF0 NF1 ON0 NN1\n"
        "; CP_TOOLCHANGE_WIPE CT0 FL0\nT1\n"
        "G1 X150 Y0 E0.5 F60\nG1 X200 Y0 E1 F60\n"
        "; NOZZLE_CHANGE_END OF0 NF1 ON0 NN1\n"
        "; MACHINE_END_GCODE_START\n";
    const auto processed = process_post(gcode, preheat_config(), true);
    INFO("preheat transition output: " << processed.output);
    REQUIRE(processed.result.nozzle_group_result != nullptr);
    CHECK(processed.output.find("M104 T1 S25 N0 ;Multi extruder pre cooling") != std::string::npos);
    CHECK(processed.output.find("M104 T1 S220 N0 ;Multi extruder pre heating") != std::string::npos);
    CHECK(processed.output.find("M632 S1 W") != std::string::npos);
    CHECK(processed.output.find("M633") != std::string::npos);
    REQUIRE(processed.result.moves.size() > 2);
}


TEST_CASE("Preheating is a gate for inserted temperature commands", "[GCodeProcessor][GCodeProcessorReachable][GcpPreheat]")
{
    const std::string gcode =
        "; MACHINE_START_GCODE_END\nM83\nT0\nG1 X100 Y0 Z0.2 E1 F60\n"
        "; NOZZLE_CHANGE_START OF0 NF1 ON0 NN1\n"
        "; CP_TOOLCHANGE_WIPE CT0 FL0\nT1\n"
        "G1 X150 Y0 E0.5 F60\nG1 X200 Y0 E1 F60\n"
        "; NOZZLE_CHANGE_END OF0 NF1 ON0 NN1\n"
        "; MACHINE_END_GCODE_START\n";
    auto disabled = preheat_config();
    disabled.enable_pre_heating.value = false;
    const auto without = process_post(gcode, disabled, true);
    const auto with = process_post(gcode, preheat_config(), true);
    CHECK(without.output.find("Multi extruder pre heating") == std::string::npos);
    CHECK(without.output.find("Multi extruder pre cooling") == std::string::npos);
    CHECK(with.output.find("Multi extruder pre heating") != std::string::npos);
    CHECK(with.output.find("Multi extruder pre cooling") != std::string::npos);
}

TEST_CASE("The three supported filament-change spellings drive preheat usage", "[GCodeProcessor][GCodeProcessorReachable][GcpPreheat]")
{
    // The preheat usage pass parses all three spellings. ;VT is not a general preview tool change,
    // so exact injected commands are the shared public oracle for this case.
    const auto marker = GENERATE(std::string("T1"), std::string(";VT1 H1"), std::string("M1020 S1 H1"));
    const std::string gcode =
        "; MACHINE_START_GCODE_END\nM83\nT0\nG1 X100 Y0 Z0.2 E1 F60\n"
        "; NOZZLE_CHANGE_START OF0 NF1 ON0 NN1\n"
        "; CP_TOOLCHANGE_WIPE CT0 FL0\n" + marker + "\n"
        "G1 X150 Y0 E0.5 F60\nG1 X200 Y0 E1 F60\n"
        "; NOZZLE_CHANGE_END OF0 NF1 ON0 NN1\n"
        "; MACHINE_END_GCODE_START\n";
    const auto processed = process_post(gcode, preheat_config(), true);
    INFO("filament marker: " << marker);
    INFO("preheat output: " << processed.output);
    REQUIRE(processed.result.nozzle_group_result != nullptr);
    REQUIRE(processed.result.moves.size() > 2);
    CHECK(processed.output.find(marker) != std::string::npos);
    CHECK(processed.output.find("M104 T1 S25 N0 ;Multi extruder pre cooling") != std::string::npos);
    CHECK(processed.output.find("M104 T1 S220 N0 ;Multi extruder pre heating") != std::string::npos);
    CHECK(processed.output.find("M632 S1 W") != std::string::npos);
    CHECK(processed.output.find("M633") != std::string::npos);
    CHECK(std::isfinite(processed.result.print_statistics.modes[0].time));
}

TEST_CASE("An optional first-layer flag may be omitted from a wipe marker", "[GCodeProcessor][GCodeProcessorReachable][GcpPreheat]")
{
    const std::string gcode =
        "; MACHINE_START_GCODE_END\nM83\nT0\nG1 X20 Y0 Z0.2 E1 F600\n"
        "; NOZZLE_CHANGE_START OF0 NF1 ON0 NN1\n"
        "; CP_TOOLCHANGE_WIPE CT0\nT1\n"
        "G1 X40 Y0 E1 F600\n"
        "; NOZZLE_CHANGE_END OF0 NF1 ON0 NN1\n; MACHINE_END_GCODE_START\n";
    const auto processed = process_post(gcode, preheat_config(), true);
    REQUIRE(processed.result.moves.size() > 2);
    CHECK(processed.output.find("CP_TOOLCHANGE_WIPE CT0") != std::string::npos);
    CHECK(std::isfinite(processed.result.print_statistics.modes[0].time));
}

TEST_CASE("Nozzle-change contact metadata still permits a measured preheat transition", "[GCodeProcessor][GCodeProcessorReachable][GcpPreheat]")
{
    const std::string gcode =
        "; MACHINE_START_GCODE_END\nM83\nT0\nG1 X100 Y0 Z0.2 E1 F60\n"
        "; NOZZLE_CHANGE_START OF0 NF1 ON0 NN1\n"
        "; CP_TOOLCHANGE_WIPE CT1 FL1\nT1\n"
        "G1 X150 Y0 E0.5 F60\nG1 X200 Y0 E1 F60\n"
        "; NOZZLE_CHANGE_END OF0 NF1 ON0 NN1\n"
        "; MACHINE_END_GCODE_START\n";
    const auto processed = process_post(gcode, preheat_config(), true);
    CHECK(processed.output.find("Multi extruder pre heating") != std::string::npos);
    CHECK(processed.output.find("S220 N0 ;Multi extruder pre heating") != std::string::npos);
}


TEST_CASE("A completed extruder with a later idle window emits cooling only", "[GCodeProcessor][GCodeProcessorReachable][GcpPreheat]")
{
    // T0 finishes before the long T1 moves, leaving an idle window with no
    // future T0 filament. The processor must cool T0 without scheduling heat.
    const std::string gcode =
        "; MACHINE_START_GCODE_END\nM83\nT0\nG1 X1 Y0 Z0.2 E1 F600\n"
        "; NOZZLE_CHANGE_START OF0 NF1 ON0 NN1\n"
        "; CP_TOOLCHANGE_WIPE CT1 FL0\nT1\n"
        "; NOZZLE_CHANGE_END OF0 NF1 ON0 NN1\n"
        "G1 X150 Y0 E0.5 F60\nG1 X200 Y0 E0.5 F60\n"
        "; MACHINE_END_GCODE_START\n";
    const auto processed = process_post(gcode, preheat_config(), true);
    INFO("cooling-only output: " << processed.output);
    REQUIRE(processed.result.moves.size() > 2);
    CHECK(processed.output.find("M104 T0 S25 N0 ;Multi extruder pre cooling") != std::string::npos);
    CHECK(processed.output.find("Multi extruder pre heating") == std::string::npos);
    CHECK(processed.output.find("M632") == std::string::npos);
    CHECK(processed.output.find("M633") == std::string::npos);
}

TEST_CASE("Preheating relocates a back-solved heat command from a skippable move interval",
          "[GCodeProcessor][GCodeProcessorReachable][GcpPreheat][GCodeProcessorSkippable]")
{
    const std::string gcode =
        "; MACHINE_START_GCODE_END\n"
        "M83\nT0\nG1 X100 Y0 Z0.2 E1 F60\n"
        "; NOZZLE_CHANGE_START OF0 NF1 ON0 NN1\n"
        "; CP_TOOLCHANGE_WIPE CT0 FL0\nT1\n"
        "; SKIPPABLE_START\n; SKIPTYPE: timelapse\nG4 S30\n"
        "G1 X101 Y0 E0.2 F6000\n; SKIPPABLE_END\n"
        "G1 X125 Y0 E1 F60\n"
        "; NOZZLE_CHANGE_END OF0 NF1 ON0 NN1\n"
        "; MACHINE_END_GCODE_START\n";

    const auto processed = process_post(gcode, preheat_config(), true);
    INFO("preheat/skippable output: " << processed.output);

    const size_t skip_start = processed.output.find("; SKIPPABLE_START");
    const size_t skip_end = processed.output.find("; SKIPPABLE_END");
    const size_t heat = processed.output.find("M104 T1 S220 N0 ;Multi extruder pre heating");
    const size_t map_start = processed.output.find("M632 S1 W");
    const size_t map_end = processed.output.find("M633\n");
    const size_t in_skip_move = processed.output.find("G1 X101 Y0 E0.2 F6000");
    const size_t post_skip_move = processed.output.find("G1 X125 Y0 E1 F60");
    REQUIRE(skip_start != std::string::npos);
    REQUIRE(skip_end != std::string::npos);
    REQUIRE(heat != std::string::npos);
    REQUIRE(map_start != std::string::npos);
    REQUIRE(map_end != std::string::npos);
    REQUIRE(in_skip_move != std::string::npos);
    REQUIRE(post_skip_move != std::string::npos);
    CHECK(heat < skip_start);
    CHECK(map_start < skip_start);
    CHECK(map_end < skip_start);
    CHECK(skip_start < in_skip_move);
    CHECK(in_skip_move < skip_end);
    CHECK(skip_end < post_skip_move);
    REQUIRE(processed.result.moves.size() > 2);
}

namespace preheat_variance_contracts {

namespace {

std::string measured_idle_window()
{
    std::string gcode = "; MACHINE_START_GCODE_END\nG90\nM83\nT0\n";
    // Ten-second moves make a thermal scheduling difference observable without
    // depending on exact acceleration rounding or internal time-block indices.
    for (int x = 10; x <= 200; x += 10) {
        if (x == 110)
            gcode += "; NOZZLE_CHANGE_START OF0 NF1 ON0 NN1\n"
                     "; CP_TOOLCHANGE_WIPE CT0 FL0\nT1\n";
        gcode += "G1 X" + std::to_string(x) + " Y0 Z0.2 E0.1 F60\n";
    }
    return gcode + "; NOZZLE_CHANGE_END OF0 NF1 ON0 NN1\n"
                   "; MACHINE_END_GCODE_START\n";
}

struct TemperatureCommand {
    float tool;
    float temperature;
    float preceding_x;
};

std::vector<TemperatureCommand> temperature_commands(const std::string &output)
{
    std::vector<TemperatureCommand> commands;
    GCodeReader reader;
    float last_x = 0.f;
    reader.parse_buffer(output, [&](GCodeReader &, const GCodeReader::GCodeLine &line) {
        if (line.cmd_is("G1") && line.has_x())
            last_x = line.x();
        if (!line.cmd_is("M104"))
            return;
        float tool = 0.f;
        float temperature = 0.f;
        float generated = -1.f;
        REQUIRE(line.has_value('T', tool));
        REQUIRE(line.has_value('S', temperature));
        REQUIRE(line.has_value('N', generated));
        CHECK_THAT(generated, WithinAbs(0.f, 1e-6f));
        commands.push_back({tool, temperature, last_x});
    });
    return commands;
}

TemperatureCommand heating_command(const std::vector<TemperatureCommand> &commands,
                                   float expected_temperature)
{
    const auto matches = [expected_temperature](const TemperatureCommand &command) {
        return std::abs(command.temperature - expected_temperature) < 1e-4f;
    };
    REQUIRE(std::count_if(commands.begin(), commands.end(), matches) == 1);
    return *std::find_if(commands.begin(), commands.end(), matches);
}

float cooling_temperature(const std::vector<TemperatureCommand> &commands)
{
    REQUIRE(commands.size() >= 2);
    return std::min_element(commands.begin(), commands.end(),
        [](const TemperatureCommand &a, const TemperatureCommand &b) {
            return a.temperature < b.temperature;
        })->temperature;
}

} // namespace

TEST_CASE("Preheating uses the next filament temperature delta independently",
          "[GCodeProcessor][GcpPreheat][GcpPreheatVariance]")
{
    const double delta = GENERATE(0., 10., 35.);
    auto config = preheat_config();
    config.filament_preheat_temperature_delta.values = {7., delta};
    const auto processed = process_post(measured_idle_window(), config, true);
    const auto commands = temperature_commands(processed.output);
    const float expected = static_cast<float>(230. - delta);
    const auto heat = heating_command(commands, expected);
    CHECK_THAT(heat.tool, WithinAbs(1.f, 1e-6f));
    CHECK_THAT(cooling_temperature(commands), WithinAbs(25.f, 1e-6f));
    CHECK(heat.preceding_x < 200.f);
}

TEST_CASE("Preheating translates logical extruders through the physical map",
          "[GCodeProcessor][GcpPreheat][GcpPreheatVariance]")
{
    const bool reversed = GENERATE(false, true);
    auto config = preheat_config();
    config.physical_extruder_map.values = reversed ? std::vector<int>{1, 0}
                                                   : std::vector<int>{0, 1};
    const auto processed = process_post(measured_idle_window(), config, true);
    const auto commands = temperature_commands(processed.output);
    const auto heat = heating_command(commands, 220.f);
    CHECK_THAT(heat.tool, WithinAbs(reversed ? 0.f : 1.f, 1e-6f));
    CHECK_THAT(cooling_temperature(commands), WithinAbs(25.f, 1e-6f));
    // Mapping the physical heater must not remap the logical filament grouping.
    check_group_mapping(processed.result);
    for (const auto &command : commands)
        CHECK_THAT(command.tool, WithinAbs(reversed ? 0.f : 1.f, 1e-6f));
}

TEST_CASE("A faster target heater starts later without changing its temperature",
          "[GCodeProcessor][GcpPreheat][GcpPreheatVariance]")
{
    auto slow = preheat_config();
    auto fast = preheat_config();
    auto unrelated = preheat_config();
    slow.hotend_heating_rate.values = {4., 2.};
    fast.hotend_heating_rate.values = {4., 8.};
    unrelated.hotend_heating_rate.values = {1., 8.};
    const std::string gcode = measured_idle_window();
    const auto slow_output = process_post(gcode, slow, true);
    const auto fast_output = process_post(gcode, fast, true);
    const auto unrelated_output = process_post(gcode, unrelated, true);
    const auto slow_heat = heating_command(temperature_commands(slow_output.output), 220.f);
    const auto fast_heat = heating_command(temperature_commands(fast_output.output), 220.f);
    const auto unchanged_heat = heating_command(temperature_commands(unrelated_output.output), 220.f);
    CHECK_THAT(slow_heat.tool, WithinAbs(1.f, 1e-6f));
    CHECK_THAT(fast_heat.tool, WithinAbs(1.f, 1e-6f));
    CHECK(fast_heat.preceding_x > slow_heat.preceding_x);
    CHECK(fast_heat.preceding_x < 200.f);
    CHECK_THAT(unchanged_heat.preceding_x, WithinAbs(fast_heat.preceding_x, 1e-4f));
}

TEST_CASE("A slower target cooler retains more heat during the same idle window",
          "[GCodeProcessor][GcpPreheat][GcpPreheatVariance]")
{
    auto slow = preheat_config();
    auto fast = preheat_config();
    auto unrelated = preheat_config();
    slow.hotend_cooling_rate.values = {4., 0.1};
    fast.hotend_cooling_rate.values = {4., 4.};
    unrelated.hotend_cooling_rate.values = {0.2, 4.};
    const std::string gcode = measured_idle_window();
    const auto slow_output = process_post(gcode, slow, true);
    const auto fast_output = process_post(gcode, fast, true);
    const auto unrelated_output = process_post(gcode, unrelated, true);
    const auto slow_commands = temperature_commands(slow_output.output);
    const auto fast_commands = temperature_commands(fast_output.output);
    const auto unchanged_commands = temperature_commands(unrelated_output.output);
    const float slow_temperature = cooling_temperature(slow_commands);
    const float fast_temperature = cooling_temperature(fast_commands);
    CHECK(slow_temperature > fast_temperature);
    CHECK(slow_temperature < 220.f);
    CHECK_THAT(fast_temperature, WithinAbs(25.f, 1e-6f));
    CHECK_THAT(cooling_temperature(unchanged_commands), WithinAbs(fast_temperature, 1e-6f));
    CHECK_THAT(heating_command(slow_commands, 220.f).tool, WithinAbs(1.f, 1e-6f));
    CHECK_THAT(heating_command(fast_commands, 220.f).tool, WithinAbs(1.f, 1e-6f));
}



std::vector<float> post_extrusion_cooling_temperatures(const std::string &output)
{
    std::vector<float> temperatures;
    GCodeReader reader;
    reader.parse_buffer(output, [&temperatures](GCodeReader &, const GCodeReader::GCodeLine &line) {
        if (!line.cmd_is("M104") || line.raw().find("pre cooling in post extrusion") == std::string::npos)
            return;
        float temperature = 0.f;
        if (line.has_value('S', temperature))
            temperatures.push_back(temperature);
    });
    return temperatures;
}

TEST_CASE("Partial free cooling respects its target and preheating avoids skippable motion",
          "[GCodeProcessor][GcpPreheat][GcpPreheatVariance][GCodeProcessorSkippable]")
{
    std::string gcode =
        "; MACHINE_START_GCODE_END\n"
        "G90\nM83\nT0\nG1 X10 Y0 Z0.2 E0.1 F60\n"
        "; NOZZLE_CHANGE_START OF0 NF1 ON0 NN1\n"
        "; CP_TOOLCHANGE_WIPE CT0 FL0\nG1 X20 Y0 E0.1 F60\n"
        "; NOZZLE_CHANGE_END OF0 NF1 ON0 NN1\n"
        "T1\nG1 X30 Y0 E0.1 F60\n"
        "; NOZZLE_CHANGE_START OF1 NF0 ON1 NN0\n"
        "; CP_TOOLCHANGE_WIPE CT0 FL0\nG1 X90 Y0 E0.1 F60\n"
        "; NOZZLE_CHANGE_END OF1 NF0 ON1 NN0\n"
        "T0\nG1 X150 Y0 E0.1 F60\n"
        "; SKIPPABLE_START\n; SKIPTYPE: timelapse\n";
    for (int x = 151; x <= 210; ++x)
        gcode += "G1 X" + std::to_string(x) + " Y0 E0.1 F60\n";
    gcode +=
        "; SKIPPABLE_END\nG1 X215 Y0 E0.1 F600\n"
        "; NOZZLE_CHANGE_START OF0 NF1 ON0 NN1\n"
        "; CP_TOOLCHANGE_WIPE CT0 FL0\nG1 X220 Y0 E0.1 F600\n"
        "; NOZZLE_CHANGE_END OF0 NF1 ON0 NN1\n"
        "T1\nG1 X225 Y0 E0.1 F60\n"
        "; MACHINE_END_GCODE_START\n";

    auto fast_cooling = preheat_config();
    fast_cooling.filament_pre_cooling_temperature.values = {0, 190};
    fast_cooling.hotend_cooling_rate.values = {4., 4.};
    const auto fast = process_post(gcode, fast_cooling, true);

    auto slow_cooling = fast_cooling;
    slow_cooling.hotend_cooling_rate.values = {4., 0.1};
    const auto slow = process_post(gcode, slow_cooling, true);

    const auto fast_partial = post_extrusion_cooling_temperatures(fast.output);
    const auto slow_partial = post_extrusion_cooling_temperatures(slow.output);
    REQUIRE(fast_partial.size() == 1);
    REQUIRE(slow_partial.size() == 1);
    CHECK_THAT(fast_partial.front(), WithinAbs(190.f, 1e-3f));
    CHECK(slow_partial.front() > fast_partial.front());
    CHECK(slow_partial.front() < 230.f);

    const size_t skip_start = fast.output.find("; SKIPPABLE_START");
    const size_t skip_end = fast.output.find("; SKIPPABLE_END");
    std::vector<size_t> preheat_positions;
    std::vector<size_t> nozzle_map_start_positions;
    std::vector<size_t> nozzle_map_end_positions;
    std::vector<size_t> skipped_move_positions;
    size_t search_offset = 0;
    GCodeReader reader;
    reader.parse_buffer(fast.output, [&](GCodeReader &, const GCodeReader::GCodeLine &line) {
        if (line.raw().empty())
            return;
        const size_t line_position = fast.output.find(line.raw(), search_offset);
        if (line_position == std::string::npos)
            return;
        search_offset = line_position + line.raw().size();

        if (line.cmd_is("M104")) {
            float tool = 0.f;
            float temperature = 0.f;
            float generated = -1.f;
            if (line.has_value('T', tool) && line.has_value('S', temperature) &&
                line.has_value('N', generated) && std::abs(tool - 1.f) < 1e-4f &&
                std::abs(temperature - 220.f) < 1e-4f && std::abs(generated) < 1e-4f &&
                line.comment().find("Multi extruder pre heating") != std::string_view::npos)
                preheat_positions.push_back(line_position);
        } else if (line.cmd_is("M632")) {
            float next_filament = 0.f;
            if (line.has_value('S', next_filament) && std::abs(next_filament - 1.f) < 1e-4f && line.has('W'))
                nozzle_map_start_positions.push_back(line_position);
        } else if (line.cmd_is("M633")) {
            nozzle_map_end_positions.push_back(line_position);
        }
        if (line.cmd_is("G1")) {
            float x = 0.f;
            float y = 0.f;
            float extrusion = 0.f;
            float feedrate = 0.f;
            if (line.has_value('X', x) && line.has_value('Y', y) && line.has_value('E', extrusion) &&
                line.has_value('F', feedrate) && std::abs(x - 200.f) < 1e-4f &&
                std::abs(y) < 1e-4f && std::abs(extrusion - 0.1f) < 1e-4f &&
                std::abs(feedrate - 60.f) < 1e-4f)
                skipped_move_positions.push_back(line_position);
        }
    });
    REQUIRE(skip_start != std::string::npos);
    REQUIRE(skip_end != std::string::npos);
    REQUIRE(preheat_positions.size() == 1);
    REQUIRE(skipped_move_positions.size() == 1);
    const size_t heat = preheat_positions.front();
    const size_t skipped_move = skipped_move_positions.front();
    const auto nozzle_map_start_it = std::find_if(nozzle_map_start_positions.rbegin(), nozzle_map_start_positions.rend(),
        [heat](size_t position) { return position < heat; });
    const auto nozzle_map_end_it = std::find_if(nozzle_map_end_positions.begin(), nozzle_map_end_positions.end(),
        [heat](size_t position) { return position > heat; });
    REQUIRE(nozzle_map_start_it != nozzle_map_start_positions.rend());
    REQUIRE(nozzle_map_end_it != nozzle_map_end_positions.end());
    const size_t nozzle_map_start = *nozzle_map_start_it;
    const size_t nozzle_map_end = *nozzle_map_end_it;
    CHECK(nozzle_map_start < heat);
    CHECK(heat < nozzle_map_end);
    CHECK(heat < skip_start);
    CHECK(nozzle_map_start < skip_start);
    CHECK(nozzle_map_end < skip_start);
    CHECK(skip_start < skipped_move);
    CHECK(skipped_move < skip_end);
}

} // namespace preheat_variance_contracts

namespace gcp_build_volume_contracts {

namespace {

void parse_build_volume_gcode(const std::string &gcode, const FullPrintConfig &config,
                              GCodeProcessor &processor)
{
    const bool previous_bbl = GCodeProcessor::s_IsBBLPrinter;
    const ScopeGuard restore_bbl([previous_bbl] { GCodeProcessor::s_IsBBLPrinter = previous_bbl; });
    GCodeProcessor::s_IsBBLPrinter = false;

    processor.apply_config(config);
    processor.initialize("gcp-build-volume.gcode");
    processor.initialize_result_moves();
    processor.process_buffer(gcode);
}

void require_extrusion_witness(const GCodeProcessorResult &result,
                               const std::set<int> &expected_filaments,
                               size_t minimum_count)
{
    size_t count = 0;
    std::set<int> seen_filaments;
    for (const GCodeProcessorResult::MoveVertex &move : result.moves) {
        if (move.type != EMoveType::Extrude || move.extrusion_role == erCustom)
            continue;
        REQUIRE(std::isfinite(move.delta_extruder));
        REQUIRE(move.delta_extruder > 0.f);
        REQUIRE(move.object_label_id == 7);
        CHECK_THAT(move.print_z, WithinAbs(5.f, 1e-6f));
        ++count;
        seen_filaments.insert(static_cast<int>(move.extruder_id));
    }
    REQUIRE(count >= minimum_count);
    CHECK(seen_filaments == expected_filaments);
}

void require_custom_extrusion_witness(const GCodeProcessorResult &result)
{
    size_t count = 0;
    for (const GCodeProcessorResult::MoveVertex &move : result.moves) {
        if (move.type != EMoveType::Extrude || move.extrusion_role != erCustom)
            continue;
        REQUIRE(std::isfinite(move.delta_extruder));
        REQUIRE(move.delta_extruder > 0.f);
        REQUIRE(move.object_label_id == 7);
        CHECK_THAT(move.print_z, WithinAbs(500.f, 1e-6f));
        CHECK_THAT(move.position.z(), WithinAbs(500.f, 1e-3f));
        ++count;
    }
    REQUIRE(count >= 1);
}

} // namespace

TEST_CASE("The processor reports build-volume errors and clears them on recheck",
          "[GCodeProcessor][GCodeProcessorBuildVolume]")
{
    FullPrintConfig config;
    config.gcode_flavor.value = gcfMarlinFirmware;
    config.filament_diameter.values = {1.75};
    config.filament_map.values = {1};

    const std::string gcode =
        "G21\nG90\nM83\nT0\n"
        "; start printing object: 7\n; Z_HEIGHT:5\n;TYPE:Outer wall\n"
        "G1 X120 Y10 Z5 E1 F600\nG1 X130 Y10 E1\n"
        ";TYPE:Custom\n; Z_HEIGHT:500\nG1 X500 Y500 Z500 E1\n";
    GCodeProcessor processor;
    parse_build_volume_gcode(gcode, config, processor);
    const GCodeProcessorResult &parsed = processor.get_result();
    require_extrusion_witness(parsed, {0}, 2);
    require_custom_extrusion_witness(parsed);

    const Pointfs plate{{0., 0.}, {100., 0.}, {100., 100.}, {0., 100.}};
    const bool invalid = processor.check_multi_extruder_gcode_valid(
        1, plate, 4., Pointfs{}, std::vector<Polygons>(1), std::vector<double>{100.},
        std::vector<int>{1}, std::vector<std::set<int>>(1));
    REQUIRE_FALSE(invalid);
    CHECK(parsed.gcode_check_result.error_code == (4 | 8));
    REQUIRE(parsed.gcode_check_result.print_area_error_infos.count(0) == 1);
    REQUIRE(parsed.gcode_check_result.print_height_error_infos.count(0) == 1);
    const std::pair<int, int> expected_pair{0, 7};
    REQUIRE(parsed.gcode_check_result.print_area_error_infos.at(0).size() == 1);
    REQUIRE(parsed.gcode_check_result.print_height_error_infos.at(0).size() == 1);
    CHECK(parsed.gcode_check_result.print_area_error_infos.at(0).front() == expected_pair);
    CHECK(parsed.gcode_check_result.print_height_error_infos.at(0).front() == expected_pair);

    const Pointfs generous_plate{{0., 0.}, {140., 0.}, {140., 100.}, {0., 100.}};
    const bool valid = processor.check_multi_extruder_gcode_valid(
        1, generous_plate, 100., Pointfs{}, std::vector<Polygons>(1), std::vector<double>{100.},
        std::vector<int>{1}, std::vector<std::set<int>>(1));
    REQUIRE(valid);
    CHECK(parsed.gcode_check_result.error_code == 0);
    CHECK(parsed.gcode_check_result.print_area_error_infos.empty());
    CHECK(parsed.gcode_check_result.print_height_error_infos.empty());
    CHECK(parsed.limit_filament_maps == std::vector<int>{0});
}

TEST_CASE("The processor maps multi-extruder area, height and material restrictions",
          "[GCodeProcessor][GCodeProcessorBuildVolume]")
{
    FullPrintConfig config;
    config.gcode_flavor.value = gcfMarlinFirmware;
    config.filament_diameter.values = {1.75, 1.75};
    config.filament_map.values = {2, 1};

    const std::string gcode =
        "G21\nG90\nM83\nT0\n"
        "; start printing object: 7\n; Z_HEIGHT:5\n;TYPE:Outer wall\n"
        "G1 X10 Y10 Z5 E1 F600\nG1 X20 Y10 E1\nT1\n"
        "G1 X30 Y10 E1\nG1 X40 Y10 E1\n"
        ";TYPE:Custom\n; Z_HEIGHT:500\nG1 X500 Y500 Z500 E1\n";
    GCodeProcessor processor;
    parse_build_volume_gcode(gcode, config, processor);
    const GCodeProcessorResult &parsed = processor.get_result();
    require_extrusion_witness(parsed, {0, 1}, 4);
    require_custom_extrusion_witness(parsed);

    const Pointfs plate{{-100., -100.}, {100., -100.}, {100., 100.}, {-100., 100.}};
    std::vector<Polygons> unprintable_areas(2);
    unprintable_areas[1].emplace_back(Polygon::new_scale(
        Pointfs{{8., 8.}, {22., 8.}, {22., 12.}, {8., 12.}}));
    std::vector<std::set<int>> unprintable_filament_types(2);
    unprintable_filament_types[0].insert(1);

    const bool valid = processor.check_multi_extruder_gcode_valid(
        2, plate, 100., Pointfs{}, unprintable_areas, std::vector<double>{100., 4.},
        std::vector<int>{2, 1}, unprintable_filament_types);
    REQUIRE_FALSE(valid);
    CHECK(parsed.gcode_check_result.error_code == (1 | 2));
    REQUIRE(parsed.gcode_check_result.print_area_error_infos.count(1) == 1);
    REQUIRE(parsed.gcode_check_result.print_height_error_infos.count(1) == 1);
    REQUIRE(parsed.gcode_check_result.print_area_error_infos.at(1).size() == 1);
    REQUIRE(parsed.gcode_check_result.print_height_error_infos.at(1).size() == 1);
    const std::pair<int, int> expected_pair{0, 7};
    CHECK(parsed.gcode_check_result.print_area_error_infos.at(1).front() == expected_pair);
    CHECK(parsed.gcode_check_result.print_height_error_infos.at(1).front() == expected_pair);
    CHECK(parsed.gcode_check_result.print_area_error_infos.count(0) == 0);
    CHECK(parsed.gcode_check_result.print_height_error_infos.count(0) == 0);
    REQUIRE(parsed.limit_filament_maps.size() == 2);
    CHECK(parsed.limit_filament_maps[0] == 2);
    CHECK(parsed.limit_filament_maps[1] == 1);
}

} // namespace gcp_build_volume_contracts


} // namespace gcode_processor_reachable_contracts

namespace silent_mode_variance_contracts {

FullPrintConfig config_with_silent_mode(bool enabled)
{
    FullPrintConfig config;
    config.gcode_flavor.value = gcfMarlinFirmware;
    config.silent_mode.value = enabled;
    config.machine_max_speed_x.values = {500., 200.};
    config.machine_max_speed_y.values = {500., 200.};
    config.machine_max_speed_z.values = {12., 12.};
    config.machine_max_speed_e.values = {120., 120.};
    config.machine_max_acceleration_x.values = {1000., 200.};
    config.machine_max_acceleration_y.values = {1000., 1000.};
    config.machine_max_acceleration_z.values = {500., 200.};
    config.machine_max_acceleration_e.values = {5000., 5000.};
    config.machine_max_acceleration_extruding.values = {1500., 1250.};
    config.machine_max_acceleration_retracting.values = {1500., 1250.};
    config.machine_max_acceleration_travel.values = {0., 0.};
    return config;
}

} // namespace silent_mode_variance_contracts

TEST_CASE("Imported G-code ignores the obsolete silent_mode setting",
          "[GCodeProcessor][GCodeProcessorPublic][SilentModeVariance]")
{
    const std::string gcode =
        "G90\nM83\n"
        "G1 X100 Y0 Z0.2 E10 F600\n"
        "G1 X0 Y0 E10 F600\n";

    const auto normal = public_processor_contracts::process_file(
        public_processor_contracts::orca_header(silent_mode_variance_contracts::config_with_silent_mode(false)) + gcode);
    const auto silent = public_processor_contracts::process_file(
        public_processor_contracts::orca_header(silent_mode_variance_contracts::config_with_silent_mode(true)) + gcode);

    REQUIRE(public_processor_contracts::normal_time(normal) > 0.0f);
    REQUIRE(public_processor_contracts::normal_time(silent) > 0.0f);
    CHECK_THAT(public_processor_contracts::normal_time(silent),
               WithinAbs(public_processor_contracts::normal_time(normal), 1e-6f));
    CHECK_THAT(public_processor_contracts::stealth_time(normal), WithinAbs(0.0f, 1e-6f));
    // The G-code importer applies PrintConfig's legacy-key filter, which
    // discards silent_mode even though the option remains registered.
    CHECK_THAT(public_processor_contracts::stealth_time(silent), WithinAbs(0.0f, 1e-6f));
}
