#include <array>
#include <utility>
#include <vector>
#include <boost/nowide/fstream.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <catch2/catch_message.hpp>
#include "libslic3r/CommonDefs.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PresetBundle.hpp"
#include "libslic3r/Preset.hpp"
#include <catch2/generators/catch_generators.hpp>
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


namespace processor_dynamic_filament_map_contracts {

TEST_CASE("Static nozzle grouping preserves a complete second-extruder-only map",
          "[GCodeProcessor][GCodeProcessorPublic][FilamentMapIdentity]")
{
    FullPrintConfig config = full_fixture_config();
    config.filament_colour.values = {"#112233", "#445566"};
    config.filament_map.values = {2, 2};
    GCodeProcessor processor;
    processor.apply_config(config);
    processor.init_filament_maps_and_nozzle_type_when_import_only_gcode();
    processor.ensure_nozzle_group_result(2);
    const auto *group = dynamic_cast<const MultiNozzleUtils::LayeredNozzleGroupResult *>(
        processor.get_result().nozzle_group_result.get());
    REQUIRE(group != nullptr);
    REQUIRE(group->get_extruder_map().size() == 2);
    CHECK(group->get_extruder_map()[0] == 1);
    CHECK(group->get_extruder_map()[1] == 1);
    CHECK(group->get_extruder_count() == 2);
}

TEST_CASE("Orca import preserves explicit trailing and second-extruder-only maps",
          "[GCodeProcessor][GCodeProcessorPublic][FilamentMapIdentity]")
{
    const bool previous_bbl = GCodeProcessor::s_IsBBLPrinter;
    const ScopeGuard restore([previous_bbl] { GCodeProcessor::s_IsBBLPrinter = previous_bbl; });
    GCodeProcessor::s_IsBBLPrinter = true;
    const bool trailing_entry = GENERATE(true, false);
    FullPrintConfig full = full_fixture_config();
    // Match colour/map lengths so the public loader retains both explicit entries.
    full.filament_colour.values = {"#112233", "#445566"};
    full.filament_map.values = {2, trailing_entry ? 1 : 2};
    if (trailing_entry)
        full.filament_diameter.values = {1.75};
    DynamicPrintConfig config;
    config.apply(full);
    std::string input = "; generated by OrcaSlicer\n; CONFIG_BLOCK_START\n";
    for (const auto &key : config.keys()) {
        const auto *option = config.option(key);
        REQUIRE(option != nullptr);
        input += "; " + key + " = " + option->serialize() + "\n";
    }
    // Getter-only import: the one-diameter row must never execute a T1 move.
    input += "; CONFIG_BLOCK_END\nG90\n";
    REQUIRE(input.size() <= 512 * 1024);
    ScopedTemporaryFile file(".gcode");
    {
        std::ofstream output(file.string());
        REQUIRE(output.good());
        output << input;
        output.flush();
        REQUIRE(output.good());
    }
    GCodeProcessor processor;
    processor.process_file(file.string());
    const auto &result = processor.get_result();
    CHECK(result.filaments_count == (trailing_entry ? 1u : 2u));
    const auto *group = dynamic_cast<const MultiNozzleUtils::LayeredNozzleGroupResult *>(
        result.nozzle_group_result.get());
    REQUIRE(group != nullptr);
    REQUIRE(group->get_extruder_map().size() == 2);
    CHECK(group->get_extruder_map()[0] == 1);
    CHECK(group->get_extruder_map()[1] == (trailing_entry ? 0 : 1));
    CHECK(group->get_extruder_count() == 2);
}

} // namespace processor_dynamic_filament_map_contracts

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
    const bool previous_bbl = GCodeProcessor::s_IsBBLPrinter;
    const ScopeGuard restore_bbl([previous_bbl] { GCodeProcessor::s_IsBBLPrinter = previous_bbl; });
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

TEST_CASE("Nullable nozzle type resolves through printer presets before G-code import",
          "[GCodeProcessor][NozzleTypeSettings][GCodeProcessorPublic]")
{
    namespace fs = boost::filesystem;
    ScopedTemporaryDir temp("nullable-nozzle-preset");
    PresetBundle bundle;
    bundle.filament_presets = {bundle.filaments.default_preset().name};

    const std::string parent_name = "Nullable Nozzle Parent";
    const fs::path machine_dir = temp.path() / PRESET_PRINTER_NAME;
    const fs::path parent_file = machine_dir / "base" / (parent_name + ".json");
    fs::create_directories(parent_file.parent_path());

    DynamicPrintConfig parent_config = bundle.printers.default_preset().config;
    auto *parent_nozzle = parent_config.option<ConfigOptionEnumsGenericNullable>("nozzle_type");
    REQUIRE(parent_nozzle != nullptr);
    parent_nozzle->values = {int(ntHardenedSteel)};
    parent_config.option<ConfigOptionFloats>("nozzle_diameter")->values = {0.4};
    parent_config.option<ConfigOptionInt>("nozzle_hrc")->value = 61;
    parent_config.option<ConfigOptionString>("inherits", true)->value.clear();
    parent_config.option<ConfigOptionString>("printer_settings_id", true)->value = parent_name;
    parent_config.save_to_json(parent_file.string(), parent_name, "User", "1.0.0");
    REQUIRE(fs::is_regular_file(parent_file));

    PresetsConfigSubstitutions substitutions;
    bundle.printers.load_presets(temp.path().string(), PRESET_PRINTER_NAME, substitutions,
                                 ForwardCompatibilitySubstitutionRule::EnableSilent);
    REQUIRE(bundle.printers.error_count() == 0);
    const Preset *loaded_parent = bundle.printers.find_preset(parent_name);
    REQUIRE(loaded_parent != nullptr);
    const auto *loaded_parent_nozzle = loaded_parent->config.option<ConfigOptionEnumsGenericNullable>("nozzle_type");
    REQUIRE(loaded_parent_nozzle != nullptr);
    REQUIRE(loaded_parent_nozzle->values.size() == 1);
    REQUIRE_FALSE(loaded_parent_nozzle->is_nil(0));
    CHECK(loaded_parent_nozzle->values[0] == int(ntHardenedSteel));

    const auto write_child = [&](const std::string &name, NozzleType type, bool nil_value, bool inherits_parent) {
        DynamicPrintConfig child_config = inherits_parent ? parent_config : bundle.printers.default_preset().config;
        child_config.option<ConfigOptionString>("inherits", true)->value = inherits_parent ? parent_name : "";
        child_config.option<ConfigOptionString>("printer_settings_id", true)->value = name;
        child_config.option<ConfigOptionInt>("nozzle_hrc")->value = 61;
        auto *nozzle = child_config.option<ConfigOptionEnumsGenericNullable>("nozzle_type");
        REQUIRE(nozzle != nullptr);
        nozzle->values = {int(type)};
        if (nil_value)
            nozzle->set_at_to_nil(0);
        REQUIRE(nozzle->is_nil(0) == nil_value);
        const fs::path file = machine_dir / (name + ".json");
        child_config.save_to_json(file.string(), name, "User", "1.0.0");
        REQUIRE(fs::is_regular_file(file));
    };

    const std::string inherited_name = "Nullable Nozzle Child";
    const std::string explicit_name = "Explicit Brass Nozzle Child";
    const std::string root_nil_name = "Root Nil Nozzle";
    write_child(inherited_name, ntHardenedSteel, true, true);
    write_child(explicit_name, ntBrass, false, true);
    write_child(root_nil_name, ntUndefine, true, false);

    bundle.printers.load_presets(temp.path().string(), PRESET_PRINTER_NAME, substitutions,
                                 ForwardCompatibilitySubstitutionRule::EnableSilent);
    REQUIRE(bundle.printers.error_count() == 0);

    struct Row {
        const char *name;
        const char *parent;
        NozzleType expected;
    };
    const std::array<Row, 3> rows = {{
        {inherited_name.c_str(), parent_name.c_str(), ntHardenedSteel},
        {explicit_name.c_str(), parent_name.c_str(), ntBrass},
        {root_nil_name.c_str(), "", ntUndefine},
    }};

    for (const Row &row : rows) {
        INFO("printer preset=" << row.name);
        REQUIRE(bundle.printers.select_preset_by_name(row.name, true));
        const Preset &selected = bundle.printers.get_edited_preset();
        CHECK(selected.inherits() == row.parent);

        const auto *selected_nozzle = selected.config.option<ConfigOptionEnumsGenericNullable>("nozzle_type");
        REQUIRE(selected_nozzle != nullptr);
        REQUIRE(selected_nozzle->values.size() == 1);
        REQUIRE_FALSE(selected_nozzle->is_nil(0));
        CHECK(selected_nozzle->values[0] == int(row.expected));

        DynamicPrintConfig resolved = bundle.full_config(false);
        const auto *resolved_nozzle = resolved.option<ConfigOptionEnumsGenericNullable>("nozzle_type");
        const auto *resolved_diameter = resolved.option<ConfigOptionFloats>("nozzle_diameter");
        REQUIRE(resolved_nozzle != nullptr);
        REQUIRE(resolved_diameter != nullptr);
        REQUIRE(resolved_nozzle->values.size() == 1);
        REQUIRE(resolved_diameter->values.size() == 1);
        REQUIRE_FALSE(resolved_nozzle->is_nil(0));
        CHECK(resolved_nozzle->values[0] == int(row.expected));

        FullPrintConfig processor_config;
        // Printer bundles also contain host options outside FullPrintConfig.
        processor_config.apply(resolved, true);
        // The time planner needs both a travel block and an extrusion block.
        const auto result = process_file(orca_header(processor_config) +
            "M83\nG90\nG1 X0 Y0 Z0.2 F600\nG1 X10 Y0 E1 F600\n");
        REQUIRE(result.nozzle_type.size() == 1);
        CHECK(result.nozzle_type[0] == row.expected);
        CHECK(result.nozzle_hrc == 61);
        const auto extrusions = moves_of(result, EMoveType::Extrude);
        REQUIRE(extrusions.size() == 1);
        CHECK_THAT(extrusions.front()->delta_extruder, WithinAbs(1., 1e-4));
        CHECK(normal_time(result) > 0.f);
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

} // namespace public_processor_contracts
