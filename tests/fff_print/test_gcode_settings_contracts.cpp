#include <catch2/catch_all.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "catch2/catch_test_macros.hpp"
#include "catch2/generators/catch_generators.hpp"
#include "catch2/matchers/catch_matchers.hpp"
#include "catch2/catch_message.hpp"
#include "libslic3r/GCodeReader.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/TriangleMesh.hpp"

#include "test_helpers.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <exception>
#include <libslic3r/Config.hpp>
#include <libslic3r/libslic3r.h>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>
#include <catch2/catch_message.hpp>
#include <catch2/catch_test_macros.hpp>
#include "libslic3r/Config.hpp"
#include "libslic3r/libslic3r.h"
#include <set>

using namespace Slic3r;
using namespace Slic3r::Test;

namespace {

// A small, solid fixture still has enough perimeter and travel moves to exercise
// the exporter while keeping each settings variant cheap to process.
DynamicPrintConfig exporter_config(GCodeFlavor flavor, bool relative_e, bool comments)
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        { "layer_height",             0.2 },
        { "initial_layer_print_height", 0.2 },
        { "nozzle_diameter",          0.4 },
        { "wall_loops",               2 },
        { "top_shell_layers",         0 },
        { "bottom_shell_layers",      0 },
        { "top_shell_thickness",      0 },
        { "bottom_shell_thickness",   0 },
        { "sparse_infill_density",    "20%" },
        { "z_hop",                    0 },
    });
    config.set_key_value("gcode_flavor", new ConfigOptionEnum<GCodeFlavor>(flavor));
    config.set_key_value("use_relative_e_distances", new ConfigOptionBool(relative_e));
    config.set_key_value("gcode_comments", new ConfigOptionBool(comments));
    config.set_deserialize_strict("layer_change_gcode", relative_e ? "G92 E0\n" : "");
    return config;
}

std::string export_cube(const DynamicPrintConfig &config)
{
    Model model;
    Print print;
    TriangleMesh shape = cube(8.0);
    shape.scale(Vec3f(1.0f, 1.0f, 0.25f));

    ModelObject *object = model.add_object();
    object->name = "gcode-settings-contract.stl";
    object->add_volume(std::move(shape));
    object->add_instance();
    object->ensure_on_bed();
    print.auto_assign_extruders(object);
    print.apply(model, config);

    REQUIRE(print.validate().string.empty());
    return Test::gcode(print);
}

struct Motion {
    std::string command;
    double dx = 0.0;
    double dy = 0.0;
    double dz = 0.0;
    double de = 0.0;
    bool has_feedrate = false;
    double feedrate = 0.0;
};

std::vector<Motion> normalized_motions(const std::string &gcode, const DynamicPrintConfig &config)
{
    std::vector<Motion> motions;
    GCodeReader reader;
    reader.apply_config(config);
    reader.parse_buffer(gcode, [&motions](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        if (!(line.cmd_is("G0") || line.cmd_is("G1") || line.cmd_is("G2") || line.cmd_is("G3")))
            return;

        Motion motion;
        motion.command = std::string(line.cmd());
        motion.dx = line.dist_X(self);
        motion.dy = line.dist_Y(self);
        motion.dz = line.dist_Z(self);
        motion.de = line.dist_E(self);
        motion.has_feedrate = line.has_f();
        motion.feedrate = line.f();
        motions.emplace_back(std::move(motion));
    });
    return motions;
}

struct DepositionTotals {
    double filament_length = 0.0;
    double path_length = 0.0;
    size_t moves = 0;
};

DepositionTotals deposition_totals(const std::string &gcode, const DynamicPrintConfig &config)
{
    DepositionTotals totals;
    GCodeReader reader;
    reader.apply_config(config);
    reader.parse_buffer(gcode, [&totals](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        if (!line.extruding(self) || line.dist_XY(self) <= EPSILON)
            return;
        totals.filament_length += line.dist_E(self);
        totals.path_length += line.dist_XY(self);
        ++totals.moves;
    });
    return totals;
}

std::optional<int> command_value(const std::string &line, std::string_view command, char axis)
{
    std::istringstream words(line);
    std::string word;
    if (!(words >> word) || word != command)
        return std::nullopt;

    while (words >> word) {
        if (word.size() < 2 || word.front() != axis)
            continue;
        try {
            size_t parsed = 0;
            const int value = std::stoi(word.substr(1), &parsed);
            if (parsed == word.size() - 1)
                return value;
        } catch (const std::exception &) {
            return std::nullopt;
        }
    }
    return std::nullopt;
}

std::vector<int> temperature_commands(const std::string &gcode)
{
    std::vector<int> result;
    std::istringstream lines(gcode);
    for (std::string line; std::getline(lines, line);) {
        for (const std::string_view command : { std::string_view("M104"), std::string_view("M109") }) {
            if (const auto value = command_value(line, command, 'S')) {
                result.push_back(*value);
                break;
            }
        }
    }
    return result;
}

std::vector<int> fan_commands(const std::string &gcode)
{
    std::vector<int> result;
    std::istringstream lines(gcode);
    for (std::string line; std::getline(lines, line);) {
        if (const auto value = command_value(line, "M106", 'S'))
            result.push_back(*value);
        else {
            std::istringstream words(line);
            std::string command;
            if (words >> command && command == "M107")
                result.push_back(0);
        }
    }
    return result;
}

bool contains_value(const std::vector<int> &values, int expected)
{
    return std::find(values.begin(), values.end(), expected) != values.end();
}

} // namespace

TEST_CASE("Relative and absolute extrusion preserve deposited motion across firmware flavors",
          "[GCodeSettings]")
{
    const GCodeFlavor flavor = GENERATE(gcfMarlinFirmware, gcfKlipper, gcfRepRapFirmware);
    DYNAMIC_SECTION("firmware flavor " << static_cast<int>(flavor)) {
        DynamicPrintConfig relative_config = exporter_config(flavor, true, true);
        DynamicPrintConfig absolute_config = exporter_config(flavor, false, true);

        const std::string relative_gcode = export_cube(relative_config);
        const std::string absolute_gcode = export_cube(absolute_config);
        const DepositionTotals relative = deposition_totals(relative_gcode, relative_config);
        const DepositionTotals absolute = deposition_totals(absolute_gcode, absolute_config);

        REQUIRE(relative.moves > 0);
        REQUIRE(absolute.moves > 0);
        CHECK(relative_gcode.find("M83") != std::string::npos);
        CHECK(absolute_gcode.find("M82") != std::string::npos);
        // E words are emitted at five decimal places. Relative mode rounds each
        // segment's extrusion increment, while absolute mode rounds cumulative E;
        // summing parsed moves therefore has a bounded quantization difference.
        CHECK_THAT(relative.filament_length,
                   Catch::Matchers::WithinAbs(absolute.filament_length, 1e-3));
        CHECK_THAT(relative.path_length,
                   Catch::Matchers::WithinRel(absolute.path_length, 1e-6));

        // The E values are deliberately interpreted through each file's mode;
        // comparing raw E words would confuse addressing format with deposited material.
        REQUIRE(relative.filament_length > 0.0);
        REQUIRE(absolute.filament_length > 0.0);
        REQUIRE(relative.path_length > 0.0);
        REQUIRE(absolute.path_length > 0.0);
    }
}

TEST_CASE("Disabling G-code comments preserves the normalized motion stream", "[GCodeSettings]")
{
    DynamicPrintConfig with_comments_config = exporter_config(gcfMarlinFirmware, true, true);
    DynamicPrintConfig without_comments_config = exporter_config(gcfMarlinFirmware, true, false);
    with_comments_config.set_deserialize_strict("default_acceleration", "1200");
    without_comments_config.set_deserialize_strict("default_acceleration", "1200");
    const std::string with_comments = export_cube(with_comments_config);
    const std::string without_comments = export_cube(without_comments_config);

    REQUIRE(with_comments != without_comments);
    CHECK(with_comments.find(" ; adjust acceleration") != std::string::npos);
    CHECK(without_comments.find(" ; adjust acceleration") == std::string::npos);

    const std::vector<Motion> expected = normalized_motions(with_comments, with_comments_config);
    const std::vector<Motion> actual = normalized_motions(without_comments, without_comments_config);
    REQUIRE_FALSE(expected.empty());
    REQUIRE(expected.size() == actual.size());
    for (size_t i = 0; i < expected.size(); ++i) {
        INFO("motion index = " << i);
        CHECK(expected[i].command == actual[i].command);
        CHECK_THAT(expected[i].dx, Catch::Matchers::WithinAbs(actual[i].dx, 1e-5));
        CHECK_THAT(expected[i].dy, Catch::Matchers::WithinAbs(actual[i].dy, 1e-5));
        CHECK_THAT(expected[i].dz, Catch::Matchers::WithinAbs(actual[i].dz, 1e-5));
        CHECK_THAT(expected[i].de, Catch::Matchers::WithinAbs(actual[i].de, 1e-5));
        CHECK(expected[i].has_feedrate == actual[i].has_feedrate);
        if (expected[i].has_feedrate)
            CHECK_THAT(expected[i].feedrate, Catch::Matchers::WithinAbs(actual[i].feedrate, 1e-5));
    }
}

TEST_CASE("Exporter settings emit thermal cooling and machine limit commands", "[GCodeSettings]")
{
    DynamicPrintConfig config = exporter_config(gcfMarlinFirmware, true, true);
    config.set_deserialize_strict({
        { "slow_down_for_layer_cooling",    true },
        { "nozzle_temperature_initial_layer", 211 },
        { "nozzle_temperature",            203 },
        { "initial_layer_fan_speed",       0 },
        { "full_fan_speed_layer",          2 },
        { "close_fan_the_first_x_layers",  0 },
        { "fan_min_speed",                 100 },
        { "fan_max_speed",                 100 },
        { "emit_machine_limits_to_gcode",  true },
        { "machine_max_acceleration_x",    "1000,1000" },
        { "machine_max_acceleration_y",    "1100,1100" },
        { "machine_max_acceleration_z",    "100,100" },
        { "machine_max_acceleration_e",    "5000,5000" },
        { "machine_max_acceleration_extruding",  "1200,1200" },
        { "machine_max_acceleration_retracting", "1400,1400" },
        { "machine_max_acceleration_travel", "1600,1600" },
        { "machine_max_speed_x",            "100,100" },
        { "machine_max_speed_y",            "110,110" },
        { "machine_max_speed_z",            "10,10" },
        { "machine_max_speed_e",            "50,50" },
    });

    const std::string gcode = export_cube(config);
    const std::vector<int> temperatures = temperature_commands(gcode);
    const std::vector<int> fans = fan_commands(gcode);

    REQUIRE(contains_value(temperatures, 211));
    REQUIRE(contains_value(temperatures, 203));
    REQUIRE(contains_value(fans, 0));
    REQUIRE(contains_value(fans, 255));
    CHECK(gcode.find("M201 X1000 Y1100 Z100 E5000") != std::string::npos);
    CHECK(gcode.find("M203 X100 Y110 Z10 E50") != std::string::npos);
}

using namespace Slic3r;
using namespace Slic3r::Test;

namespace initial_layer_fan_speed_contracts {
namespace {

constexpr int kNormalCoolingPercent = 80;
constexpr std::size_t kMaxGCodeBytes = 4u * 1024u * 1024u;
constexpr std::size_t kMaxGCodeLines = 100000u;

DynamicPrintConfig initial_layer_fan_config(int initial_layer_fan_speed)
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        {"layer_height", 0.2},
        {"initial_layer_print_height", 0.2},
        {"nozzle_diameter", 0.4},
        {"wall_loops", 2},
        {"top_shell_layers", 0},
        {"bottom_shell_layers", 0},
        {"sparse_infill_density", "20%"},
        {"minimum_sparse_infill_area", 0.0},
        {"skirt_loops", 0},
        {"brim_type", "no_brim"},
        {"raft_layers", 0},
        {"enable_support", false},
        {"gcode_comments", true},
        {"gcode_flavor", "marlin"},
        {"use_relative_e_distances", true},
        {"layer_change_gcode", "G92 E0\n"},
        {"close_fan_the_first_x_layers", 0},
        {"full_fan_speed_layer", 0},
        {"fan_min_speed", 25},
        {"fan_max_speed", kNormalCoolingPercent},
        {"slow_down_layer_time", 1000.0},
        {"fan_cooling_layer_time", 1001.0},
        {"slow_down_for_layer_cooling", false},
        {"reduce_fan_stop_start_freq", false},
        {"part_cooling_fan_min_pwm", 0},
        {"initial_layer_fan_speed", initial_layer_fan_speed}
    });
    return config;
}

struct DepositionEvidence {
    double path_length = 0.0;
    double filament_length = 0.0;
    std::size_t extrusion_moves = 0;
    bool finite = true;
};

struct FanEvidence {
    DepositionEvidence deposition;
    std::set<int> first_layer_deposition_fan_states;
    std::set<int> second_layer_deposition_fan_states;
    std::size_t layer_changes = 0;
};

std::string export_four_layer_cube(const DynamicPrintConfig &config)
{
    Print print;
    Model model;
    TriangleMesh shape = cube(8.0);
    shape.scale(Vec3f(1.0f, 1.0f, 0.1f));
    init_print({shape}, print, model, config);
    const StringObjectException validation = print.validate();
    INFO("validation.string: " << validation.string);
    INFO("validation.opt_key: " << validation.opt_key);
    REQUIRE(validation.string.empty());

    const std::string output = gcode(print);
    REQUIRE_FALSE(output.empty());
    REQUIRE(output.size() <= kMaxGCodeBytes);
    REQUIRE(std::count(output.begin(), output.end(), '\n') < kMaxGCodeLines);
    return output;
}

std::optional<int> fan_pwm_command(const std::string &line)
{
    std::istringstream words(line);
    std::string command;
    if (!(words >> command))
        return std::nullopt;
    if (command == "M107")
        return 0;
    if (command != "M106")
        return std::nullopt;

    std::optional<int> pwm;
    for (std::string word; words >> word;) {
        if (word.size() < 2)
            continue;
        if (word.front() == 'P')
            return std::nullopt; // P2/P3 are auxiliary/exhaust fans, not part cooling.
        if (word.front() != 'S')
            continue;
        try {
            std::size_t parsed = 0;
            const int value = std::stoi(word.substr(1), &parsed);
            if (parsed == word.size() - 1)
                pwm = value;
        } catch (...) {
            return std::nullopt;
        }
    }
    return pwm;
}

FanEvidence inspect_four_layer_cube(const std::string &gcode, const DynamicPrintConfig &config)
{
    FanEvidence evidence;
    GCodeReader reader;
    reader.apply_config(config);
    std::optional<int> part_fan_state;
    reader.parse_buffer(gcode, [&evidence, &part_fan_state](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        if (line.raw().rfind(";LAYER_CHANGE", 0) == 0)
            ++evidence.layer_changes;
        if (const auto pwm = fan_pwm_command(line.raw()); pwm.has_value())
            part_fan_state = *pwm;
        if (!line.extruding(self) || line.dist_XY(self) <= EPSILON)
            return;
        const double path = line.dist_XY(self);
        const double filament = line.dist_E(self);
        evidence.deposition.finite = evidence.deposition.finite && std::isfinite(path) && std::isfinite(filament);
        if (!std::isfinite(path) || !std::isfinite(filament))
            return;
        evidence.deposition.path_length += path;
        evidence.deposition.filament_length += filament;
        ++evidence.deposition.extrusion_moves;
        // Fan commands persist until replaced. Observe state during actual
        // deposition: a stable fan need not emit a new command on layer two.
        if (evidence.layer_changes == 1 || evidence.layer_changes == 2) {
            REQUIRE(part_fan_state.has_value());
            if (evidence.layer_changes == 1)
                evidence.first_layer_deposition_fan_states.insert(*part_fan_state);
            else
                evidence.second_layer_deposition_fan_states.insert(*part_fan_state);
        }
    });
    return evidence;
}

int pwm_for_percent(int percent)
{
    if (percent <= 0)
        return 0;
    return static_cast<int>(255.5 * static_cast<double>(percent) / 100.0);
}

void require_valid_evidence(const FanEvidence &evidence)
{
    REQUIRE(evidence.deposition.finite);
    REQUIRE(evidence.layer_changes == 4);
    REQUIRE(evidence.deposition.extrusion_moves > 0);
    REQUIRE(evidence.deposition.path_length > 0.0);
    REQUIRE(evidence.deposition.filament_length > 0.0);
    REQUIRE_FALSE(evidence.first_layer_deposition_fan_states.empty());
    REQUIRE_FALSE(evidence.second_layer_deposition_fan_states.empty());
}

void require_same_deposition(const DepositionEvidence &expected, const DepositionEvidence &observed)
{
    REQUIRE(expected.extrusion_moves > 0);
    REQUIRE(observed.extrusion_moves > 0);
    CHECK(observed.extrusion_moves == expected.extrusion_moves);
    CHECK_THAT(observed.path_length, Catch::Matchers::WithinRel(expected.path_length, 1e-6));
    CHECK_THAT(observed.filament_length, Catch::Matchers::WithinAbs(expected.filament_length, 1e-3));
}

} // namespace
} // namespace initial_layer_fan_speed_contracts

TEST_CASE("Initial-layer fan values override only the first layer and preserve deposition",
          "[GCodeSettings][InitialLayerFanSpeed][SettingsVariance]")
{
    using namespace initial_layer_fan_speed_contracts;
    const int initial_layer_fan_speed = GENERATE(-1, 0, 37, 100);
    CAPTURE(initial_layer_fan_speed);

    const DynamicPrintConfig baseline_config = initial_layer_fan_config(-1);
    const DynamicPrintConfig variant_config = initial_layer_fan_config(initial_layer_fan_speed);
    const FanEvidence baseline = inspect_four_layer_cube(
        export_four_layer_cube(baseline_config), baseline_config);
    const FanEvidence variant = inspect_four_layer_cube(
        export_four_layer_cube(variant_config), variant_config);
    require_valid_evidence(baseline);
    require_valid_evidence(variant);
    require_same_deposition(baseline.deposition, variant.deposition);

    const std::set<int> normal_state{pwm_for_percent(kNormalCoolingPercent)};
    const std::set<int> expected_first_state{
        initial_layer_fan_speed < 0 ? pwm_for_percent(kNormalCoolingPercent)
                                    : pwm_for_percent(initial_layer_fan_speed)};
    CHECK(baseline.first_layer_deposition_fan_states == normal_state);
    CHECK(baseline.second_layer_deposition_fan_states == normal_state);
    CHECK(variant.first_layer_deposition_fan_states == expected_first_state);
    CHECK(variant.second_layer_deposition_fan_states == normal_state);
    if (initial_layer_fan_speed >= 0)
        CHECK(variant.first_layer_deposition_fan_states != baseline.first_layer_deposition_fan_states);
}
