#include <catch2/catch_message.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "libslic3r/GCode/FanMover.hpp"
#include "libslic3r/GCodeReader.hpp"
#include "libslic3r/GCodeWriter.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/libslic3r.h"

#include <cmath>
#include <cstddef>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace Slic3r {
namespace {

struct ParsedLine {
    std::string raw;
    std::string command;
    bool has_x = false;
    bool has_y = false;
    bool has_z = false;
    bool has_e = false;
    float start_x = 0.f;
    float start_y = 0.f;
    float start_z = 0.f;
    float x = 0.f;
    float y = 0.f;
    float z = 0.f;
    float e = 0.f;
    float feed_mm_min = 0.f;
    float cumulative_e = 0.f;
};

std::vector<ParsedLine> parse_output(const std::string &gcode, const GCodeConfig &config)
{
    GCodeReader reader;
    reader.apply_config(config);

    float cumulative_e = 0.f;
    const bool relative_e = config.use_relative_e_distances.value;
    std::vector<ParsedLine> lines;
    reader.parse_buffer(gcode, [&lines, &cumulative_e, relative_e](
                            GCodeReader &reader, const GCodeReader::GCodeLine &line) {
        ParsedLine parsed;
        parsed.raw = line.raw();
        parsed.command = std::string(line.cmd());
        if (line.cmd_is("G1")) {
            parsed.has_x = line.has(Axis::X);
            parsed.has_y = line.has(Axis::Y);
            parsed.has_z = line.has(Axis::Z);
            parsed.has_e = line.has(Axis::E);
            // GCodeReader calls this callback before applying the current line, so its position
            // is the actual move start, including preceding G0 moves.
            parsed.start_x = reader.x();
            parsed.start_y = reader.y();
            parsed.start_z = reader.z();
            parsed.x = line.new_X(reader);
            parsed.y = line.new_Y(reader);
            parsed.z = line.new_Z(reader);
            parsed.e = parsed.has_e ? line.e() : reader.e();
            parsed.feed_mm_min = line.new_F(reader);
            if (parsed.has_e) {
                if (relative_e)
                    cumulative_e += parsed.e;
                else
                    cumulative_e = parsed.e;
            }
            parsed.cumulative_e = cumulative_e;
        }
        lines.emplace_back(std::move(parsed));
    });
    return lines;
}

std::size_t first_line_containing(const std::vector<ParsedLine> &lines, std::string_view token)
{
    for (std::size_t i = 0; i < lines.size(); ++i)
        if (lines[i].raw.find(token) != std::string::npos)
            return i;
    return lines.size();
}

std::size_t first_move(const std::vector<ParsedLine> &lines)
{
    for (std::size_t i = 0; i < lines.size(); ++i)
        if (lines[i].command == "G1")
            return i;
    return lines.size();
}

std::vector<ParsedLine> moves(const std::vector<ParsedLine> &lines)
{
    std::vector<ParsedLine> out;
    for (const ParsedLine &line : lines)
        if (line.command == "G1")
            out.push_back(line);
    return out;
}

float total_extrusion(const std::vector<ParsedLine> &g1, bool relative_e)
{
    if (!relative_e)
        return g1.back().e;

    float total = 0.f;
    for (const ParsedLine &line : g1)
        if (line.has_e)
            total += line.e;
    return total;
}

std::vector<std::size_t> move_indices(const std::vector<ParsedLine> &lines)
{
    std::vector<std::size_t> out;
    for (std::size_t i = 0; i < lines.size(); ++i)
        if (lines[i].command == "G1")
            out.push_back(i);
    return out;
}

struct FanCommandCase {
    GCodeFlavor flavor;
    const char *command;
};

} // namespace

TEST_CASE("FanMover recognizes firmware-specific fan commands and moves them before the target move", "[FanMover]")
{
    const FanCommandCase fan_case = GENERATE(
        FanCommandCase{gcfMarlinFirmware, "M106 S255"},
        FanCommandCase{gcfMarlinFirmware, "M106 P1 S255"},
        FanCommandCase{gcfMach3,          "M106 P255"},
        FanCommandCase{gcfMachinekit,     "M106 P255"},
        FanCommandCase{gcfMakerWare,      "M126 T255"},
        FanCommandCase{gcfSailfish,       "M126 T255"});

    GCodeWriter writer;
    writer.config.gcode_flavor.value = fan_case.flavor;
    FanMover mover(writer, 0.5f, false, false, false, 0.f);

    const std::string input = std::string("G1 F600 X10 Y2 Z0.2 E1\n") + fan_case.command +
                              "\nG1 F600 X20 Y4 Z0.4 E2";
    const std::string output = mover.process_gcode(input, true);
    const std::vector<ParsedLine> lines = parse_output(output, writer.config);

    const std::size_t fan_index = first_line_containing(lines, fan_case.command);
    const std::vector<std::size_t> move_positions = move_indices(lines);
    REQUIRE(fan_index < lines.size());
    REQUIRE(move_positions.size() >= 2);
    CHECK(move_positions.front() < fan_index);
    CHECK(fan_index < move_positions[1]);
}

TEST_CASE("FanMover rejects the auxiliary M106 form while accepting P1 for generic firmware", "[FanMover]")
{
    GCodeWriter writer;
    writer.config.gcode_flavor.value = gcfMarlinFirmware;
    FanMover mover(writer, 0.5f, false, false, false, 0.f);

    const std::string output = mover.process_gcode(
        "G1 F600 X10 E1\nM106 P2 S255\nM106 P1 S255\nG1 F600 X20 E2", true);
    const std::vector<ParsedLine> lines = parse_output(output, writer.config);

    const std::vector<std::size_t> move_positions = move_indices(lines);
    const std::size_t p2_index = first_line_containing(lines, "M106 P2 S255");
    const std::size_t p1_index = first_line_containing(lines, "M106 P1 S255");
    REQUIRE(move_positions.size() >= 2);
    REQUIRE(p2_index < lines.size());
    REQUIRE(p1_index < lines.size());
    CHECK(move_positions.front() < p1_index);
    CHECK(p1_index < move_positions[1]);
    CHECK(move_positions[1] < p2_index);
}

TEST_CASE("FanMover tracks an explicit fan-off command before a repeated target", "[FanMover]")
{
    GCodeWriter writer;
    writer.config.gcode_flavor.value = gcfMarlinFirmware;
    FanMover mover(writer, 0.5f, false, false, false, 0.f);

    const std::string output = mover.process_gcode(
        "G1 F600 X10 E1\nM106 S128\nG1 F600 X20 E2\nM107\nM106 S128\nG1 F600 X30 E3", true);
    const std::vector<ParsedLine> lines = parse_output(output, writer.config);
    const std::vector<ParsedLine> g1 = moves(lines);

    // The zero-speed command is a real state transition. It is removed as a stale lower-speed
    // command when the repeated target is scheduled, so that target still splits the next move.
    CHECK(output.find("M107") == std::string::npos);
    REQUIRE(g1.size() >= 5);
    CHECK_THAT(g1.back().x, Catch::Matchers::WithinAbs(30.f, 0.001));
    CHECK_THAT(g1.back().e, Catch::Matchers::WithinAbs(3.f, 0.001));
}

TEST_CASE("FanMover applies delay only to overhangs when requested", "[FanMover]")
{
    GCodeWriter writer;
    writer.config.gcode_flavor.value = gcfMarlinFirmware;

    SECTION("overhang perimeter is split before the fan command") {
        FanMover mover(writer, 0.5f, false, false, true, 0.f);
        const std::string output = mover.process_gcode(
            ";TYPE:Overhang wall\nG1 F600 X10 Y2 Z0.2 E1\nM106 S255\nG1 F600 X20 Y4 Z0.4 E2", true);
        const std::vector<ParsedLine> lines = parse_output(output, writer.config);
        const std::vector<ParsedLine> g1 = moves(lines);

        const std::vector<std::size_t> move_positions = move_indices(lines);
        const std::size_t fan_index = first_line_containing(lines, "M106 S255");
        REQUIRE(move_positions.size() >= 3);
        REQUIRE(fan_index < lines.size());
        REQUIRE(move_positions.front() < fan_index);
        REQUIRE(fan_index < move_positions[1]);
        REQUIRE(g1.size() >= 3);
        CHECK_THAT(g1.back().x, Catch::Matchers::WithinAbs(20.f, 0.001));
        CHECK_THAT(g1.back().y, Catch::Matchers::WithinAbs(4.f, 0.001));
        CHECK_THAT(g1.back().z, Catch::Matchers::WithinAbs(0.4f, 0.001));
        CHECK_THAT(g1.back().e, Catch::Matchers::WithinAbs(2.f, 0.001));
    }

    SECTION("ordinary perimeter remains in order") {
        FanMover mover(writer, 0.5f, false, false, true, 0.f);
        const std::string output = mover.process_gcode(
            ";TYPE:External perimeter\nG1 F600 X10 E1\nM106 S255\nG1 F600 X20 E2", true);
        const std::vector<ParsedLine> lines = parse_output(output, writer.config);

        const std::size_t move_index = first_move(lines);
        const std::size_t fan_index = first_line_containing(lines, "M106 S255");
        REQUIRE(move_index < lines.size());
        REQUIRE(fan_index < lines.size());
        CHECK(move_index < fan_index);
    }
}

TEST_CASE("FanMover treats custom G-code as a delay barrier", "[FanMover]")
{
    GCodeWriter writer;
    writer.config.gcode_flavor.value = gcfMarlinFirmware;
    FanMover mover(writer, 0.5f, false, false, false, 0.f);

    const std::string output = mover.process_gcode(
        "G1 F600 X10 E1\n; custom gcode start\nM106 S255\n; custom gcode end\nG1 F600 X20 E2", true);
    const std::vector<ParsedLine> lines = parse_output(output, writer.config);

    const std::size_t start = first_line_containing(lines, "; custom gcode");
    const std::size_t fan = first_line_containing(lines, "M106 S255");
    const std::size_t end = first_line_containing(lines, "; custom gcode end");
    REQUIRE(start < lines.size());
    REQUIRE(fan < lines.size());
    REQUIRE(end < lines.size());
    CHECK(start < fan);
    CHECK(fan < end);
}

TEST_CASE("FanMover inserts a kickstart and conserves absolute or relative extrusion", "[FanMover]")
{
    const bool relative_e = GENERATE(false, true);
    GCodeWriter writer;
    writer.config.gcode_flavor.value = gcfMarlinFirmware;
    writer.config.use_relative_e_distances.value = relative_e;
    FanMover mover(writer, 0.f, false, relative_e, false, 0.5f);

    const std::string second_move = relative_e ? "G1 F600 X20 Y4 Z0.4 E1" : "G1 F600 X20 Y4 Z0.4 E2";
    const std::string output = mover.process_gcode(
        std::string("G1 F600 X10 Y2 Z0.2 E1\nM106 S128\n") + second_move, true);
    const std::vector<ParsedLine> lines = parse_output(output, writer.config);
    const std::vector<ParsedLine> g1 = moves(lines);

    REQUIRE(first_line_containing(lines, "M106 S255") < lines.size());
    REQUIRE(first_line_containing(lines, "M106 S128") < lines.size());
    REQUIRE(g1.size() >= 3);
    CHECK_THAT(g1.back().x, Catch::Matchers::WithinAbs(20.f, 0.001));
    CHECK_THAT(g1.back().y, Catch::Matchers::WithinAbs(4.f, 0.001));
    CHECK_THAT(g1.back().z, Catch::Matchers::WithinAbs(0.4f, 0.001));
    CHECK_THAT(total_extrusion(g1, relative_e), Catch::Matchers::WithinAbs(2.f, 0.001));
}

TEST_CASE("FanMover does not kick-start a ten-percent fan change", "[FanMover]")
{
    GCodeWriter writer;
    writer.config.gcode_flavor.value = gcfMarlinFirmware;
    FanMover mover(writer, 0.f, false, false, false, 0.5f);

    const std::string output = mover.process_gcode(
        "G1 F600 X10 E1\nM106 S26\nG1 F600 X20 E2", true);

    // S26 maps to 10% with FanMover's integer conversion. The kickstart threshold is strictly greater than 10%.
    CHECK(output.find("M106 S26") != std::string::npos);
    CHECK(output.find("M106 S255") == std::string::npos);
}

TEST_CASE("FanMover preserves a kickstart command until an explicit flush", "[FanMover]")
{
    GCodeWriter writer;
    writer.config.gcode_flavor.value = gcfMarlinFirmware;
    FanMover mover(writer, 0.f, false, false, false, 0.5f);

    const std::string pending = mover.process_gcode("M106 S128", false);
    REQUIRE(pending.empty());

    const std::string flushed = mover.process_gcode("", true);
    CHECK(flushed.find("M106 S255") != std::string::npos);
}

TEST_CASE("FanMover combines delayed overhang cooling with kickstart timing and preserves the move path", "[FanMover]")
{
    struct KickstartTimelineCase {
        float kickstart_seconds;
        const char *second_move_end_x;
        float final_x;
        const char *label;
    };

    const bool relative_e = GENERATE(false, true);
    const KickstartTimelineCase timing = GENERATE(
        KickstartTimelineCase{0.42f, "7",   7.f, "near start"},
        KickstartTimelineCase{0.74f, "7",   7.f, "asymmetric interior"},
        KickstartTimelineCase{0.90f, "4.7", 4.7f, "near end"});
    CAPTURE(relative_e);
    CAPTURE(timing.label);

    GCodeWriter writer;
    writer.config.gcode_flavor.value = gcfMarlinFirmware;
    writer.config.use_relative_e_distances.value = relative_e;
    FanMover mover(writer, 0.5f, false, relative_e, true, timing.kickstart_seconds);

    // F600 is 10 mm/s. The first 2 mm takes 0.2 s, less than the 0.5 s overhang
    // delay, and S128 maps to the 50% target used by FanMover's kickstart duration.
    // The requested target times are therefore 0.21, 0.37, and 0.45 s. The latter
    // two fall inside the following 5 mm and 2.7 mm moves; the first is just after
    // the 2 mm endpoint. The interior row uses 34% of its second move so a reversed
    // split fraction also changes the physical E distribution.
    const std::string second_e = relative_e ? "1" : "2";
    const std::string second_move = std::string("G1 F600 X") + timing.second_move_end_x +
                                    " Y0 Z0 E" + second_e;
    const std::string input = std::string(
        ";TYPE:Overhang wall\n"
        "G1 F600 X2 Y0 Z0 E1\n"
        "M106 S128\n") + second_move;
    const std::string output = mover.process_gcode(input, true);
    const std::vector<ParsedLine> lines = parse_output(output, writer.config);
    const std::vector<ParsedLine> g1 = moves(lines);
    const std::size_t first_move_position = first_move(lines);
    const std::size_t baseline_position = first_line_containing(lines, "M106 S255");
    const std::size_t target_position = first_line_containing(lines, "M106 S128");

    REQUIRE(baseline_position < lines.size());
    REQUIRE(target_position < lines.size());
    REQUIRE(first_move_position < lines.size());
    REQUIRE(g1.size() >= 2);
    CHECK(baseline_position < first_move_position);
    CHECK(baseline_position < target_position);

    double elapsed_motion_seconds = 0.0;
    double target_elapsed_seconds = -1.0;
    for (std::size_t i = 0; i < lines.size(); ++i) {
        if (i == target_position)
            target_elapsed_seconds = elapsed_motion_seconds;
        const ParsedLine &line = lines[i];
        if (line.command != "G1")
            continue;

        const double dx = double(line.x) - line.start_x;
        const double dy = double(line.y) - line.start_y;
        const double dz = double(line.z) - line.start_z;
        const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
        CHECK_THAT(line.feed_mm_min, Catch::Matchers::WithinAbs(600.0, 0.001));
        if (distance > 0.0) {
            REQUIRE(line.feed_mm_min > 0.f);
            elapsed_motion_seconds += distance / (line.feed_mm_min / 60.0);
        }
    }
    REQUIRE(target_elapsed_seconds >= 0.0);
    const double requested_kickstart_seconds = double(timing.kickstart_seconds) * 0.5;
    const double following_input_move_seconds = double(timing.final_x - 2.f) / 10.0;
    const double placement_tolerance_seconds = following_input_move_seconds * 0.1 + 0.001;
    INFO("target time=" << target_elapsed_seconds << " requested=" << requested_kickstart_seconds
                         << " tolerance=" << placement_tolerance_seconds);
    CHECK_THAT(target_elapsed_seconds,
               Catch::Matchers::WithinAbs(requested_kickstart_seconds, placement_tolerance_seconds));

    REQUIRE(g1.front().has_x);
    CHECK_THAT(g1.front().x, Catch::Matchers::WithinAbs(2.f, 0.001));
    float previous_x = 0.f;
    for (const ParsedLine &move : g1) {
        CHECK(move.has_x);
        CHECK(move.has_e);
        CHECK_THAT(move.y, Catch::Matchers::WithinAbs(0.f, 0.001));
        CHECK_THAT(move.z, Catch::Matchers::WithinAbs(0.f, 0.001));
        CHECK(move.x + 0.001f >= previous_x);
        CHECK(move.x <= timing.final_x + 0.001f);
        const double expected_cumulative_e = move.x <= 2.f
            ? double(move.x) / 2.0
            : 1.0 + (double(move.x) - 2.0) / double(timing.final_x - 2.f);
        CHECK_THAT(move.cumulative_e, Catch::Matchers::WithinAbs(expected_cumulative_e, 0.001));
        previous_x = move.x;
    }
    CHECK_THAT(g1.back().x, Catch::Matchers::WithinAbs(timing.final_x, 0.001));
    CHECK_THAT(g1.back().cumulative_e, Catch::Matchers::WithinAbs(2.f, 0.001));
}

} // namespace Slic3r
