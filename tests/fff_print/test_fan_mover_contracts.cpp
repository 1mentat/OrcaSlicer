#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "libslic3r/GCode/FanMover.hpp"
#include "libslic3r/GCodeReader.hpp"
#include "libslic3r/GCodeWriter.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/libslic3r.h"

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
    float x = 0.f;
    float y = 0.f;
    float z = 0.f;
    float e = 0.f;
};

std::vector<ParsedLine> parse_output(const std::string &gcode, const GCodeConfig &config)
{
    GCodeReader reader;
    reader.apply_config(config);

    std::vector<ParsedLine> lines;
    reader.parse_buffer(gcode, [&lines](GCodeReader &reader, const GCodeReader::GCodeLine &line) {
        ParsedLine parsed;
        parsed.raw = line.raw();
        parsed.command = std::string(line.cmd());
        if (line.cmd_is("G1")) {
            parsed.has_x = line.has(Axis::X);
            parsed.has_y = line.has(Axis::Y);
            parsed.has_z = line.has(Axis::Z);
            parsed.has_e = line.has(Axis::E);
            parsed.x = parsed.has_x ? line.x() : reader.x();
            parsed.y = parsed.has_y ? line.y() : reader.y();
            parsed.z = parsed.has_z ? line.z() : reader.z();
            parsed.e = parsed.has_e ? line.e() : reader.e();
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

} // namespace Slic3r
