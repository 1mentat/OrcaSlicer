#include <boost/nowide/fstream.hpp>
#include <cstddef>
#include <ios>
#include <string>
#include <vector>

#include "test_utils.hpp"
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <catch2/matchers/catch_matchers.hpp>
#include "libslic3r/Config.hpp"
#include "libslic3r/GCodeReader.hpp"
#include "libslic3r/PrintConfig.hpp"

using namespace Slic3r;

namespace {

// The extrusion of the second of two moves that both give E1.
float second_move_extrusion(GCodeReader &reader)
{
    float extrusion = -1.f;
    reader.parse_buffer("G1 X1 E1\nG1 X2 E1\n", [&extrusion](GCodeReader &reader, const GCodeReader::GCodeLine &line) {
        extrusion = line.dist_E(reader);
    });
    return extrusion;
}

} // namespace

TEST_CASE("A G-code reader measures extrusion as relative or absolute as its config says", "[GCodeReader]")
{
    const bool  relative = GENERATE(false, true);
    const float expected = relative ? 1.f : 0.f;

    GCodeConfig config;
    config.use_relative_e_distances.value = relative;
    GCodeReader applied;
    applied.apply_config(config);
    CHECK(applied.config().use_relative_e_distances.value == relative);
    CHECK_THAT(second_move_extrusion(applied), Catch::Matchers::WithinAbs(expected, 1e-6));

    DynamicPrintConfig dynamic;
    dynamic.set_key_value("use_relative_e_distances", new ConfigOptionBool(relative));
    GCodeReader applied_dynamic;
    applied_dynamic.apply_config(dynamic);
    CHECK_THAT(second_move_extrusion(applied_dynamic), Catch::Matchers::WithinAbs(expected, 1e-6));

    GCodeReader copy = applied;
    CHECK(copy.config().use_relative_e_distances.value == relative);
    CHECK_THAT(second_move_extrusion(copy), Catch::Matchers::WithinAbs(expected, 1e-6));
}

TEST_CASE("A G-code reader without a config uses the default one", "[GCodeReader]")
{
    const bool  relative = GCodeConfig().use_relative_e_distances.value;
    GCodeReader reader;
    CHECK(reader.config().use_relative_e_distances.value == relative);
    CHECK_THAT(second_move_extrusion(reader), Catch::Matchers::WithinAbs(relative ? 1.f : 0.f, 1e-6));
}

TEST_CASE("GCodeReader reports a missing file through its boolean contract",
          "[GCodeReader][GCodeReaderPublic]")
{
    ScopedTemporaryDir directory("orca-gcode-reader-missing");
    const std::string missing = (directory.path() / "not-there.gcode").string();
    GCodeReader reader;
    bool callback_invoked = false;
    const bool parsed = reader.parse_file_raw(missing,
        [&](GCodeReader&, const char*, const char*) { callback_invoked = true; });
    CHECK_FALSE(parsed);
    CHECK_FALSE(callback_invoked);
}

namespace {

void write_reader_fixture(const ScopedTemporaryFile &file, const std::string &contents)
{
    boost::nowide::ofstream output(file.string(), std::ios::binary);
    REQUIRE(output.is_open());
    output.write(contents.data(), static_cast<std::streamsize>(contents.size()));
    output.close();
    REQUIRE_FALSE(output.fail());
}

} // namespace

TEST_CASE("File parsing preserves long lines and the final unterminated move",
          "[GCodeReader][GCodeReaderFileStreaming]")
{
    const bool relative_e = GENERATE(false, true);
    const std::string first = "  N7 G1 X3 E1 F600";
    // A comment longer than the reader's input block forces line accumulation.
    const std::string comment = ";" + std::string(700000, 'p');
    const std::string second = "\tN8 G1 X7 E2";
    const std::string last = "N9 G1 X11 E3";
    const std::string contents = first + "\r\n" + comment + "\n" + second + "\r" + last;
    ScopedTemporaryFile file(".gcode");
    write_reader_fixture(file, contents);

    GCodeReader raw_reader;
    std::vector<std::string> raw_lines;
    REQUIRE(raw_reader.parse_file_raw(file.string(),
        [&raw_lines](GCodeReader &, const char *begin, const char *end) {
            raw_lines.emplace_back(begin, end);
        }));
    const std::vector<std::string> expected_lines = {first, comment, second, last};
    CHECK(raw_lines == expected_lines);

    GCodeConfig config;
    config.use_relative_e_distances.value = relative_e;
    GCodeReader reader;
    reader.apply_config(config);
    std::size_t moves = 0;
    double extrusion = 0.0;
    double distance = 0.0;
    std::vector<std::size_t> line_ends = {123};
    REQUIRE(reader.parse_file(file.string(),
        [&](GCodeReader &state, const GCodeReader::GCodeLine &line) {
            if (!line.cmd_is("G1"))
                return;
            ++moves;
            extrusion += line.dist_E(state);
            distance += line.dist_XY(state);
        }, line_ends));
    CHECK(moves == 3);
    CHECK_THAT(extrusion, Catch::Matchers::WithinAbs(relative_e ? 6.0 : 3.0, 1e-6));
    CHECK_THAT(distance, Catch::Matchers::WithinAbs(11.0, 1e-6));
    CHECK_THAT(reader.x(), Catch::Matchers::WithinAbs(11.0, 1e-6));
    const std::vector<std::size_t> expected_ends = {first.size() + 2, first.size() + 2 + comment.size() + 1};
    CHECK(line_ends == expected_ends);
}

TEST_CASE("A file callback can stop reading and the reader can start a fresh parse",
          "[GCodeReader][GCodeReaderFileStreaming]")
{
    ScopedTemporaryFile file(".gcode");
    write_reader_fixture(file, "G1 X1\nG1 X2\nG1 X3\n");
    GCodeReader reader;
    std::size_t callbacks = 0;
    REQUIRE(reader.parse_file(file.string(),
        [&](GCodeReader &state, const GCodeReader::GCodeLine &) {
            ++callbacks;
            state.quit_parsing();
        }));
    CHECK(callbacks == 1);
    CHECK_THAT(reader.x(), Catch::Matchers::WithinAbs(1.0, 1e-6));

    callbacks = 0;
    REQUIRE(reader.parse_file_raw(file.string(),
        [&](GCodeReader &state, const char *, const char *) {
            ++callbacks;
            state.quit_parsing();
        }));
    CHECK(callbacks == 1);

    reader.reset();
    callbacks = 0;
    REQUIRE(reader.parse_file(file.string(),
        [&](GCodeReader &, const GCodeReader::GCodeLine &) { ++callbacks; }));
    CHECK(callbacks == 3);
    CHECK_THAT(reader.x(), Catch::Matchers::WithinAbs(3.0, 1e-6));
}

TEST_CASE("Parsed files distinguish an empty file from a missing file",
          "[GCodeReader][GCodeReaderFileStreaming]")
{
    ScopedTemporaryFile empty(".gcode");
    write_reader_fixture(empty, "");
    ScopedTemporaryDir directory("orca-gcode-reader-parsed-missing");
    const std::string missing = (directory.path() / "absent.gcode").string();
    GCodeReader reader;
    std::size_t callbacks = 0;
    const auto callback = [&](GCodeReader &, const GCodeReader::GCodeLine &) { ++callbacks; };
    std::vector<std::size_t> line_ends = {123};
    REQUIRE(reader.parse_file(empty.string(), callback, line_ends));
    CHECK(line_ends.empty());
    CHECK(callbacks == 0);
    line_ends = {456};
    CHECK_FALSE(reader.parse_file(missing, callback, line_ends));
    CHECK(line_ends.empty());
    CHECK(callbacks == 0);
    CHECK_FALSE(reader.parse_file(missing, callback));
    CHECK(callbacks == 0);
}
