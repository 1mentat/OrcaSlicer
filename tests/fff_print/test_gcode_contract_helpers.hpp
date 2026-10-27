#pragma once

#include <cmath>
#include <cstddef>
#include <fstream>
#include <iterator>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <catch2/catch_all.hpp>
#include <catch2/catch_message.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/catch_test_macros.hpp>

#include "libslic3r/GCode/GCodeProcessor.hpp"
#include "libslic3r/GCodeReader.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/libslic3r.h"

#include "test_helpers.hpp"
#include "test_utils.hpp"

namespace gcode_print_variance_contracts {
using namespace Slic3r;
using std::size_t;

struct ExportedPrint {
    std::string text;
    bool        label_object_enabled = false;
};

struct MotionEvidence {
    size_t extrusion_moves = 0;
    size_t z_only_moves = 0;
    double xy_path = 0.0;
    double filament = 0.0;
};

inline void require_valid(const Print &print)
{
    const auto error = print.validate();
    INFO("Print validation error: " << error.string << " [" << error.opt_key << "]");
    REQUIRE(error.string.empty());
}

inline ExportedPrint export_print(Print &print)
{
    ExportedPrint result;
    GCodeProcessorResult processor_result;
    print.set_status_silent();
    print.process();
    ScopedTemporaryFile temp(".gcode");
    print.export_gcode(temp.string(), &processor_result, nullptr);
    std::ifstream input(temp.string(), std::ios::binary);
    result.text.assign((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
    result.label_object_enabled = processor_result.label_object_enabled;
    REQUIRE_FALSE(result.text.empty());
    return result;
}

inline size_t count_text(const std::string &text, const std::string &needle)
{
    size_t count = 0;
    for (size_t pos = text.find(needle); pos != std::string::npos; pos = text.find(needle, pos + needle.size()))
        ++count;
    return count;
}

inline MotionEvidence motion_evidence(const std::string &text, const DynamicPrintConfig &config)
{
    MotionEvidence evidence;
    GCodeReader reader;
    reader.apply_config(config);
    reader.parse_buffer(text, [&evidence](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        if (!(line.cmd_is("G0") || line.cmd_is("G1") || line.cmd_is("G2") || line.cmd_is("G3")))
            return;

        const double xy = line.dist_XY(self);
        if (std::abs(line.dist_Z(self)) > EPSILON && xy <= EPSILON)
            ++evidence.z_only_moves;
        if (!line.extruding(self) || xy <= EPSILON)
            return;
        ++evidence.extrusion_moves;
        evidence.xy_path += xy;
        evidence.filament += std::abs(line.dist_E(self));
    });
    return evidence;
}

inline void require_same_material(const MotionEvidence &lhs, const MotionEvidence &rhs)
{
    REQUIRE(lhs.extrusion_moves > 0);
    REQUIRE(lhs.extrusion_moves == rhs.extrusion_moves);
    CHECK_THAT(rhs.xy_path, Catch::Matchers::WithinRel(lhs.xy_path, 1e-6));
    CHECK_THAT(rhs.filament, Catch::Matchers::WithinRel(lhs.filament, 1e-6));
}

inline ExportedPrint fresh_custom_mesh_export(const DynamicPrintConfig &config, TriangleMesh mesh, bool bbl = false)
{
    Model model;
    Print print;
    Test::init_print({std::move(mesh)}, print, model, config);
    print.is_BBL_printer() = bbl;
    require_valid(print);
    return export_print(print);
}

namespace calibration_gcode_followup {

inline constexpr size_t kCalibrationLayerCount = 6;

struct OuterWallFeedEvidence {
    std::vector<std::vector<double>> feeds;
    bool                             finite = true;
};

inline OuterWallFeedEvidence parse_outer_wall_feeds(const std::string &gcode,
                                             const DynamicPrintConfig &config)
{
    OuterWallFeedEvidence evidence;
    evidence.feeds.resize(kCalibrationLayerCount);
    GCodeReader reader;
    reader.apply_config(config);
    int current_layer = -1;
    bool outer_wall = false;
    reader.parse_buffer(gcode, [&evidence, &current_layer, &outer_wall](GCodeReader &self,
                                                                         const GCodeReader::GCodeLine &line) {
        const std::string_view comment = line.comment();
        if (comment.find("LAYER_CHANGE") != std::string_view::npos)
            ++current_layer;
        if (comment.rfind("TYPE:", 0) == 0)
            outer_wall = comment == "TYPE:Outer wall";
        if (current_layer < 0 || current_layer >= static_cast<int>(evidence.feeds.size()) ||
            !outer_wall || !line.extruding(self) || line.dist_XY(self) <= EPSILON)
            return;
        const double feed = double(line.new_F(self));
        if (!std::isfinite(feed) || feed <= 0.)
            evidence.finite = false;
        else
            evidence.feeds[static_cast<size_t>(current_layer)].push_back(feed);
    });
    return evidence;
}

} // namespace calibration_gcode_followup
} // namespace gcode_print_variance_contracts
