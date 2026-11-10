#pragma once

#include "libslic3r/GCode.hpp"
#include "libslic3r/GCodeWriter.hpp"
#include "libslic3r/Config.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/PrintConfig.hpp"

namespace gcode_broad_contracts {
using namespace Slic3r;

class TestGCode : public GCode
{
public:
    using GCode::change_layer;
    using GCode::cur_extruder_index;
    using GCode::mass_load_limited_machine_acceleration;
    using GCode::calc_max_volumetric_speed;
    using GCode::set_extruders;
    using GCode::set_last_pos;
    using GCode::to_lift_type;
};

inline DynamicPrintConfig base_config()
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        {"gcode_flavor", "marlin2"},
        {"use_relative_e_distances", true},
        {"layer_height", 0.2},
        {"initial_layer_print_height", 0.2},
        {"nozzle_diameter", 0.4},
        {"wall_loops", 1},
        {"top_shell_layers", 1},
        {"bottom_shell_layers", 1},
        {"top_shell_thickness", 0.0},
        {"bottom_shell_thickness", 0.0},
        {"sparse_infill_density", "20%"},
        {"line_width", 0.5},
        {"initial_layer_line_width", 0.5},
        {"outer_wall_line_width", 0.5},
        {"inner_wall_line_width", 0.5},
        {"sparse_infill_line_width", 0.5},
        {"internal_solid_infill_line_width", 0.5},
        {"top_surface_line_width", 0.5},
        {"skin_infill_line_width", 0.5},
        {"skirt_loops", 0},
        {"brim_type", "no_brim"},
        {"raft_layers", 0},
        {"machine_start_gcode", ""},
        {"machine_end_gcode", ""},
        {"file_start_gcode", ""},
        {"before_layer_change_gcode", ""},
        {"layer_change_gcode", "G92 E0\n"},
        {"printing_by_object_gcode", ""},
        {"gcode_label_objects", false},
        {"exclude_object", false},
        {"gcode_comments", true},
        {"slow_down_for_layer_cooling", false},
        {"fan_cooling_layer_time", 0.0},
        {"slow_down_layer_time", 0.0},
        {"retraction_length", 0.8},
        {"retraction_minimum_travel", 0.0},
        {"z_hop", 0.0},
    });
    return config;
}

inline PrintConfig direct_print_config()
{
    DynamicPrintConfig dynamic = base_config();
    PrintConfig config;
    config.apply(dynamic, true);
    config.extruder_offset.values     = {Vec2d(1.25, -2.5)};
    config.filament_diameter.values   = {1.75};
    config.filament_flow_ratio.values = {1.0};
    return config;
}

inline void init_direct_gcode(TestGCode& gcode, bool relative_e = true)
{
    PrintConfig config                    = direct_print_config();
    config.use_relative_e_distances.value = relative_e;
    gcode.apply_print_config(config);
    gcode.set_extruders({0});
    gcode.writer().set_extruder(0);
}

} // namespace gcode_broad_contracts
