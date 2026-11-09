#include <string>
#include <vector>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "libslic3r/Config.hpp"
#include "libslic3r/Exception.hpp"
#include "libslic3r/Flow.hpp"

using namespace Slic3r;

TEST_CASE("Flow width hints select role defaults for the chosen nozzle", "[Flow][FlowWidthContracts]")
{
    DynamicConfig config;
    config.set_key_value("nozzle_diameter", new ConfigOptionFloats({0.4, 0.6}));
    config.set_key_value("line_width", new ConfigOptionFloatOrPercent(0., false));
    const ConfigOptionFloatOrPercent automatic(0., false);
    for (const std::string &key : std::vector<std::string>{
             "line_width", "initial_layer_line_width", "inner_wall_line_width", "outer_wall_line_width",
             "sparse_infill_line_width", "internal_solid_infill_line_width"}) {
        DYNAMIC_SECTION(key) {
            CHECK_THAT(Flow::extrusion_width(key, &automatic, config, 1), Catch::Matchers::WithinAbs(0.675, 1e-6));
        }
    }
    CHECK_THAT(Flow::extrusion_width("top_surface_line_width", &automatic, config, 1), Catch::Matchers::WithinAbs(0.6, 1e-6));
    CHECK_THAT(Flow::extrusion_width("support_line_width", &automatic, config, 1), Catch::Matchers::WithinAbs(0.6, 1e-6));
}

TEST_CASE("Flow width hints resolve explicit values before global and bridge fallbacks", "[Flow][FlowWidthContracts]")
{
    DynamicConfig config;
    config.set_key_value("nozzle_diameter", new ConfigOptionFloats({0.4, 0.6}));
    config.set_key_value("line_width", new ConfigOptionFloatOrPercent(125., true));
    config.set_key_value("internal_solid_infill_line_width", new ConfigOptionFloatOrPercent(130., true));
    const ConfigOptionFloatOrPercent automatic(0., false), absolute(0.72, false), percentage(150., true);
    CHECK_THAT(Flow::extrusion_width("outer_wall_line_width", &absolute, config, 1), Catch::Matchers::WithinAbs(0.72, 1e-6));
    CHECK_THAT(Flow::extrusion_width("outer_wall_line_width", &percentage, config, 1), Catch::Matchers::WithinAbs(0.9, 1e-6));
    CHECK_THAT(Flow::extrusion_width("outer_wall_line_width", &automatic, config, 1), Catch::Matchers::WithinAbs(0.75, 1e-6));
    config.set_key_value("bridge_line_width", new ConfigOptionFloatOrPercent(200., true));
    CHECK_THAT(Flow::extrusion_width("bridge_line_width", config, 1), Catch::Matchers::WithinAbs(1.2, 1e-6));
    config.set_key_value("bridge_line_width", new ConfigOptionFloatOrPercent(0.81, false));
    CHECK_THAT(Flow::extrusion_width("bridge_line_width", config, 1), Catch::Matchers::WithinAbs(0.81, 1e-6));
    config.set_key_value("bridge_line_width", new ConfigOptionFloatOrPercent(0., true));
    CHECK_THAT(Flow::extrusion_width("bridge_line_width", config, 1), Catch::Matchers::WithinAbs(0.78, 1e-6));
    config.set_key_value("internal_solid_infill_line_width", new ConfigOptionFloatOrPercent(0., false));
    CHECK_THAT(Flow::extrusion_width("bridge_line_width", config, 1), Catch::Matchers::WithinAbs(0.75, 1e-6));
}

TEST_CASE("Flow reports missing width dependencies and rejects nonpositive geometry", "[Flow][FlowWidthContracts]")
{
    DynamicConfig config;
    const ConfigOptionFloatOrPercent automatic(0., false);
    CHECK_THROWS_AS(Flow::extrusion_width("outer_wall_line_width", &automatic, config), FlowErrorMissingVariable);
    config.set_key_value("nozzle_diameter", new ConfigOptionFloats({0.4}));
    CHECK_THROWS_AS(Flow::extrusion_width("outer_wall_line_width", &automatic, config), FlowErrorMissingVariable);
    CHECK_THROWS_AS(Flow::extrusion_width("bridge_line_width", &automatic, config), FlowErrorMissingVariable);
    config.set_key_value("line_width", new ConfigOptionFloatOrPercent(0., false));
    CHECK_THROWS_AS(Flow::extrusion_width("invalid_width_key", &automatic, config), Slic3r::RuntimeError);
    CHECK_THROWS_AS(Flow::new_from_config_width(frPerimeter, automatic, 0.4f, 0.f), Slic3r::InvalidArgument);
    CHECK_THROWS_AS(Flow::new_from_config_width(frPerimeter, automatic, 0.4f, -0.1f), Slic3r::InvalidArgument);
    CHECK_THROWS_AS(Flow::rounded_rectangle_extrusion_spacing(0.01f, 0.2f), FlowErrorNegativeSpacing);
    CHECK_THROWS_AS(Flow::bridging_flow(0.f, 0.4f).mm3_per_mm(), FlowErrorNegativeFlow);
    CHECK_THROWS_AS(Flow(1.f, 0.2f, 0.4f).with_spacing(0.01f), Slic3r::InvalidArgument);
}

TEST_CASE("Flow cross sections preserve requested area for ordinary rounded rectangles", "[Flow][FlowWidthContracts]")
{
    const Flow original(1.f, 0.2f, 0.4f);
    const Flow larger = original.with_cross_section(0.3f);
    CHECK_THAT(larger.mm3_per_mm(), Catch::Matchers::WithinAbs(0.3, 1e-6));
    CHECK_THAT(larger.spacing(), Catch::Matchers::WithinAbs(original.spacing(), 1e-6));
    CHECK_THAT(larger.nozzle_diameter(), Catch::Matchers::WithinAbs(0.4, 1e-6));
    const Flow smaller = original.with_cross_section(0.15f);
    CHECK_THAT(smaller.mm3_per_mm(), Catch::Matchers::WithinAbs(0.15, 1e-6));
    CHECK_THAT(smaller.height(), Catch::Matchers::WithinAbs(0.2, 1e-6));
    const Flow same = original.with_cross_section(float(original.mm3_per_mm()));
    CHECK_THAT(same.width(), Catch::Matchers::WithinAbs(original.width(), 1e-6));
    CHECK_THAT(same.height(), Catch::Matchers::WithinAbs(original.height(), 1e-6));
    CHECK_THAT(same.spacing(), Catch::Matchers::WithinAbs(original.spacing(), 1e-6));
    CHECK_THAT(original.width(), Catch::Matchers::WithinAbs(1., 1e-6));
}
