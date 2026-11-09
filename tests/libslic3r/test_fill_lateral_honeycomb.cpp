#include "infill_geometry_helpers.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <vector>

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_message.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "libslic3r/Config.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/Fill/FillBase.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/Polyline.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/Surface.hpp"
#include "libslic3r/libslic3r.h"

using namespace Slic3r;

namespace infill_setting_geometry_contracts {

static Polylines lateral_honeycomb_paths(double angle_degrees)
{
    const ConfigOptionDef *angle_definition = print_config_def.get("infill_overhang_angle");
    REQUIRE(angle_definition != nullptr);
    REQUIRE(angle_definition->type == coFloat);
    CHECK_THAT(angle_definition->min, Catch::Matchers::WithinAbs(15., 1e-12));
    CHECK_THAT(angle_definition->max, Catch::Matchers::WithinAbs(75., 1e-12));
    REQUIRE(angle_definition->is_value_valid(15.));
    REQUIRE(angle_definition->is_value_valid(75.));

    // This setting belongs to PrintRegionConfig. Feed its canonical-key value
    // into the public Fill API so the oracle sees emitted geometry, not a
    // copied version of the period calculation.
    PrintRegionConfig region_config;
    region_config.set_deserialize_strict({{"infill_overhang_angle", angle_degrees}});
    REQUIRE_THAT(region_config.infill_overhang_angle.value, Catch::Matchers::WithinAbs(angle_degrees, 1e-6));

    ExPolygon expolygon({Point::new_scale(0., 0.), Point::new_scale(20., 0.),
                         Point::new_scale(20., 20.), Point::new_scale(0., 20.)});
    Surface surface(stInternal, expolygon);
    std::unique_ptr<Fill> filler(Fill::new_from_type(ipLateralHoneycomb));
    filler->angle = 0.f;
    filler->fixed_angle = true;
    filler->layer_id = 0;
    filler->z = 0.45;
    filler->spacing = 0.45;

    FillParams params;
    params.pattern = ipLateralHoneycomb;
    params.density = 0.43f;
    params.dont_adjust = true;
    params.anchor_length_max = 0.f;
    params.infill_overhang_angle = region_config.infill_overhang_angle;
    params.config = &region_config;
    return filler->fill_surface(&surface, params);
}

} // namespace infill_setting_geometry_contracts

TEST_CASE("Lateral honeycomb ridge spacing follows infill overhang angle", "[Fill][InfillOverhangAngle]")
{
    using namespace infill_setting_geometry_contracts;

    // The registered bounds are 15 and 75 degrees. At this fixed physical z,
    // the lower setting is in the one-family part of the lattice cycle and the
    // upper setting is in the two-family part; their emitted line counts must
    // therefore differ independently of implementation internals.
    const size_t minimum_angle_lines = long_axis_line_positions(lateral_honeycomb_paths(15.), true).size();
    const size_t maximum_angle_lines = long_axis_line_positions(lateral_honeycomb_paths(75.), true).size();
    CAPTURE(minimum_angle_lines, maximum_angle_lines);
    REQUIRE(minimum_angle_lines >= 8);
    CHECK(maximum_angle_lines > minimum_angle_lines + 2);
}
