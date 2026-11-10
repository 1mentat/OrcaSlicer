#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_message.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "libslic3r/BeltTransform.hpp"
#include "libslic3r/Config.hpp"
#include "libslic3r/Geometry.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/libslic3r.h"

#include <array>
#include <cmath>

using namespace Slic3r;

TEST_CASE("Belt rotation enum values select distinct public transform and floor semantics", "[GCodeWriter][SettingsEnum][BeltRotationEnum]")
{
    struct Row {
        const char *serialized;
        BeltRotationAxis axis;
        Vec3d expected_x;
        Vec3d expected_y;
        Vec3d expected_z;
        double tilt_x;
        double tilt_y;
        bool floor_active;
        double shear;
        int from_axis;
    };

    const double c = std::cos(Geometry::deg2rad(30.));
    const double s = std::sin(Geometry::deg2rad(30.));
    const double t = std::tan(Geometry::deg2rad(30.));
    const std::array<Row, 4> rows = {{
        {"none", BeltRotationAxis::None, Vec3d(1., 0., 0.), Vec3d(0., 1., 0.), Vec3d(0., 0., 1.), 0., 0., false, 0., 1},
        {"x",    BeltRotationAxis::X,    Vec3d(1., 0., 0.), Vec3d(0., c, s), Vec3d(0., -s, c), 30., 0., true, t, 1},
        {"y",    BeltRotationAxis::Y,    Vec3d(c, 0., -s), Vec3d(0., 1., 0.), Vec3d(s, 0., c), 0., 30., true, -t, 0},
        {"z",    BeltRotationAxis::Z,    Vec3d(c, s, 0.), Vec3d(-s, c, 0.), Vec3d(0., 0., 1.), 0., 0., false, 0., 1},
    }};

    for (const Row &row : rows) {
        INFO("belt_slice_rotation=" << row.serialized);

        DynamicPrintConfig parsed = DynamicPrintConfig::full_print_config();
        parsed.set_deserialize_strict({
            {"belt_slice_rotation", row.serialized},
            {"belt_slice_rotation_angle", 30.},
        });
        const auto *parsed_axis = parsed.option<ConfigOptionEnum<BeltRotationAxis>>("belt_slice_rotation");
        REQUIRE(parsed_axis != nullptr);
        CHECK(parsed_axis->value == row.axis);

        PrintConfig config;
        config.belt_slice_rotation.value = parsed_axis->value;
        config.belt_slice_rotation_angle.value = 30.;

        bool has_rotation = false;
        const Matrix3d matrix = BeltTransformPipeline::build_rotation_matrix(config, &has_rotation);
        CHECK(has_rotation == (row.axis != BeltRotationAxis::None));
        CHECK_THAT((matrix * Vec3d::UnitX()).x(), Catch::Matchers::WithinAbs(row.expected_x.x(), 1e-12));
        CHECK_THAT((matrix * Vec3d::UnitX()).y(), Catch::Matchers::WithinAbs(row.expected_x.y(), 1e-12));
        CHECK_THAT((matrix * Vec3d::UnitX()).z(), Catch::Matchers::WithinAbs(row.expected_x.z(), 1e-12));
        CHECK_THAT((matrix * Vec3d::UnitY()).x(), Catch::Matchers::WithinAbs(row.expected_y.x(), 1e-12));
        CHECK_THAT((matrix * Vec3d::UnitY()).y(), Catch::Matchers::WithinAbs(row.expected_y.y(), 1e-12));
        CHECK_THAT((matrix * Vec3d::UnitY()).z(), Catch::Matchers::WithinAbs(row.expected_y.z(), 1e-12));
        CHECK_THAT((matrix * Vec3d::UnitZ()).x(), Catch::Matchers::WithinAbs(row.expected_z.x(), 1e-12));
        CHECK_THAT((matrix * Vec3d::UnitZ()).y(), Catch::Matchers::WithinAbs(row.expected_z.y(), 1e-12));
        CHECK_THAT((matrix * Vec3d::UnitZ()).z(), Catch::Matchers::WithinAbs(row.expected_z.z(), 1e-12));

        const auto tilt = BeltTransformPipeline::physical_tilt(row.axis, 30.);
        CHECK_THAT(tilt.tilt_x_deg, Catch::Matchers::WithinAbs(row.tilt_x, 1e-12));
        CHECK_THAT(tilt.tilt_y_deg, Catch::Matchers::WithinAbs(row.tilt_y, 1e-12));

        BeltTransformPipeline::BeltFloorParams floor;
        const bool has_floor = BeltTransformPipeline::floor_shear(config, floor);
        CHECK(has_floor == row.floor_active);
        CHECK_THAT(floor.shear_factor, Catch::Matchers::WithinAbs(row.shear, 1e-12));
        CHECK(floor.from_axis == row.from_axis);
    }
}
