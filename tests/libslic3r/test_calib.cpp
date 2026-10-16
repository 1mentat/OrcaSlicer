#include <regex>
#include <cmath>
#include <cstddef>
#include <vector>
#include "libslic3r/GCode.hpp"
#include "libslic3r/BoundingBox.hpp"
#include <catch2/catch_all.hpp>

#include <algorithm>
#include "libslic3r/CustomGCode.hpp"
#include <sstream>
#include <string>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "libslic3r/calib.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/Point.hpp"

using namespace Slic3r;

namespace {

// The width-resolution getters are protected; expose them so the resolution can be asserted directly.
struct PaPatternProbe : public CalibPressureAdvancePattern
{
    using CalibPressureAdvancePattern::CalibPressureAdvancePattern;
    using CalibPressureAdvancePattern::line_width;
    using CalibPressureAdvancePattern::line_width_first_layer;
};

} // namespace

TEST_CASE("Zero calibration line width resolves to a positive default", "[Calib][Regression]")
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        {"line_width", "0"},
        {"initial_layer_line_width", "0"},
    });

    Model model;
    model.add_object("cube", "", make_cube(20, 20, 20))->add_instance();

    Calib_Params params;
    params.mode = CalibMode::Calib_PA_Pattern;

    PaPatternProbe pattern(params, config, /* is_bbl_machine */ true, *model.objects.front(), Vec3d(0, 0, 0));

    REQUIRE(pattern.line_width() > 0.);
    REQUIRE(pattern.line_width_first_layer() > 0.);
}

namespace {

struct EndState { double final_e; double max_e; };

EndState simulate_absolute_e(const std::string &gcode)
{
    double final_e = 0.;
    double max_e   = 0.;

    std::istringstream lines(gcode);
    std::string        line;
    while (std::getline(lines, line)) {
        std::istringstream words(line);
        std::string        op;
        if (!(words >> op))
            continue;
        if (op != "G1" && op != "G0" && op != "G92")
            continue;

        std::string word;
        while (words >> word) {
            if (word.size() >= 2 && word[0] == 'E') {
                final_e = std::stod(word.substr(1));
                max_e   = std::max(max_e, final_e);
                break;
            }
        }
    }

    return {final_e, max_e};
}

} // namespace

TEST_CASE("PA pattern resets the extruder after the final layer in absolute E mode", "[Calib][Regression]")
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        {"use_relative_e_distances", "0"},
        {"line_width", "0.45"},
        {"initial_layer_line_width", "0.45"},
    });

    Model model;
    model.add_object("cube", "", make_cube(20, 20, 20))->add_instance();

    Calib_Params params;
    params.mode  = CalibMode::Calib_PA_Pattern;
    params.start = 0.;
    params.end   = 0.08;
    params.step  = 0.002;

    CalibPressureAdvancePattern pattern(params, config, /* is_bbl_machine */ false, *model.objects.front(), Vec3d(0, 0, 0));
    const CustomGCode::Info     info = pattern.generate_custom_gcodes(config, /* is_bbl_machine */ false, *model.objects.front(),
                                                                      Vec3d(0, 0, 0));

    std::string gcode;
    for (const CustomGCode::Item &item : info.gcodes)
        gcode += item.extra;

    const EndState state = simulate_absolute_e(gcode);

    REQUIRE(state.max_e > 1.);
    REQUIRE_THAT(state.final_e, Catch::Matchers::WithinAbs(0., 1e-9));
}

namespace calibration_contracts {

namespace {

class ProbeGCode final : public GCode
{
public:
    void apply_config(const DynamicPrintConfig &config)
    {
        m_config.apply(config);
        m_writer.apply_print_config(m_config);
        m_writer.set_extruders({0});
        m_writer.set_extruder(0);
    }
};

DynamicPrintConfig calib_config()
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        {"gcode_flavor", "marlin2"},
        {"nozzle_diameter", "0.4"},
        {"filament_diameter", "1.75"},
        {"layer_height", "0.2"},
        {"initial_layer_print_height", "0.2"},
        {"initial_layer_line_width", "0.5"},
        {"line_width", "0.5"},
        {"wall_loops", "2"},
        {"outer_wall_speed", "60"},
        {"initial_layer_speed", "30"},
        {"outer_wall_acceleration", "500"},
        {"filament_flow_ratio", "1.0"},
        {"use_relative_e_distances", "1"},
        {"printable_area", "0x0,200x0,200x200,0x200"},
    });
    return config;
}

void configure_gcode(ProbeGCode &gcode, const DynamicPrintConfig &config)
{
    gcode.apply_config(config);
}

std::vector<double> pressure_advance_values(const std::string &gcode)
{
    std::vector<double> result;
    const std::regex re(R"(M900 K([-+0-9.eE]+))");
    for (std::sregex_iterator it(gcode.begin(), gcode.end(), re), end; it != end; ++it)
        result.push_back(std::stod((*it)[1].str()));
    return result;
}

std::vector<double> feed_rates(const std::string &gcode)
{
    std::vector<double> result;
    const std::regex re(R"(G1 F([-+0-9.eE]+))");
    for (std::sregex_iterator it(gcode.begin(), gcode.end(), re), end; it != end; ++it)
        result.push_back(std::stod((*it)[1].str()));
    return result;
}

ModelObject *calibration_object(Model &model)
{
    return model.add_object("calibration cube", "", make_cube(20., 20., 20.));
}

CalibPressureAdvancePattern make_pattern(const DynamicPrintConfig &config, ModelObject &object,
                                         Calib_Params &params,
                                         double start = 0., double end = 0.02, double step = 0.01)
{
    // The pattern retains a reference to params; its caller owns that lifetime.
    params.mode = CalibMode::Calib_PA_Pattern;
    params.start = start;
    params.end = end;
    params.step = step;
    return CalibPressureAdvancePattern(params, config, false, object, Vec3d::Zero());
}

std::string join_gcode(const CustomGCode::Info &info)
{
    std::string result;
    for (const CustomGCode::Item &item : info.gcodes)
        result += item.extra;
    return result;
}

} // namespace

TEST_CASE("Pressure advance speed honors minimum and volumetric limits", "[Calib][CalibrationContracts]")
{
    DynamicPrintConfig config = calib_config();
    config.set_deserialize_strict({
        {"outer_wall_speed", "30"},
        {"filament_max_volumetric_speed", "20"},
    });
    const float high_limit = CalibPressureAdvance::find_optimal_PA_speed(config, 0.5, 0.2);
    REQUIRE_THAT(high_limit, Catch::Matchers::WithinAbs(100., 1e-5));

    config.set_deserialize_strict("filament_max_volumetric_speed", "0.1");
    const float low_limit = CalibPressureAdvance::find_optimal_PA_speed(config, 0.5, 0.2);
    REQUIRE(low_limit >= 0.f);
    REQUIRE(low_limit < high_limit);
}

TEST_CASE("Pressure advance speed follows an explicit wall speed above the floor", "[Calib][CalibrationContracts]")
{
    DynamicPrintConfig config = calib_config();
    config.set_deserialize_strict({
        {"outer_wall_speed", "145"},
        {"filament_max_volumetric_speed", "1000"},
    });
    const float result = CalibPressureAdvance::find_optimal_PA_speed(config, 0.5, 0.2);
    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(145., 1e-5));
}

TEST_CASE("Pressure advance line extents include the optional number box", "[Calib][CalibrationContracts]")
{
    DynamicPrintConfig config = calib_config();
    ProbeGCode gcode;
    configure_gcode(gcode, config);
    CalibPressureAdvanceLine line(&gcode);
    const BoundingBoxf bed(Vec2d(0.f, 0.f), Vec2d(200., 200.));

    const BoundingBoxf with_numbers = line.print_extents(bed);
    line.draw_numbers() = false;
    const BoundingBoxf without_numbers = line.print_extents(bed);

    REQUIRE_THAT(with_numbers.min.x(), Catch::Matchers::WithinAbs(50., 1e-4));
    REQUIRE_THAT(with_numbers.max.x(), Catch::Matchers::WithinAbs(154.6, 1e-4));
    REQUIRE_THAT(with_numbers.min.y(), Catch::Matchers::WithinAbs(0., 1e-5));
    REQUIRE_THAT(with_numbers.max.y(), Catch::Matchers::WithinAbs(200., 1e-5));
    REQUIRE_THAT(with_numbers.max.x() - without_numbers.max.x(), Catch::Matchers::WithinAbs(24., 1e-4));
}

TEST_CASE("Pressure advance line shortens its long segment on a narrow bed", "[Calib][CalibrationContracts]")
{
    DynamicPrintConfig config = calib_config();
    ProbeGCode gcode;
    configure_gcode(gcode, config);
    CalibPressureAdvanceLine line(&gcode);
    const BoundingBoxf bed(Vec2d(0.f, 0.f), Vec2d(100., 200.));
    const BoundingBoxf extents = line.print_extents(bed);

    REQUIRE_THAT(extents.min.x(), Catch::Matchers::WithinAbs(10., 1e-4));
    REQUIRE_THAT(extents.max.x(), Catch::Matchers::WithinAbs(94.6, 1e-4));
    REQUIRE_THAT(extents.max.y() - extents.min.y(), Catch::Matchers::WithinAbs(200., 1e-5));
}

TEST_CASE("Pressure advance line emits each requested calibration value", "[Calib][CalibrationContracts]")
{
    DynamicPrintConfig config = calib_config();
    ProbeGCode gcode;
    configure_gcode(gcode, config);
    CalibPressureAdvanceLine line(&gcode);
    line.draw_numbers() = false;
    line.set_speed(80., 20.);
    const std::string output = line.generate_test(0.02, 0.01, 3);
    const std::vector<double> values = pressure_advance_values(output);

    // The prime, extra anchor after the first line, and final reset use zero PA.
    const std::vector<double> expected = {0., 0.02, 0., 0.03, 0.04, 0.};
    REQUIRE(values.size() == expected.size());
    for (std::size_t i = 0; i < expected.size(); ++i)
        CHECK_THAT(values[i], Catch::Matchers::WithinAbs(expected[i], 1e-5));
}

TEST_CASE("Pressure advance line speed settings reach both feed-rate classes", "[Calib][CalibrationContracts]")
{
    DynamicPrintConfig config = calib_config();
    ProbeGCode gcode;
    configure_gcode(gcode, config);
    CalibPressureAdvanceLine line(&gcode);
    line.draw_numbers() = false;
    line.set_speed(80., 20.);
    const std::vector<double> rates = feed_rates(line.generate_test(0.02, 0.01, 3));

    REQUIRE_FALSE(rates.empty());
    CHECK(std::find_if(rates.begin(), rates.end(), [](double f) { return std::abs(f - 1200.) < 1e-4; }) != rates.end());
    CHECK(std::find_if(rates.begin(), rates.end(), [](double f) { return std::abs(f - 4800.) < 1e-4; }) != rates.end());
}

TEST_CASE("Pressure advance line draw-number toggle enables annotation geometry", "[Calib][CalibrationContracts]")
{
    DynamicPrintConfig config = calib_config();
    ProbeGCode gcode;
    configure_gcode(gcode, config);
    CalibPressureAdvanceLine line(&gcode);
    line.draw_numbers() = true;
    const std::string with_numbers = line.generate_test(0., 0.01, 2);
    line.draw_numbers() = false;
    const std::string without_numbers = line.generate_test(0., 0.01, 2);

    CHECK(with_numbers.find("Bottom surface") != std::string::npos);
    CHECK(without_numbers.find("Bottom surface") == std::string::npos);
    REQUIRE_FALSE(with_numbers.empty());
    REQUIRE_FALSE(without_numbers.empty());
}

TEST_CASE("Delta pressure advance line scales its reported bed extent", "[Calib][CalibrationContracts]")
{
    DynamicPrintConfig config = calib_config();
    config.set_deserialize_strict({{"printable_area", "0x0,200x0,220x100,200x200,0x200,-20x100"}});
    ProbeGCode gcode;
    configure_gcode(gcode, config);
    CalibPressureAdvanceLine line(&gcode);
    const BoundingBoxf bed(Vec2d(-100., -100.), Vec2d(100., 100.));
    const BoundingBoxf extents = line.print_extents(bed);

    REQUIRE(line.is_delta());
    const double scaled_width = 200. * (1. / 1.41421);
    REQUIRE_THAT(extents.max.y() - extents.min.y(), Catch::Matchers::WithinAbs(scaled_width, 1e-3));
}

TEST_CASE("Pressure advance pattern returns one custom item per layer", "[Calib][CalibrationContracts]")
{
    DynamicPrintConfig config = calib_config();
    Model model;
    ModelObject *object = calibration_object(model);
    object->add_instance();
    Calib_Params params;
    CalibPressureAdvancePattern pattern = make_pattern(config, *object, params);
    const CustomGCode::Info info = pattern.generate_custom_gcodes(config, false, *object, Vec3d::Zero());

    REQUIRE(info.gcodes.size() == 4);
    CHECK_THAT(info.gcodes[0].print_z, Catch::Matchers::WithinAbs(0.2, 1e-5));
    CHECK_THAT(info.gcodes[1].print_z, Catch::Matchers::WithinAbs(0.4, 1e-5));
    CHECK_THAT(info.gcodes[2].print_z, Catch::Matchers::WithinAbs(0.6, 1e-5));
    CHECK_THAT(info.gcodes[3].print_z, Catch::Matchers::WithinAbs(0.8, 1e-5));
}

TEST_CASE("Pressure advance pattern contains the requested PA sequence and layer markers", "[Calib][CalibrationContracts]")
{
    DynamicPrintConfig config = calib_config();
    Model model;
    ModelObject *object = calibration_object(model);
    object->add_instance();
    Calib_Params params;
    CalibPressureAdvancePattern pattern = make_pattern(config, *object, params, 0.1, 0.13, 0.01);
    const std::string output = join_gcode(pattern.generate_custom_gcodes(config, false, *object, Vec3d::Zero()));

    const std::vector<double> values = pressure_advance_values(output);
    REQUIRE_FALSE(values.empty());
    CHECK(std::find_if(values.begin(), values.end(), [](double v) { return std::abs(v - 0.1) < 1e-4; }) != values.end());
    CHECK(std::find_if(values.begin(), values.end(), [](double v) { return std::abs(v - 0.11) < 1e-4; }) != values.end());
    CHECK(std::find_if(values.begin(), values.end(), [](double v) { return std::abs(v - 0.12) < 1e-4; }) != values.end());
    CHECK(std::find_if(values.begin(), values.end(), [](double v) { return std::abs(v - 0.13) < 1e-4; }) != values.end());
    CHECK(output.find("; start pressure advance pattern for layer") != std::string::npos);
    CHECK(output.find("; end pressure advance pattern for layer") != std::string::npos);
}

TEST_CASE("Pressure advance pattern flow value increases with configured perimeter speed", "[Calib][CalibrationContracts]")
{
    DynamicPrintConfig slow_config = calib_config();
    DynamicPrintConfig fast_config = slow_config;
    fast_config.set_deserialize_strict({{"outer_wall_speed", "120"}});
    Model slow_model;
    ModelObject *slow_object = calibration_object(slow_model);
    slow_object->add_instance();
    Model fast_model;
    ModelObject *fast_object = calibration_object(fast_model);
    fast_object->add_instance();

    Calib_Params slow_params;
    Calib_Params fast_params;
    CalibPressureAdvancePattern slow = make_pattern(slow_config, *slow_object, slow_params);
    CalibPressureAdvancePattern fast = make_pattern(fast_config, *fast_object, fast_params);
    REQUIRE(slow.flow_val() > 0.);
    REQUIRE(fast.flow_val() > slow.flow_val());
}

TEST_CASE("Pressure advance pattern start offset is retained and affects generated coordinates", "[Calib][CalibrationContracts]")
{
    DynamicPrintConfig config = calib_config();
    Model model;
    ModelObject *object = calibration_object(model);
    object->add_instance();
    Calib_Params params;
    CalibPressureAdvancePattern pattern = make_pattern(config, *object, params);
    pattern.set_start_offset(Vec3d(17., 23., 0.));
    const Vec3d offset = pattern.get_start_offset();
    CHECK_THAT(offset.x(), Catch::Matchers::WithinAbs(17., 1e-5));
    CHECK_THAT(offset.y(), Catch::Matchers::WithinAbs(23., 1e-5));
    const std::string output = join_gcode(pattern.generate_custom_gcodes(config, false, *object, Vec3d::Zero()));
    CHECK(std::regex_search(output, std::regex(R"((^|\s)X17(\.0*)?(\s|$))")));
    CHECK(std::regex_search(output, std::regex(R"((^|\s)Y23(\.0*)?(\s|$))")));
}

TEST_CASE("Pressure advance pattern emits BBL pressure-advance syntax when requested", "[Calib][CalibrationContracts]")
{
    DynamicPrintConfig config = calib_config();
    Model model;
    ModelObject *object = calibration_object(model);
    object->add_instance();
    Calib_Params params;
    CalibPressureAdvancePattern pattern = make_pattern(config, *object, params, 0., 0.01, 0.01);
    const std::string output = join_gcode(pattern.generate_custom_gcodes(config, true, *object, Vec3d::Zero()));

    CHECK(output.find("M900 K") != std::string::npos);
    CHECK(output.find("L1000 M10") != std::string::npos);
}

} // namespace calibration_contracts
