#include <miniz.h>
#include <memory>
#include <cstdint>
#include "test_utils.hpp"
#include "libslic3r/Utils.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/miniz_extension.hpp"
#include "libslic3r/PNGReadWrite.hpp"
#include "libslic3r/Format/SL1.hpp"
#include <catch2/generators/catch_generators.hpp>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include "libslic3r/SLA/Hollowing.hpp"
#include "libslic3r/ObjectID.hpp"
#include "libslic3r/BoundingBox.hpp"
#include <string>
#include <algorithm>
#include <catch2/catch_all.hpp>
#include <catch2/catch_message.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <math.h>
#include <cmath>
#include <cstddef>
#include <libslic3r/Config.hpp>
#include <utility>
#include <libslic3r/Point.hpp>
#include <vector>

#include "catch2/catch_test_macros.hpp"
#include "catch2/matchers/catch_matchers.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/PrintBase.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/libslic3r.h"
#include "libslic3r/SLA/SupportPoint.hpp"
#include "libslic3r/SLA/SupportTree.hpp"
#include "libslic3r/SLA/SupportTreeBuilder.hpp"
#include "libslic3r/SLA/SupportTreeBuildsteps.hpp"
#include "libslic3r/TriangleMeshSlicer.hpp"
#include "libslic3r/SLAPrint.hpp"
#include "libslic3r/TriangleMesh.hpp"

using namespace Slic3r;

namespace {

DynamicPrintConfig sla_config(double layer_height)
{
    DynamicPrintConfig config;
    config.apply(SLAFullPrintConfig::defaults());

    config.set_key_value("printer_technology", new ConfigOptionEnum<PrinterTechnology>(ptSLA));
    config.set_key_value("layer_height", new ConfigOptionFloat(layer_height));
    config.set_key_value("initial_layer_height", new ConfigOptionFloat(layer_height));
    config.set_key_value("supports_enable", new ConfigOptionBool(false));
    config.set_key_value("pad_enable", new ConfigOptionBool(false));
    config.set_key_value("hollowing_enable", new ConfigOptionBool(false));
    config.set_key_value("slice_closing_radius", new ConfigOptionFloat(0.005));

    // Keep the statistics path physical and deterministic for a headless print.
    config.set_key_value("display_width", new ConfigOptionFloat(50.0));
    config.set_key_value("display_height", new ConfigOptionFloat(50.0));
    config.set_key_value("area_fill", new ConfigOptionFloat(50.0));
    config.set_key_value("exposure_time", new ConfigOptionFloat(2.0));
    config.set_key_value("initial_exposure_time", new ConfigOptionFloat(3.0));
    config.set_key_value("min_exposure_time", new ConfigOptionFloat(0.0));
    config.set_key_value("max_exposure_time", new ConfigOptionFloat(100.0));
    config.set_key_value("min_initial_exposure_time", new ConfigOptionFloat(0.0));
    config.set_key_value("max_initial_exposure_time", new ConfigOptionFloat(100.0));
    config.set_key_value("fast_tilt_time", new ConfigOptionFloat(1.0));
    config.set_key_value("slow_tilt_time", new ConfigOptionFloat(2.0));
    return config;
}

class SlaCubeFixture
{
public:
    explicit SlaCubeFixture(DynamicPrintConfig config,
                            TriangleMesh          mesh = {},
                            sla::SupportPoints    support_points = {})
    {
        ModelObject *object = model.add_object();
        if (mesh.empty())
            mesh = make_cube(10.0, 10.0, 4.0);
        object->add_volume(std::move(mesh));
        if (!support_points.empty()) {
            object->sla_support_points = std::move(support_points);
            object->sla_points_status = sla::PointsStatus::UserModified;
        }
        object->add_instance();
        object->ensure_on_bed();
        apply_status = print.apply(model, std::move(config));
    }

    Model              model;
    SLAPrint           print;
    SLAPrint::ApplyStatus apply_status = SLAPrint::APPLY_STATUS_UNCHANGED;
};

double support_slice_area(const SLAPrintObject &object)
{
    double result = 0.;
    for (const SLAPrintObject::SliceRecord &record : object.get_slice_index())
        for (const ExPolygon &expolygon : record.get_slice(soSupport))
            result += std::abs(expolygon.area());
    return result;
}

struct SlaRunSummary
{
    std::size_t layer_count = 0;
    double      object_material = 0.0;
};

SlaRunSummary process_fixture(double layer_height)
{
    SlaCubeFixture fixture(sla_config(layer_height));
    REQUIRE(fixture.apply_status != SLAPrint::APPLY_STATUS_UNCHANGED);
    REQUIRE(fixture.print.objects().size() == 1);

    fixture.print.process();

    REQUIRE(fixture.print.finished());
    REQUIRE(fixture.print.is_step_done(slaposObjectSlice));
    REQUIRE(fixture.print.is_step_done(slaposSliceSupports));

    const SLAPrintObject *object = fixture.print.objects().front();
    const auto &slice_index = object->get_slice_index();
    REQUIRE(!slice_index.empty());
    REQUIRE(slice_index.size() == fixture.print.print_layers().size());

    for (const SLAPrintObject::SliceRecord &record : slice_index) {
        REQUIRE(record.is_valid());
        CHECK(record.get_slice(soSupport).empty());

        const ExPolygons &model_slice = record.get_slice(soModel);
        REQUIRE(model_slice.size() == 1);
        CHECK(model_slice.front().holes.empty());
        const double area_mm2 = model_slice.front().area() /
                                (scaled<double>(1.0) * scaled<double>(1.0));
        CHECK_THAT(area_mm2, Catch::Matchers::WithinAbs(100.0, 1e-4));
    }

    for (std::size_t i = 1; i < fixture.print.print_layers().size(); ++i)
        CHECK(fixture.print.print_layers()[i - 1].level() < fixture.print.print_layers()[i].level());

    const SLAPrintStatistics &stats = fixture.print.print_statistics();
    CHECK(stats.layers_times.size() == slice_index.size());
    CHECK(stats.fast_layers_count == slice_index.size());
    CHECK(stats.slow_layers_count == 0);
    CHECK_THAT(stats.support_used_material, Catch::Matchers::WithinAbs(0.0, 1e-9));
    CHECK(stats.objects_used_material > 0.0);
    CHECK(stats.estimated_print_time > 0.0);

    return {slice_index.size(), stats.objects_used_material};
}

struct SlaStatisticsRun
{
    std::vector<double> layer_times;
    std::vector<double> model_slice_areas;
    double object_material = 0.0;
    double support_material = 0.0;
    double estimated_time = 0.0;
    std::size_t fast_layers = 0;
    std::size_t slow_layers = 0;
};

DynamicPrintConfig sla_statistics_config(double initial_exposure,
                                         double regular_exposure,
                                         double area_fill,
                                         double fast_tilt,
                                         double slow_tilt)
{
    DynamicPrintConfig config = sla_config(0.5);
    config.set_key_value("faded_layers", new ConfigOptionInt(3));
    config.set_key_value("initial_exposure_time", new ConfigOptionFloat(initial_exposure));
    config.set_key_value("exposure_time", new ConfigOptionFloat(regular_exposure));
    config.set_key_value("area_fill", new ConfigOptionFloat(area_fill));
    config.set_key_value("fast_tilt_time", new ConfigOptionFloat(fast_tilt));
    config.set_key_value("slow_tilt_time", new ConfigOptionFloat(slow_tilt));
    return config;
}

SlaStatisticsRun run_sla_statistics(DynamicPrintConfig config)
{
    SlaCubeFixture fixture(std::move(config), make_cube(10.0, 10.0, 3.0));
    REQUIRE(fixture.apply_status != SLAPrint::APPLY_STATUS_UNCHANGED);
    REQUIRE(fixture.print.validate().string.empty());
    fixture.print.process();
    REQUIRE(fixture.print.finished());
    REQUIRE(fixture.print.objects().size() == 1);

    const SLAPrintObject *object = fixture.print.objects().front();
    const auto &slice_index = object->get_slice_index();
    REQUIRE(slice_index.size() == fixture.print.print_layers().size());
    REQUIRE(slice_index.size() == 6);

    SlaStatisticsRun result;
    result.model_slice_areas.reserve(slice_index.size());
    const double mm2_scale = scaled<double>(1.0) * scaled<double>(1.0);
    for (const SLAPrintObject::SliceRecord &record : slice_index) {
        REQUIRE(record.is_valid());
        CHECK(record.get_slice(soSupport).empty());
        const ExPolygons &model_slice = record.get_slice(soModel);
        REQUIRE(model_slice.size() == 1);
        const double model_slice_area = model_slice.front().area() / mm2_scale;
        CHECK_THAT(model_slice_area, Catch::Matchers::WithinAbs(100.0, 1e-4));
        result.model_slice_areas.push_back(model_slice_area);
    }

    const SLAPrintStatistics &stats = fixture.print.print_statistics();
    result.layer_times = stats.layers_times;
    result.object_material = stats.objects_used_material;
    result.support_material = stats.support_used_material;
    result.estimated_time = stats.estimated_print_time;
    result.fast_layers = stats.fast_layers_count;
    result.slow_layers = stats.slow_layers_count;
    REQUIRE(result.layer_times.size() == slice_index.size());
    CHECK_THAT(result.object_material, Catch::Matchers::WithinAbs(300.0, 1e-3));
    CHECK_THAT(result.support_material, Catch::Matchers::WithinAbs(0.0, 1e-9));
    double layer_time_sum = 0.0;
    for (const double layer_time : result.layer_times) {
        CHECK(std::isfinite(layer_time));
        CHECK(layer_time >= 0.0);
        layer_time_sum += layer_time;
    }
    CHECK_THAT(result.estimated_time, Catch::Matchers::WithinAbs(layer_time_sum, 1e-8));
    return result;
}

void check_same_sla_geometry(const SlaStatisticsRun &expected, const SlaStatisticsRun &actual)
{
    CHECK(actual.model_slice_areas.size() == expected.model_slice_areas.size());
    if (actual.model_slice_areas.size() == expected.model_slice_areas.size())
        for (std::size_t i = 0; i < actual.model_slice_areas.size(); ++i)
            CHECK_THAT(actual.model_slice_areas[i], Catch::Matchers::WithinAbs(expected.model_slice_areas[i], 1e-7));
    CHECK_THAT(actual.object_material, Catch::Matchers::WithinRel(expected.object_material, 1e-8));
    CHECK_THAT(actual.support_material, Catch::Matchers::WithinAbs(expected.support_material, 1e-9));
}

void check_sla_layer_times(const SlaStatisticsRun &run, std::vector<double> expected)
{
    REQUIRE(run.layer_times.size() == expected.size());
    std::sort(expected.begin(), expected.end());
    std::vector<double> actual = run.layer_times;
    std::sort(actual.begin(), actual.end());
    for (std::size_t i = 0; i < expected.size(); ++i)
        CHECK_THAT(actual[i], Catch::Matchers::WithinAbs(expected[i], 1e-8));
}

struct PillarConnectionRun
{
    double support_volume = 0.0;
    double model_volume = 0.0;
    std::size_t support_vertices = 0;
    std::size_t support_faces = 0;
    std::size_t separated_support_slices = 0;
    double max_support_centroid_spacing_mm = 0.0;
};

DynamicPrintConfig pillar_connection_config(const SLAPillarConnectionMode mode)
{
    DynamicPrintConfig config = sla_config(1.0);
    config.set_deserialize_strict({
        { "supports_enable",                 true },
        { "support_buildplate_only",         false },
        { "support_object_elevation",        15.0 },
        { "support_head_front_diameter",     1.0 },
        { "support_head_penetration",        0.2 },
        { "support_head_width",              1.0 },
        { "support_pillar_diameter",         2.0 },
        { "support_base_diameter",           4.0 },
        { "support_base_height",             1.0 },
        { "support_base_safety_distance",    0.5 },
        { "support_critical_angle",          45.0 },
        // Keep the two manually supplied heads out of one ground cluster: their 3 mm
        // 3D separation must exceed this clustering limit, while interconnect() still
        // accepts the pair through support_max_pillar_link_distance below.
        { "support_max_bridge_length",       2.5 },
        { "support_max_pillar_link_distance", 10.0 },
        { "support_max_bridges_on_pillar",   8 },
        { "support_pillar_widening_factor",  0.0 },
    });
    config.set_key_value("support_pillar_connection_mode",
                         new ConfigOptionEnum<SLAPillarConnectionMode>(mode));
    return config;
}

PillarConnectionRun process_pillar_connection(const SLAPillarConnectionMode mode,
                                               const double                    spacing)
{
    DynamicPrintConfig config = pillar_connection_config(mode);
    sla::SupportPoints points{
        sla::SupportPoint(float(5.0 - spacing / 2.0), 5.f, 0.f, 0.5f, true),
        sla::SupportPoint(float(5.0 + spacing / 2.0), 5.f, 0.f, 0.5f, false),
    };
    SlaCubeFixture fixture(std::move(config), make_cube(10.0, 10.0, 4.0), std::move(points));
    REQUIRE(fixture.apply_status != SLAPrint::APPLY_STATUS_UNCHANGED);
    REQUIRE(fixture.print.validate().string.empty());
    REQUIRE(fixture.print.objects().size() == 1);

    fixture.print.process();
    REQUIRE(fixture.print.finished());
    const SLAPrintObject *object = fixture.print.objects().front();
    REQUIRE(object->is_step_done(slaposSupportTree));

    TriangleMesh support = object->support_mesh();
    REQUIRE_FALSE(support.empty());
    const double support_volume = std::abs(static_cast<double>(support.volume()));
    const double model_volume = std::abs(static_cast<double>(fixture.model.objects.front()->raw_mesh().volume()));
    REQUIRE(std::isfinite(support_volume));
    REQUIRE(std::isfinite(model_volume));
    REQUIRE(support_volume > 0.0);
    REQUIRE(model_volume > 0.0);

    // A public slice-level witness that the two heads stayed separate.  Base polygons
    // may overlap at the bottom, so search every support slice and use the largest
    // separation between distinct positive-area ExPolygon centroids.
    std::size_t separated_support_slices = 0;
    double max_support_centroid_spacing_mm = 0.0;
    for (const SLAPrintObject::SliceRecord &record : object->get_slice_index()) {
        std::vector<Vec2d> centers;
        for (const ExPolygon &expolygon : record.get_slice(soSupport)) {
            if (expolygon.empty() || expolygon.area() <= 0.0)
                continue;
            const Point center = expolygon.contour.centroid();
            centers.emplace_back(unscaled<double>(center.x()), unscaled<double>(center.y()));
        }
        double slice_spacing = 0.0;
        for (std::size_t i = 0; i < centers.size(); ++i)
            for (std::size_t j = i + 1; j < centers.size(); ++j)
                slice_spacing = std::max(slice_spacing, (centers[i] - centers[j]).norm());
        if (slice_spacing > 2.0) {
            ++separated_support_slices;
            max_support_centroid_spacing_mm = std::max(max_support_centroid_spacing_mm, slice_spacing);
        }
    }
    REQUIRE(separated_support_slices > 0);
    REQUIRE(max_support_centroid_spacing_mm > 2.0);
    return {support_volume, model_volume, support.its.vertices.size(), support.its.indices.size(),
            separated_support_slices, max_support_centroid_spacing_mm};
}

} // namespace

TEST_CASE("SLA cube processing creates one area-preserving model slice per layer", "[SLAPrint][SLAPipeline]")
{
    const SlaRunSummary run = process_fixture(1.0);

    CHECK(run.layer_count == 4);
    CHECK_THAT(run.object_material, Catch::Matchers::WithinRel(400.0, 0.02));
}

TEST_CASE("SLA layer height changes the number of sliced layers without changing volume", "[SLAPrint][SLAPipeline]")
{
    const SlaRunSummary coarse = process_fixture(1.0);
    const SlaRunSummary fine   = process_fixture(0.5);

    CHECK(coarse.layer_count == 4);
    CHECK(fine.layer_count == 8);
    CHECK_THAT(fine.object_material, Catch::Matchers::WithinRel(coarse.object_material, 0.02));
}

TEST_CASE("SLA exposure validation rejects values outside printer bounds", "[SLAPrint][SLAPipeline]")
{
    SlaCubeFixture valid(sla_config(1.0));
    CHECK(valid.print.validate().string.empty());

    DynamicPrintConfig invalid_config = sla_config(1.0);
    invalid_config.set_key_value("max_exposure_time", new ConfigOptionFloat(5.0));
    invalid_config.set_key_value("exposure_time", new ConfigOptionFloat(6.0));
    SlaCubeFixture invalid(std::move(invalid_config));
    CHECK_FALSE(invalid.print.validate().string.empty());
}

TEST_CASE("SLA exposure limits accept inclusive endpoints and reject either side",
          "[SLAPrint][SLAPipeline][SLAExposureBounds]")
{
    DynamicPrintConfig zero_config = sla_config(1.0);
    zero_config.set_key_value("min_exposure_time", new ConfigOptionFloat(0.0));
    zero_config.set_key_value("exposure_time", new ConfigOptionFloat(0.0));
    zero_config.set_key_value("max_exposure_time", new ConfigOptionFloat(0.0));
    zero_config.set_key_value("min_initial_exposure_time", new ConfigOptionFloat(0.0));
    zero_config.set_key_value("initial_exposure_time", new ConfigOptionFloat(0.0));
    zero_config.set_key_value("max_initial_exposure_time", new ConfigOptionFloat(0.0));
    SlaCubeFixture zero_endpoint(std::move(zero_config));
    CHECK(zero_endpoint.print.validate().string.empty());

    DynamicPrintConfig equality_config = sla_config(1.0);
    equality_config.set_key_value("min_exposure_time", new ConfigOptionFloat(2.0));
    equality_config.set_key_value("exposure_time", new ConfigOptionFloat(2.0));
    equality_config.set_key_value("max_exposure_time", new ConfigOptionFloat(2.0));
    equality_config.set_key_value("min_initial_exposure_time", new ConfigOptionFloat(3.0));
    equality_config.set_key_value("initial_exposure_time", new ConfigOptionFloat(3.0));
    equality_config.set_key_value("max_initial_exposure_time", new ConfigOptionFloat(3.0));
    SlaCubeFixture equality_endpoint(std::move(equality_config));
    CHECK(equality_endpoint.print.validate().string.empty());

    const auto rejects_bound = [](const char *key, double value) {
        DynamicPrintConfig config = sla_config(1.0);
        config.set_key_value(key, new ConfigOptionFloat(value));
        SlaCubeFixture fixture(std::move(config));
        return !fixture.print.validate().string.empty();
    };
    CHECK(rejects_bound("min_exposure_time", 2.1));
    CHECK(rejects_bound("max_exposure_time", 1.9));
    CHECK(rejects_bound("min_initial_exposure_time", 3.1));
    CHECK(rejects_bound("max_initial_exposure_time", 2.9));
}

TEST_CASE("SLA exposure durations determine fade-weighted layer statistics",
          "[SLAPrint][SLAPipeline][SLAExposureStatistics]")
{
    DynamicPrintConfig nominal_config = sla_statistics_config(4.0, 2.0, 100.0, 0.0, 0.0);
    const SlaStatisticsRun nominal = run_sla_statistics(nominal_config);
    CHECK(nominal.fast_layers == 6);
    CHECK(nominal.slow_layers == 0);
    // For faded_layers=3, the six exposure weights are 3 initial layers and
    // fade fractions 3/4, 1/2, and 1/4: 4.5*initial + 1.5*regular.
    CHECK_THAT(nominal.estimated_time, Catch::Matchers::WithinAbs(21.0, 1e-8));
    check_sla_layer_times(nominal, { 2.5, 3.0, 3.5, 4.0, 4.0, 4.0 });

    DynamicPrintConfig regular_config = nominal_config;
    regular_config.set_key_value("exposure_time", new ConfigOptionFloat(2.5));
    const SlaStatisticsRun regular = run_sla_statistics(std::move(regular_config));
    check_same_sla_geometry(nominal, regular);
    CHECK(regular.fast_layers == 6);
    CHECK(regular.slow_layers == 0);
    CHECK_THAT(regular.estimated_time - nominal.estimated_time,
               Catch::Matchers::WithinAbs(0.75, 1e-8));
    check_sla_layer_times(regular, { 2.875, 3.25, 3.625, 4.0, 4.0, 4.0 });

    DynamicPrintConfig initial_config = nominal_config;
    initial_config.set_key_value("initial_exposure_time", new ConfigOptionFloat(5.0));
    const SlaStatisticsRun initial = run_sla_statistics(std::move(initial_config));
    check_same_sla_geometry(nominal, initial);
    CHECK(initial.fast_layers == 6);
    CHECK(initial.slow_layers == 0);
    CHECK_THAT(initial.estimated_time - nominal.estimated_time,
               Catch::Matchers::WithinAbs(4.5, 1e-8));
    check_sla_layer_times(initial, { 2.75, 3.5, 4.25, 5.0, 5.0, 5.0 });

    const SlaStatisticsRun zero = run_sla_statistics(sla_statistics_config(0.0, 0.0, 100.0, 0.0, 0.0));
    check_same_sla_geometry(nominal, zero);
    CHECK(zero.fast_layers == 6);
    CHECK(zero.slow_layers == 0);
    CHECK_THAT(zero.estimated_time, Catch::Matchers::WithinAbs(0.0, 1e-8));
    check_sla_layer_times(zero, { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 });
}

TEST_CASE("SLA fast and slow tilt durations affect only their classified layers",
          "[SLAPrint][SLAPipeline][SLATiltStatistics]")
{
    DynamicPrintConfig fast_config = sla_statistics_config(2.0, 2.0, 100.0, 0.0, 0.0);
    const SlaStatisticsRun fast_zero = run_sla_statistics(fast_config);
    fast_config.set_key_value("fast_tilt_time", new ConfigOptionFloat(2.0));
    const SlaStatisticsRun fast_two = run_sla_statistics(std::move(fast_config));
    check_same_sla_geometry(fast_zero, fast_two);
    CHECK(fast_zero.fast_layers == 6);
    CHECK(fast_zero.slow_layers == 0);
    CHECK_THAT(fast_two.estimated_time - fast_zero.estimated_time,
               Catch::Matchers::WithinAbs(12.0, 1e-8));
    check_sla_layer_times(fast_zero, { 2.0, 2.0, 2.0, 2.0, 2.0, 2.0 });
    check_sla_layer_times(fast_two, { 4.0, 4.0, 4.0, 4.0, 4.0, 4.0 });

    DynamicPrintConfig slow_config = sla_statistics_config(2.0, 2.0, 0.0, 0.0, 0.0);
    const SlaStatisticsRun slow_zero = run_sla_statistics(slow_config);
    slow_config.set_key_value("slow_tilt_time", new ConfigOptionFloat(3.0));
    const SlaStatisticsRun slow_three = run_sla_statistics(std::move(slow_config));
    check_same_sla_geometry(slow_zero, slow_three);
    CHECK(slow_zero.fast_layers == 0);
    CHECK(slow_zero.slow_layers == 6);
    CHECK_THAT(slow_three.estimated_time - slow_zero.estimated_time,
               Catch::Matchers::WithinAbs(18.0, 1e-8));
    check_sla_layer_times(slow_zero, { 2.0, 2.0, 2.0, 2.0, 2.0, 2.0 });
    check_sla_layer_times(slow_three, { 5.0, 5.0, 5.0, 5.0, 5.0, 5.0 });
}

TEST_CASE("SLA area fill selects fast versus slow layer timing",
          "[SLAPrint][SLAPipeline][SLAAreaFillStatistics]")
{
    DynamicPrintConfig slow_config = sla_statistics_config(2.0, 2.0, 0.0, 0.0, 2.0);
    const SlaStatisticsRun slow = run_sla_statistics(slow_config);

    DynamicPrintConfig fast_config = slow_config;
    fast_config.set_key_value("area_fill", new ConfigOptionFloat(100.0));
    const SlaStatisticsRun fast = run_sla_statistics(std::move(fast_config));
    check_same_sla_geometry(slow, fast);
    CHECK(slow.fast_layers == 0);
    CHECK(slow.slow_layers == 6);
    CHECK(fast.fast_layers == 6);
    CHECK(fast.slow_layers == 0);
    CHECK_THAT(slow.estimated_time, Catch::Matchers::WithinAbs(24.0, 1e-8));
    CHECK_THAT(fast.estimated_time, Catch::Matchers::WithinAbs(12.0, 1e-8));
    CHECK_THAT(slow.estimated_time - fast.estimated_time,
               Catch::Matchers::WithinAbs(12.0, 1e-8));
    check_sla_layer_times(slow, { 4.0, 4.0, 4.0, 4.0, 4.0, 4.0 });
    check_sla_layer_times(fast, { 2.0, 2.0, 2.0, 2.0, 2.0, 2.0 });
}

TEST_CASE("SLA hollowing creates a smaller interior volume", "[SLAFeatures][SLAHollowingToggle]")
{
    // Run the disabled control through the same public task boundary.  The
    // hollowing step is still marked complete, but its public interior product
    // must remain absent when hollowing_enable is false.
    DynamicPrintConfig disabled_config = sla_config(1.0);
    disabled_config.set_key_value("hollowing_enable", new ConfigOptionBool(false));
    disabled_config.set_key_value("hollowing_min_thickness", new ConfigOptionFloat(1.0));
    disabled_config.set_key_value("hollowing_quality", new ConfigOptionFloat(0.2));
    disabled_config.set_key_value("hollowing_closing_distance", new ConfigOptionFloat(0.25));
    SlaCubeFixture disabled(std::move(disabled_config), make_cube(20.0, 20.0, 10.0));
    PrintBase::TaskParams disabled_task;
    disabled_task.to_object_step = slaposHollowing;
    disabled.print.set_task(disabled_task);
    disabled.print.process();
    REQUIRE(disabled.print.objects().size() == 1);
    const SLAPrintObject *disabled_object = disabled.print.objects().front();
    REQUIRE(disabled_object->is_step_done(slaposHollowing));
    CHECK(disabled_object->hollowed_interior_mesh().empty());

    DynamicPrintConfig config = sla_config(1.0);
    config.set_key_value("hollowing_enable", new ConfigOptionBool(true));
    config.set_key_value("hollowing_min_thickness", new ConfigOptionFloat(1.0));
    config.set_key_value("hollowing_quality", new ConfigOptionFloat(0.2));
    config.set_key_value("hollowing_closing_distance", new ConfigOptionFloat(0.25));

    SlaCubeFixture fixture(std::move(config), make_cube(20.0, 20.0, 10.0));
    const double solid_volume = fixture.model.objects.front()->raw_mesh().volume();

    // The production drill_holes step is currently disabled, so stop after
    // hollowing rather than asserting on the empty downstream hollow mesh.
    PrintBase::TaskParams task;
    task.to_object_step = slaposHollowing;
    fixture.print.set_task(task);
    fixture.print.process();

    REQUIRE(fixture.print.objects().size() == 1);
    const SLAPrintObject *object = fixture.print.objects().front();
    REQUIRE(object->is_step_done(slaposHollowing));

    const indexed_triangle_set &interior = object->hollowed_interior_mesh();
    REQUIRE_FALSE(interior.empty());
    TriangleMesh interior_mesh(interior);
    const double interior_volume = std::abs(interior_mesh.volume());
    CHECK(interior_volume > 0.);
    CHECK(interior_volume < solid_volume);
}

TEST_CASE("SLA pad processing creates printable pad geometry and slices", "[SLAFeatures][SLAPadToggle]")
{
    // A complete run with the feature disabled must not expose a pad mesh or
    // a pad milestone product.  The following enabled run is the positive
    // control for the same public getters and slice-level output.
    DynamicPrintConfig disabled_config = sla_config(1.0);
    disabled_config.set_key_value("pad_enable", new ConfigOptionBool(false));
    disabled_config.set_key_value("pad_wall_thickness", new ConfigOptionFloat(1.0));
    disabled_config.set_key_value("pad_wall_height", new ConfigOptionFloat(1.0));
    disabled_config.set_key_value("pad_brim_size", new ConfigOptionFloat(2.0));
    SlaCubeFixture disabled(std::move(disabled_config));
    disabled.print.process();
    REQUIRE(disabled.print.objects().size() == 1);
    const SLAPrintObject *disabled_object = disabled.print.objects().front();
    REQUIRE(disabled_object->is_step_done(slaposPad));
    CHECK_FALSE(disabled_object->has_mesh(slaposPad));
    CHECK(disabled_object->pad_mesh().empty());
    CHECK_THAT(support_slice_area(*disabled_object), Catch::Matchers::WithinAbs(0.0, 1e-9));

    DynamicPrintConfig config = sla_config(1.0);
    config.set_key_value("pad_enable", new ConfigOptionBool(true));
    config.set_key_value("pad_wall_thickness", new ConfigOptionFloat(1.0));
    config.set_key_value("pad_wall_height", new ConfigOptionFloat(1.0));
    config.set_key_value("pad_brim_size", new ConfigOptionFloat(2.0));

    SlaCubeFixture fixture(std::move(config));
    REQUIRE(fixture.apply_status != SLAPrint::APPLY_STATUS_UNCHANGED);
    fixture.print.process();

    REQUIRE(fixture.print.finished());
    const SLAPrintObject *object = fixture.print.objects().front();
    REQUIRE(object->is_step_done(slaposPad));
    REQUIRE(object->has_mesh(slaposPad));
    REQUIRE_FALSE(object->pad_mesh().empty());
    TriangleMesh pad_mesh = object->pad_mesh();
    CHECK(pad_mesh.volume() > 0.f);
    CHECK(support_slice_area(*object) > 0.);
}

TEST_CASE("SLA manual supports elevate the object and emit support slices", "[SLAFeatures]")
{
    DynamicPrintConfig config = sla_config(1.0);
    config.set_key_value("supports_enable", new ConfigOptionBool(true));
    config.set_key_value("support_object_elevation", new ConfigOptionFloat(5.0));
    config.set_key_value("support_head_front_diameter", new ConfigOptionFloat(1.0));
    config.set_key_value("support_pillar_diameter", new ConfigOptionFloat(1.0));
    config.set_key_value("support_base_diameter", new ConfigOptionFloat(2.0));
    config.set_key_value("support_base_height", new ConfigOptionFloat(1.0));

    sla::SupportPoints points{sla::SupportPoint(5.f, 5.f, 0.f, 0.5f, true)};
    SlaCubeFixture fixture(std::move(config), make_cube(10.0, 10.0, 4.0), std::move(points));
    REQUIRE(fixture.apply_status != SLAPrint::APPLY_STATUS_UNCHANGED);
    fixture.print.process();

    REQUIRE(fixture.print.finished());
    const SLAPrintObject *object = fixture.print.objects().front();
    CHECK_THAT(object->get_elevation(), Catch::Matchers::WithinAbs(5.0, 1e-6));
    REQUIRE(object->is_step_done(slaposSupportTree));
    REQUIRE(object->get_support_points().size() == 1);
    REQUIRE_FALSE(object->support_mesh().empty());
    TriangleMesh support_mesh = object->support_mesh();
    CHECK(support_mesh.volume() > 0.f);
    CHECK(object->support_mesh().bounding_box().min.z() < -EPSILON);
    CHECK(support_slice_area(*object) > 0.);
}


TEST_CASE("SLA buildplate-only setting omits blocked support while retaining a clear buildplate route",
          "[SLAPrint][SLASupportRouting][SettingsVariance][BuildplateOnlySupport]")
{
    const bool buildplate_only = GENERATE(false, true);
    CAPTURE(buildplate_only);

    DynamicPrintConfig config = sla_config(1.0);
    config.set_deserialize_strict({
        { "supports_enable",                       true },
        { "support_object_elevation",              5.0 },
        { "support_head_front_diameter",           1.0 },
        { "support_head_penetration",              0.2 },
        { "support_head_width",                    1.0 },
        { "support_pillar_diameter",               1.0 },
        { "support_small_pillar_diameter_percent", 50.0 },
        { "support_base_diameter",                 2.0 },
        { "support_base_height",                   1.0 },
        { "support_base_safety_distance",          0.5 },
        { "support_critical_angle",                45.0 },
        { "support_max_bridge_length",             0.5 },
        { "support_max_pillar_link_distance",      10.0 },
        { "support_max_bridges_on_pillar",         8 },
        { "support_pillar_widening_factor",        0.0 },
    });
    config.set_key_value("support_pillar_connection_mode",
                         new ConfigOptionEnum<SLAPillarConnectionMode>(slapcmDynamic));
    config.set_key_value("support_buildplate_only", new ConfigOptionBool(buildplate_only));

    // ensure_on_bed() leaves the lower block at z=0. Elevation 5 mm clears
    // the support head width for public SLAPrint validation without moving
    // the model or manual points in model coordinates. The ceiling is shifted
    // right: (8,5,12) is blocked by the lower block, while (12,5,12) is clear
    // to the build plate and spans the empty height gap in both settings.
    // A third clear head below the lower block grounds that component too.
    TriangleMesh model = make_cube(10.0, 10.0, 5.0);
    TriangleMesh ceiling = make_cube(6.0, 6.0, 2.0);
    ceiling.translate(Vec3f(7.f, 2.f, 12.f));
    model.merge(ceiling);
    sla::SupportPoints points{
        sla::SupportPoint(8.f, 5.f, 12.f, 0.5f, true),
        sla::SupportPoint(12.f, 5.f, 12.f, 0.5f, true),
        sla::SupportPoint(1.f, 1.f, 0.f, 0.5f, true),
    };

    SlaCubeFixture fixture(std::move(config), std::move(model), std::move(points));
    REQUIRE(fixture.apply_status != SLAPrint::APPLY_STATUS_UNCHANGED);
    INFO("SLA support_buildplate_only validation: " << fixture.print.validate().string);
    REQUIRE(fixture.print.validate().string.empty());
    fixture.print.process();

    REQUIRE(fixture.print.finished());
    REQUIRE(fixture.print.objects().size() == 1);
    const SLAPrintObject *object = fixture.print.objects().front();
    REQUIRE(object->is_step_done(slaposSupportTree));
    REQUIRE(object->get_support_points().size() == 3);
    CHECK_THAT(object->get_elevation(), Catch::Matchers::WithinAbs(5.0, 1e-6));

    // Both setting values preserve model bounds and the same exact slice profile.
    const BoundingBoxf3 model_bounds = object->transformed_mesh().bounding_box();
    CHECK_THAT(model_bounds.min.x(), Catch::Matchers::WithinAbs(0.0, 1e-4));
    CHECK_THAT(model_bounds.min.y(), Catch::Matchers::WithinAbs(0.0, 1e-4));
    CHECK_THAT(model_bounds.min.z(), Catch::Matchers::WithinAbs(0.0, 1e-4));
    CHECK_THAT(model_bounds.max.x(), Catch::Matchers::WithinAbs(13.0, 1e-4));
    CHECK_THAT(model_bounds.max.y(), Catch::Matchers::WithinAbs(10.0, 1e-4));
    CHECK_THAT(model_bounds.max.z(), Catch::Matchers::WithinAbs(14.0, 1e-4));

    const auto &slice_index = object->get_slice_index();
    REQUIRE(slice_index.size() == fixture.print.print_layers().size());
    std::size_t lower_body_layers = 0;
    std::size_t upper_body_layers = 0;
    double total_model_slice_area_mm2 = 0.0;
    const double scale2 = scaled<double>(1.0) * scaled<double>(1.0);
    for (const SLAPrintObject::SliceRecord &record : slice_index) {
        const ExPolygons &model_slice = record.get_slice(soModel);
        double area_mm2 = 0.0;
        for (const ExPolygon &expolygon : model_slice)
            area_mm2 += std::abs(expolygon.area()) / scale2;
        total_model_slice_area_mm2 += area_mm2;

        if (area_mm2 > 70.0) {
            ++lower_body_layers;
            CHECK_THAT(area_mm2, Catch::Matchers::WithinAbs(100.0, 1e-4));
        } else if (area_mm2 > 1.0) {
            ++upper_body_layers;
            CHECK_THAT(area_mm2, Catch::Matchers::WithinAbs(36.0, 1e-4));
        } else {
            CHECK_THAT(area_mm2, Catch::Matchers::WithinAbs(0.0, 1e-4));
            CHECK(model_slice.empty());
        }

        if (!model_slice.empty()) {
            REQUIRE(model_slice.size() == 1);
            CHECK(model_slice.front().holes.empty());
        }
    }
    CHECK(lower_body_layers == 5);
    CHECK(upper_body_layers == 2);
    CHECK_THAT(total_model_slice_area_mm2, Catch::Matchers::WithinAbs(572.0, 1e-4));

    // The clear head at x=12 supplies support in both rows. At z=5.1, only
    // normal routing may also contact the intervening body near blocked x=8.
    const TriangleMesh support = object->support_mesh();
    REQUIRE_FALSE(support.empty());
    CHECK(support_slice_area(*object) > 0.0);
    const Polygons section = slice_mesh(support.its, 5.1f, MeshSlicingParams{});
    REQUIRE_FALSE(section.empty());
    // Clip to the independently known lower-block footprint, which excludes
    // the clear pillar at x=12 while allowing model-facing contact to drift.
    const Polygon blocked_region{
        Point::new_scale(0.0, 0.0), Point::new_scale(10.0, 0.0),
        Point::new_scale(10.0, 10.0), Point::new_scale(0.0, 10.0),
    };
    const ExPolygons blocked_section = intersection_ex(section, Polygons{blocked_region});
    double blocked_area_mm2 = 0.0;
    for (const ExPolygon &expolygon : blocked_section)
        blocked_area_mm2 += std::abs(expolygon.area()) / scale2;
    if (buildplate_only)
        CHECK_THAT(blocked_area_mm2, Catch::Matchers::WithinAbs(0.0, 1e-6));
    else
        CHECK(blocked_area_mm2 > 0.0);
}

TEST_CASE("SLA blocked model-facing supports anchor to the intervening body",
          "[SLASupportRouting][ModelFacingSupport]")
{
    // The lower block occupies x/y=[0,10] and ends at z=5. A second component
    // has an underside at z=12. The support head fits in the 7 mm gap, but its
    // bridge limit is too short to move outside the lower block and reach ground.
    TriangleMesh model = make_cube(10.0, 10.0, 5.0);
    TriangleMesh ceiling = make_cube(6.0, 6.0, 2.0);
    ceiling.translate(Vec3f(2.f, 2.f, 12.f));
    model.merge(ceiling);

    sla::SupportTreeConfig config;
    config.object_elevation_mm = 0.0;
    config.ground_facing_only = false;
    config.bridge_slope = M_PI / 4.0;
    config.max_bridge_length_mm = 0.5;
    config.head_front_radius_mm = 0.5;
    config.head_back_radius_mm = 0.5;
    config.head_fallback_radius_mm = 0.25;
    config.head_width_mm = 1.0;
    config.head_penetration_mm = 0.2;
    config.base_radius_mm = 1.0;
    config.base_height_mm = 1.0;

    const sla::SupportPoints points{sla::SupportPoint(5.f, 5.f, 12.f, 0.5f, true)};
    const sla::SupportableMesh input(model.its, points, config);
    sla::SupportTreeBuilder tree;
    // false means normal completion; true means the support-tree build aborted.
    REQUIRE_FALSE(sla::SupportTreeBuildsteps::execute(tree, input));
    REQUIRE(tree.heads().size() == 1);

    const sla::Head &head = tree.heads().front();
    REQUIRE(head.is_valid());
    REQUIRE(head.pillar_id >= 0);
    CHECK(head.bridge_id == sla::SupportTreeNode::ID_UNSET);
    const sla::Pillar &anchor_pillar = tree.pillars().at(size_t(head.pillar_id));

    // The configured head spans 2.8 mm: two 0.5 mm end radii at each end,
    // a 1 mm shaft, less 0.2 mm penetration. Its 45-degree tail rises about
    // 2 mm. From the junction near z=9.7, the downward ray meets the lower
    // block at z=5, placing the anchor-pillar endpoint near z=7.
    CHECK(anchor_pillar.endpoint().z() > 6.5);
    CHECK(anchor_pillar.endpoint().z() < 7.5);

    TriangleMesh support{tree.retrieve_mesh(sla::MeshType::Support)};
    REQUIRE_FALSE(support.empty());
    const auto support_bounds = support.bounding_box();
    CHECK(support_bounds.min.z() > 4.0f);
    CHECK(support_bounds.min.z() < 5.5f);

    // A positive mesh cross-section just above the independently known z=5
    // contact confirms that the anchor reaches the intervening model surface.
    const Polygons contact_slice = slice_mesh(support.its, 5.1f, MeshSlicingParams{});
    REQUIRE_FALSE(contact_slice.empty());
    double contact_area = 0.0;
    for (const Polygon &polygon : contact_slice)
        contact_area += std::abs(double(polygon.area()));
    CHECK(contact_area > 0.0);
}

TEST_CASE("SLA tall isolated pillars acquire two printable neighbors",
          "[SLASupportRouting][TallPillarBracing]")
{
    // A 1 mm-wide model leaves the two bracing locations clear. Elevation 50
    // makes the primary pillar exceed the public 35 mm dual-support threshold.
    const TriangleMesh model = make_cube(1.0, 1.0, 1.0);
    sla::SupportTreeConfig config;
    config.object_elevation_mm = 50.0;
    config.bridge_slope = M_PI / 4.0;
    config.head_front_radius_mm = 0.5;
    config.head_back_radius_mm = 0.5;
    config.head_fallback_radius_mm = 0.25;
    config.head_width_mm = 1.0;
    config.head_penetration_mm = 0.2;
    config.base_radius_mm = 1.0;
    config.base_height_mm = 1.0;
    config.max_pillar_link_distance_mm = 10.0;
    config.pillar_connection_mode = sla::PillarConnectionMode::dynamic;

    const sla::SupportPoints points{sla::SupportPoint(0.5f, 0.5f, 0.f, 0.5f, true)};
    const sla::SupportableMesh input(model.its, points, config);
    sla::SupportTreeBuilder tree;
    // false means normal completion; true means the support-tree build aborted.
    REQUIRE_FALSE(sla::SupportTreeBuildsteps::execute(tree, input));
    REQUIRE(tree.pillars().size() >= 3);

    const sla::Pillar &primary = tree.pillars().front();
    CHECK(primary.height > sla::SupportTreeConfig::max_dual_pillar_height_mm);
    CHECK(primary.links >= 2);
    size_t ground_anchored_pillars = 0;
    for (const sla::Pillar &pillar : tree.pillars())
        if (std::abs(pillar.endpoint().z() + 50.0) < 0.1)
            ++ground_anchored_pillars;
    CHECK(ground_anchored_pillars >= 3);

    TriangleMesh support{tree.retrieve_mesh(sla::MeshType::Support)};
    REQUIRE_FALSE(support.empty());
    CHECK(support.bounding_box().min.z() < -49.9f);
}

namespace sla_lifecycle_contracts {

using namespace Slic3r;

namespace {

ModelObject* add_cube_object(Model& model, double size = 10.0, double x_offset = 0.0)
{
    ModelObject* object = model.add_object();
    object->name        = "SLA lifecycle cube";
    object->add_volume(make_cube(size, size, 4.0));
    ModelInstance* instance = object->add_instance();
    instance->set_offset(X, x_offset);
    object->ensure_on_bed();
    return object;
}

class LifecycleFixture
{
public:
    explicit LifecycleFixture(DynamicPrintConfig cfg = sla_config(1.0)) : config(std::move(cfg))
    {
        add_cube_object(model);
        apply_status = print.apply(model, config);
    }

    void process_all()
    {
        print.process();
        REQUIRE(print.finished());
    }

    Model model;
    DynamicPrintConfig config;
    SLAPrint print;
    SLAPrint::ApplyStatus apply_status = SLAPrint::APPLY_STATUS_UNCHANGED;
};

double model_slice_area_mm2(const SLAPrintObject& object)
{
    for (const auto& record : object.get_slice_index()) {
        const ExPolygons& slice = record.get_slice(soModel);
        if (!slice.empty())
            return std::abs(slice.front().area()) / (scaled<double>(1.0) * scaled<double>(1.0));
    }
    return 0.0;
}

std::size_t model_layer_count(const SLAPrintObject& object) { return object.get_slice_index().size(); }

} // namespace

TEST_CASE("SLA reapplying an unchanged model keeps completed state", "[SLAPrint][SLALifecycle]")
{
    LifecycleFixture fixture;
    REQUIRE(fixture.apply_status != SLAPrint::APPLY_STATUS_UNCHANGED);
    fixture.process_all();

    const auto* object = fixture.print.objects().front();
    REQUIRE(object->is_step_done(slaposObjectSlice));
    REQUIRE(fixture.print.finished());
    const auto object_id = object->id();
    const auto status    = fixture.print.apply(fixture.model, fixture.config);

    CHECK(status == SLAPrint::APPLY_STATUS_UNCHANGED);
    CHECK(fixture.print.objects().size() == 1);
    CHECK(fixture.print.objects().front()->id() == object_id);
    CHECK(fixture.print.objects().front()->is_step_done(slaposObjectSlice));
    CHECK(fixture.print.finished());
}

TEST_CASE("SLA raster-only changes invalidate merge while retaining object slices", "[SLAPrint][SLALifecycle]")
{
    LifecycleFixture fixture;
    fixture.process_all();
    const double old_area = model_slice_area_mm2(*fixture.print.objects().front());

    DynamicPrintConfig changed = fixture.config;
    changed.set_key_value("exposure_time", new ConfigOptionFloat(4.0));
    const auto status = fixture.print.apply(fixture.model, std::move(changed));

    CHECK(status == SLAPrint::APPLY_STATUS_INVALIDATED);
    CHECK_FALSE(fixture.print.finished());
    CHECK(fixture.print.objects().front()->is_step_done(slaposObjectSlice));
    fixture.print.process();
    REQUIRE(fixture.print.finished());
    CHECK_THAT(model_slice_area_mm2(*fixture.print.objects().front()), Catch::Matchers::WithinAbs(old_area, 1e-4));
}

TEST_CASE("SLA layer-height changes invalidate slicing and alter layer count", "[SLAPrint][SLALifecycle]")
{
    LifecycleFixture fixture;
    fixture.process_all();
    REQUIRE(model_layer_count(*fixture.print.objects().front()) == 4);

    DynamicPrintConfig changed = fixture.config;
    changed.set_key_value("layer_height", new ConfigOptionFloat(0.5));
    changed.set_key_value("initial_layer_height", new ConfigOptionFloat(0.5));
    const auto status = fixture.print.apply(fixture.model, std::move(changed));

    CHECK(status == SLAPrint::APPLY_STATUS_INVALIDATED);
    CHECK_FALSE(fixture.print.objects().front()->is_step_done(slaposObjectSlice));
    fixture.print.process();
    REQUIRE(fixture.print.finished());
    CHECK(model_layer_count(*fixture.print.objects().front()) == 8);
    CHECK_THAT(model_slice_area_mm2(*fixture.print.objects().front()), Catch::Matchers::WithinAbs(100.0, 1e-4));
}

TEST_CASE("SLA relative correction rebuilds transformed geometry", "[SLAPrint][SLALifecycle]")
{
    DynamicPrintConfig initial = sla_config(1.0);
    initial.set_key_value("relative_correction", new ConfigOptionFloats({1.0, 1.0}));
    initial.set_key_value("relative_correction_x", new ConfigOptionFloat(1.0));
    initial.set_key_value("relative_correction_y", new ConfigOptionFloat(1.0));
    initial.set_key_value("relative_correction_z", new ConfigOptionFloat(1.0));
    LifecycleFixture fixture(std::move(initial));
    fixture.process_all();
    const auto* before     = fixture.print.objects().front();
    const double old_width = before->transformed_mesh().bounding_box().size().x();

    DynamicPrintConfig changed = fixture.config;
    changed.set_key_value("relative_correction", new ConfigOptionFloats({1.0, 1.0}));
    changed.set_key_value("relative_correction_x", new ConfigOptionFloat(1.5));
    changed.set_key_value("relative_correction_y", new ConfigOptionFloat(1.0));
    changed.set_key_value("relative_correction_z", new ConfigOptionFloat(1.0));
    const auto status = fixture.print.apply(fixture.model, std::move(changed));

    CHECK(status == SLAPrint::APPLY_STATUS_INVALIDATED);
    CHECK_FALSE(fixture.print.objects().front()->is_step_done(slaposObjectSlice));
    const auto* after = fixture.print.objects().front();
    CHECK_THAT(after->transformed_mesh().bounding_box().size().x(), Catch::Matchers::WithinRel(old_width * 1.5, 1e-5));
    CHECK_THAT(fixture.print.relative_correction().x(), Catch::Matchers::WithinAbs(1.5, 1e-6));
    fixture.print.process();
    REQUIRE(fixture.print.finished());
    CHECK_THAT(model_slice_area_mm2(*fixture.print.objects().front()), Catch::Matchers::WithinRel(150.0, 0.01));
}

TEST_CASE("SLA model extension and deletion synchronize print objects by ID", "[SLAPrint][SLALifecycle]")
{
    LifecycleFixture fixture;
    fixture.process_all();
    const ObjectID first_id = fixture.model.objects.front()->id();

    add_cube_object(fixture.model, 6.0, 20.0);
    const auto added_status = fixture.print.apply(fixture.model, fixture.config);
    CHECK(added_status == SLAPrint::APPLY_STATUS_INVALIDATED);
    REQUIRE(fixture.print.objects().size() == 2);
    CHECK(fixture.print.get_print_object_by_model_object_id(first_id) != nullptr);
    fixture.print.process();
    REQUIRE(fixture.print.finished());
    CHECK(fixture.print.print_layers().size() == 4);
    CHECK(fixture.print.print_layers().front().slices().size() == 2);

    const ObjectID removed_id = fixture.model.objects.back()->id();
    fixture.model.delete_object(1);
    const auto removed_status = fixture.print.apply(fixture.model, fixture.config);
    CHECK(removed_status == SLAPrint::APPLY_STATUS_INVALIDATED);
    REQUIRE(fixture.print.objects().size() == 1);
    CHECK(fixture.print.get_print_object_by_model_object_id(first_id) != nullptr);
    CHECK(fixture.print.get_print_object_by_model_object_id(removed_id) == nullptr);
    fixture.print.process();
    REQUIRE(fixture.print.finished());
    CHECK(fixture.print.print_layers().front().slices().size() == 1);
}

TEST_CASE("SLA instance scale changes replace transformed print geometry", "[SLAPrint][SLALifecycle]")
{
    LifecycleFixture fixture;
    fixture.process_all();
    const auto* before          = fixture.print.objects().front();
    const Transform3d old_trafo = before->trafo();

    fixture.model.objects.front()->instances.front()->set_scaling_factor(Vec3d(1.5, 1.5, 1.5));
    const auto status = fixture.print.apply(fixture.model, fixture.config);

    CHECK(status == SLAPrint::APPLY_STATUS_INVALIDATED);
    const auto* after = fixture.print.objects().front();
    CHECK_FALSE(after->trafo().isApprox(old_trafo));
    CHECK_THAT(after->transformed_mesh().bounding_box().size().x(), Catch::Matchers::WithinAbs(15.0, 1e-4));
    CHECK_FALSE(after->is_step_done(slaposObjectSlice));
    fixture.print.process();
    REQUIRE(fixture.print.finished());
    CHECK_THAT(model_slice_area_mm2(*fixture.print.objects().front()), Catch::Matchers::WithinRel(225.0, 0.01));
}

TEST_CASE("SLA drain holes transform with inverse-transpose normals and stick-out", "[SLAPrint][SLALifecycle]")
{
    LifecycleFixture fixture;
    ModelObject* object = fixture.model.objects.front();
    object->instances.front()->set_scaling_factor(Vec3d(2.0, 3.0, 4.0));
    const Vec3f input_normal = Vec3f(1.f, 1.f, 1.f).normalized();
    object->sla_drain_holes  = {sla::DrainHole(Vec3f(1.f, 2.f, 1.f), input_normal, 2.f, 5.f)};
    fixture.print.apply(fixture.model, fixture.config);

    REQUIRE(fixture.print.objects().size() == 1);
    const auto* print_object = fixture.print.objects().front();
    const auto holes         = print_object->transformed_drainhole_points();
    REQUIRE(holes.size() == 1);
    const sla::DrainHole& hole  = holes.front();
    const Vec3f base_pos        = (print_object->trafo() * Vec3d(1., 2., 1.)).cast<float>();
    const Vec3f expected_normal = Vec3f(input_normal.x() / 2.f, input_normal.y() / 3.f, input_normal.z() / 4.f);

    CHECK_THAT(hole.radius, Catch::Matchers::WithinAbs(2.0, 1e-5));
    CHECK_THAT(hole.height, Catch::Matchers::WithinAbs(6.0, 1e-5));
    CHECK_THAT((base_pos - hole.pos).norm(), Catch::Matchers::WithinAbs(1.0, 1e-4));
    const Vec3f expected_pos = base_pos - expected_normal.normalized();
    for (int axis = 0; axis < 3; ++axis)
        CHECK_THAT(hole.pos[axis], Catch::Matchers::WithinAbs(expected_pos[axis], 1e-5));
    CHECK_THAT(hole.normal.x(), Catch::Matchers::WithinAbs(expected_normal.x(), 1e-5));
    CHECK_THAT(hole.normal.y(), Catch::Matchers::WithinAbs(expected_normal.y(), 1e-5));
    CHECK_THAT(hole.normal.z(), Catch::Matchers::WithinAbs(expected_normal.z(), 1e-5));
}

TEST_CASE("SLA drain-hole edits invalidate the drill-hole step", "[SLAPrint][SLALifecycle]")
{
    LifecycleFixture fixture;
    fixture.process_all();
    REQUIRE(fixture.print.objects().front()->is_step_done(slaposDrillHoles));

    fixture.model.objects.front()->sla_drain_holes = {sla::DrainHole(Vec3f(1.f, 1.f, 0.f), Vec3f(0.f, 0.f, 1.f), 1.f, 2.f)};
    const auto status                              = fixture.print.apply(fixture.model, fixture.config);

    CHECK(status == SLAPrint::APPLY_STATUS_INVALIDATED);
    CHECK_FALSE(fixture.print.objects().front()->is_step_done(slaposDrillHoles));
    CHECK_FALSE(fixture.print.objects().front()->is_step_done(slaposObjectSlice));
    CHECK(fixture.print.objects().front()->transformed_drainhole_points().size() == 1);
}

TEST_CASE("SLA secondary instance edits synchronize print instances", "[SLAPrint][SLALifecycle]")
{
    LifecycleFixture fixture;
    fixture.process_all();
    REQUIRE(fixture.print.objects().front()->instances().size() == 1);

    ModelInstance* second = fixture.model.objects.front()->add_instance();
    second->set_offset(X, 20.0);
    const auto status = fixture.print.apply(fixture.model, fixture.config);

    CHECK(status == SLAPrint::APPLY_STATUS_INVALIDATED);
    REQUIRE(fixture.print.objects().size() == 1);
    const auto& instances = fixture.print.objects().front()->instances();
    REQUIRE(instances.size() == 2);
    const auto copied = std::find_if(instances.begin(), instances.end(),
                                     [&](const SLAPrintObject::Instance& instance) { return instance.instance_id == second->id(); });
    REQUIRE(copied != instances.end());
    CHECK(copied->shift.x() == scaled<coord_t>(20.));
    CHECK(copied->shift.y() == 0);
    CHECK_THAT(copied->rotation, Catch::Matchers::WithinAbs(0., 1e-6));
}

TEST_CASE("SLA transformed support points follow the print transformation", "[SLAPrint][SLALifecycle]")
{
    LifecycleFixture fixture;
    ModelObject* object = fixture.model.objects.front();
    object->instances.front()->set_scaling_factor(Vec3d(1.5, 1.5, 1.5));
    object->sla_support_points = {sla::SupportPoint(1.f, 2.f, 3.f, 0.5f)};
    fixture.print.apply(fixture.model, fixture.config);

    REQUIRE(fixture.print.objects().size() == 1);
    const auto* print_object = fixture.print.objects().front();
    const auto points        = print_object->transformed_support_points();
    REQUIRE(points.size() == 1);
    const Vec3f expected = (print_object->trafo() * Vec3d(1., 2., 3.)).cast<float>();
    CHECK_THAT(points.front().pos.x(), Catch::Matchers::WithinAbs(expected.x(), 1e-5));
    CHECK_THAT(points.front().pos.y(), Catch::Matchers::WithinAbs(expected.y(), 1e-5));
    CHECK_THAT(points.front().pos.z(), Catch::Matchers::WithinAbs(expected.z(), 1e-5));
    CHECK_THAT(points.front().head_front_radius, Catch::Matchers::WithinAbs(0.5, 1e-5));
}

TEST_CASE("SLA mesh getters expose only completed support and pad products", "[SLAPrint][SLALifecycle]")
{
    DynamicPrintConfig support_config = sla_config(1.0);
    support_config.set_key_value("supports_enable", new ConfigOptionBool(true));
    support_config.set_key_value("support_object_elevation", new ConfigOptionFloat(5.0));
    support_config.set_key_value("support_head_front_diameter", new ConfigOptionFloat(1.0));
    support_config.set_key_value("support_pillar_diameter", new ConfigOptionFloat(1.0));
    support_config.set_key_value("support_base_diameter", new ConfigOptionFloat(2.0));
    support_config.set_key_value("support_base_height", new ConfigOptionFloat(1.0));
    LifecycleFixture support_fixture(std::move(support_config));
    support_fixture.model.objects.front()->sla_support_points = {sla::SupportPoint(5.f, 5.f, 0.f, 0.5f, true)};
    support_fixture.model.objects.front()->sla_points_status  = sla::PointsStatus::UserModified;
    support_fixture.print.apply(support_fixture.model, support_fixture.config);
    REQUIRE(support_fixture.print.objects().size() == 1);
    const auto* support_object = support_fixture.print.objects().front();
    CHECK_FALSE(support_object->has_mesh(slaposSupportTree));
    CHECK(support_object->get_mesh(slaposSupportTree).empty());
    support_fixture.process_all();
    REQUIRE(support_object->is_step_done(slaposSupportTree));
    CHECK(support_object->has_mesh(slaposSupportTree));
    TriangleMesh support_mesh = support_object->support_mesh();
    CHECK_THAT(support_object->get_mesh(slaposSupportTree).volume(),
               Catch::Matchers::WithinRel(static_cast<float>(support_mesh.volume()), 1e-5f));

    DynamicPrintConfig pad_config = sla_config(1.0);
    pad_config.set_key_value("pad_enable", new ConfigOptionBool(true));
    pad_config.set_key_value("pad_wall_thickness", new ConfigOptionFloat(1.0));
    pad_config.set_key_value("pad_wall_height", new ConfigOptionFloat(1.0));
    pad_config.set_key_value("pad_brim_size", new ConfigOptionFloat(2.0));
    LifecycleFixture pad_fixture(std::move(pad_config));
    REQUIRE(pad_fixture.print.objects().size() == 1);
    const auto* pad_object = pad_fixture.print.objects().front();
    CHECK_FALSE(pad_object->has_mesh(slaposPad));
    pad_fixture.process_all();
    REQUIRE(pad_object->is_step_done(slaposPad));
    CHECK(pad_object->has_mesh(slaposPad));
    TriangleMesh pad_mesh = pad_object->pad_mesh();
    CHECK_THAT(pad_object->get_mesh(slaposPad).volume(), Catch::Matchers::WithinRel(static_cast<float>(pad_mesh.volume()), 1e-5f));
}

TEST_CASE("SLA slice-closing changes invalidate object slices", "[SLAPrint][SLALifecycle]")
{
    LifecycleFixture fixture;
    fixture.process_all();
    REQUIRE(fixture.print.objects().front()->is_step_done(slaposObjectSlice));

    DynamicPrintConfig changed = fixture.config;
    changed.set_key_value("slice_closing_radius", new ConfigOptionFloat(0.01));
    const auto status = fixture.print.apply(fixture.model, std::move(changed));

    CHECK(status == SLAPrint::APPLY_STATUS_INVALIDATED);
    CHECK_FALSE(fixture.print.objects().front()->is_step_done(slaposObjectSlice));
}

TEST_CASE("SLA support geometry settings invalidate completed support trees", "[SLAPrint][SLALifecycle]")
{
    DynamicPrintConfig config = sla_config(1.0);
    config.set_key_value("supports_enable", new ConfigOptionBool(true));
    config.set_key_value("support_object_elevation", new ConfigOptionFloat(5.0));
    config.set_key_value("support_head_front_diameter", new ConfigOptionFloat(1.0));
    config.set_key_value("support_pillar_diameter", new ConfigOptionFloat(1.0));
    config.set_key_value("support_base_diameter", new ConfigOptionFloat(2.0));
    config.set_key_value("support_base_height", new ConfigOptionFloat(1.0));
    LifecycleFixture fixture(std::move(config));
    fixture.model.objects.front()->sla_support_points = {sla::SupportPoint(5.f, 5.f, 0.f, 0.5f, true)};
    fixture.model.objects.front()->sla_points_status  = sla::PointsStatus::UserModified;
    fixture.print.apply(fixture.model, fixture.config);
    fixture.process_all();
    REQUIRE(fixture.print.objects().front()->is_step_done(slaposSupportTree));

    DynamicPrintConfig changed = fixture.config;
    changed.set_key_value("support_pillar_diameter", new ConfigOptionFloat(1.5));
    const auto status = fixture.print.apply(fixture.model, std::move(changed));

    CHECK(status == SLAPrint::APPLY_STATUS_INVALIDATED);
    CHECK_FALSE(fixture.print.objects().front()->is_step_done(slaposSupportTree));
}

TEST_CASE("SLA pad geometry settings invalidate completed pads", "[SLAPrint][SLALifecycle]")
{
    DynamicPrintConfig config = sla_config(1.0);
    config.set_key_value("pad_enable", new ConfigOptionBool(true));
    config.set_key_value("pad_wall_thickness", new ConfigOptionFloat(1.0));
    config.set_key_value("pad_wall_height", new ConfigOptionFloat(1.0));
    config.set_key_value("pad_brim_size", new ConfigOptionFloat(2.0));
    LifecycleFixture fixture(std::move(config));
    fixture.process_all();
    REQUIRE(fixture.print.objects().front()->is_step_done(slaposPad));

    DynamicPrintConfig changed = fixture.config;
    changed.set_key_value("pad_wall_height", new ConfigOptionFloat(2.0));
    const auto status = fixture.print.apply(fixture.model, std::move(changed));

    CHECK(status == SLAPrint::APPLY_STATUS_INVALIDATED);
    CHECK_FALSE(fixture.print.objects().front()->is_step_done(slaposPad));
}

TEST_CASE("SLA task limits are reversible after finalize", "[SLAPrint][SLALifecycle]")
{
    LifecycleFixture fixture;
    PrintBase::TaskParams task;
    task.to_object_step = slaposObjectSlice;
    fixture.print.set_task(task);
    fixture.print.process();

    REQUIRE(fixture.print.objects().size() == 1);
    CHECK(fixture.print.objects().front()->is_step_done(slaposObjectSlice));
    CHECK_FALSE(fixture.print.objects().front()->is_step_done(slaposSupportTree));
    CHECK_FALSE(fixture.print.finished());

    fixture.print.finalize();
    fixture.print.process();
    CHECK(fixture.print.finished());
}

TEST_CASE("SLA support validation reports user-modified support points that are missing", "[SLAPrint][SLALifecycle]")
{
    DynamicPrintConfig config = sla_config(1.0);
    config.set_key_value("supports_enable", new ConfigOptionBool(true));
    config.set_key_value("support_object_elevation", new ConfigOptionFloat(5.0));
    LifecycleFixture fixture(std::move(config));
    fixture.model.objects.front()->sla_points_status = sla::PointsStatus::UserModified;
    fixture.print.apply(fixture.model, fixture.config);

    const StringObjectException error = fixture.print.validate();
    CHECK_FALSE(error.string.empty());
    CHECK(error.object != nullptr);
}

TEST_CASE("SLA statistics populate placeholders and output paths", "[SLAPrint][SLALifecycle]")
{
    LifecycleFixture fixture;
    fixture.process_all();
    const SLAPrintStatistics& stats = fixture.print.print_statistics();
    const DynamicConfig values      = stats.config();
    REQUIRE(values.option("objects_used_material") != nullptr);
    REQUIRE(values.option("support_used_material") != nullptr);
    CHECK_THAT(values.option("objects_used_material")->getFloat(), Catch::Matchers::WithinAbs(stats.objects_used_material, 1e-6));
    CHECK_THAT(values.option("support_used_material")->getFloat(), Catch::Matchers::WithinAbs(stats.support_used_material, 1e-6));

    const std::string path = stats.finalize_output_path("results/{print_time}_{objects_used_material}.sl1");
    CHECK(path.find('{') == std::string::npos);
    CHECK(path.find('}') == std::string::npos);
    const bool has_sl1_extension = path.size() >= 4 && path.compare(path.size() - 4, 4, ".sl1") == 0;
    CHECK(has_sl1_extension);

    SLAPrintStatistics known;
    known.objects_used_material = 120.;
    known.support_used_material = 30.;
    known.total_cost            = 4.;
    known.total_weight          = 8.;
    CHECK(known.finalize_output_path("results/{objects_used_material}_{support_used_material}_{total_cost}_{total_weight}.sl1") ==
          "results/120_30_4_8.sl1");
    CHECK(known.finalize_output_path("results/{missing_statistic}.sl1") == "results/{missing_statistic}.sl1");

    const DynamicConfig placeholders   = SLAPrintStatistics::placeholders();
    const auto* print_time_placeholder = dynamic_cast<const ConfigOptionString*>(placeholders.option("print_time"));
    const auto* material_placeholder   = dynamic_cast<const ConfigOptionString*>(placeholders.option("objects_used_material"));
    REQUIRE(print_time_placeholder != nullptr);
    REQUIRE(material_placeholder != nullptr);
    CHECK(print_time_placeholder->value == "{print_time}");
    CHECK(material_placeholder->value == "{objects_used_material}");
}

TEST_CASE("SLA clearing a processed print releases object ownership", "[SLAPrint][SLALifecycle]")
{
    LifecycleFixture fixture;
    fixture.process_all();
    REQUIRE_FALSE(fixture.print.empty());
    fixture.print.clear();

    CHECK(fixture.print.empty());
    CHECK(fixture.print.objects().empty());
    CHECK_FALSE(fixture.print.finished());
}

TEST_CASE("SLA empty models remain empty through processing", "[SLAPrint][SLALifecycle]")
{
    SLAPrint print;
    DynamicPrintConfig config = sla_config(1.0);
    Model empty_model;
    const auto status = print.apply(empty_model, std::move(config));

    CHECK(status != SLAPrint::APPLY_STATUS_INVALIDATED);
    CHECK(print.empty());
    print.process();
    CHECK(print.empty());
    CHECK_FALSE(print.finished());
    CHECK(print.print_layers().empty());
}

} // namespace sla_lifecycle_contracts

namespace sla_drain_transform_contracts {
TEST_CASE("SLA drain-hole normals remain perpendicular after rotated unequal scaling", "[SLAPrint][SLADrainTransforms]")
{
    const bool corrected = GENERATE(false, true);
    auto config          = sla_config(1.0);
    config.set_key_value("relative_correction", new ConfigOptionFloats({1.0, 1.0}));
    const Vec3d correction = corrected ? Vec3d(1.5, 0.8, 1.2) : Vec3d(1., 1., 1.);
    config.set_key_value("relative_correction_x", new ConfigOptionFloat(correction.x()));
    config.set_key_value("relative_correction_y", new ConfigOptionFloat(correction.y()));
    config.set_key_value("relative_correction_z", new ConfigOptionFloat(correction.z()));
    sla_lifecycle_contracts::LifecycleFixture fixture(std::move(config));
    auto* object   = fixture.model.objects.front();
    auto* instance = object->instances.front();
    instance->set_scaling_factor(Vec3d(2., 3., 4.));
    instance->set_rotation(Vec3d(PI / 2., 0., 0.));
    const Vec3f n           = Vec3f(1., 1., 1.).normalized();
    object->sla_drain_holes = {sla::DrainHole(Vec3f(1., 2., 1.), n, 2., 5.)};
    fixture.print.apply(fixture.model, fixture.config);
    REQUIRE(fixture.print.objects().size() == 1);
    const auto* printed = fixture.print.objects().front();
    const auto holes    = printed->transformed_drainhole_points();
    REQUIRE(holes.size() == 1);
    const auto& hole = holes.front();
    // A quarter-turn maps (nx/2,ny/3,nz/4) to (nx/2,-nz/4,ny/3).
    const Vec3f expected(n.x() / float(2. * correction.x()), -n.z() / float(4. * correction.y()), n.y() / float(3. * correction.z()));
    for (int axis = 0; axis < 3; ++axis)
        CHECK_THAT(hole.normal[axis], Catch::Matchers::WithinAbs(expected[axis], 1e-5));
    // Covectors annihilate two independently transformed source-plane tangents.
    const Vec3f tangent1 = (printed->trafo().linear() * Vec3d(1., -1., 0.)).cast<float>();
    const Vec3f tangent2 = (printed->trafo().linear() * Vec3d(0., 1., -1.)).cast<float>();
    CHECK_THAT(hole.normal.dot(tangent1), Catch::Matchers::WithinAbs(0., 1e-5));
    CHECK_THAT(hole.normal.dot(tangent2), Catch::Matchers::WithinAbs(0., 1e-5));
    const Vec3f base         = (printed->trafo() * Vec3d(1., 2., 1.)).cast<float>();
    const Vec3f expected_pos = base - expected.normalized();
    for (int axis = 0; axis < 3; ++axis)
        CHECK_THAT(hole.pos[axis], Catch::Matchers::WithinAbs(expected_pos[axis], 1e-5));
    CHECK_THAT(hole.height, Catch::Matchers::WithinAbs(6., 1e-5));
    CHECK_THAT(hole.radius, Catch::Matchers::WithinAbs(2., 1e-5));
}
} // namespace sla_drain_transform_contracts

namespace sla_pad_variance_contracts {

struct PadVarianceRun {
    double wall_thickness = 0.;
    double brim_size      = 0.;
    double z_span         = 0.;
    double footprint_x    = 0.;
    double footprint_y    = 0.;
    double pad_volume     = 0.;
    double support_area   = 0.;
    std::size_t model_records = 0;
    double model_area = 0.;
};

PadVarianceRun run_pad(double wall_thickness, double brim_size,
                       double wall_height = 1.0, double wall_slope = 45.0,
                       double layer_height = 1.0)
{
    DynamicPrintConfig config = sla_config(layer_height);
    config.set_key_value("pad_enable", new ConfigOptionBool(true));
    config.set_key_value("pad_wall_thickness", new ConfigOptionFloat(wall_thickness));
    config.set_key_value("pad_wall_height", new ConfigOptionFloat(wall_height));
    config.set_key_value("pad_brim_size", new ConfigOptionFloat(brim_size));
    config.set_key_value("pad_wall_slope", new ConfigOptionFloat(wall_slope));
    config.set_key_value("pad_max_merge_distance", new ConfigOptionFloat(50.0));
    config.set_key_value("pad_around_object", new ConfigOptionBool(false));
    config.set_key_value("pad_around_object_everywhere", new ConfigOptionBool(false));
    config.set_key_value("relative_correction", new ConfigOptionFloats({1.0, 1.0}));
    config.set_key_value("relative_correction_x", new ConfigOptionFloat(1.0));
    config.set_key_value("relative_correction_y", new ConfigOptionFloat(1.0));
    config.set_key_value("relative_correction_z", new ConfigOptionFloat(1.0));
    config.set_key_value("material_correction", new ConfigOptionFloats({1.0, 1.0, 1.0}));
    config.set_key_value("material_correction_x", new ConfigOptionFloat(1.0));
    config.set_key_value("material_correction_y", new ConfigOptionFloat(1.0));
    config.set_key_value("material_correction_z", new ConfigOptionFloat(1.0));
    config.set_key_value("absolute_correction", new ConfigOptionFloat(0.0));
    config.set_key_value("elefant_foot_compensation", new ConfigOptionFloat(0.0));

    SlaCubeFixture fixture(std::move(config), make_cube(10.0, 10.0, 4.0));
    REQUIRE(fixture.apply_status != SLAPrint::APPLY_STATUS_UNCHANGED);
    const auto validation = fixture.print.validate();
    INFO("pad validation: " << validation.string);
    REQUIRE(validation.string.empty());

    fixture.print.process();
    REQUIRE(fixture.print.finished());
    REQUIRE(fixture.print.objects().size() == 1);

    const SLAPrintObject *object = fixture.print.objects().front();
    REQUIRE(object->is_step_done(slaposPad));
    TriangleMesh pad = object->pad_mesh();
    REQUIRE_FALSE(pad.empty());

    const auto bbox = pad.bounding_box();
    const double z_span      = bbox.max.z() - bbox.min.z();
    const double footprint_x = bbox.max.x() - bbox.min.x();
    const double footprint_y = bbox.max.y() - bbox.min.y();
    const double pad_volume  = pad.volume();
    const double support_area = support_slice_area(*object);

    CHECK(std::isfinite(z_span));
    CHECK(std::isfinite(footprint_x));
    CHECK(std::isfinite(footprint_y));
    CHECK(std::isfinite(pad_volume));
    CHECK(std::isfinite(support_area));
    REQUIRE(z_span > 0.);
    REQUIRE(footprint_x > 0.);
    REQUIRE(footprint_y > 0.);
    REQUIRE(pad_volume > 0.);
    REQUIRE(support_area > 0.);

    const auto &slice_index = object->get_slice_index();
    REQUIRE_FALSE(slice_index.empty());
    std::size_t model_records = 0;
    double model_area = 0.;
    for (const SLAPrintObject::SliceRecord &record : slice_index) {
        const ExPolygons &model_slice = record.get_slice(soModel);
        if (model_slice.empty())
            continue;
        ++model_records;
        REQUIRE(model_slice.size() == 1);
        const double area_mm2 = model_slice.front().area() /
                                (scaled<double>(1.0) * scaled<double>(1.0));
        CHECK_THAT(area_mm2, Catch::Matchers::WithinAbs(100.0, 1e-4));
        model_area += area_mm2;
    }
    REQUIRE(model_records > 0);
    CHECK(std::isfinite(model_area));

    return {wall_thickness, brim_size, z_span, footprint_x, footprint_y,
            pad_volume, support_area, model_records, model_area};
}

} // namespace sla_pad_variance_contracts

TEST_CASE("SLA pad wall thickness controls pad height",
          "[SLAFeatures][SLAPadVariance][SLAPadThicknessVariance]")
{
    const std::vector<double> values{0.8, 1.2, 2.0};
    std::vector<sla_pad_variance_contracts::PadVarianceRun> runs;
    runs.reserve(values.size());
    for (const double value : values) {
        INFO("pad_wall_thickness=" << value);
        // Keep brim comfortably above PadConfig::bottom_offset() for the
        // thickest wall row, including finite-precision validation margins.
        runs.emplace_back(sla_pad_variance_contracts::run_pad(value, 4.0));
    }

    REQUIRE(runs.size() == values.size());
    for (std::size_t i = 0; i < runs.size(); ++i)
        CHECK_THAT(runs[i].z_span,
                   Catch::Matchers::WithinAbs(values[i] + 1.0, 1e-4));
    for (std::size_t i = 1; i < runs.size(); ++i)
        CHECK(runs[i].z_span > runs[i - 1].z_span);
}

TEST_CASE("SLA pad brim size expands the pad footprint",
          "[SLAFeatures][SLAPadVariance][SLAPadBrimVariance]")
{
    const std::vector<double> values{2.0, 5.0, 8.0};
    std::vector<sla_pad_variance_contracts::PadVarianceRun> runs;
    runs.reserve(values.size());
    for (const double value : values) {
        INFO("pad_brim_size=" << value);
        runs.emplace_back(sla_pad_variance_contracts::run_pad(0.8, value));
    }

    REQUIRE(runs.size() == values.size());
    for (const auto &run : runs)
        REQUIRE(run.model_records > 0);
    for (const auto &run : runs)
        CHECK_THAT(run.z_span, Catch::Matchers::WithinAbs(1.8, 1e-4));
    const auto &reference = runs.front();
    for (std::size_t i = 1; i < runs.size(); ++i) {
        CHECK(runs[i].model_records == reference.model_records);
        CHECK_THAT(runs[i].model_area,
                   Catch::Matchers::WithinAbs(reference.model_area, 1e-4));
    }
    for (std::size_t i = 1; i < runs.size(); ++i) {
        const double brim_delta = values[i] - values[i - 1];
        // PadConfig derives the waffle offset from brim_size + wing_distance.
        // Keep a conservative fraction of the configured delta as the
        // public extent witness instead of freezing exact offset coordinates.
        const double minimum_growth = 0.25 * brim_delta;
        CHECK(runs[i].footprint_x > runs[i - 1].footprint_x + minimum_growth);
        CHECK(runs[i].footprint_y > runs[i - 1].footprint_y + minimum_growth);
    }
}

namespace sla_correction_variance_contracts {

struct Run {
    std::vector<std::size_t> model_indices;
    std::vector<float>       model_levels;
    std::vector<double>      model_areas;
    double                   bbox_x = 0.;
    double                   bbox_y = 0.;
    double                   bbox_z = 0.;
};

DynamicPrintConfig correction_config()
{
    DynamicPrintConfig config = sla_config(1.0);
    // Keep all other transform and printer corrections neutral so each test
    // observes one source setting family through the public pipeline.
    config.set_key_value("relative_correction", new ConfigOptionFloats({1.0, 1.0}));
    config.set_key_value("relative_correction_x", new ConfigOptionFloat(1.0));
    config.set_key_value("relative_correction_y", new ConfigOptionFloat(1.0));
    config.set_key_value("relative_correction_z", new ConfigOptionFloat(1.0));
    config.set_key_value("material_correction", new ConfigOptionFloats({1.0, 1.0, 1.0}));
    config.set_key_value("material_correction_x", new ConfigOptionFloat(1.0));
    config.set_key_value("material_correction_y", new ConfigOptionFloat(1.0));
    config.set_key_value("material_correction_z", new ConfigOptionFloat(1.0));
    config.set_key_value("absolute_correction", new ConfigOptionFloat(0.0));
    config.set_key_value("elefant_foot_compensation", new ConfigOptionFloat(0.0));
    return config;
}

Run process_cube(DynamicPrintConfig config)
{
    SlaCubeFixture fixture(std::move(config), make_cube(10.0, 10.0, 4.0));
    REQUIRE(fixture.apply_status != SLAPrint::APPLY_STATUS_UNCHANGED);
    const auto validation = fixture.print.validate();
    INFO("SLA correction validation: " << validation.string);
    REQUIRE(validation.string.empty());

    fixture.print.process();
    REQUIRE(fixture.print.finished());
    REQUIRE(fixture.print.objects().size() == 1);
    const SLAPrintObject *object = fixture.print.objects().front();
    REQUIRE(object->is_step_done(slaposObjectSlice));

    const auto &slice_index = object->get_slice_index();
    REQUIRE_FALSE(slice_index.empty());
    Run result;
    for (const SLAPrintObject::SliceRecord &record : slice_index) {
        const ExPolygons &model_slice = record.get_slice(soModel);
        if (model_slice.empty())
            continue;
        REQUIRE(model_slice.size() == 1);
        const double area = std::abs(model_slice.front().area()) /
                            (scaled<double>(1.0) * scaled<double>(1.0));
        REQUIRE(std::isfinite(area));
        REQUIRE(std::isfinite(record.slice_level()));
        REQUIRE(area > 0.);
        result.model_indices.push_back(record.get_slice_idx(soModel));
        result.model_levels.push_back(record.slice_level());
        result.model_areas.push_back(area);
    }
    REQUIRE(result.model_areas.size() >= 2);
    REQUIRE(result.model_levels.size() == result.model_areas.size());

    const auto size = object->transformed_mesh().bounding_box().size();
    result.bbox_x = size.x();
    result.bbox_y = size.y();
    result.bbox_z = size.z();
    CHECK(std::isfinite(result.bbox_x));
    CHECK(std::isfinite(result.bbox_y));
    CHECK(std::isfinite(result.bbox_z));
    CHECK(result.bbox_x > 0.);
    CHECK(result.bbox_y > 0.);
    CHECK(result.bbox_z > 0.);
    return result;
}

} // namespace sla_correction_variance_contracts

TEST_CASE("SLA absolute correction erodes and expands every model slice",
          "[SLAFeatures][SLAAbsoluteCorrection]")
{
    const std::vector<double> values{-0.5, 0.0, 0.5};
    std::vector<sla_correction_variance_contracts::Run> runs;
    runs.reserve(values.size());
    for (const double value : values) {
        INFO("absolute_correction=" << value);
        DynamicPrintConfig config = sla_correction_variance_contracts::correction_config();
        config.set_key_value("absolute_correction", new ConfigOptionFloat(value));
        runs.emplace_back(sla_correction_variance_contracts::process_cube(std::move(config)));
    }

    REQUIRE(runs.size() == values.size());
    REQUIRE(runs[1].model_areas.size() > 0);
    for (const auto &run : runs) {
        REQUIRE(run.model_areas.size() == runs[1].model_areas.size());
        REQUIRE(run.model_levels.size() == runs[1].model_levels.size());
        REQUIRE(run.model_indices.size() == runs[1].model_indices.size());
    }
    for (std::size_t i = 0; i < runs[1].model_areas.size(); ++i) {
        CHECK_THAT(runs[1].model_areas[i], Catch::Matchers::WithinAbs(100.0, 0.02));
        CHECK(runs[0].model_indices[i] == runs[1].model_indices[i]);
        CHECK(runs[2].model_indices[i] == runs[1].model_indices[i]);
        CHECK_THAT(runs[0].model_levels[i], Catch::Matchers::WithinAbs(runs[1].model_levels[i], 1e-6));
        CHECK_THAT(runs[2].model_levels[i], Catch::Matchers::WithinAbs(runs[1].model_levels[i], 1e-6));
        // apply_printer_corrections offsets the 10 mm square by the configured
        // signed distance on each side: -0.5 -> 81/100 and +0.5 -> 121/100.
        // A 1% tolerance covers scaled integer conversion while retaining a
        // clear gate mutation signal on every aligned model slice.
        CHECK_THAT(runs[0].model_areas[i] / runs[1].model_areas[i],
                   Catch::Matchers::WithinRel(0.81, 0.01));
        CHECK_THAT(runs[2].model_areas[i] / runs[1].model_areas[i],
                   Catch::Matchers::WithinRel(1.21, 0.01));
    }
    for (const auto &run : runs) {
        CHECK_THAT(run.bbox_x, Catch::Matchers::WithinAbs(runs[1].bbox_x, 1e-6));
        CHECK_THAT(run.bbox_y, Catch::Matchers::WithinAbs(runs[1].bbox_y, 1e-6));
        CHECK_THAT(run.bbox_z, Catch::Matchers::WithinAbs(runs[1].bbox_z, 1e-6));
    }
}

TEST_CASE("SLA material correction scales transformed slice geometry",
          "[SLAFeatures][SLAMaterialCorrection]")
{
    DynamicPrintConfig base_config = sla_correction_variance_contracts::correction_config();
    DynamicPrintConfig material_config = base_config;
    // The array has three values deliberately: SLAPrint::relative_correction()
    // enters the material branch only when material_correction has >= 2 values.
    material_config.set_key_value("material_correction", new ConfigOptionFloats({1.0, 1.0, 1.0}));
    material_config.set_key_value("material_correction_x", new ConfigOptionFloat(1.5));
    material_config.set_key_value("material_correction_y", new ConfigOptionFloat(0.8));
    material_config.set_key_value("material_correction_z", new ConfigOptionFloat(1.0));

    const auto base = sla_correction_variance_contracts::process_cube(std::move(base_config));
    const auto material = sla_correction_variance_contracts::process_cube(std::move(material_config));

    REQUIRE(base.model_areas.size() > 0);
    REQUIRE(material.model_areas.size() == base.model_areas.size());
    REQUIRE(material.model_levels.size() == base.model_levels.size());
    REQUIRE(material.model_indices.size() == base.model_indices.size());
    for (std::size_t i = 0; i < base.model_areas.size(); ++i) {
        CHECK_THAT(base.model_areas[i], Catch::Matchers::WithinAbs(100.0, 0.02));
        CHECK(material.model_indices[i] == base.model_indices[i]);
        CHECK_THAT(material.model_levels[i], Catch::Matchers::WithinAbs(base.model_levels[i], 1e-6));
        CHECK_THAT(material.model_areas[i] / base.model_areas[i],
                   Catch::Matchers::WithinRel(1.2, 0.01));
    }
    CHECK_THAT(material.bbox_x, Catch::Matchers::WithinRel(base.bbox_x * 1.5, 1e-5));
    CHECK_THAT(material.bbox_y, Catch::Matchers::WithinRel(base.bbox_y * 0.8, 1e-5));
    CHECK_THAT(material.bbox_z, Catch::Matchers::WithinRel(base.bbox_z, 1e-5));
}

TEST_CASE("SLA pillar connection modes preserve intended near and far bridge topology",
          "[SLAPrint][SLAPipeline][SupportPillarConnection]")
{
    constexpr double near_spacing = 3.0;
    constexpr double far_spacing  = 5.0;

    const PillarConnectionRun near_zigzag = process_pillar_connection(slapcmZigZag, near_spacing);
    const PillarConnectionRun near_cross  = process_pillar_connection(slapcmCross, near_spacing);
    const PillarConnectionRun near_dynamic = process_pillar_connection(slapcmDynamic, near_spacing);
    const PillarConnectionRun far_zigzag  = process_pillar_connection(slapcmZigZag, far_spacing);
    const PillarConnectionRun far_cross   = process_pillar_connection(slapcmCross, far_spacing);
    const PillarConnectionRun far_dynamic = process_pillar_connection(slapcmDynamic, far_spacing);

    for (const PillarConnectionRun &run : {near_zigzag, near_cross, near_dynamic,
                                            far_zigzag, far_cross, far_dynamic}) {
        CHECK_THAT(run.model_volume, Catch::Matchers::WithinRel(400.0, 1e-5));
        CHECK(run.support_vertices > 0);
        CHECK(run.support_faces > 0);
        CHECK(run.separated_support_slices > 0);
        CHECK(run.max_support_centroid_spacing_mm > 2.0);
    }

    // The explicit Cross mode adds the second, crossing stick.  This positive
    // mesh-volume witness prevents the comparisons below from becoming a
    // mode-label-only or nonempty-mesh test.
    REQUIRE(near_cross.support_volume > near_zigzag.support_volume + 1e-6);
    REQUIRE(far_cross.support_volume > far_zigzag.support_volume + 1e-6);

    // The public SLAPrint mapping and the production dynamic threshold agree:
    // spacing 3 <= 2 * base radius 2 is zig-zag, while spacing 5 > 4 is cross.
    // max_bridge_length is only reduced enough to keep these points in separate
    // ground clusters; interconnect uses max_pillar_link_distance for this pair.
    CHECK_THAT(near_dynamic.support_volume,
               Catch::Matchers::WithinRel(near_zigzag.support_volume, 1e-6));
    CHECK_THAT(far_dynamic.support_volume,
               Catch::Matchers::WithinRel(far_cross.support_volume, 1e-6));
}

namespace sla_active_next_contracts {

struct BrimRun {
    double volume = 0.0;
    double width = 0.0;
    double depth = 0.0;
    double z_span = 0.0;
};

BrimRun run_everywhere_brim()
{
    DynamicPrintConfig config = sla_config(1.0);
    config.set_key_value("pad_enable", new ConfigOptionBool(true));
    config.set_key_value("pad_around_object", new ConfigOptionBool(true));
    config.set_key_value("pad_around_object_everywhere", new ConfigOptionBool(true));
    config.set_key_value("pad_wall_thickness", new ConfigOptionFloat(1.0));
    config.set_key_value("pad_wall_height", new ConfigOptionFloat(1.0));
    config.set_key_value("pad_brim_size", new ConfigOptionFloat(4.0));
    config.set_key_value("pad_wall_slope", new ConfigOptionFloat(45.0));
    config.set_key_value("pad_max_merge_distance", new ConfigOptionFloat(50.0));

    SlaCubeFixture fixture(std::move(config), make_cube(10.0, 10.0, 4.0));
    REQUIRE(fixture.apply_status != SLAPrint::APPLY_STATUS_UNCHANGED);
    INFO("SLA brim validation: " << fixture.print.validate().string);
    REQUIRE(fixture.print.validate().string.empty());
    fixture.print.process();
    REQUIRE(fixture.print.finished());
    REQUIRE(fixture.print.objects().size() == 1);

    const SLAPrintObject *object = fixture.print.objects().front();
    REQUIRE(object->is_step_done(slaposPad));
    TriangleMesh pad = object->pad_mesh();
    REQUIRE_FALSE(pad.empty());
    const auto bbox = pad.bounding_box();
    BrimRun result;
    result.volume = std::abs(static_cast<double>(pad.volume()));
    result.width = bbox.max.x() - bbox.min.x();
    result.depth = bbox.max.y() - bbox.min.y();
    result.z_span = bbox.max.z() - bbox.min.z();
    REQUIRE(std::isfinite(result.volume));
    REQUIRE(std::isfinite(result.width));
    REQUIRE(std::isfinite(result.depth));
    REQUIRE(std::isfinite(result.z_span));
    REQUIRE(result.volume > 0.0);
    REQUIRE(result.width > 10.0);
    REQUIRE(result.depth > 10.0);
    REQUIRE(result.z_span > 0.0);
    return result;
}

} // namespace sla_active_next_contracts

TEST_CASE("SLA automatic support generation creates printable support geometry",
          "[SLAPrint][SLAFeatures][SLAActiveNext][SLAAutoSupports]")
{
    DynamicPrintConfig config = sla_config(1.0);
    config.set_key_value("supports_enable", new ConfigOptionBool(true));
    config.set_key_value("support_object_elevation", new ConfigOptionFloat(5.0));
    config.set_key_value("support_points_density_relative", new ConfigOptionInt(100));
    config.set_key_value("support_points_minimal_distance", new ConfigOptionFloat(1.0));
    config.set_key_value("support_head_front_diameter", new ConfigOptionFloat(1.0));
    config.set_key_value("support_head_penetration", new ConfigOptionFloat(0.2));
    config.set_key_value("support_head_width", new ConfigOptionFloat(1.0));
    config.set_key_value("support_pillar_diameter", new ConfigOptionFloat(2.0));
    config.set_key_value("support_base_diameter", new ConfigOptionFloat(4.0));
    config.set_key_value("support_base_height", new ConfigOptionFloat(1.0));
    config.set_key_value("support_base_safety_distance", new ConfigOptionFloat(0.5));

    SlaCubeFixture fixture(std::move(config), make_cube(10.0, 10.0, 4.0));
    REQUIRE(fixture.apply_status != SLAPrint::APPLY_STATUS_UNCHANGED);
    INFO("SLA automatic support validation: " << fixture.print.validate().string);
    REQUIRE(fixture.print.validate().string.empty());
    fixture.print.process();
    REQUIRE(fixture.print.finished());
    REQUIRE(fixture.print.objects().size() == 1);

    const SLAPrintObject *object = fixture.print.objects().front();
    REQUIRE(object->is_step_done(slaposSupportPoints));
    REQUIRE(object->is_step_done(slaposSupportTree));
    REQUIRE_FALSE(object->get_support_points().empty());
    TriangleMesh support = object->support_mesh();
    REQUIRE_FALSE(support.empty());
    REQUIRE(std::isfinite(static_cast<double>(support.volume())));
    REQUIRE(std::abs(static_cast<double>(support.volume())) > 0.0);
    REQUIRE(std::isfinite(support_slice_area(*object)));
    REQUIRE(support_slice_area(*object) > 0.0);
}

TEST_CASE("SLA large layers use the slow exposure statistics path",
          "[SLAPrint][SLAPipeline][SLAActiveNext][SLASlowLayers]")
{
    SlaCubeFixture fixture(sla_config(1.0), make_cube(40.0, 40.0, 4.0));
    REQUIRE(fixture.apply_status != SLAPrint::APPLY_STATUS_UNCHANGED);
    INFO("SLA large-layer validation: " << fixture.print.validate().string);
    REQUIRE(fixture.print.validate().string.empty());
    fixture.print.process();
    REQUIRE(fixture.print.finished());
    REQUIRE(fixture.print.objects().size() == 1);

    const auto &stats = fixture.print.print_statistics();
    const auto *object = fixture.print.objects().front();
    REQUIRE_FALSE(object->get_slice_index().empty());
    REQUIRE(stats.layers_times.size() == object->get_slice_index().size());
    REQUIRE(stats.slow_layers_count > 0);
    CHECK(stats.fast_layers_count == 0);
    REQUIRE(stats.estimated_print_time > 0.0);
    for (const double layer_time : stats.layers_times) {
        CHECK(std::isfinite(layer_time));
        CHECK(layer_time > 0.0);
    }
}

TEST_CASE("SLA support-backed pads preserve both support and pad products",
          "[SLAPrint][SLAFeatures][SLAActiveNext][SLASupportPad]")
{
    DynamicPrintConfig config = sla_config(1.0);
    config.set_key_value("supports_enable", new ConfigOptionBool(true));
    config.set_key_value("pad_enable", new ConfigOptionBool(true));
    config.set_key_value("pad_around_object", new ConfigOptionBool(false));
    config.set_key_value("pad_wall_thickness", new ConfigOptionFloat(1.0));
    config.set_key_value("pad_wall_height", new ConfigOptionFloat(1.0));
    config.set_key_value("pad_brim_size", new ConfigOptionFloat(4.0));
    config.set_key_value("pad_wall_slope", new ConfigOptionFloat(45.0));
    config.set_key_value("pad_max_merge_distance", new ConfigOptionFloat(50.0));
    config.set_key_value("support_object_elevation", new ConfigOptionFloat(5.0));
    config.set_key_value("support_head_front_diameter", new ConfigOptionFloat(1.0));
    config.set_key_value("support_head_penetration", new ConfigOptionFloat(0.2));
    config.set_key_value("support_head_width", new ConfigOptionFloat(1.0));
    config.set_key_value("support_pillar_diameter", new ConfigOptionFloat(2.0));
    config.set_key_value("support_base_diameter", new ConfigOptionFloat(4.0));
    config.set_key_value("support_base_height", new ConfigOptionFloat(1.0));
    config.set_key_value("support_base_safety_distance", new ConfigOptionFloat(0.5));
    sla::SupportPoints points{sla::SupportPoint(5.f, 5.f, 0.f, 0.5f, true)};

    SlaCubeFixture fixture(std::move(config), make_cube(10.0, 10.0, 4.0), std::move(points));
    REQUIRE(fixture.apply_status != SLAPrint::APPLY_STATUS_UNCHANGED);
    INFO("SLA support-pad validation: " << fixture.print.validate().string);
    REQUIRE(fixture.print.validate().string.empty());
    fixture.print.process();
    REQUIRE(fixture.print.finished());
    REQUIRE(fixture.print.objects().size() == 1);

    const SLAPrintObject *object = fixture.print.objects().front();
    REQUIRE(object->is_step_done(slaposSupportTree));
    REQUIRE(object->is_step_done(slaposPad));
    TriangleMesh support = object->support_mesh();
    TriangleMesh pad = object->pad_mesh();
    REQUIRE_FALSE(support.empty());
    REQUIRE_FALSE(pad.empty());
    REQUIRE(std::abs(static_cast<double>(support.volume())) > 0.0);
    REQUIRE(std::abs(static_cast<double>(pad.volume())) > 0.0);
    REQUIRE(support_slice_area(*object) > 0.0);
}

TEST_CASE("SLA everywhere brim surrounds the model with a positive public footprint",
          "[SLAPrint][SLAFeatures][SLAActiveNext][SLABrimEverywhere]")
{
    const auto run = sla_active_next_contracts::run_everywhere_brim();
    CHECK(run.width > 10.0);
    CHECK(run.depth > 10.0);
    CHECK(run.z_span > 0.0);
}

TEST_CASE("SLA overlapping objects are unioned before material statistics",
          "[SLAPrint][SLALifecycle][SLAActiveNext][SLAMergedSlices]")
{
    sla_lifecycle_contracts::LifecycleFixture fixture;
    fixture.process_all();
    sla_lifecycle_contracts::add_cube_object(fixture.model, 10.0, 4.0);
    const auto status = fixture.print.apply(fixture.model, fixture.config);
    REQUIRE(status == SLAPrint::APPLY_STATUS_INVALIDATED);
    REQUIRE(fixture.print.objects().size() == 2);
    fixture.print.process();
    REQUIRE(fixture.print.finished());
    REQUIRE(fixture.print.print_layers().size() == 4);
    REQUIRE(fixture.print.print_statistics().objects_used_material > 0.0);
    // Two 10x10x4 cubes translated by 4 mm have a 140 mm2 union on every
    // one of the four 1 mm model layers. This is a public material statistic,
    // independent of the internal polygon container ordering.
    CHECK_THAT(fixture.print.print_statistics().objects_used_material,
               Catch::Matchers::WithinRel(560.0, 0.02));
}

namespace sla_hollowing_thickness_followup_contracts {

struct HollowRun {
    double volume = 0.;
    double width = 0.;
    double depth = 0.;
};

HollowRun run_hollowing(double min_thickness)
{
    DynamicPrintConfig config = sla_config(1.0);
    config.set_key_value("hollowing_enable", new ConfigOptionBool(true));
    config.set_key_value("hollowing_min_thickness", new ConfigOptionFloat(min_thickness));
    config.set_key_value("hollowing_quality", new ConfigOptionFloat(0.2));
    config.set_key_value("hollowing_closing_distance", new ConfigOptionFloat(0.25));

    SlaCubeFixture fixture(std::move(config), make_cube(20.0, 20.0, 10.0));
    REQUIRE(fixture.apply_status != SLAPrint::APPLY_STATUS_UNCHANGED);
    REQUIRE(fixture.model.objects.size() == 1);
    const ModelObject *source = fixture.model.objects.front();
    REQUIRE(source != nullptr);
    REQUIRE(std::abs(static_cast<double>(source->raw_mesh().volume())) > 0.);
    INFO("hollowing validation: " << fixture.print.validate().string);
    REQUIRE(fixture.print.validate().string.empty());

    PrintBase::TaskParams task;
    task.to_object_step = slaposHollowing;
    fixture.print.set_task(task);
    fixture.print.process();

    REQUIRE(fixture.print.objects().size() == 1);
    const SLAPrintObject *object = fixture.print.objects().front();
    REQUIRE(object != nullptr);
    REQUIRE(object->is_step_done(slaposHollowing));
    // Compare in the same public print-object coordinate frame.
    const auto source_bbox = object->transformed_mesh().bounding_box();
    REQUIRE(source_bbox.defined);
    const indexed_triangle_set &interior = object->hollowed_interior_mesh();
    REQUIRE_FALSE(interior.empty());

    TriangleMesh interior_mesh(interior);
    const auto bbox = interior_mesh.bounding_box();
    const double bounds_tolerance = 0.05;
    CHECK(bbox.min.x() >= source_bbox.min.x() - bounds_tolerance);
    CHECK(bbox.max.x() <= source_bbox.max.x() + bounds_tolerance);
    CHECK(bbox.min.y() >= source_bbox.min.y() - bounds_tolerance);
    CHECK(bbox.max.y() <= source_bbox.max.y() + bounds_tolerance);
    CHECK(bbox.min.z() >= source_bbox.min.z() - bounds_tolerance);
    CHECK(bbox.max.z() <= source_bbox.max.z() + bounds_tolerance);
    HollowRun result;
    result.volume = std::abs(static_cast<double>(interior_mesh.volume()));
    result.width = bbox.max.x() - bbox.min.x();
    result.depth = bbox.max.y() - bbox.min.y();
    REQUIRE(std::isfinite(result.volume));
    REQUIRE(std::isfinite(result.width));
    REQUIRE(std::isfinite(result.depth));
    REQUIRE(result.volume > 0.);
    REQUIRE(result.width > 0.);
    REQUIRE(result.depth > 0.);
    return result;
}

} // namespace sla_hollowing_thickness_followup_contracts

TEST_CASE("SLA hollowing wall thickness contracts the public interior",
          "[SLAPrint][SLAHollowing][SLAHollowingThickness]")
{
    const auto thin = sla_hollowing_thickness_followup_contracts::run_hollowing(1.0);
    const auto thick = sla_hollowing_thickness_followup_contracts::run_hollowing(3.0);

    REQUIRE(thin.volume > 0.);
    REQUIRE(thick.volume > 0.);
    CHECK(thick.volume < thin.volume);
    CHECK(thick.width < thin.width);
    CHECK(thick.depth < thin.depth);
}

namespace sla_support_pad_followup_v4 {

struct AroundPadSummary {
    bool   empty = true;
    double volume = 0.0;
    double width = 0.0;
    double depth = 0.0;
    double z_span = 0.0;
};

void enable_common_pad(DynamicPrintConfig &config, bool everywhere)
{
    config.set_key_value("pad_enable", new ConfigOptionBool(true));
    config.set_key_value("pad_around_object", new ConfigOptionBool(true));
    config.set_key_value("pad_around_object_everywhere", new ConfigOptionBool(everywhere));
    config.set_key_value("pad_wall_thickness", new ConfigOptionFloat(1.0));
    config.set_key_value("pad_wall_height", new ConfigOptionFloat(1.0));
    config.set_key_value("pad_brim_size", new ConfigOptionFloat(4.0));
    config.set_key_value("pad_wall_slope", new ConfigOptionFloat(45.0));
    config.set_key_value("pad_max_merge_distance", new ConfigOptionFloat(50.0));
    config.set_key_value("pad_object_gap", new ConfigOptionFloat(1.0));
}

AroundPadSummary run_around_object_pad(bool everywhere)
{
    DynamicPrintConfig config = sla_config(1.0);
    enable_common_pad(config, everywhere);

    SlaCubeFixture fixture(std::move(config), make_cube(10.0, 10.0, 4.0));
    REQUIRE(fixture.apply_status != SLAPrint::APPLY_STATUS_UNCHANGED);
    INFO("SLA around-object pad validation: " << fixture.print.validate().string);
    REQUIRE(fixture.print.validate().string.empty());
    fixture.print.process();
    REQUIRE(fixture.print.finished());
    REQUIRE(fixture.print.objects().size() == 1);

    const SLAPrintObject *object = fixture.print.objects().front();
    REQUIRE(object->is_step_done(slaposPad));
    TriangleMesh pad = object->pad_mesh();
    AroundPadSummary result;
    result.empty = pad.empty();
    if (!everywhere) {
        REQUIRE(result.empty);
        return result;
    }

    REQUIRE_FALSE(result.empty);
    result.volume = std::abs(static_cast<double>(pad.volume()));
    const auto bbox = pad.bounding_box();
    result.width = bbox.max.x() - bbox.min.x();
    result.depth = bbox.max.y() - bbox.min.y();
    result.z_span = bbox.max.z() - bbox.min.z();
    REQUIRE(std::isfinite(result.volume));
    REQUIRE(std::isfinite(result.width));
    REQUIRE(std::isfinite(result.depth));
    REQUIRE(std::isfinite(result.z_span));
    REQUIRE(result.volume > 0.0);
    REQUIRE(result.width > 10.0);
    REQUIRE(result.depth > 10.0);
    REQUIRE(result.z_span > 0.0);
    return result;
}

} // namespace sla_support_pad_followup_v4

TEST_CASE("SLA around-object and everywhere pads expose distinct public footprints",
          "[SLAPrint][SLAFeatures][SLAAroundPad]")
{
    const auto around = sla_support_pad_followup_v4::run_around_object_pad(false);
    const auto everywhere = sla_support_pad_followup_v4::run_around_object_pad(true);
    CHECK(around.empty);
    REQUIRE_FALSE(everywhere.empty);
    CHECK(everywhere.width > 10.0);
    CHECK(everywhere.depth > 10.0);
    CHECK(everywhere.z_span > 0.0);
    CHECK(everywhere.volume > 0.0);
}

TEST_CASE("SLA everywhere pads retain a positive product after filtering manual base points",
          "[SLAPrint][SLAFeatures][SLAAroundPad][SLAManualSupportPoints]")
{
    DynamicPrintConfig config = sla_config(1.0);
    config.set_key_value("supports_enable", new ConfigOptionBool(true));
    sla_support_pad_followup_v4::enable_common_pad(config, true);
    config.set_key_value("support_head_front_diameter", new ConfigOptionFloat(1.0));
    config.set_key_value("support_pillar_diameter", new ConfigOptionFloat(1.0));
    config.set_key_value("support_base_diameter", new ConfigOptionFloat(2.0));
    config.set_key_value("support_base_height", new ConfigOptionFloat(1.0));
    config.set_key_value("support_base_safety_distance", new ConfigOptionFloat(1.0));

    sla::SupportPoints points{
        sla::SupportPoint(2.f, 2.f, 0.f, 0.5f, true),
        sla::SupportPoint(8.f, 8.f, 0.f, 0.5f, false),
    };
    SlaCubeFixture fixture(std::move(config), make_cube(10.0, 10.0, 4.0), std::move(points));
    REQUIRE(fixture.apply_status != SLAPrint::APPLY_STATUS_UNCHANGED);
    INFO("SLA everywhere support validation: " << fixture.print.validate().string);
    REQUIRE(fixture.print.validate().string.empty());
    fixture.print.process();
    REQUIRE(fixture.print.finished());
    REQUIRE(fixture.print.objects().size() == 1);

    const SLAPrintObject *object = fixture.print.objects().front();
    REQUIRE(object->is_step_done(slaposSupportPoints));
    REQUIRE(object->is_step_done(slaposSupportTree));
    REQUIRE(object->is_step_done(slaposPad));
    REQUIRE(object->get_support_points().empty());
    TriangleMesh pad = object->pad_mesh();
    REQUIRE_FALSE(pad.empty());
    CHECK(std::isfinite(static_cast<double>(pad.volume())));
    CHECK(std::abs(static_cast<double>(pad.volume())) > 0.0);
}

TEST_CASE("SLA around-object pads retain a footprint from real elevated supports",
          "[SLAPrint][SLAFeatures][SLAAroundPad][SLAIntersector]")
{
    DynamicPrintConfig config = sla_config(1.0);
    config.set_key_value("supports_enable", new ConfigOptionBool(true));
    sla_support_pad_followup_v4::enable_common_pad(config, false);
    // Keep the elevated island outside the grounded cube footprint. With buildplate-only
    // support enabled, the pillar must descend unobstructed to the bed.
    config.set_key_value("support_buildplate_only", new ConfigOptionBool(true));
    config.set_key_value("support_head_front_diameter", new ConfigOptionFloat(1.0));
    config.set_key_value("support_head_penetration", new ConfigOptionFloat(0.2));
    config.set_key_value("support_head_width", new ConfigOptionFloat(1.0));
    config.set_key_value("support_pillar_diameter", new ConfigOptionFloat(2.0));
    config.set_key_value("support_base_diameter", new ConfigOptionFloat(4.0));
    config.set_key_value("support_base_height", new ConfigOptionFloat(1.0));
    config.set_key_value("support_base_safety_distance", new ConfigOptionFloat(1.0));

    TriangleMesh model_mesh = make_cube(10.0, 10.0, 1.0);
    TriangleMesh elevated_island = make_cube(6.0, 6.0, 2.0);
    elevated_island.translate(12.f, 2.f, 5.f);
    model_mesh.merge(elevated_island);
    sla::SupportPoints points{sla::SupportPoint(15.f, 5.f, 5.f, 0.5f, true)};

    SlaCubeFixture fixture(std::move(config), std::move(model_mesh), std::move(points));
    REQUIRE(fixture.apply_status != SLAPrint::APPLY_STATUS_UNCHANGED);
    INFO("SLA elevated-support pad validation: " << fixture.print.validate().string);
    REQUIRE(fixture.print.validate().string.empty());
    fixture.print.process();
    REQUIRE(fixture.print.finished());
    REQUIRE(fixture.print.objects().size() == 1);

    const SLAPrintObject *object = fixture.print.objects().front();
    REQUIRE(object->is_step_done(slaposSupportTree));
    REQUIRE(object->is_step_done(slaposPad));
    REQUIRE(object->get_support_points().size() == 1);
    TriangleMesh support = object->support_mesh();
    TriangleMesh pad = object->pad_mesh();
    REQUIRE_FALSE(support.empty());
    REQUIRE_FALSE(pad.empty());
    REQUIRE(std::isfinite(static_cast<double>(support.volume())));
    REQUIRE(std::isfinite(static_cast<double>(pad.volume())));
    REQUIRE(std::abs(static_cast<double>(support.volume())) > 0.0);
    REQUIRE(std::abs(static_cast<double>(pad.volume())) > 0.0);
}

namespace sla_settings_variance_followup {

struct SupportEnableRun {
    bool   support_empty = true;
    double support_volume = 0.0;
    double support_slice_area_scaled2 = 0.0;
};

SupportEnableRun run_support_enable(bool enabled)
{
    DynamicPrintConfig config = sla_config(1.0);
    config.set_key_value("supports_enable", new ConfigOptionBool(enabled));
    config.set_key_value("support_object_elevation", new ConfigOptionFloat(5.0));
    config.set_key_value("support_head_front_diameter", new ConfigOptionFloat(1.0));
    config.set_key_value("support_head_penetration", new ConfigOptionFloat(0.2));
    config.set_key_value("support_head_width", new ConfigOptionFloat(1.0));
    config.set_key_value("support_pillar_diameter", new ConfigOptionFloat(1.0));
    config.set_key_value("support_base_diameter", new ConfigOptionFloat(2.0));
    config.set_key_value("support_base_height", new ConfigOptionFloat(1.0));
    config.set_key_value("support_base_safety_distance", new ConfigOptionFloat(0.5));

    sla::SupportPoints points{sla::SupportPoint(5.f, 5.f, 0.f, 0.5f, true)};
    SlaCubeFixture fixture(std::move(config), make_cube(10.0, 10.0, 4.0), std::move(points));
    REQUIRE(fixture.apply_status != SLAPrint::APPLY_STATUS_UNCHANGED);
    INFO("SLA supports_enable validation: " << fixture.print.validate().string);
    REQUIRE(fixture.print.validate().string.empty());
    fixture.print.process();
    REQUIRE(fixture.print.finished());
    REQUIRE(fixture.print.objects().size() == 1);

    const SLAPrintObject *object = fixture.print.objects().front();
    REQUIRE(object->is_step_done(slaposObjectSlice));
    TriangleMesh support = object->support_mesh();
    SupportEnableRun result;
    result.support_empty = support.empty();
    result.support_slice_area_scaled2 = support_slice_area(*object);
    if (!enabled) {
        REQUIRE(result.support_empty);
        CHECK_THAT(result.support_slice_area_scaled2, Catch::Matchers::WithinAbs(0.0, 1e-9));
        return result;
    }

    REQUIRE(object->is_step_done(slaposSupportTree));
    REQUIRE_FALSE(result.support_empty);
    result.support_volume = std::abs(static_cast<double>(support.volume()));
    REQUIRE(std::isfinite(result.support_volume));
    REQUIRE(std::isfinite(result.support_slice_area_scaled2));
    REQUIRE(result.support_volume > 0.0);
    REQUIRE(result.support_slice_area_scaled2 > 0.0);
    return result;
}

struct PadAroundRun {
    bool   pad_empty = true;
    double pad_volume = 0.0;
    double width = 0.0;
    double depth = 0.0;
    double z_span = 0.0;
};

PadAroundRun run_pad_around_object(bool around_object)
{
    DynamicPrintConfig config = sla_config(1.0);
    config.set_key_value("pad_enable", new ConfigOptionBool(true));
    config.set_key_value("pad_around_object", new ConfigOptionBool(around_object));
    config.set_key_value("pad_around_object_everywhere", new ConfigOptionBool(false));
    config.set_key_value("pad_wall_thickness", new ConfigOptionFloat(1.0));
    config.set_key_value("pad_wall_height", new ConfigOptionFloat(1.0));
    config.set_key_value("pad_brim_size", new ConfigOptionFloat(4.0));
    config.set_key_value("pad_wall_slope", new ConfigOptionFloat(45.0));
    config.set_key_value("pad_max_merge_distance", new ConfigOptionFloat(50.0));
    config.set_key_value("pad_object_gap", new ConfigOptionFloat(1.0));

    SlaCubeFixture fixture(std::move(config), make_cube(10.0, 10.0, 4.0));
    REQUIRE(fixture.apply_status != SLAPrint::APPLY_STATUS_UNCHANGED);
    INFO("SLA pad_around_object validation: " << fixture.print.validate().string);
    REQUIRE(fixture.print.validate().string.empty());
    fixture.print.process();
    REQUIRE(fixture.print.finished());
    REQUIRE(fixture.print.objects().size() == 1);

    const SLAPrintObject *object = fixture.print.objects().front();
    REQUIRE(object->is_step_done(slaposPad));
    TriangleMesh pad = object->pad_mesh();
    PadAroundRun result;
    result.pad_empty = pad.empty();
    if (around_object) {
        REQUIRE(result.pad_empty);
        return result;
    }

    REQUIRE_FALSE(result.pad_empty);
    result.pad_volume = std::abs(static_cast<double>(pad.volume()));
    const auto bbox = pad.bounding_box();
    result.width = bbox.max.x() - bbox.min.x();
    result.depth = bbox.max.y() - bbox.min.y();
    result.z_span = bbox.max.z() - bbox.min.z();
    REQUIRE(std::isfinite(result.pad_volume));
    REQUIRE(std::isfinite(result.width));
    REQUIRE(std::isfinite(result.depth));
    REQUIRE(std::isfinite(result.z_span));
    REQUIRE(result.pad_volume > 0.0);
    REQUIRE(result.width > 10.0);
    REQUIRE(result.depth > 10.0);
    REQUIRE(result.z_span > 0.0);
    return result;
}

} // namespace sla_settings_variance_followup

TEST_CASE("SLA supports_enable controls manual support production on one fixture",
          "[SLAPrint][SLAFeatures][SLASupportsEnableVariance]")
{
    const auto disabled = sla_settings_variance_followup::run_support_enable(false);
    const auto enabled = sla_settings_variance_followup::run_support_enable(true);

    REQUIRE(disabled.support_empty);
    CHECK_THAT(disabled.support_slice_area_scaled2, Catch::Matchers::WithinAbs(0.0, 1e-9));
    REQUIRE_FALSE(enabled.support_empty);
    CHECK(enabled.support_volume > 0.0);
    CHECK(enabled.support_slice_area_scaled2 > 0.0);
}

TEST_CASE("SLA pad_around_object selects the standard or around-object pad",
          "[SLAPrint][SLAFeatures][SLAPadAroundObjectVariance]")
{
    const auto standard = sla_settings_variance_followup::run_pad_around_object(false);
    const auto around   = sla_settings_variance_followup::run_pad_around_object(true);

    REQUIRE_FALSE(standard.pad_empty);
    CHECK(standard.pad_volume > 0.0);
    CHECK(standard.width > 10.0);
    CHECK(standard.depth > 10.0);
    CHECK(standard.z_span > 0.0);
    REQUIRE(around.pad_empty);
}

TEST_CASE("SLA elephant foot correction fades by layer and preserves narrow features",
          "[SLAPrint][SLAElephantFootFade]")
{
    struct Settings {
        double compensation;
        int faded_layers;
        double minimum_width;
    };
    const std::vector<Settings> settings{
        {0.0, 4, 0.2},
        {0.3, 4, 0.2},
        {0.3, 3, 0.2},
        {0.3, 4, 30.0},
        {0.3, 20, 0.2},
    };
    std::vector<sla_correction_variance_contracts::Run> runs;
    for (const Settings &row : settings) {
        auto config = sla_correction_variance_contracts::correction_config();
        config.set_key_value("elefant_foot_compensation", new ConfigOptionFloat(row.compensation));
        config.set_key_value("faded_layers", new ConfigOptionInt(row.faded_layers));
        config.set_key_value("elefant_foot_min_width", new ConfigOptionFloat(row.minimum_width));
        runs.emplace_back(sla_correction_variance_contracts::process_cube(std::move(config)));
    }

    for (const auto &run : runs) {
        REQUIRE(run.model_areas.size() == 4); // Four 1 mm layers in the 4 mm cube.
        CHECK_THAT(run.bbox_x, Catch::Matchers::WithinAbs(10.0, 1e-6));
        CHECK_THAT(run.bbox_y, Catch::Matchers::WithinAbs(10.0, 1e-6));
        CHECK_THAT(run.bbox_z, Catch::Matchers::WithinAbs(4.0, 1e-6));
        for (std::size_t i = 0; i < run.model_areas.size(); ++i) {
            CHECK(run.model_indices[i] == runs.front().model_indices[i]);
            CHECK_THAT(run.model_levels[i], Catch::Matchers::WithinAbs(runs.front().model_levels[i], 1e-6));
        }
    }
    for (double area : runs[0].model_areas)
        CHECK_THAT(area, Catch::Matchers::WithinAbs(100.0, 0.02));

    // A wider-than-object minimum prevents compensation from eroding the
    // feature. The ordinary 0.2 mm minimum allows the 10 mm square to shrink.
    for (double area : runs[3].model_areas)
        CHECK_THAT(area, Catch::Matchers::WithinAbs(100.0, 0.02));
    const auto &four = runs[1].model_areas;
    const auto &three = runs[2].model_areas;
    CHECK(four[0] < four[1] - 0.02);
    CHECK(four[1] < four[2] - 0.02);
    CHECK(four[2] < four[3] - 0.02);
    CHECK_THAT(four[3], Catch::Matchers::WithinAbs(100.0, 0.02));

    // Three fade layers reach the unchanged section one layer earlier, while
    // the configured first-layer correction remains identical in both rows.
    CHECK_THAT(three[0], Catch::Matchers::WithinAbs(four[0], 0.02));
    CHECK(three[1] > four[1] + 0.02);
    CHECK(three[1] < 100.0 - 0.02);
    CHECK_THAT(three[2], Catch::Matchers::WithinAbs(100.0, 0.02));
    CHECK_THAT(three[3], Catch::Matchers::WithinAbs(100.0, 0.02));

    // The registered maximum is clamped to the number of printable layers.
    for (std::size_t i = 0; i < four.size(); ++i)
        CHECK_THAT(runs[4].model_areas[i], Catch::Matchers::WithinAbs(four[i], 0.02));
}

namespace sla_raster_active_contracts {

class ArchiveReader
{
public:
    explicit ArchiveReader(const std::string &path)
        : opened(open_zip_reader(&archive.arch, path))
    {}

    ~ArchiveReader()
    {
        if (opened)
            close_zip_reader(&archive.arch);
    }

    bool is_open() const { return opened; }
    MZ_Archive archive;

private:
    bool opened;
};

std::vector<std::uint8_t> read_entry(ArchiveReader &reader, const std::string &name)
{
    const int index = mz_zip_reader_locate_file(&reader.archive.arch, name.c_str(), nullptr, 0);
    REQUIRE(index >= 0);

    mz_zip_archive_file_stat stat;
    REQUIRE(mz_zip_reader_file_stat(&reader.archive.arch, mz_uint(index), &stat));

    size_t extracted_size = 0;
    void *raw = mz_zip_reader_extract_to_heap(&reader.archive.arch, mz_uint(index), &extracted_size, 0);
    std::unique_ptr<void, decltype(&mz_free)> owner(raw, &mz_free);
    REQUIRE(owner != nullptr);
    REQUIRE(extracted_size == size_t(stat.m_uncomp_size));

    const auto *begin = static_cast<const std::uint8_t *>(owner.get());
    return std::vector<std::uint8_t>(begin, begin + extracted_size);
}

struct RasterLayerSummary
{
    std::vector<std::uint8_t> mask;
    std::size_t               active_pixels = 0;
    double                    area_mm2 = 0.;
    double                    centroid_x = 0.;
    double                    centroid_y = 0.;
    double                    width_mm = 0.;
    double                    height_mm = 0.;
};

RasterLayerSummary summarize_png(const std::vector<std::uint8_t> &bytes,
                                 const std::size_t expected_pixels,
                                 const double       pixel_mm)
{
    png::ImageGreyscale image;
    const png::ReadBuf input{bytes.data(), bytes.size()};
    REQUIRE(png::decode_png(input, image));
    REQUIRE(image.cols == expected_pixels);
    REQUIRE(image.rows == expected_pixels);
    REQUIRE(image.buf.size() == expected_pixels * expected_pixels);

    RasterLayerSummary result;
    result.mask.resize(image.buf.size());
    std::size_t min_col = image.cols;
    std::size_t min_row = image.rows;
    std::size_t max_col = 0;
    std::size_t max_row = 0;
    double sum_x = 0.;
    double sum_y = 0.;

    for (std::size_t row = 0; row < image.rows; ++row) {
        for (std::size_t col = 0; col < image.cols; ++col) {
            const std::uint8_t value = image.get(row, col);
            const bool active = value >= 128;
            result.mask[row * image.cols + col] = active ? 1 : 0;
            if (!active)
                continue;

            ++result.active_pixels;
            min_col = std::min(min_col, col);
            min_row = std::min(min_row, row);
            max_col = std::max(max_col, col);
            max_row = std::max(max_row, row);
            sum_x += (double(col) + 0.5) * pixel_mm;
            sum_y += (double(image.rows - row) - 0.5) * pixel_mm;
        }
    }

    REQUIRE(result.active_pixels > 0);
    result.area_mm2 = double(result.active_pixels) * pixel_mm * pixel_mm;
    result.centroid_x = sum_x / double(result.active_pixels);
    result.centroid_y = sum_y / double(result.active_pixels);
    result.width_mm = double(max_col - min_col + 1) * pixel_mm;
    result.height_mm = double(max_row - min_row + 1) * pixel_mm;
    return result;
}

struct ExportProfile
{
    SLAMaterialSpeed material_speed = slamsFast;
    std::string material_name;
    std::string printer_model;
    std::string printer_variant;
    std::string printer_profile;
    std::string print_profile;
};

struct RasterRun
{
    std::vector<RasterLayerSummary> layers;
    std::vector<coord_t>             levels;
    std::string                      config_ini;
};

RasterRun run_raster_export(ExportProfile profile = {})
{
    DynamicPrintConfig config = sla_correction_variance_contracts::correction_config();
    config.set_key_value("material_print_speed", new ConfigOptionEnum<SLAMaterialSpeed>(profile.material_speed));
    config.set_key_value("sla_material_settings_id", new ConfigOptionString(profile.material_name));
    config.set_key_value("printer_model", new ConfigOptionString(profile.printer_model));
    config.set_key_value("printer_variant", new ConfigOptionString(profile.printer_variant));
    config.set_key_value("printer_settings_id", new ConfigOptionString(profile.printer_profile));
    config.set_key_value("sla_print_settings_id", new ConfigOptionString(profile.print_profile));
    config.set_key_value("layer_height", new ConfigOptionFloat(1.0));
    config.set_key_value("initial_layer_height", new ConfigOptionFloat(1.0));
    config.set_key_value("display_width", new ConfigOptionFloat(32.0));
    config.set_key_value("display_height", new ConfigOptionFloat(32.0));
    config.set_key_value("display_pixels_x", new ConfigOptionInt(128));
    config.set_key_value("display_pixels_y", new ConfigOptionInt(128));
    config.set_key_value("display_orientation",
                         new ConfigOptionEnum<SLADisplayOrientation>(sladoLandscape));
    config.set_key_value("display_mirror_x", new ConfigOptionBool(false));
    config.set_key_value("display_mirror_y", new ConfigOptionBool(false));
    config.set_key_value("gamma_correction", new ConfigOptionFloat(0.0));

    // SLAPrint::apply supplies the pinned display configuration to this default
    // archive through SLAArchive::apply; no second printer-config copy is needed.
    SL1Archive archive;
    SLAPrint print;
    // The archive is attached before apply so the public rasterize step is enabled
    // by the same lifecycle used by BackgroundSlicingProcess.
    print.set_printer(&archive);

    Model model;
    ModelObject *object = model.add_object();
    REQUIRE(object != nullptr);
    // make_cube() uses vertices from (0,0,0) to (size_x,size_y,size_z);
    // the explicit upper-volume translation therefore keeps both tier centers
    // at (4,4) while placing the union inside the 32 mm raster bed.
    object->add_volume(make_cube(8.0, 8.0, 2.0));
    TriangleMesh upper = make_cube(4.0, 4.0, 2.0);
    upper.translate(Vec3f(2.f, 2.f, 1.99f));
    object->add_volume(std::move(upper));
    object->add_instance();
    object->ensure_on_bed();

    const auto apply_status = print.apply(model, std::move(config));
    REQUIRE(apply_status != SLAPrint::APPLY_STATUS_UNCHANGED);
    INFO("SLA raster validation: " << print.validate().string);
    REQUIRE(print.validate().string.empty());

    print.process();
    REQUIRE(print.finished());
    // minZ=0, maxZ=3.99, and 1 mm initial/layer heights produce the public
    // slice centers 0.5, 1.5, and 2.5; the next center is above maxZ.
    REQUIRE(print.print_layers().size() == 3);
    for (std::size_t i = 1; i < print.print_layers().size(); ++i)
        REQUIRE(print.print_layers()[i - 1].level() < print.print_layers()[i].level());

    const auto transformed_box = print.objects().front()->transformed_mesh().bounding_box();
    CHECK_THAT(transformed_box.min.x(), Catch::Matchers::WithinAbs(0.0, 1e-5));
    CHECK_THAT(transformed_box.min.y(), Catch::Matchers::WithinAbs(0.0, 1e-5));
    CHECK_THAT(transformed_box.min.z(), Catch::Matchers::WithinAbs(0.0, 1e-5));
    CHECK_THAT(transformed_box.max.x(), Catch::Matchers::WithinAbs(8.0, 1e-5));
    CHECK_THAT(transformed_box.max.y(), Catch::Matchers::WithinAbs(8.0, 1e-5));
    CHECK_THAT(transformed_box.max.z(), Catch::Matchers::WithinAbs(3.99, 1e-4));

    ScopedTemporaryFile output(".sl1");
    archive.export_print(output.string(), print, "raster-contract");

    ArchiveReader reader(output.string());
    REQUIRE(reader.is_open());
    REQUIRE(mz_zip_reader_locate_file(&reader.archive.arch, "config.ini", nullptr, 0) >= 0);
    REQUIRE(mz_zip_reader_locate_file(&reader.archive.arch, "prusaslicer.ini", nullptr, 0) >= 0);

    RasterRun result;
    const auto config_ini = read_entry(reader, "config.ini");
    result.config_ini.assign(reinterpret_cast<const char *>(config_ini.data()), config_ini.size());
    result.levels.reserve(print.print_layers().size());
    result.layers.reserve(print.print_layers().size());
    for (const auto &layer : print.print_layers())
        result.levels.push_back(layer.level());

    constexpr std::size_t pixels = 128;
    constexpr double pixel_mm = 32.0 / double(pixels);
    for (std::size_t i = 0; i < result.levels.size(); ++i) {
        const std::string name = string_printf("raster-contract%.5d.png", int(i));
        result.layers.emplace_back(summarize_png(read_entry(reader, name), pixels, pixel_mm));
    }
    return result;
}

} // namespace sla_raster_active_contracts

TEST_CASE("SLA attached SL1 archive rasterizes stepped model layers",
          "[SLAPrint][SLAFeatures][SLARasterPipeline]")
{
    const auto run = sla_raster_active_contracts::run_raster_export();

    REQUIRE(run.levels.size() == 3);
    REQUIRE(run.layers.size() == run.levels.size());
    REQUIRE(run.layers[0].mask == run.layers[1].mask);
    REQUIRE(run.layers[0].mask != run.layers[2].mask);

    for (const auto &layer : run.layers) {
        CHECK(std::isfinite(layer.area_mm2));
        CHECK(std::isfinite(layer.centroid_x));
        CHECK(std::isfinite(layer.centroid_y));
        CHECK_THAT(layer.centroid_x, Catch::Matchers::WithinAbs(4.0, 0.5));
        CHECK_THAT(layer.centroid_y, Catch::Matchers::WithinAbs(4.0, 0.5));
    }

    CHECK_THAT(run.layers[0].area_mm2, Catch::Matchers::WithinAbs(64.0, 1.0));
    CHECK_THAT(run.layers[1].area_mm2, Catch::Matchers::WithinAbs(64.0, 1.0));
    CHECK_THAT(run.layers[2].area_mm2, Catch::Matchers::WithinAbs(16.0, 1.0));
    CHECK_THAT(run.layers[0].width_mm, Catch::Matchers::WithinAbs(8.0, 0.5));
    CHECK_THAT(run.layers[0].height_mm, Catch::Matchers::WithinAbs(8.0, 0.5));
    CHECK_THAT(run.layers[2].width_mm, Catch::Matchers::WithinAbs(4.0, 0.5));
    CHECK_THAT(run.layers[2].height_mm, Catch::Matchers::WithinAbs(4.0, 0.5));
}

TEST_CASE("SLA SL1 export records material speed and profile identity values",
          "[SLAPrint][SLAPipeline][SL1Settings]")
{
    const std::string speed = GENERATE(std::string("slow"), std::string("fast"));
    sla_raster_active_contracts::ExportProfile profile;
    profile.material_speed = speed == "slow" ? slamsSlow : slamsFast;
    profile.material_name = "resin-" + speed;
    profile.printer_model = "model-" + speed;
    profile.printer_variant = "variant-" + speed;
    profile.printer_profile = "printer-profile-" + speed;
    profile.print_profile = "print-profile-" + speed;

    const auto run = sla_raster_active_contracts::run_raster_export(profile);
    const std::string expected_user_profile = speed == "slow" ? "1" : "0";
    CHECK(run.config_ini.find("expUserProfile = " + expected_user_profile + "\n") != std::string::npos);
    CHECK(run.config_ini.find("materialName = " + profile.material_name + "\n") != std::string::npos);
    CHECK(run.config_ini.find("printerModel = " + profile.printer_model + "\n") != std::string::npos);
    CHECK(run.config_ini.find("printerVariant = " + profile.printer_variant + "\n") != std::string::npos);
    CHECK(run.config_ini.find("printerProfile = " + profile.printer_profile + "\n") != std::string::npos);
    CHECK(run.config_ini.find("printProfile = " + profile.print_profile + "\n") != std::string::npos);
}


namespace sla_pad_geometry_boundary_contracts {

struct AroundPadRun {
    bool   empty = true;
    double volume = 0.;
    double footprint_x = 0.;
    double footprint_y = 0.;
    double support_volume = 0.;
    double model_area = 0.;
};

AroundPadRun run_around_pad(double object_gap, double connector_stride,
                            double connector_width, double connector_penetration,
                            double support_base_safety_distance)
{
    DynamicPrintConfig config = sla_config(1.0);
    config.set_key_value("supports_enable", new ConfigOptionBool(true));
    sla_support_pad_followup_v4::enable_common_pad(config, false);
    config.set_key_value("pad_object_gap", new ConfigOptionFloat(object_gap));
    config.set_key_value("pad_object_connector_stride", new ConfigOptionFloat(connector_stride));
    config.set_key_value("pad_object_connector_width", new ConfigOptionFloat(connector_width));
    config.set_key_value("pad_object_connector_penetration", new ConfigOptionFloat(connector_penetration));
    config.set_key_value("support_buildplate_only", new ConfigOptionBool(true));
    config.set_key_value("support_head_front_diameter", new ConfigOptionFloat(1.0));
    config.set_key_value("support_head_penetration", new ConfigOptionFloat(0.2));
    config.set_key_value("support_head_width", new ConfigOptionFloat(1.0));
    config.set_key_value("support_pillar_diameter", new ConfigOptionFloat(2.0));
    config.set_key_value("support_base_diameter", new ConfigOptionFloat(4.0));
    config.set_key_value("support_base_height", new ConfigOptionFloat(1.0));
    config.set_key_value("support_base_safety_distance",
                         new ConfigOptionFloat(support_base_safety_distance));

    TriangleMesh model_mesh = make_cube(10.0, 10.0, 1.0);
    TriangleMesh elevated_island = make_cube(6.0, 6.0, 2.0);
    elevated_island.translate(12.f, 2.f, 5.f);
    model_mesh.merge(elevated_island);
    sla::SupportPoints points{sla::SupportPoint(15.f, 5.f, 5.f, 0.5f, true)};

    SlaCubeFixture fixture(std::move(config), std::move(model_mesh), std::move(points));
    REQUIRE(fixture.apply_status != SLAPrint::APPLY_STATUS_UNCHANGED);
    const auto validation = fixture.print.validate();
    INFO("embedded pad validation: " << validation.string);
    REQUIRE(validation.string.empty());
    fixture.print.process();
    REQUIRE(fixture.print.finished());
    REQUIRE(fixture.print.objects().size() == 1);

    const SLAPrintObject *object = fixture.print.objects().front();
    REQUIRE(object->is_step_done(slaposSupportTree));
    REQUIRE(object->is_step_done(slaposPad));
    REQUIRE(object->get_support_points().size() == 1);
    TriangleMesh pad = object->pad_mesh();
    TriangleMesh support = object->support_mesh();
    REQUIRE_FALSE(support.empty());

    AroundPadRun result;
    result.empty = pad.empty();
    result.support_volume = std::abs(static_cast<double>(support.volume()));
    REQUIRE(std::isfinite(result.support_volume));
    REQUIRE(result.support_volume > 0.);
    if (!result.empty) {
        result.volume = std::abs(static_cast<double>(pad.volume()));
        const auto bbox = pad.bounding_box();
        result.footprint_x = bbox.max.x() - bbox.min.x();
        result.footprint_y = bbox.max.y() - bbox.min.y();
        REQUIRE(std::isfinite(result.volume));
        REQUIRE(std::isfinite(result.footprint_x));
        REQUIRE(std::isfinite(result.footprint_y));
        REQUIRE(result.volume > 0.);
        REQUIRE(result.footprint_x > 0.);
        REQUIRE(result.footprint_y > 0.);
    }

    const double area_scale = scaled<double>(1.0) * scaled<double>(1.0);
    for (const SLAPrintObject::SliceRecord &record : object->get_slice_index())
        for (const ExPolygon &slice : record.get_slice(soModel))
            result.model_area += slice.area() / area_scale;
    REQUIRE(std::isfinite(result.model_area));
    REQUIRE(result.model_area > 0.);
    return result;
}

bool geometry_differs(const AroundPadRun &a, const AroundPadRun &b)
{
    if (a.empty != b.empty)
        return true;
    if (a.empty)
        return false;
    constexpr double tolerance = 1e-3;
    return std::abs(a.volume - b.volume) > tolerance ||
           std::abs(a.footprint_x - b.footprint_x) > tolerance ||
           std::abs(a.footprint_y - b.footprint_y) > tolerance;
}

void check_same_model_and_support(const AroundPadRun &reference,
                                 const AroundPadRun &candidate)
{
    CHECK_THAT(candidate.model_area,
               Catch::Matchers::WithinAbs(reference.model_area, 1e-4));
    CHECK_THAT(candidate.support_volume,
               Catch::Matchers::WithinAbs(reference.support_volume, 1e-4));
}

} // namespace sla_pad_geometry_boundary_contracts

// pad_blueprint samples from the ground plane in layer_height steps below the
// full-height endpoint. A 1 mm stack at a 1 mm stride has no interior sample,
// so these endpoint sweeps use 0.5 mm layers to exercise generated geometry.
TEST_CASE("SLA pad wall thickness honors its registered boundaries",
          "[SLAPrint][SLAFeatures][SLAPadWallThicknessBoundary]")
{
    const std::vector<double> values{0.0, 1.0, 30.0};
    std::vector<sla_pad_variance_contracts::PadVarianceRun> runs;
    for (const double value : values) {
        INFO("pad_wall_thickness=" << value);
        runs.emplace_back(sla_pad_variance_contracts::run_pad(value, 30.0, 1.0, 90.0, 0.5));
    }
    REQUIRE(runs.size() == values.size());
    for (std::size_t i = 0; i < runs.size(); ++i) {
        CHECK_THAT(runs[i].z_span, Catch::Matchers::WithinAbs(values[i] + 1.0, 1e-4));
        CHECK_THAT(runs[i].model_area,
                   Catch::Matchers::WithinAbs(runs.front().model_area, 1e-4));
    }
    CHECK(runs.front().z_span < runs.back().z_span);
}

TEST_CASE("SLA pad wall height honors its registered boundaries",
          "[SLAPrint][SLAFeatures][SLAPadWallHeightBoundary]")
{
    const std::vector<double> values{0.0, 1.0, 30.0};
    std::vector<sla_pad_variance_contracts::PadVarianceRun> runs;
    for (const double value : values) {
        INFO("pad_wall_height=" << value);
        runs.emplace_back(sla_pad_variance_contracts::run_pad(1.0, 30.0, value, 90.0, 0.5));
    }
    REQUIRE(runs.size() == values.size());
    for (std::size_t i = 0; i < runs.size(); ++i) {
        CHECK_THAT(runs[i].z_span, Catch::Matchers::WithinAbs(values[i] + 1.0, 1e-4));
        CHECK_THAT(runs[i].model_area,
                   Catch::Matchers::WithinAbs(runs.front().model_area, 1e-4));
    }
    CHECK(runs.front().z_span < runs.back().z_span);
}

TEST_CASE("SLA pad brim rejects zero and expands through its valid range",
          "[SLAPrint][SLAFeatures][SLAPadBrimBoundary]")
{
    DynamicPrintConfig invalid_config = sla_config(1.0);
    invalid_config.set_key_value("pad_enable", new ConfigOptionBool(true));
    invalid_config.set_key_value("pad_wall_thickness", new ConfigOptionFloat(0.8));
    invalid_config.set_key_value("pad_wall_height", new ConfigOptionFloat(1.0));
    invalid_config.set_key_value("pad_brim_size", new ConfigOptionFloat(0.0));
    invalid_config.set_key_value("pad_wall_slope", new ConfigOptionFloat(90.0));
    SlaCubeFixture invalid_fixture(std::move(invalid_config), make_cube(10.0, 10.0, 4.0));
    REQUIRE(invalid_fixture.apply_status != SLAPrint::APPLY_STATUS_UNCHANGED);
    const auto rejection = invalid_fixture.print.validate();
    CHECK(rejection.string.find("Pad brim size is too small") != std::string::npos);

    const std::vector<double> values{0.1, 5.0, 30.0};
    std::vector<sla_pad_variance_contracts::PadVarianceRun> runs;
    for (const double value : values) {
        INFO("pad_brim_size=" << value);
        runs.emplace_back(sla_pad_variance_contracts::run_pad(0.8, value, 1.0, 90.0));
    }
    REQUIRE(runs.size() == values.size());
    for (std::size_t i = 1; i < runs.size(); ++i) {
        CHECK(runs[i].footprint_x > runs[i - 1].footprint_x);
        CHECK(runs[i].footprint_y > runs[i - 1].footprint_y);
        CHECK_THAT(runs[i].model_area,
                   Catch::Matchers::WithinAbs(runs.front().model_area, 1e-4));
    }
}

TEST_CASE("SLA pad wall slope boundaries change the public footprint",
          "[SLAPrint][SLAFeatures][SLAPadWallSlopeBoundary]")
{
    const std::vector<double> values{45.0, 67.5, 90.0};
    std::vector<sla_pad_variance_contracts::PadVarianceRun> runs;
    for (const double value : values) {
        INFO("pad_wall_slope=" << value);
        runs.emplace_back(sla_pad_variance_contracts::run_pad(1.0, 30.0, 1.0, value));
    }
    REQUIRE(runs.size() == values.size());
    for (const auto &run : runs)
        CHECK_THAT(run.z_span, Catch::Matchers::WithinAbs(2.0, 1e-4));
    for (std::size_t i = 1; i < runs.size(); ++i) {
        CHECK(runs[i - 1].footprint_x > runs[i].footprint_x);
        CHECK(runs[i - 1].footprint_y > runs[i].footprint_y);
        CHECK_THAT(runs[i].model_area,
                   Catch::Matchers::WithinAbs(runs.front().model_area, 1e-4));
    }
}

TEST_CASE("SLA around-object pad gap boundaries change pad geometry",
          "[SLAPrint][SLAFeatures][SLAPadObjectGapBoundary]")
{
    const auto no_gap = sla_pad_geometry_boundary_contracts::run_around_pad(0.0, 1.0, 0.5, 0.3, 10.0);
    const auto representative = sla_pad_geometry_boundary_contracts::run_around_pad(1.0, 1.0, 0.5, 0.3, 10.0);
    const auto maximum = sla_pad_geometry_boundary_contracts::run_around_pad(10.0, 1.0, 0.5, 0.3, 10.0);

    // Production permits an empty pad when around-object mode is enabled and
    // everywhere mode is off. Accept either valid endpoint result, but require
    // the zero-gap geometry to differ from the positive-gap reference.
    CHECK(sla_pad_geometry_boundary_contracts::geometry_differs(no_gap, representative));
    REQUIRE_FALSE(representative.empty);
    REQUIRE_FALSE(maximum.empty);
    CHECK(sla_pad_geometry_boundary_contracts::geometry_differs(maximum, representative));
    sla_pad_geometry_boundary_contracts::check_same_model_and_support(representative, no_gap);
    sla_pad_geometry_boundary_contracts::check_same_model_and_support(representative, maximum);
}

TEST_CASE("SLA around-object connector boundaries change pad geometry",
          "[SLAPrint][SLAFeatures][SLAPadConnectorBoundary]")
{
    const auto reference = sla_pad_geometry_boundary_contracts::run_around_pad(1.0, 2.0, 1.0, 1.0, 1.0);
    const auto no_stride = sla_pad_geometry_boundary_contracts::run_around_pad(1.0, 0.0, 1.0, 1.0, 1.0);
    const auto no_width = sla_pad_geometry_boundary_contracts::run_around_pad(1.0, 2.0, 0.0, 1.0, 1.0);
    const auto zero_penetration = sla_pad_geometry_boundary_contracts::run_around_pad(1.0, 2.0, 1.0, 0.0, 1.0);

    REQUIRE_FALSE(reference.empty);
    REQUIRE_FALSE(no_stride.empty);
    REQUIRE_FALSE(no_width.empty);
    REQUIRE_FALSE(zero_penetration.empty);
    CHECK(sla_pad_geometry_boundary_contracts::geometry_differs(reference, no_stride));
    CHECK(sla_pad_geometry_boundary_contracts::geometry_differs(reference, no_width));
    CHECK(sla_pad_geometry_boundary_contracts::geometry_differs(reference, zero_penetration));
    sla_pad_geometry_boundary_contracts::check_same_model_and_support(reference, no_stride);
    sla_pad_geometry_boundary_contracts::check_same_model_and_support(reference, no_width);
    sla_pad_geometry_boundary_contracts::check_same_model_and_support(reference, zero_penetration);
}
