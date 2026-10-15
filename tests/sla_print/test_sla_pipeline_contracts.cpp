#include <catch2/catch_all.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

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
#include "libslic3r/PrintBase.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/libslic3r.h"
#include "libslic3r/SLA/SupportPoint.hpp"
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

TEST_CASE("SLA hollowing creates a smaller interior volume", "[SLAFeatures]")
{
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

TEST_CASE("SLA pad processing creates printable pad geometry and slices", "[SLAFeatures]")
{
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
