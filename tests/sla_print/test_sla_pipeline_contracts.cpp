#include <catch2/catch_all.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cstddef>
#include <vector>

#include "libslic3r/Model.hpp"
#include "libslic3r/PrintConfig.hpp"
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
    explicit SlaCubeFixture(DynamicPrintConfig config)
    {
        ModelObject *object = model.add_object();
        object->add_volume(make_cube(10.0, 10.0, 4.0));
        object->add_instance();
        object->ensure_on_bed();
        apply_status = print.apply(model, std::move(config));
    }

    Model              model;
    SLAPrint           print;
    SLAPrint::ApplyStatus apply_status = SLAPrint::APPLY_STATUS_UNCHANGED;
};

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
