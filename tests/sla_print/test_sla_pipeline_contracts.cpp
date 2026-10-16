#include <catch2/generators/catch_generators.hpp>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include "libslic3r/SLA/Hollowing.hpp"
#include "libslic3r/ObjectID.hpp"
#include "libslic3r/BoundingBox.hpp"
#include <string>
#include <algorithm>
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
