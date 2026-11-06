#include <boost/filesystem/path.hpp>
#include <memory>
#include <string>
#include <map>
#include <cstddef>
#include <array>
#include <cmath>
#include "libslic3r/libslic3r.h"
#include <tuple>
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/BuildVolume.hpp"
#include <catch2/catch_message.hpp>
#include <catch2/generators/catch_generators.hpp>
#include "libslic3r/ObjectID.hpp"
#include <catch2/catch_all.hpp>
#include <algorithm>
#include <utility>
#include <vector>
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/Point.hpp"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "libslic3r/Model.hpp"
#include "libslic3r/Geometry.hpp"

using namespace Slic3r;

// convex_hull_2d does not clip geometry below the bed, so these cases avoid
// sinking transforms.
TEST_CASE("A part's 2D convex hull is its footprint projected onto the bed", "[Model]")
{
    Model model;
    ModelObject* object = model.add_object();
    // Keep the cube's raw coordinates ([0,20] on every axis): the default
    // add_volume re-centers the geometry, which would move the footprint.
    object->add_volume(make_cube(20, 20, 20), ModelVolumeType::MODEL_PART, false);

    SECTION("identity transform yields the 20 mm square") {
        const Polygon hull   = object->convex_hull_2d(Geometry::Transformation{}.get_matrix());
        const BoundingBox bb = hull.bounding_box();
        CHECK(hull.size() == 4);
        CHECK(bb.min.x() == scaled(0.));
        CHECK(bb.min.y() == scaled(0.));
        CHECK(bb.max.x() == scaled(20.));
        CHECK(bb.max.y() == scaled(20.));
    }

    SECTION("scaling and offset move and grow the footprint") {
        Geometry::Transformation t;
        t.set_scaling_factor({2, 2, 2}); // cube now spans [0,40]
        t.set_offset({10, 5, 0});        // then shift +10 in X, +5 in Y

        const Polygon hull   = object->convex_hull_2d(t.get_matrix());
        const BoundingBox bb = hull.bounding_box();
        CHECK(hull.size() == 4);
        CHECK(bb.min.x() == scaled(10.));
        CHECK(bb.min.y() == scaled(5.));
        CHECK(bb.max.x() == scaled(50.));
        CHECK(bb.max.y() == scaled(45.));
    }
}

TEST_CASE("An object's raw mesh keeps the triangles of each part on its own vertices", "[Model]")
{
    Model model;
    ModelObject *object = model.add_object();
    object->add_volume(make_cube(10, 10, 10), ModelVolumeType::MODEL_PART, false);
    TriangleMesh second = make_cube(10, 10, 10);
    second.translate(30, 0, 0);
    object->add_volume(std::move(second), ModelVolumeType::MODEL_PART, false);

    // Two separate cubes stay two closed components, one around each cube.
    const std::vector<indexed_triangle_set> parts = its_split(object->raw_indexed_triangle_set());
    REQUIRE(parts.size() == 2);
    std::vector<double> min_x;
    for (const indexed_triangle_set &part : parts) {
        CHECK(part.indices.size() == 12);
        min_x.push_back(bounding_box(part).min.x());
    }
    std::sort(min_x.begin(), min_x.end());
    CHECK_THAT(min_x.front(), Catch::Matchers::WithinAbs(0., 1e-4));
    CHECK_THAT(min_x.back(), Catch::Matchers::WithinAbs(30., 1e-4));
}

TEST_CASE("Model lifecycle preserves material identity and mesh sharing", "[Model][ModelLifecycleSharing]")
{
    Model model;
    ModelObject *object = model.add_object();
    REQUIRE(object != nullptr);
    object->name = "Lifecycle part";
    object->input_file = "lifecycle.stl";
    ModelVolume *part = object->add_volume(make_cube(10, 10, 10), ModelVolumeType::MODEL_PART, false);
    REQUIRE(part != nullptr);
    const ObjectID object_id = object->id();
    const ObjectID volume_id = part->id();

    CHECK(model.add_default_instances());
    REQUIRE(object->instances.size() == 1);
    CHECK(model.add_default_instances());
    CHECK(object->instances.size() == 1);

    ModelMaterial *material = model.add_material("red");
    REQUIRE(material != nullptr);
    material->apply({ { "color", "#ff0000" } });
    part->set_material_id("red");
    CHECK(part->material() == material);
    CHECK(material->attributes.at("color") == "#ff0000");
    CHECK(object->materials_count() == 1);

    ModelVolume *shared = object->add_volume_with_shared_mesh(*part, ModelVolumeType::NEGATIVE_VOLUME);
    REQUIRE(shared != nullptr);
    REQUIRE(object->volumes.size() == 2);
    CHECK(shared->mesh_ptr() == part->mesh_ptr());
    CHECK(shared->is_negative_volume());
    CHECK(object->parts_count() == 1);
    CHECK(object->facets_count() == 12);

    ModelObject *clone = model.add_object(*object);
    REQUIRE(clone != nullptr);
    REQUIRE(clone->volumes.size() == object->volumes.size());
    REQUIRE(clone->volumes.size() == 2);
    REQUIRE(clone->volumes.front() != nullptr);
    CHECK(clone->id() != object_id);
    CHECK(clone->volumes.front()->id() != volume_id);
    CHECK(clone->volumes.front()->mesh_ptr() == part->mesh_ptr());
    CHECK(clone->instances.size() == 1);

    CHECK(model.delete_object(clone));
    CHECK_FALSE(model.delete_object(ObjectID()));
    CHECK(model.delete_object(object_id));
    CHECK(model.objects.empty());
}



namespace model_bounds_contracts {
namespace {

void check_vec3d(const Vec3d &actual, const Vec3d &expected, double epsilon = 1e-5)
{
    CHECK_THAT(actual.x(), Catch::Matchers::WithinAbs(expected.x(), epsilon));
    CHECK_THAT(actual.y(), Catch::Matchers::WithinAbs(expected.y(), epsilon));
    CHECK_THAT(actual.z(), Catch::Matchers::WithinAbs(expected.z(), epsilon));
}


} // namespace
} // namespace model_bounds_contracts

namespace model_bounds_contracts {

TEST_CASE("Model instance transforms have analytic world and local bounds", "[Model][ModelInstanceBounds]")
{
    Model model;
    ModelObject *object = model.add_object();
    REQUIRE(object != nullptr);
    REQUIRE(object->add_volume(make_cube(10, 10, 10), ModelVolumeType::MODEL_PART, false) != nullptr);
    ModelInstance *instance = object->add_instance(Vec3d(20, 30, -5), Vec3d(2, 1, 1), Vec3d::Zero(), Vec3d::Ones());
    REQUIRE(instance != nullptr);

    const BoundingBoxf3 world_box = object->instance_bounding_box(*instance);
    check_vec3d(world_box.min, Vec3d(20, 30, -5), 1e-5);
    check_vec3d(world_box.max, Vec3d(40, 40, 5), 1e-5);
    const BoundingBoxf3 no_offset_box = object->instance_bounding_box(*instance, true);
    check_vec3d(no_offset_box.min, Vec3d(0, 0, 0), 1e-5);
    check_vec3d(no_offset_box.max, Vec3d(20, 10, 10), 1e-5);
    check_vec3d(object->raw_mesh_bounding_box().min, Vec3d(0, 0, 0), 1e-5);
    check_vec3d(object->raw_mesh_bounding_box().max, Vec3d(10, 10, 10), 1e-5);
    CHECK_THAT(object->min_z(), Catch::Matchers::WithinAbs(-5., 1e-6));
    CHECK_THAT(object->max_z(), Catch::Matchers::WithinAbs(5., 1e-6));
}

} // namespace model_bounds_contracts

namespace model_bounds_contracts {

TEST_CASE("Model instance centering and bed placement retain projected dimensions", "[Model][ModelInstancePlacement]")
{
    Model model;
    ModelObject *object = model.add_object();
    REQUIRE(object != nullptr);
    REQUIRE(object->add_volume(make_cube(10, 10, 10), ModelVolumeType::MODEL_PART, false) != nullptr);
    ModelInstance *instance = object->add_instance(Vec3d(20, 30, -5), Vec3d(2, 1, 1), Vec3d::Zero(), Vec3d::Ones());
    REQUIRE(instance != nullptr);

    CHECK(model.center_instances_around_point(Vec2d(0, 0)));
    CHECK_FALSE(model.center_instances_around_point(Vec2d(0, 0)));
    const Vec3d centered = object->bounding_box_exact().center();
    CHECK_THAT(centered.x(), Catch::Matchers::WithinAbs(0., 1e-5));
    CHECK_THAT(centered.y(), Catch::Matchers::WithinAbs(0., 1e-5));

    object->translate_instances(Vec3d(0, 0, -10));
    CHECK_THAT(object->min_z(), Catch::Matchers::WithinAbs(-15., 1e-6));
    object->ensure_on_bed();
    CHECK_THAT(object->min_z(), Catch::Matchers::WithinAbs(0., 1e-6));

    const Polygon instance_hull = instance->convex_hull_2d();
    CHECK(instance_hull.bounding_box().size().x() == scaled(20.));
    CHECK(instance_hull.bounding_box().size().y() == scaled(10.));
}

} // namespace model_bounds_contracts

namespace model_bounds_contracts {

TEST_CASE("Model grid duplication rejects empty and multiple-object models", "[Model][ModelGridRejection]")
{
    Model empty_model;
    CHECK_THROWS(empty_model.duplicate_objects_grid(1, 1, 1.));

    Model two_objects;
    for (int i = 0; i < 2; ++i) {
        ModelObject *object = two_objects.add_object();
        REQUIRE(object != nullptr);
        REQUIRE(object->add_volume(make_cube(1, 1, 1), ModelVolumeType::MODEL_PART, false) != nullptr);
    }
    REQUIRE(two_objects.objects.size() == 2);
    CHECK_THROWS(two_objects.duplicate_objects_grid(1, 1, 1.));
}

} // namespace model_bounds_contracts

namespace model_bounds_contracts {

TEST_CASE("Model grid duplication uses mesh dimensions for spacing", "[Model][ModelGridSpacing]")
{
    const bool has_initial_instance = GENERATE(false, true);
    CAPTURE(has_initial_instance);
    Model model;
    ModelObject *object = model.add_object();
    REQUIRE(object != nullptr);
    REQUIRE(object->add_volume(make_cube(10, 20, 7), ModelVolumeType::MODEL_PART, false) != nullptr);
    if (has_initial_instance)
        REQUIRE(object->add_instance() != nullptr);

    model.duplicate_objects_grid(2, 2, 5.);
    REQUIRE(object->instances.size() == 4);
    for (const ModelInstance *instance : object->instances)
        REQUIRE(instance != nullptr);
    check_vec3d(model.bounding_box_exact().size(), Vec3d(25, 45, 7), 1e-5);
}

} // namespace model_bounds_contracts

namespace model_bounds_contracts {

TEST_CASE("Model print-volume classification follows instance movement", "[Model][ModelPrintVolumeState]")
{
    Model state_model;
    ModelObject *state_object = state_model.add_object();
    REQUIRE(state_object != nullptr);
    REQUIRE(state_object->add_volume(make_cube(10, 10, 10), ModelVolumeType::MODEL_PART, false) != nullptr);
    ModelInstance *state_instance = state_object->add_instance();
    REQUIRE(state_instance != nullptr);
    const BuildVolume bed({Vec2d(0, 0), Vec2d(100, 0), Vec2d(100, 100), Vec2d(0, 100)}, 100., {}, {});
    CHECK(state_model.update_print_volume_state(bed) == 1);
    CHECK(state_instance->print_volume_state == ModelInstancePVS_Inside);
    state_instance->set_offset(Vec3d(95, 0, 0));
    state_object->invalidate_bounding_box();
    CHECK(state_model.update_print_volume_state(bed) == 0);
    CHECK(state_instance->print_volume_state == ModelInstancePVS_Partly_Outside);
    state_instance->set_offset(Vec3d(200, 0, 0));
    state_object->invalidate_bounding_box();
    CHECK(state_model.update_print_volume_state(bed) == 0);
    CHECK(state_instance->print_volume_state == ModelInstancePVS_Fully_Outside);
}

} // namespace model_bounds_contracts


namespace model_bounds_contracts {

TEST_CASE("Model object splitting preserves disconnected components and positions", "[Model][ModelObjectSplit]")
{
    Model split_model;
    ModelObject *source = split_model.add_object();
    REQUIRE(source != nullptr);
    source->name = "Disconnected";
    TriangleMesh disconnected = make_cube(10, 10, 10);
    TriangleMesh second = make_cube(10, 10, 10);
    second.translate(Vec3f(30, 0, 0));
    disconnected.merge(second);
    REQUIRE(source->add_volume(std::move(disconnected), ModelVolumeType::MODEL_PART, false) != nullptr);
    REQUIRE(source->add_instance() != nullptr);
    ModelObjectPtrs split_objects;
    source->split(&split_objects, false);
    REQUIRE(split_objects.size() == 2);
    for (ModelObject *object : split_objects) {
        REQUIRE(object != nullptr);
        REQUIRE(object->volumes.size() == 1);
        REQUIRE(object->volumes.front() != nullptr);
        CHECK(object->parts_count() == 1);
        CHECK(object->facets_count() == 12);
        REQUIRE(object->instances.size() == 1);
        REQUIRE(object->instances.front() != nullptr);
    }
    std::vector<double> split_min_x;
    for (ModelObject *object : split_objects)
        split_min_x.push_back(object->instance_bounding_box(0).min.x());
    std::sort(split_min_x.begin(), split_min_x.end());
    REQUIRE(split_min_x.size() == 2);
    CHECK_THAT(split_min_x[0], Catch::Matchers::WithinAbs(0., 1e-5));
    CHECK_THAT(split_min_x[1], Catch::Matchers::WithinAbs(30., 1e-5));
}

} // namespace model_bounds_contracts

namespace model_bounds_contracts {

TEST_CASE("Model volume merge preserves geometry and facet count", "[Model][ModelObjectMerge]")
{
    Model merge_model;
    ModelObject *merged = merge_model.add_object();
    REQUIRE(merged != nullptr);
    REQUIRE(merged->add_volume(make_cube(10, 10, 10), ModelVolumeType::MODEL_PART, false) != nullptr);
    TriangleMesh second = make_cube(10, 10, 10);
    second.translate(Vec3f(30, 0, 0));
    REQUIRE(merged->add_volume(std::move(second), ModelVolumeType::MODEL_PART, false) != nullptr);
    REQUIRE(merged->add_instance() != nullptr);
    CHECK(merged->parts_count() == 2);
    CHECK(merged->facets_count() == 24);
    merged->merge();
    REQUIRE(merged->volumes.size() == 1);
    REQUIRE(merged->volumes.front() != nullptr);
    CHECK(merged->parts_count() == 1);
    CHECK(merged->facets_count() == 24);
    check_vec3d(merged->raw_mesh_bounding_box().size(), Vec3d(40, 10, 10), 1e-5);
}

} // namespace model_bounds_contracts

namespace model_bounds_contracts {

TEST_CASE("Model volume types preserve material and extruder semantics", "[Model][ModelVolumeSemantics]")
{
    Model model;
    ModelObject *object = model.add_object();
    REQUIRE(object != nullptr);
    ModelVolume *part = object->add_volume(make_cube(10, 10, 10), ModelVolumeType::MODEL_PART, false);
    ModelVolume *negative = object->add_volume(make_cube(10, 10, 10), ModelVolumeType::NEGATIVE_VOLUME, false);
    REQUIRE(part != nullptr);
    REQUIRE(negative != nullptr);
    object->config.set("extruder", 2);
    part->set_material_id("part-material");
    negative->set_material_id("negative-material");
    part->config.set("extruder", 3);

    CHECK(object->parts_count() == 1);
    CHECK(object->facets_count() == 12);
    CHECK(object->materials_count() == 2);
    CHECK(object->has_solid_mesh());
    CHECK(part->extruder_id() == 3);
    part->config.erase("extruder");
    CHECK(part->extruder_id() == 2);
    CHECK(part->get_extruders() == std::vector<int>{ 2 });
    CHECK(negative->get_extruders().empty());
    ModelVolume *modifier = object->add_volume(make_cube(10, 10, 10), ModelVolumeType::PARAMETER_MODIFIER, false);
    ModelVolume *support = object->add_volume(make_cube(10, 10, 10), ModelVolumeType::SUPPORT_BLOCKER, false);
    REQUIRE(modifier != nullptr);
    REQUIRE(support != nullptr);
    CHECK(support->get_extruders().empty());
    CHECK(modifier->is_modifier());
    CHECK(support->is_support_blocker());
    CHECK_FALSE(part->is_splittable());
    CHECK(ModelVolume::type_from_string(ModelVolume::type_to_string(ModelVolumeType::SUPPORT_ENFORCER)) == ModelVolumeType::SUPPORT_ENFORCER);
}

} // namespace model_bounds_contracts


namespace model_bounds_contracts {

TEST_CASE("Model unit conversion scales meshes and preserves conversion flags", "[Model][ModelUnitConversion]")
{
    Model unit_model;
    ModelObject *inch_object = unit_model.add_object();
    REQUIRE(inch_object != nullptr);
    ModelVolume *inch_volume = inch_object->add_volume(make_cube(1, 1, 1), ModelVolumeType::MODEL_PART, false);
    REQUIRE(inch_volume != nullptr);
    CHECK(unit_model.looks_like_imperial_units());
    unit_model.convert_from_imperial_units(true);
    check_vec3d(inch_volume->mesh().bounding_box().size(), Vec3d(25.4, 25.4, 25.4), 1e-4);
    CHECK(inch_volume->source.is_converted_from_inches);
    CHECK_FALSE(inch_volume->source.is_converted_from_meters);

    Model meter_model;
    ModelObject *meter_object = meter_model.add_object();
    REQUIRE(meter_object != nullptr);
    ModelVolume *meter_volume = meter_object->add_volume(make_cube(0.1, 0.1, 0.1), ModelVolumeType::MODEL_PART, false);
    REQUIRE(meter_volume != nullptr);
    CHECK(meter_model.looks_like_saved_in_meters());
    meter_model.convert_from_meters(true);
    check_vec3d(meter_volume->mesh().bounding_box().size(), Vec3d(100., 100., 100.), 1e-4);
    CHECK(meter_volume->source.is_converted_from_meters);
    CHECK_FALSE(meter_volume->source.is_converted_from_inches);
}

} // namespace model_bounds_contracts


namespace model_bounds_contracts {

TEST_CASE("Model cleanup removes only zero-volume objects", "[Model][ModelZeroVolumeCleanup]")
{
    Model model;
    ModelObject *empty = model.add_object();
    REQUIRE(empty != nullptr);
    TriangleMesh empty_mesh;
    REQUIRE(empty_mesh.volume() == 0.f);
    REQUIRE(empty->add_volume(std::move(empty_mesh), ModelVolumeType::MODEL_PART, false) != nullptr);
    ModelObject *solid = model.add_object();
    REQUIRE(solid != nullptr);
    REQUIRE(solid->add_volume(make_cube(1, 1, 1), ModelVolumeType::MODEL_PART, false) != nullptr);
    CHECK(model.removed_objects_with_zero_volume() == 1);
    REQUIRE(model.objects.size() == 1);
    REQUIRE(model.objects.front() != nullptr);
    CHECK(model.objects.front()->facets_count() == 12);
}

} // namespace model_bounds_contracts

namespace model_bounds_contracts {
namespace {

void check_box(const BoundingBoxf3 &box, const Vec3d &min, const Vec3d &max, double epsilon = 1e-5)
{
    check_vec3d(box.min, min, epsilon);
    check_vec3d(box.max, max, epsilon);
}

} // namespace
} // namespace model_bounds_contracts

namespace model_bounds_contracts {

TEST_CASE("ModelObject mirror reflects each axis and restores cached bounds", "[Model][ModelMirror]")
{
    const auto [axis, reflected_min, reflected_max] = GENERATE(table<Axis, Vec3d, Vec3d>({
        { X, Vec3d(-4., 0., 0.), Vec3d(0., 6., 8.) },
        { Y, Vec3d(0., -6., 0.), Vec3d(4., 0., 8.) },
        { Z, Vec3d(0., 0., -8.), Vec3d(4., 6., 0.) },
    }));

    Model model;
    ModelObject *object = model.add_object();
    REQUIRE(object != nullptr);
    REQUIRE(object->add_volume(make_cube(4.f, 6.f, 8.f), ModelVolumeType::MODEL_PART, false) != nullptr);
    REQUIRE(object->add_volume(make_cube(2.f, 3.f, 5.f), ModelVolumeType::MODEL_PART, false) != nullptr);
    REQUIRE(object->volumes.size() == 2);
    REQUIRE(object->add_instance() != nullptr);

    const BoundingBoxf3 before_approx = object->bounding_box_approx();
    const BoundingBoxf3 before_exact = object->bounding_box_exact();
    check_box(before_approx, Vec3d::Zero(), Vec3d(4., 6., 8.));
    check_box(before_exact, Vec3d::Zero(), Vec3d(4., 6., 8.));

    object->mirror(axis);

    check_box(object->bounding_box_approx(), reflected_min, reflected_max);
    check_box(object->bounding_box_exact(), reflected_min, reflected_max);
    for (const ModelVolume *volume : object->volumes) {
        REQUIRE(volume != nullptr);
        CHECK(volume->is_left_handed());
    }

    object->mirror(axis);

    check_box(object->bounding_box_approx(), Vec3d::Zero(), Vec3d(4., 6., 8.));
    check_box(object->bounding_box_exact(), Vec3d::Zero(), Vec3d(4., 6., 8.));
    for (const ModelVolume *volume : object->volumes) {
        REQUIRE(volume != nullptr);
        CHECK_FALSE(volume->is_left_handed());
    }
}

} // namespace model_bounds_contracts

namespace model_bounds_contracts {
namespace {

std::array<Vec3d, 8> centered_box_corners(const BoundingBoxf3 &bbox)
{
    const Vec3d lo = bbox.min.cast<double>();
    const Vec3d hi = bbox.max.cast<double>();
    return {
        Vec3d(lo.x(), lo.y(), lo.z()), Vec3d(lo.x(), lo.y(), hi.z()),
        Vec3d(lo.x(), hi.y(), lo.z()), Vec3d(lo.x(), hi.y(), hi.z()),
        Vec3d(hi.x(), lo.y(), lo.z()), Vec3d(hi.x(), lo.y(), hi.z()),
        Vec3d(hi.x(), hi.y(), lo.z()), Vec3d(hi.x(), hi.y(), hi.z())
    };
}

double max_corner_error(const Transform3d &chain, const BoundingBoxf3 &bbox)
{
    REQUIRE(chain.matrix().allFinite());
    double max_error = 0.;
    for (const Vec3d &corner : centered_box_corners(bbox)) {
        const double error = (chain * corner - corner).norm();
        REQUIRE(std::isfinite(error));
        max_error = std::max(max_error, error);
    }
    return max_error;
}

Vec3d transformed_box_size(const Transform3d &transform, const BoundingBoxf3 &bbox)
{
    REQUIRE(transform.matrix().allFinite());
    const auto corners = centered_box_corners(bbox);
    Vec3d minimum = transform * corners.front();
    REQUIRE(minimum.allFinite());
    Vec3d maximum = minimum;
    for (std::size_t i = 1; i < corners.size(); ++i) {
        const Vec3d point = transform * corners[i];
        REQUIRE(point.allFinite());
        minimum = minimum.cwiseMin(point);
        maximum = maximum.cwiseMax(point);
    }
    return maximum - minimum;
}

} // namespace
} // namespace model_bounds_contracts

namespace model_bounds_contracts {

TEST_CASE("Volume-to-bed transform inverts uniform scale and rotation while ignoring translation", "[Model][Geometry][VolumeBedTransformation][VolumeBedUniform]")
{
    const BoundingBoxf3 bbox(Vec3d(-2., -3., -4.), Vec3d(2., 3., 4.));
    Geometry::Transformation instance;
    instance.set_scaling_factor(Vec3d(2., 2., 2.));
    instance.set_rotation(Z, Geometry::deg2rad(17.));
    instance.set_offset(Vec3d(37., -11., 5.));

    const Geometry::Transformation volume = Geometry::Transformation::volume_to_bed_transformation(instance, bbox);

    CHECK(volume.get_offset().isApprox(Vec3d::Zero(), 1e-12));
    CHECK_THAT(max_corner_error(instance.get_matrix_no_offset() * volume.get_matrix(), bbox),
               Catch::Matchers::WithinAbs(0., 1e-9));
    CHECK_THAT(volume.get_scaling_factor(X), Catch::Matchers::WithinAbs(0.5, 1e-12));
    CHECK_THAT(volume.get_scaling_factor(Y), Catch::Matchers::WithinAbs(0.5, 1e-12));
    CHECK_THAT(volume.get_scaling_factor(Z), Catch::Matchers::WithinAbs(0.5, 1e-12));
}

} // namespace model_bounds_contracts

namespace model_bounds_contracts {

TEST_CASE("Volume-to-bed transform cancels quarter-turn anisotropic mirror", "[Model][Geometry][VolumeBedTransformation][VolumeBedQuarterTurn]")
{
    const BoundingBoxf3 bbox(Vec3d(-2., -3., -4.), Vec3d(2., 3., 4.));
    Geometry::Transformation instance;
    instance.set_scaling_factor(Vec3d(2., 3., 4.));
    instance.set_rotation(Z, 0.5 * double(PI));
    instance.set_mirror(X, -1.);
    instance.set_offset(Vec3d(-17., 23., 9.));

    const Geometry::Transformation volume = Geometry::Transformation::volume_to_bed_transformation(instance, bbox);

    CHECK(volume.get_offset().isApprox(Vec3d::Zero(), 1e-12));
    CHECK_THAT(max_corner_error(instance.get_matrix_no_offset() * volume.get_matrix(), bbox),
               Catch::Matchers::WithinAbs(0., 1e-9));
    CHECK_THAT(volume.get_scaling_factor(X), Catch::Matchers::WithinAbs(1. / 3., 1e-12));
    CHECK_THAT(volume.get_scaling_factor(Y), Catch::Matchers::WithinAbs(1. / 2., 1e-12));
    CHECK_THAT(volume.get_scaling_factor(Z), Catch::Matchers::WithinAbs(1. / 4., 1e-12));
    // Rotation/reflection factors are not unique; the physical inverse and its
    // handedness are invariant under an equivalent matrix decomposition.
    const Transform3d expected = instance.get_matrix_no_offset().inverse();
    CHECK(volume.get_matrix().isApprox(expected, 1e-12));
    CHECK(volume.is_left_handed());
}

} // namespace model_bounds_contracts

namespace model_bounds_contracts {

TEST_CASE("Volume-to-bed transform keeps general rotation out of the volume", "[Model][Geometry][VolumeBedTransformation][VolumeBedGeneralRotation]")
{
    const BoundingBoxf3 bbox(Vec3d(-2., -3., -4.), Vec3d(2., 3., 4.));
    Geometry::Transformation instance;
    instance.set_scaling_factor(Vec3d(2., 3., 4.));
    instance.set_rotation(Z, Geometry::deg2rad(23.));
    instance.set_mirror(X, -1.);
    instance.set_offset(Vec3d(8., -19., 12.));

    const Geometry::Transformation volume = Geometry::Transformation::volume_to_bed_transformation(instance, bbox);

    CHECK(volume.get_offset().isApprox(Vec3d::Zero(), 1e-12));
    CHECK_THAT(volume.get_rotation().norm(), Catch::Matchers::WithinAbs(0., 1e-12));
    CHECK(volume.get_mirror().isApprox(Vec3d::Ones(), 1e-12));
    CHECK_THAT(volume.get_scaling_factor(X), Catch::Matchers::WithinAbs(0.5, 1e-12));
    CHECK_THAT(volume.get_scaling_factor(Y), Catch::Matchers::WithinAbs(1. / 3., 1e-12));
    CHECK_THAT(volume.get_scaling_factor(Z), Catch::Matchers::WithinAbs(0.25, 1e-12));

    const Vec3d bed_size = transformed_box_size(volume.get_matrix(), bbox);
    CHECK_THAT(bed_size.x(), Catch::Matchers::WithinAbs(2., 1e-12));
    CHECK_THAT(bed_size.y(), Catch::Matchers::WithinAbs(2., 1e-12));
    CHECK_THAT(bed_size.z(), Catch::Matchers::WithinAbs(2., 1e-12));
}

} // namespace model_bounds_contracts

namespace model_extended_contracts {
namespace {

ModelObject *cube_object(Model &model, double size = 10., bool add_instance = true)
{
    ModelObject *object = model.add_object();
    REQUIRE(object != nullptr);
    REQUIRE(object->add_volume(make_cube(float(size), float(size), float(size)), ModelVolumeType::MODEL_PART, false) != nullptr);
    REQUIRE(object->volumes.size() == 1);
    REQUIRE(object->volumes.front() != nullptr);
    if (add_instance) {
        REQUIRE(object->add_instance() != nullptr);
        REQUIRE(object->instances.size() == 1);
        REQUIRE(object->instances.front() != nullptr);
    }
    return object;
}

} // namespace
} // namespace model_extended_contracts

namespace model_extended_contracts {

TEST_CASE("Model copy and clone retain topology while clone IDs are independent", "[Model][ModelCopyClone]")
{
    Model original;
    ModelObject *object = cube_object(original);
    object->name = "copy-source";
    object->volumes.front()->name = "body";
    object->volumes.front()->set_material_id("blue");
    ModelMaterial *material = original.get_material("blue");
    REQUIRE(material != nullptr);
    material->apply({ { "color", "#0000ff" } });

    const ObjectID model_id = original.id();
    const ObjectID object_id = object->id();
    const ObjectID volume_id = object->volumes.front()->id();
    Model copied(original);
    REQUIRE(copied.id() == model_id);
    REQUIRE(copied.objects.size() == 1);
    REQUIRE(copied.objects.front() != nullptr);
    REQUIRE(copied.objects.front()->volumes.size() == 1);
    REQUIRE(copied.objects.front()->volumes.front() != nullptr);
    CHECK(copied.objects.front()->id() == object_id);
    CHECK(copied.objects.front()->volumes.front()->id() == volume_id);
    CHECK(copied.objects.front() != object);
    CHECK(copied.objects.front()->volumes.front()->mesh_ptr() == object->volumes.front()->mesh_ptr());
    const ModelMaterial *copied_material = copied.objects.front()->volumes.front()->material();
    REQUIRE(copied_material != nullptr);
    REQUIRE(copied_material->attributes.count("color") == 1);
    CHECK(copied_material->attributes.at("color") == "#0000ff");

    Model cloned = original.make_clone(original);
    REQUIRE(cloned.objects.size() == 1);
    REQUIRE(cloned.objects.front() != nullptr);
    REQUIRE(cloned.objects.front()->volumes.size() == 1);
    REQUIRE(cloned.objects.front()->volumes.front() != nullptr);
    CHECK(cloned.id() != model_id);
    CHECK(cloned.objects.front()->id() != object_id);
    CHECK(cloned.objects.front()->volumes.front()->id() != volume_id);
    CHECK(cloned.objects.front()->get_model() == &cloned);
    CHECK(cloned.objects.front()->volumes.front()->get_object() == cloned.objects.front());
    CHECK(model_object_list_equal(original, copied));
    CHECK_FALSE(model_object_list_equal(original, cloned));
}

} // namespace model_extended_contracts

namespace model_extended_contracts {

TEST_CASE("Model copy and move assignment preserve parent links and ownership", "[Model][ModelAssignmentOwnership]")
{
    Model source;
    ModelObject *object = cube_object(source);
    object->name = "source";
    ModelMaterial *material = source.add_material("PLA");
    REQUIRE(material != nullptr);
    material->attributes["type"] = "PLA";
    object->volumes.front()->set_material_id("PLA");
    const ObjectID model_id = source.id();

    Model copied;
    copied = source;
    REQUIRE(copied.id() == model_id);
    REQUIRE(copied.objects.size() == 1);
    REQUIRE(copied.objects.front() != nullptr);
    REQUIRE(copied.objects.front()->volumes.size() == 1);
    REQUIRE(copied.objects.front()->volumes.front() != nullptr);
    CHECK(copied.objects.front()->get_model() == &copied);
    CHECK(copied.objects.front()->volumes.front()->get_object() == copied.objects.front());
    const ModelMaterial *copied_material = copied.get_material("PLA");
    REQUIRE(copied_material != nullptr);
    CHECK(copied_material->get_model() == &copied);

    Model moved;
    moved = std::move(copied);
    CHECK(moved.id() == model_id);
    REQUIRE(moved.objects.size() == 1);
    REQUIRE(moved.objects.front() != nullptr);
    CHECK(moved.objects.front()->get_model() == &moved);
    CHECK(copied.objects.empty());
    CHECK(copied.materials.empty());
}

} // namespace model_extended_contracts

namespace model_extended_contracts {

TEST_CASE("Model and object meshes aggregate transformed instances exactly", "[Model][ModelAggregateMeshes]")
{
    Model model;
    ModelObject *object = cube_object(model);
    object->instances.front()->set_offset(Vec3d(5, 7, 2));
    ModelInstance *second = object->add_instance();
    REQUIRE(second != nullptr);
    REQUIRE(object->instances.size() == 2);
    second->set_offset(Vec3d(25, 7, 2));
    second->set_scaling_factor(Vec3d(2, 1, 1));

    const BoundingBoxf3 object_mesh_box = object->mesh().bounding_box();
    const BoundingBoxf3 model_mesh_box = model.mesh().bounding_box();
    model_bounds_contracts::check_box(object_mesh_box, Vec3d(5, 7, 2), Vec3d(45, 17, 12));
    model_bounds_contracts::check_box(model_mesh_box, Vec3d(5, 7, 2), Vec3d(45, 17, 12));
    model_bounds_contracts::check_box(object->instance_convex_hull_bounding_box(size_t(0)), Vec3d(5, 7, 2), Vec3d(15, 17, 12));
    model_bounds_contracts::check_box(object->instance_convex_hull_bounding_box(second), Vec3d(25, 7, 2), Vec3d(45, 17, 12));
}

} // namespace model_extended_contracts

namespace model_extended_contracts {

TEST_CASE("Model instance transform helpers separate translation from vector geometry", "[Model][ModelInstanceHelpers]")
{
    Model model;
    ModelObject *object = cube_object(model);
    ModelInstance *instance = object->instances.front();
    instance->set_offset(Vec3d(11, 13, 17));
    instance->set_scaling_factor(Vec3d(2, 3, 4));
    instance->set_rotation(Z, 0.5);

    const Vec3d transformed = instance->transform_vector(Vec3d(1, 0, 0));
    const Vec3d without_offset = instance->transform_vector(Vec3d(1, 0, 0), true);
    model_bounds_contracts::check_vec3d(without_offset, Vec3d(2. * std::cos(0.5), 2. * std::sin(0.5), 0.));
    model_bounds_contracts::check_vec3d(transformed, Vec3d(11. + 2. * std::cos(0.5), 13. + 2. * std::sin(0.5), 17.));
    model_bounds_contracts::check_vec3d(transformed - without_offset, Vec3d(11, 13, 17));

    BoundingBoxf3 source_box;
    source_box.merge(Vec3d::Zero());
    source_box.merge(Vec3d::Ones());
    model_bounds_contracts::check_box(instance->transform_bounding_box(source_box),
              Vec3d(11. - 3. * std::sin(0.5), 13., 17.),
              Vec3d(11. + 2. * std::cos(0.5), 13. + 2. * std::sin(0.5) + 3. * std::cos(0.5), 21.));

    Polygon polygon({ Point(0, 0), Point(scaled(1.), 0), Point(scaled(1.), scaled(1.)), Point(0, scaled(1.)) });
    instance->transform_polygon(&polygon);
    const double rotated_unit_extent = std::abs(std::cos(0.5)) + std::abs(std::sin(0.5));
    CHECK_THAT(double(polygon.bounding_box().size().x()), Catch::Matchers::WithinAbs(scaled(2. * rotated_unit_extent), 3.));
    CHECK_THAT(double(polygon.bounding_box().size().y()), Catch::Matchers::WithinAbs(scaled(3. * rotated_unit_extent), 3.));
}

} // namespace model_extended_contracts


namespace model_extended_contracts {

TEST_CASE("Scaling to fit respects the limiting X target", "[Model][ModelFitScaling]")
{
    Model model;
    ModelObject *object = cube_object(model);
    object->instances.front()->set_offset(Vec3d(10, 20, 30));
    const Vec3d before = object->bounding_box_exact().size().cast<double>();
    object->scale_to_fit(Vec3d(15, 30, 40));
    const Vec3d after = object->bounding_box_exact().size().cast<double>();

    const double factor = 15. / before.x();
    model_bounds_contracts::check_vec3d(after, before * factor);
    CHECK_THAT(after.x(), Catch::Matchers::WithinAbs(15., 1e-5));
    CHECK(after.y() < 30.);
    CHECK(after.z() < 40.);
}

} // namespace model_extended_contracts


namespace model_extended_contracts {

TEST_CASE("Volume transformations update mesh bounds and handedness", "[Model][ModelVolumeTransforms]")
{
    Model model;
    ModelObject *object = cube_object(model, 4., false);
    ModelVolume *volume = object->volumes.front();
    volume->center_geometry_after_creation();
    model_bounds_contracts::check_box(volume->mesh().bounding_box(), Vec3d(-2, -2, -2), Vec3d(2, 2, 2));
    CHECK_THAT(volume->source.mesh_offset.x(), Catch::Matchers::WithinAbs(2., 1e-5));

    volume->translate(Vec3d(3, 4, 5));
    volume->scale(Vec3d(2, 3, 4));
    model_bounds_contracts::check_vec3d(volume->get_offset(), Vec3d(5, 6, 7));
    model_bounds_contracts::check_vec3d(volume->get_scaling_factor(), Vec3d(2, 3, 4));
    CHECK_FALSE(volume->is_left_handed());
    volume->mirror(X);
    CHECK(volume->is_left_handed());
    volume->rotate(0.5, Z);
    CHECK_THAT(volume->get_rotation(Z), Catch::Matchers::WithinAbs(0.5, 1e-5));
    REQUIRE(volume->get_convex_hull_shared_ptr() != nullptr);
    CHECK(volume->get_convex_hull().facets_count() > 0);
}

} // namespace model_extended_contracts


namespace model_extended_contracts {

TEST_CASE("Model volume split preserves each disconnected solid and its assignment", "[Model][ModelVolumeSplit]")
{
    Model model;
    ModelObject *object = model.add_object();
    REQUIRE(object != nullptr);
    TriangleMesh disconnected = make_cube(10, 10, 10);
    TriangleMesh second = make_cube(10, 10, 10);
    second.translate(Vec3f(30, 0, 0));
    disconnected.merge(second);
    ModelVolume *volume = object->add_volume(std::move(disconnected), ModelVolumeType::MODEL_PART, false);
    REQUIRE(volume != nullptr);
    volume->name = "multi";
    volume->config.set("extruder", 2);
    REQUIRE(object->add_instance() != nullptr);
    REQUIRE(volume->is_splittable());

    const size_t created = volume->split(4, false);
    REQUIRE(created == 2);
    REQUIRE(object->volumes.size() == 2);
    std::vector<double> min_x;
    for (const ModelVolume *part : object->volumes) {
        REQUIRE(part != nullptr);
        REQUIRE(part->is_model_part());
        CHECK(part->mesh().facets_count() == 12);
        CHECK(part->get_extruders() == std::vector<int>{ 2 });
        REQUIRE(part->get_convex_hull_shared_ptr() != nullptr);
        CHECK(part->get_convex_hull().facets_count() > 0);
        min_x.push_back(part->mesh().transformed_bounding_box(part->get_matrix()).min.x());
    }
    std::sort(min_x.begin(), min_x.end());
    CHECK_THAT(min_x[0], Catch::Matchers::WithinAbs(0., 1e-5));
    CHECK_THAT(min_x[1], Catch::Matchers::WithinAbs(30., 1e-5));
}

} // namespace model_extended_contracts

namespace model_extended_contracts {

TEST_CASE("Merging selected volumes retains unselected volume geometry", "[Model][ModelSelectedMerge]")
{
    Model model;
    ModelObject *object = model.add_object();
    REQUIRE(object != nullptr);
    ModelVolume *left = object->add_volume(make_cube(10, 10, 10), ModelVolumeType::MODEL_PART, false);
    REQUIRE(left != nullptr);
    left->set_offset(Vec3d(0, 0, 0));
    ModelVolume *right = object->add_volume(make_cube(10, 10, 10), ModelVolumeType::MODEL_PART, false);
    REQUIRE(right != nullptr);
    right->set_offset(Vec3d(20, 0, 0));
    ModelVolume *modifier = object->add_volume(make_cube(5, 5, 5), ModelVolumeType::PARAMETER_MODIFIER, false);
    REQUIRE(modifier != nullptr);
    modifier->set_offset(Vec3d(40, 0, 0));
    REQUIRE(object->add_instance() != nullptr);

    std::vector<int> selected{ 0, 1 };
    // merge_volumes returns detached objects; Model provides their owning cleanup.
    Model merged_model;
    merged_model.objects = object->merge_volumes(selected);
    const ModelObjectPtrs &result = merged_model.objects;
    REQUIRE(result.size() == 1);
    REQUIRE(result.front() != nullptr);
    REQUIRE(result.front()->volumes.size() == 2);
    REQUIRE(result.front()->volumes.front() != nullptr);
    REQUIRE(result.front()->volumes.back() != nullptr);
    CHECK(result.front()->volumes.front()->is_model_part());
    CHECK(result.front()->volumes.back()->is_modifier());
    model_bounds_contracts::check_box(result.front()->volumes.front()->mesh().transformed_bounding_box(result.front()->volumes.front()->get_matrix()),
              Vec3d(0, 0, 0), Vec3d(30, 10, 10));
    model_bounds_contracts::check_box(result.front()->volumes.back()->mesh().transformed_bounding_box(result.front()->volumes.back()->get_matrix()),
              Vec3d(40, 0, 0), Vec3d(45, 5, 5));
}

} // namespace model_extended_contracts

namespace model_extended_contracts {

TEST_CASE("Multipart conversion preserves world geometry and material volume count", "[Model][ModelMultipartConversion]")
{
    Model model;
    ModelObject *first = cube_object(model, 10.);
    first->name = "first";
    first->input_file = (boost::filesystem::path("model-input") / "first.stl").make_preferred().string();
    first->instances.front()->set_offset(Vec3d(3, 4, 0));
    ModelObject *second = cube_object(model, 5.);
    second->name = "second";
    second->input_file = (boost::filesystem::path("model-input") / "second.stl").make_preferred().string();
    second->instances.front()->set_offset(Vec3d(30, 4, 5));
    const BoundingBoxf3 before = model.mesh().bounding_box();
    REQUIRE(model.looks_like_multipart_object());

    model.convert_multipart_object(2);
    REQUIRE(model.objects.size() == 1);
    REQUIRE(model.objects.front() != nullptr);
    REQUIRE(model.objects.front()->volumes.size() == 2);
    REQUIRE(model.objects.front()->volumes[0] != nullptr);
    REQUIRE(model.objects.front()->volumes[1] != nullptr);
    CHECK(model.objects.front()->instances.empty());
    model_bounds_contracts::check_box(model.objects.front()->full_raw_mesh_bounding_box(), before.min.cast<double>(), before.max.cast<double>());
}

} // namespace model_extended_contracts

namespace model_extended_contracts {

TEST_CASE("Multipart detection distinguishes equal and different minimum Z", "[Model][ModelMultipartDetection]")
{
    Model model;
    cube_object(model, 10.);
    ModelObject *second = cube_object(model, 5.);
    second->instances.front()->set_offset(Vec3d(30, 0, 0));
    CHECK_FALSE(model.looks_like_multipart_object());
    second->instances.front()->set_offset(Vec3d(30, 0, 2));
    second->invalidate_bounding_box();
    CHECK(model.looks_like_multipart_object());
}

} // namespace model_extended_contracts
