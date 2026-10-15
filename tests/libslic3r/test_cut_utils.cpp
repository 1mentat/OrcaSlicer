#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <algorithm>
#include <cstddef>
#include <cmath>
#include <vector>

#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/CutUtils.hpp"
#include "libslic3r/Geometry.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "test_utils.hpp"

using namespace Slic3r;

namespace {

constexpr double COORD_EPSILON = 1e-4;

double mesh_volume(const ModelVolume *volume)
{
    TriangleMesh mesh = volume->mesh();
    return std::abs(static_cast<double>(mesh.volume()));
}

BoundingBoxf3 world_bounding_box(const ModelVolume *volume)
{
    return volume->mesh().transformed_bounding_box(volume->get_matrix());
}

ModelObject *add_centered_cube(Model &model, double x, double y, double z)
{
    ModelObject *object = model.add_object();
    object->add_volume(make_cube(x, y, z));
    object->add_instance();
    return object;
}

} // namespace

TEST_CASE("Cut plane conserves a cube as two watertight halves", "[CutUtils]")
{
    Model source_model;
    ModelObject *source_object = add_centered_cube(source_model, 20., 20., 20.);
    const BoundingBoxf3 source_world_box = world_bounding_box(source_object->volumes.front());
    CHECK_THAT(source_world_box.min.z(), Catch::Matchers::WithinAbs(0., COORD_EPSILON));
    CHECK_THAT(source_world_box.max.z(), Catch::Matchers::WithinAbs(20., COORD_EPSILON));
    const double source_volume = mesh_volume(source_object->volumes.front());

    const ModelObjectCutAttributes attributes = ModelObjectCutAttribute::KeepUpper |
                                                 ModelObjectCutAttribute::KeepLower;
    const Transform3d cut_plane = Geometry::translation_transform(Vec3d(0., 0., 10.));
    Cut cut(source_object, 0, cut_plane, attributes);
    const ModelObjectPtrs &parts = cut.perform_with_plane();

    REQUIRE(parts.size() == 2);
    double split_volume = 0.;
    bool found_lower = false;
    bool found_upper = false;
    for (const ModelObject *part : parts) {
        REQUIRE(part != nullptr);
        REQUIRE(part->volumes.size() == 1);
        const ModelVolume *volume = part->volumes.front();
        REQUIRE(volume->is_model_part());
        REQUIRE_FALSE(volume->mesh().empty());
        CHECK(its_num_open_edges(volume->mesh().its) == 0);

        const BoundingBoxf3 box = world_bounding_box(volume);
        split_volume += mesh_volume(volume);
        if (box.max.z() <= 10. + COORD_EPSILON)
            found_lower = true;
        if (box.min.z() >= 10. - COORD_EPSILON)
            found_upper = true;
    }

    CHECK(found_lower);
    CHECK(found_upper);
    CHECK_THAT(split_volume, Catch::Matchers::WithinRel(source_volume, 1e-5));
}

TEST_CASE("Cut plane honors a single retained side", "[CutUtils]")
{
    Model source_model;
    ModelObject *source_object = add_centered_cube(source_model, 20., 20., 20.);

    const Transform3d cut_plane = Geometry::translation_transform(Vec3d(0., 0., 10.));
    Cut cut(source_object, 0, cut_plane, ModelObjectCutAttribute::KeepUpper);
    const ModelObjectPtrs &parts = cut.perform_with_plane();

    REQUIRE(parts.size() == 1);
    REQUIRE(parts.front()->volumes.size() == 1);
    const BoundingBoxf3 box = world_bounding_box(parts.front()->volumes.front());
    CHECK(box.min.z() >= 10. - COORD_EPSILON);
    CHECK(its_num_open_edges(parts.front()->volumes.front()->mesh().its) == 0);
}

TEST_CASE("Cut plane preserves modifiers and splits an unprocessed dowel", "[CutUtils]")
{
    Model source_model;
    ModelObject *source_object = add_centered_cube(source_model, 20., 20., 20.);

    ModelVolume *modifier = source_object->add_volume(make_cube(8., 8., 8.), ModelVolumeType::PARAMETER_MODIFIER);
    modifier->set_offset(Vec3d(0., 0., 10.));
    ModelVolume *connector = source_object->add_volume(make_cube(4., 4., 4.), ModelVolumeType::NEGATIVE_VOLUME);
    connector->set_offset(Vec3d(0., 0., 10.));
    connector->cut_info = ModelVolume::CutInfo(CutConnectorType::Dowel, 0.1f, 0.2f, false);

    const ModelObjectCutAttributes attributes = ModelObjectCutAttribute::KeepUpper |
                                                 ModelObjectCutAttribute::KeepLower;
    const Transform3d cut_plane = Geometry::translation_transform(Vec3d(0., 0., 10.));
    Cut cut(source_object, 0, cut_plane, attributes);
    const ModelObjectPtrs &parts = cut.perform_with_plane();

    REQUIRE(parts.size() == 2);
    bool found_lower_connector = false;
    bool found_upper_connector = false;
    for (const ModelObject *part : parts) {
        bool found_modifier = false;
        const ModelVolume *part_connector = nullptr;
        for (const ModelVolume *volume : part->volumes) {
            if (volume->type() == ModelVolumeType::PARAMETER_MODIFIER)
                found_modifier = true;
            if (volume->is_cut_connector())
                part_connector = volume;
        }
        CHECK(found_modifier);
        REQUIRE(part_connector != nullptr);
        CHECK(part_connector->is_negative_volume());
        CHECK(part_connector->cut_info.connector_type == CutConnectorType::Dowel);
        CHECK_THAT(part_connector->cut_info.radius_tolerance, Catch::Matchers::WithinAbs(0.1, 1e-6));
        CHECK_THAT(part_connector->cut_info.height_tolerance, Catch::Matchers::WithinAbs(0.2, 1e-6));

        const BoundingBoxf3 connector_box = world_bounding_box(part_connector);
        // CutUtils offsets the two preview halves by 0.05 mm in opposite
        // directions, so classify the connector halves by their centers.
        if (connector_box.center().z() < 10.)
            found_lower_connector = true;
        if (connector_box.center().z() > 10.)
            found_upper_connector = true;
    }

    CHECK(found_lower_connector);
    CHECK(found_upper_connector);
}

TEST_CASE("Contour cut distributes and merges selected model parts", "[CutUtils]")
{
    Model source_model;
    ModelObject *source_object = source_model.add_object();
    ModelVolume *upper_a = source_object->add_volume(make_cube(5., 10., 10.));
    upper_a->set_offset(Vec3d(-7.5, 0., 0.));
    ModelVolume *upper_b = source_object->add_volume(make_cube(5., 10., 10.));
    upper_b->set_offset(Vec3d(-2.5, 0., 0.));
    ModelVolume *lower = source_object->add_volume(make_cube(5., 10., 10.));
    lower->set_offset(Vec3d(5., 0., 0.));
    source_object->add_volume(make_cube(30., 12., 12.), ModelVolumeType::PARAMETER_MODIFIER);
    source_object->add_instance();

    const ModelObjectCutAttributes attributes = ModelObjectCutAttribute::KeepUpper |
                                                 ModelObjectCutAttribute::KeepLower;
    Cut cut(source_object, 0, Transform3d::Identity(), attributes);
    const std::vector<Cut::Part> selected_parts {
        { true,  false },
        { true,  false },
        { false, false },
        { true,  true  },
    };
    const ModelObjectPtrs &parts = cut.perform_by_contour(source_object, selected_parts, 0);

    REQUIRE(parts.size() == 2);
    for (const ModelObject *part : parts) {
        size_t model_parts = 0;
        bool has_modifier = false;
        double model_volume = 0.;
        BoundingBoxf3 model_box;
        bool model_box_initialized = false;
        for (const ModelVolume *volume : part->volumes) {
            if (volume->is_model_part()) {
                ++model_parts;
                model_volume += mesh_volume(volume);
                if (!model_box_initialized) {
                    model_box = world_bounding_box(volume);
                    model_box_initialized = true;
                } else {
                    model_box.merge(world_bounding_box(volume));
                }
            }
            if (volume->type() == ModelVolumeType::PARAMETER_MODIFIER)
                has_modifier = true;
        }
        REQUIRE(model_parts == 1);
        REQUIRE(model_box_initialized);
        const bool is_lower_part = model_box.min.x() > COORD_EPSILON;
        // The modifier's original [0, 30] world-space box intersects only the
        // lower selected part ([2.5, 7.5]); distribution must use world bounds.
        CHECK(has_modifier == is_lower_part);
        const double expected_volume = is_lower_part ? 500. : 1000.;
        CHECK_THAT(model_volume, Catch::Matchers::WithinRel(expected_volume, 1e-5));
    }
}

TEST_CASE("Cut plane with no retained side returns no objects", "[CutUtils]")
{
    Model source_model;
    ModelObject *source_object = add_centered_cube(source_model, 10., 10., 10.);

    Cut cut(source_object, 0, Transform3d::Identity(), ModelObjectCutAttribute::KeepAsParts);
    CHECK(cut.perform_with_plane().empty());
}
