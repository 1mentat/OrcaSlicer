#include <catch2/catch_all.hpp>
#include <catch2/catch_message.hpp>
#include <catch2/matchers/catch_matchers.hpp>

#include <catch2/catch_test_macros.hpp>
#include "libslic3r/Format/DRC.hpp"
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/TriangleMesh.hpp"

#include <boost/nowide/fstream.hpp>

#include <draco/attributes/geometry_attribute.h>
#include <cstdint>
#include <draco/attributes/geometry_indices.h>
#include <draco/compression/config/compression_shared.h>
#include <draco/compression/encode.h>
#include <draco/core/draco_types.h>
#include <draco/core/encoder_buffer.h>
#include <draco/mesh/mesh.h>
#include <string>
#include <ios>

#include "test_utils.hpp"

#include <boost/iostreams/device/mapped_file.hpp>
#include <boost/filesystem/path.hpp>
#include <draco/compression/decode.h>
#include <draco/core/decoder_buffer.h>
#include <draco/core/status.h>
#include <draco/attributes/point_attribute.h>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>
#include <utility>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "libslic3r/Model.hpp"

using namespace Slic3r;

namespace {

// Encodes one triangle whose vertices carry a single attribute of the given type.
// The decoder accepts both a mesh without POSITION and a face index past the point count.
void write_drc_triangle(const std::string &path, draco::GeometryAttribute::Type attribute_type, uint32_t second_point)
{
    draco::Mesh mesh;
    mesh.set_num_points(3);
    draco::GeometryAttribute attribute;
    attribute.Init(attribute_type, nullptr, 3, draco::DT_FLOAT32, false, sizeof(float) * 3, 0);
    const int attribute_id = mesh.AddAttribute(attribute, true, 3);
    const float points[3][3] = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}};
    for (int i = 0; i < 3; ++i)
        mesh.attribute(attribute_id)->SetAttributeValue(draco::AttributeValueIndex(i), points[i]);
    draco::Mesh::Face face;
    face[0] = draco::PointIndex(0);
    face[1] = draco::PointIndex(second_point);
    face[2] = draco::PointIndex(2);
    mesh.AddFace(face);

    draco::Encoder encoder;
    encoder.SetEncodingMethod(draco::MESH_SEQUENTIAL_ENCODING);
    draco::EncoderBuffer buffer;
    REQUIRE(encoder.EncodeMeshToBuffer(mesh, &buffer).ok());
    boost::nowide::ofstream out(path, std::ios::binary);
    out.write(buffer.data(), static_cast<std::streamsize>(buffer.size()));
}

} // namespace

TEST_CASE("A Draco triangle with positions loads", "[DRC]")
{
    ScopedTemporaryFile drc(".drc");
    write_drc_triangle(drc.string(), draco::GeometryAttribute::POSITION, 1);

    TriangleMesh mesh;
    REQUIRE(load_drc(drc.string().c_str(), &mesh));
    CHECK(mesh.facets_count() == 1);
}

TEST_CASE("A Draco mesh without a POSITION attribute fails to load", "[DRC][Regression]")
{
    ScopedTemporaryFile drc(".drc");
    write_drc_triangle(drc.string(), draco::GeometryAttribute::GENERIC, 1);

    TriangleMesh mesh;
    CHECK_FALSE(load_drc(drc.string().c_str(), &mesh));
}

TEST_CASE("A Draco face referencing a missing point fails to load", "[DRC][Regression]")
{
    ScopedTemporaryFile drc(".drc");
    write_drc_triangle(drc.string(), draco::GeometryAttribute::POSITION, 200);

    TriangleMesh mesh;
    CHECK_FALSE(load_drc(drc.string().c_str(), &mesh));
}

namespace {

struct DecodedDrcMesh {
    std::vector<Vec3f> vertices;
    std::vector<Vec3i32> faces;
    BoundingBoxf3         bounds;
};

DecodedDrcMesh decode_drc_export(const std::string &path)
{
    boost::iostreams::mapped_file_source file{boost::filesystem::path{path}};
    REQUIRE(file.is_open());

    draco::DecoderBuffer buffer;
    buffer.Init(file.data(), file.size());
    const auto geometry_type = draco::Decoder::GetEncodedGeometryType(&buffer);
    REQUIRE(geometry_type.ok());
    REQUIRE(geometry_type.value() == draco::TRIANGULAR_MESH);

    draco::Decoder decoder;
    draco::Mesh decoded;
    REQUIRE(decoder.DecodeBufferToGeometry(&buffer, &decoded).ok());

    const draco::PointAttribute *positions = decoded.GetNamedAttribute(draco::GeometryAttribute::POSITION);
    REQUIRE(positions != nullptr);

    DecodedDrcMesh result;
    result.faces.reserve(decoded.num_faces());
    result.vertices.reserve(positions->size());
    for (draco::AttributeValueIndex i(0); i < positions->size(); ++i) {
        float xyz[3] {};
        REQUIRE(positions->ConvertValue<float>(i, 3, xyz));
        REQUIRE(std::isfinite(xyz[0]));
        REQUIRE(std::isfinite(xyz[1]));
        REQUIRE(std::isfinite(xyz[2]));
        const Vec3f point(xyz[0], xyz[1], xyz[2]);
        result.vertices.push_back(point);
        result.bounds.merge(point.cast<double>());
    }
    for (draco::FaceIndex i(0); i < decoded.num_faces(); ++i) {
        const draco::Mesh::Face &face = decoded.face(i);
        Vec3i32 mapped_face;
        for (int corner = 0; corner < 3; ++corner) {
            REQUIRE(face[corner].value() < decoded.num_points());
            const draco::AttributeValueIndex mapped = positions->mapped_index(face[corner]);
            REQUIRE(mapped.value() < positions->size());
            mapped_face[corner] = static_cast<int>(mapped.value());
        }
        result.faces.push_back(mapped_face);
    }
    return result;
}

TriangleMesh drc_asymmetric_tetrahedron()
{
    std::vector<Vec3f> vertices {
        { 0.137f, -0.271f,  0.413f },
        { 3.771f,  0.239f,  0.593f },
        { 0.617f,  4.113f,  0.827f },
        { 1.341f,  0.871f,  2.937f },
    };
    std::vector<Vec3i32> faces {
        { 0, 1, 2 }, { 0, 3, 1 }, { 1, 3, 2 }, { 2, 3, 0 }
    };
    return TriangleMesh(std::move(vertices), std::move(faces));
}

double nearest_vertex_error(const std::vector<Vec3f> &actual, const TriangleMesh &expected,
                             std::vector<int> *source_for_actual = nullptr)
{
    REQUIRE(actual.size() == expected.its.vertices.size());
    std::vector<bool> used(expected.its.vertices.size(), false);
    std::vector<int> mapping(actual.size(), -1);
    double worst = 0.0;
    for (size_t actual_idx = 0; actual_idx < actual.size(); ++actual_idx) {
        const Vec3f &point = actual[actual_idx];
        double nearest = std::numeric_limits<double>::infinity();
        int nearest_idx = -1;
        for (size_t expected_idx = 0; expected_idx < expected.its.vertices.size(); ++expected_idx) {
            if (!used[expected_idx]) {
                const double error = (point - expected.its.vertices[expected_idx]).cast<double>().squaredNorm();
                if (error < nearest) {
                    nearest = error;
                    nearest_idx = static_cast<int>(expected_idx);
                }
            }
        }
        REQUIRE(nearest_idx >= 0);
        used[static_cast<size_t>(nearest_idx)] = true;
        mapping[actual_idx] = nearest_idx;
        worst = std::max(worst, std::sqrt(nearest));
    }
    if (source_for_actual != nullptr)
        *source_for_actual = std::move(mapping);
    return worst;
}

bool same_oriented_triangle(const Vec3i32 &actual, const Vec3i32 &expected)
{
    return (actual[0] == expected[0] && actual[1] == expected[1] && actual[2] == expected[2]) ||
           (actual[1] == expected[0] && actual[2] == expected[1] && actual[0] == expected[2]) ||
           (actual[2] == expected[0] && actual[0] == expected[1] && actual[1] == expected[2]);
}

void check_topology(const DecodedDrcMesh &actual, const TriangleMesh &expected,
                    const std::vector<int> &source_for_actual)
{
    REQUIRE(actual.faces.size() == expected.its.indices.size());
    REQUIRE(source_for_actual.size() == expected.its.vertices.size());
    std::vector<bool> used_expected_faces(expected.its.indices.size(), false);
    for (const Vec3i32 &decoded_face : actual.faces) {
        Vec3i32 source_face;
        for (int corner = 0; corner < 3; ++corner) {
            REQUIRE(decoded_face[corner] >= 0);
            REQUIRE(static_cast<size_t>(decoded_face[corner]) < source_for_actual.size());
            source_face[corner] = source_for_actual[static_cast<size_t>(decoded_face[corner])];
        }
        REQUIRE(source_face[0] != source_face[1]);
        REQUIRE(source_face[1] != source_face[2]);
        REQUIRE(source_face[0] != source_face[2]);
        bool found = false;
        for (size_t expected_idx = 0; expected_idx < expected.its.indices.size(); ++expected_idx) {
            if (!used_expected_faces[expected_idx] &&
                same_oriented_triangle(source_face, expected.its.indices[expected_idx])) {
                used_expected_faces[expected_idx] = true;
                found = true;
                break;
            }
        }
        CHECK(found);
    }
    for (const bool used : used_expected_faces)
        CHECK(used);
}

double quantization_step_bound(const TriangleMesh &mesh, int bits)
{
    const double max_range = mesh.bounding_box().size().maxCoeff();
    // Budget float32 dequantization/rounding in addition to the position grid.
    const double float_budget = 32.0 * std::numeric_limits<float>::epsilon() * std::max(1.0, max_range);
    if (bits <= 0)
        return float_budget;
    const double levels = static_cast<double>((uint64_t(1) << bits) - 1);
    // Conservative Euclidean bound: one full scalar grid step per axis.
    return float_budget + std::sqrt(3.0) * max_range / levels;
}

void check_bounds_close(const BoundingBoxf3 &actual, const BoundingBoxf3 &expected, double tolerance)
{
    REQUIRE(actual.defined);
    REQUIRE(expected.defined);
    for (int axis = 0; axis < 3; ++axis) {
        CHECK_THAT(actual.min(axis), Catch::Matchers::WithinAbs(expected.min(axis), tolerance));
        CHECK_THAT(actual.max(axis), Catch::Matchers::WithinAbs(expected.max(axis), tolerance));
    }
}

} // namespace

TEST_CASE("DRC export preserves topology and honors position quantization", "[DRC][DRCExport]")
{
    TriangleMesh source = drc_asymmetric_tetrahedron();
    REQUIRE(source.its.vertices.size() == 4);
    REQUIRE(source.its.indices.size() == 4);

    for (const int bits : { DRC_BITS_DEFAULT, DRC_BITS_MIN, DRC_BITS_MAX }) {
        ScopedTemporaryFile drc(".drc");
        REQUIRE(store_drc(drc.string().c_str(), &source, bits));

        const DecodedDrcMesh decoded = decode_drc_export(drc.string());
        CHECK(decoded.faces.size() == source.its.indices.size());
        CHECK(decoded.vertices.size() == source.its.vertices.size());
        check_bounds_close(decoded.bounds, source.bounding_box(), quantization_step_bound(source, bits));

        std::vector<int> source_for_decoded;
        const double error = nearest_vertex_error(decoded.vertices, source, &source_for_decoded);
        INFO("position quantization bits=" << bits << ", max nearest-vertex error=" << error);
        CHECK(std::isfinite(error));
        CHECK(error <= quantization_step_bound(source, bits));
        check_topology(decoded, source, source_for_decoded);
        if (bits == DRC_BITS_MIN)
            CHECK(error > 0.001);
        else
            CHECK(error < 0.0001);
    }
}

TEST_CASE("DRC model exports preserve public object transformations", "[DRC][DRCExport]")
{
    const TriangleMesh source = drc_asymmetric_tetrahedron();
    Model model;
    ModelObject *object = model.add_object();
    REQUIRE(object != nullptr);
    object->add_volume(source, false);
    REQUIRE(object->volumes.size() == 1);
    object->add_instance();
    REQUIRE(object->instances.size() == 1);
    object->volumes.front()->set_offset(Vec3d(5.0, -2.0, 1.0));
    object->instances.front()->set_offset(Vec3d(2.0, 3.0, 4.0));

    std::vector<Vec3f> expected_vertices {
        { 0.137f + 7.0f, -0.271f + 1.0f,  0.413f + 5.0f },
        { 3.771f + 7.0f,  0.239f + 1.0f,  0.593f + 5.0f },
        { 0.617f + 7.0f,  4.113f + 1.0f,  0.827f + 5.0f },
        { 1.341f + 7.0f,  0.871f + 1.0f,  2.937f + 5.0f },
    };
    std::vector<Vec3i32> expected_faces {
        { 0, 1, 2 }, { 0, 3, 1 }, { 1, 3, 2 }, { 2, 3, 0 }
    };
    TriangleMesh expected(std::move(expected_vertices), std::move(expected_faces));
    REQUIRE(expected.facets_count() == source.facets_count());
    REQUIRE(expected.bounding_box().defined);

    ScopedTemporaryFile object_file(".drc");
    ScopedTemporaryFile model_file(".drc");
    REQUIRE(store_drc(object_file.string().c_str(), object, DRC_BITS_MAX));
    REQUIRE(store_drc(model_file.string().c_str(), &model, DRC_BITS_MAX));

    const DecodedDrcMesh object_export = decode_drc_export(object_file.string());
    const DecodedDrcMesh model_export = decode_drc_export(model_file.string());
    CHECK(object_export.faces.size() == expected.its.indices.size());
    CHECK(model_export.faces.size() == expected.its.indices.size());
    CHECK(object_export.vertices.size() == expected.its.vertices.size());
    CHECK(model_export.vertices.size() == expected.its.vertices.size());
    check_bounds_close(object_export.bounds, expected.bounding_box(), quantization_step_bound(expected, DRC_BITS_MAX));
    check_bounds_close(model_export.bounds, expected.bounding_box(), quantization_step_bound(expected, DRC_BITS_MAX));
    std::vector<int> object_mapping;
    std::vector<int> model_mapping;
    CHECK(nearest_vertex_error(object_export.vertices, expected, &object_mapping) < 0.0001);
    CHECK(nearest_vertex_error(model_export.vertices, expected, &model_mapping) < 0.0001);
    check_topology(object_export, expected, object_mapping);
    check_topology(model_export, expected, model_mapping);
}
