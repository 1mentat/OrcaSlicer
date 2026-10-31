#include <catch2/catch_all.hpp>
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

#include <cstddef>
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

// NORMAL is added before POSITION, so a loader that read the first attribute
// instead of looking POSITION up by name would decode normals as vertices.
void write_drc_mesh(const std::string &path, const std::vector<Vec3f> &vertices,
                    const std::vector<Vec3i32> &faces, bool include_normals)
{
    draco::Mesh mesh;
    mesh.set_num_points(vertices.size());

    if (include_normals) {
        draco::GeometryAttribute normal;
        normal.Init(draco::GeometryAttribute::NORMAL, nullptr, 3, draco::DT_FLOAT32, false, sizeof(float) * 3, 0);
        const int normal_id = mesh.AddAttribute(normal, true, vertices.size());
        mesh.attribute(normal_id)->Resize(vertices.size());
        for (size_t i = 0; i < vertices.size(); ++i) {
            const float normal_value[3] = {0.f, 0.f, 1.f};
            mesh.attribute(normal_id)->SetAttributeValue(draco::AttributeValueIndex(i), normal_value);
        }
    }

    draco::GeometryAttribute position;
    position.Init(draco::GeometryAttribute::POSITION, nullptr, 3, draco::DT_FLOAT32, false, sizeof(float) * 3, 0);
    const int position_id = mesh.AddAttribute(position, true, vertices.size());
    mesh.attribute(position_id)->Resize(vertices.size());
    for (size_t i = 0; i < vertices.size(); ++i) {
        const float xyz[3] = {vertices[i].x(), vertices[i].y(), vertices[i].z()};
        mesh.attribute(position_id)->SetAttributeValue(draco::AttributeValueIndex(i), xyz);
    }

    mesh.SetNumFaces(faces.size());
    for (size_t i = 0; i < faces.size(); ++i) {
        draco::Mesh::Face face;
        face[0] = draco::PointIndex(faces[i][0]);
        face[1] = draco::PointIndex(faces[i][1]);
        face[2] = draco::PointIndex(faces[i][2]);
        mesh.SetFace(draco::FaceIndex(i), face);
    }

    draco::Encoder encoder;
    encoder.SetEncodingMethod(draco::MESH_SEQUENTIAL_ENCODING);
    encoder.SetAttributeQuantization(draco::GeometryAttribute::POSITION, DRC_BITS_MAX);
    draco::EncoderBuffer buffer;
    REQUIRE(encoder.EncodeMeshToBuffer(mesh, &buffer).ok());
    boost::nowide::ofstream out(path, std::ios::binary);
    REQUIRE(out.good());
    out.write(buffer.data(), static_cast<std::streamsize>(buffer.size()));
    REQUIRE(out.good());
}

} // namespace

TEST_CASE("DRC import preserves volume, bounds, indices, and position with normal attributes", "[DRC][DRCImport]")
{
    const std::vector<Vec3f> vertices {
        {0.f, 0.f, 0.f}, {2.f, 0.f, 0.f}, {0.f, 3.f, 0.f}, {0.f, 0.f, 4.f}
    };
    // Deliberately reverse every outward face; load_drc must normalize the public winding.
    const std::vector<Vec3i32> faces {
        {0, 1, 2}, {0, 3, 1}, {0, 2, 3}, {1, 3, 2}
    };
    ScopedTemporaryFile drc(".drc");
    write_drc_mesh(drc.string(), vertices, faces, true);

    TriangleMesh mesh;
    REQUIRE(load_drc(drc.string().c_str(), &mesh));
    REQUIRE(mesh.facets_count() == faces.size());
    REQUIRE(mesh.its.vertices.size() == vertices.size());
    CHECK_THAT(mesh.volume(), Catch::Matchers::WithinRel(4.f, 1e-5f));
    const BoundingBoxf3 bounds = mesh.bounding_box();
    REQUIRE(bounds.defined);
    CHECK_THAT(bounds.min.x(), Catch::Matchers::WithinAbs(0.f, 1e-5f));
    CHECK_THAT(bounds.min.y(), Catch::Matchers::WithinAbs(0.f, 1e-5f));
    CHECK_THAT(bounds.min.z(), Catch::Matchers::WithinAbs(0.f, 1e-5f));
    CHECK_THAT(bounds.max.x(), Catch::Matchers::WithinAbs(2.f, 1e-5f));
    CHECK_THAT(bounds.max.y(), Catch::Matchers::WithinAbs(3.f, 1e-5f));
    CHECK_THAT(bounds.max.z(), Catch::Matchers::WithinAbs(4.f, 1e-5f));
    for (const stl_triangle_vertex_indices &face : mesh.its.indices) {
        CHECK(face[0] >= 0);
        CHECK(face[1] >= 0);
        CHECK(face[2] >= 0);
        CHECK(static_cast<size_t>(face[0]) < vertices.size());
        CHECK(static_cast<size_t>(face[1]) < vertices.size());
        CHECK(static_cast<size_t>(face[2]) < vertices.size());
        CHECK(face[0] != face[1]);
        CHECK(face[1] != face[2]);
        CHECK(face[0] != face[2]);
    }
}

TEST_CASE("DRC model import records the explicit name and public file dispatch", "[DRC][DRCImport]")
{
    const std::vector<Vec3f> vertices {
        {0.f, 0.f, 0.f}, {2.f, 0.f, 0.f}, {0.f, 3.f, 0.f}, {0.f, 0.f, 4.f}
    };
    const std::vector<Vec3i32> faces {
        {0, 2, 1}, {0, 1, 3}, {0, 3, 2}, {1, 2, 3}
    };
    ScopedTemporaryFile drc(".drc");
    write_drc_mesh(drc.string(), vertices, faces, false);

    Model named_model;
    REQUIRE(load_drc(drc.string().c_str(), &named_model, "named-drc-object"));
    REQUIRE(named_model.objects.size() == 1);
    REQUIRE(named_model.objects.front() != nullptr);
    CHECK(named_model.objects.front()->name == "named-drc-object");
    CHECK(named_model.objects.front()->input_file == drc.string());
    CHECK(named_model.objects.front()->raw_mesh().facets_count() == faces.size());

    Model routed_model;
    REQUIRE_NOTHROW(routed_model = Model::read_from_file(drc.string()));
    REQUIRE(routed_model.objects.size() == 1);
    REQUIRE(routed_model.objects.front() != nullptr);
    CHECK(routed_model.objects.front()->name == drc.path().filename().string());
    CHECK(routed_model.objects.front()->input_file == drc.string());
    CHECK(routed_model.objects.front()->instances.size() == 1);
    CHECK_THAT(routed_model.objects.front()->raw_mesh().volume(), Catch::Matchers::WithinRel(4.f, 1e-5f));
}

TEST_CASE("DRC import rejects malformed bytes without changing the destination mesh", "[DRC][DRCImport][Regression]")
{
    const std::vector<Vec3f> vertices {
        {0.f, 0.f, 0.f}, {2.f, 0.f, 0.f}, {0.f, 3.f, 0.f}, {0.f, 0.f, 4.f}
    };
    const std::vector<Vec3i32> faces {
        {0, 2, 1}, {0, 1, 3}, {0, 3, 2}, {1, 2, 3}
    };
    ScopedTemporaryFile valid(".drc");
    write_drc_mesh(valid.string(), vertices, faces, false);
    TriangleMesh mesh;
    REQUIRE(load_drc(valid.string().c_str(), &mesh));
    const size_t original_facets = mesh.facets_count();
    const float original_volume = mesh.volume();

    ScopedTemporaryFile malformed(".drc");
    boost::nowide::ofstream out(malformed.string(), std::ios::binary);
    REQUIRE(out.good());
    out << "not a Draco mesh";
    out.close();

    CHECK_FALSE(load_drc(malformed.string().c_str(), &mesh));
    CHECK(mesh.facets_count() == original_facets);
    CHECK_THAT(mesh.volume(), Catch::Matchers::WithinAbs(original_volume, 1e-5f));
}
