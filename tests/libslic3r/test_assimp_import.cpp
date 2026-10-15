#include <catch2/catch_message.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include "libslic3r/Point.hpp"
#include <cstddef>
#include <catch2/catch_all.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "libslic3r/Format/AssimpImport.hpp"
#include "libslic3r/TexturePainting.hpp"
#include "test_utils.hpp"

#include <boost/nowide/fstream.hpp>

#include <algorithm>
#include <array>
#include <cstdint>
#include <fstream>
#include <ios>
#include <cmath>
#include <set>
#include <string>
#include <utility>
#include <vector>

using namespace Slic3r;
using Catch::Matchers::WithinAbs;

namespace {
void write_obj(ScopedTemporaryFile &obj_file,
               ScopedTemporaryFile &mtl_file,
               const std::string &body,
               const std::string &materials = {})
{
    {
        boost::nowide::ofstream mtl(mtl_file.string());
        mtl << (materials.empty() ? "newmtl plain\nKd 1 1 1\n" : materials);
    }
    {
        boost::nowide::ofstream obj(obj_file.string());
        obj << "mtllib " << mtl_file.path().filename().string() << "\n" << body;
    }
}

} // namespace

TEST_CASE("Assimp imports a material-colored triangle", "[AssimpImport]")
{
    ScopedTemporaryFile obj(".obj"), mtl(".mtl");
    write_obj(obj, mtl,
        "o triangle\n"
        "usemtl red\n"
        "v 0 0 0\n"
        "v 10 0 0\n"
        "v 0 10 0\n"
        "f 1 2 3\n",
        "newmtl red\nKd 0.25 0.5 0.75\n");

    TexturedMesh mesh;
    std::string error;
    REQUIRE(load_assimp_textured_model(obj.string(), mesh, &error));
    REQUIRE(error.empty());
    REQUIRE(mesh.vertices.size() == 3);
    REQUIRE(mesh.indices.size() == 1);
    REQUIRE(mesh.material_ids.size() == 1);
    REQUIRE(mesh.material_colors.size() >= 1);
    CHECK(mesh.material_ids.front() >= 0);
    CHECK(static_cast<size_t>(mesh.material_ids.front()) < mesh.material_colors.size());
    const auto &color = mesh.material_colors.at(static_cast<size_t>(mesh.material_ids.front()));
    CHECK_THAT(color[0], WithinAbs(.25f, .02f));
    CHECK_THAT(color[1], WithinAbs(.5f, .02f));
    CHECK_THAT(color[2], WithinAbs(.75f, .02f));
    CHECK(mesh.textures.empty());
    REQUIRE(mesh.material_texture_map.size() == mesh.material_colors.size());
    CHECK(std::all_of(mesh.material_texture_map.begin(), mesh.material_texture_map.end(), [](int value) { return value == -1; }));
}

TEST_CASE("Assimp triangulates a textured quad and keeps UVs", "[AssimpImport]")
{
    ScopedTemporaryFile obj(".obj"), mtl(".mtl");
    write_obj(obj, mtl,
        "v 0 0 0\n"
        "v 10 0 0\n"
        "v 10 10 0\n"
        "v 0 10 0\n"
        "vt 0 0\nvt 1 0\nvt 1 1\nvt 0 1\n"
        "f 1/1 2/2 3/3 4/4\n");

    TexturedMesh mesh;
    REQUIRE(load_assimp_textured_model(obj.string(), mesh));
    REQUIRE(mesh.vertices.size() == 4);
    REQUIRE(mesh.indices.size() == 2);
    REQUIRE(mesh.uvs.size() == 4);
    for (const auto &index : mesh.indices)
        for (int vertex : index) {
            CHECK(vertex >= 0);
            CHECK(static_cast<size_t>(vertex) < mesh.vertices.size());
        }
    for (size_t index = 0; index < mesh.vertices.size(); ++index) {
        CHECK_THAT(mesh.uvs[index][0], WithinAbs(mesh.vertices[index][0] / 10.f, 1e-6));
        CHECK_THAT(mesh.uvs[index][1], WithinAbs(mesh.vertices[index][1] / 10.f, 1e-6));
    }
    double area = 0.;
    for (const auto &face : mesh.indices) {
        const auto &a = mesh.vertices[face[0]];
        const auto &b = mesh.vertices[face[1]];
        const auto &c = mesh.vertices[face[2]];
        area += std::abs((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])) / 2.;
    }
    CHECK_THAT(area, WithinAbs(100., 1e-6));
}

TEST_CASE("Assimp keeps material partitions across objects", "[AssimpImport]")
{
    ScopedTemporaryFile obj(".obj"), mtl(".mtl");
    write_obj(obj, mtl,
        "o first\nusemtl red\n"
        "v 0 0 0\nv 10 0 0\nv 0 10 0\nf 1 2 3\n"
        "o second\nusemtl blue\n"
        "v 20 0 0\nv 30 0 0\nv 20 10 0\nf 4 5 6\n",
        "newmtl red\nKd 1 0 0\nnewmtl blue\nKd 0 0 1\n");

    TexturedMesh mesh;
    REQUIRE(load_assimp_textured_model(obj.string(), mesh));
    REQUIRE(mesh.indices.size() == 2);
    REQUIRE(mesh.material_ids.size() == 2);
    CHECK(std::set<int>(mesh.material_ids.begin(), mesh.material_ids.end()).size() == 2);
    REQUIRE(mesh.material_colors.size() >= 2);
    CHECK(std::any_of(mesh.material_colors.begin(), mesh.material_colors.end(), [](const auto &value) {
        return value[0] > .9f && value[1] < .02f && value[2] < .02f;
    }));
    CHECK(std::any_of(mesh.material_colors.begin(), mesh.material_colors.end(), [](const auto &value) {
        return value[2] > .9f && value[0] < .02f && value[1] < .02f;
    }));
}

TEST_CASE("Assimp reports a missing external texture without losing geometry", "[AssimpImport]")
{
    ScopedTemporaryFile obj(".obj"), mtl(".mtl");
    write_obj(obj, mtl,
        "usemtl textured\n"
        "v 0 0 0\nv 10 0 0\nv 0 10 0\nf 1 2 3\n",
        "newmtl textured\nKd 0.2 0.3 0.4\nmap_Kd absent-texture.png\n");

    TexturedMesh mesh;
    std::string error;
    REQUIRE(load_assimp_textured_model(obj.string(), mesh, &error));
    CHECK(error.empty());
    REQUIRE(mesh.indices.size() == 1);
    REQUIRE(mesh.material_texture_map.size() == mesh.material_colors.size());
    CHECK(std::all_of(mesh.material_texture_map.begin(), mesh.material_texture_map.end(), [](int value) { return value == -1; }));
    CHECK(mesh.textures.empty());
}

TEST_CASE("Assimp clears output and explains an invalid scene", "[AssimpImport]")
{
    ScopedTemporaryFile invalid(".obj");
    {
        boost::nowide::ofstream out(invalid.string());
        out << "this is not an OBJ scene\n";
    }
    TexturedMesh mesh;
    mesh.vertices.push_back({1.f, 2.f, 3.f});
    mesh.indices.push_back({0, 0, 0});
    std::string error;
    CHECK_FALSE(load_assimp_textured_model(invalid.string(), mesh, &error));
    CHECK(mesh.vertices.empty());
    CHECK(mesh.indices.empty());
    CHECK_FALSE(error.empty());
}

TEST_CASE("Assimp rejects a scene with vertices but no faces", "[AssimpImport]")
{
    ScopedTemporaryFile obj(".obj"), mtl(".mtl");
    write_obj(obj, mtl, "v 0 0 0\nv 1 0 0\nv 0 1 0\n");
    TexturedMesh mesh;
    std::string error;
    CHECK_FALSE(load_assimp_textured_model(obj.string(), mesh, &error));
    CHECK(mesh.vertices.empty());
    CHECK(mesh.indices.empty());
    CHECK_FALSE(error.empty());
}

namespace {
const std::vector<unsigned char> tiny_texture_png = { 137, 80, 78, 71, 13, 10, 26, 10, 0, 0, 0, 13, 73, 72, 68, 82, 0, 0, 0, 1, 0, 0, 0, 1, 8, 6, 0, 0, 0, 31, 21, 196, 137, 0, 0, 0, 13, 73, 68, 65, 84, 120, 156, 99, 248, 207, 192, 240, 31, 0, 5, 0, 1, 255, 137, 153, 61, 29, 0, 0, 0, 0, 73, 69, 78, 68, 174, 66, 96, 130 };

std::string embedded_gltf()
{
    return R"fixture({"asset":{"version":"2.0"},"buffers":[{"uri":"data:application/octet-stream;base64,AAAAAAAAAAAAAAAAAAAgQQAAAAAAAAAAAAAAAAAAIEEAAAAAAAAAAAAAAAAAAIA/AAAAAAAAAAAAAIA/AAABAAIA","byteLength":66}],"bufferViews":[{"buffer":0,"byteOffset":0,"byteLength":36},{"buffer":0,"byteOffset":36,"byteLength":24},{"buffer":0,"byteOffset":60,"byteLength":6}],"accessors":[{"bufferView":0,"componentType":5126,"count":3,"type":"VEC3","min":[0,0,0],"max":[10,10,0]},{"bufferView":1,"componentType":5126,"count":3,"type":"VEC2"},{"bufferView":2,"componentType":5123,"count":3,"type":"SCALAR"}],"images":[{"uri":"data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAYAAAAfFcSJAAAADUlEQVR4nGP4z8DwHwAFAAH/iZk9HQAAAABJRU5ErkJggg=="}],"textures":[{"source":0}],"materials":[{"pbrMetallicRoughness":{"baseColorTexture":{"index":0},"baseColorFactor":[0.2,0.3,0.4,0.5]}}],"meshes":[{"primitives":[{"attributes":{"POSITION":0,"TEXCOORD_0":1},"indices":2,"material":0}]}],"nodes":[{"mesh":0,"translation":[5,7,0]}],"scenes":[{"nodes":[0]}],"scene":0})fixture";
}

void append_u32(std::string &bytes, uint32_t value)
{
    for (unsigned shift = 0; shift < 32; shift += 8)
        bytes.push_back(static_cast<char>((value >> shift) & 0xff));
}

void check_imported_texture(const TexturedMesh &mesh)
{
    REQUIRE(mesh.textures.size() == 1);
    CHECK(mesh.textures.front().data == tiny_texture_png);
    CHECK(mesh.textures.front().width == -1);
    CHECK(mesh.textures.front().height == -1);
    REQUIRE(mesh.material_ids.size() == 1);
    const int material = mesh.material_ids.front();
    REQUIRE(material >= 0);
    REQUIRE(static_cast<size_t>(material) < mesh.material_texture_map.size());
    CHECK(mesh.material_texture_map[material] == 0);
}
} // namespace

TEST_CASE("Assimp loads external texture bytes and preserves material assignment", "[AssimpImport]")
{
    ScopedTemporaryFile obj(".obj"), mtl(".mtl"), texture(".png");
    {
        std::ofstream stream(texture.string(), std::ios::binary);
        stream.write(reinterpret_cast<const char *>(tiny_texture_png.data()),
                     static_cast<std::streamsize>(tiny_texture_png.size()));
        REQUIRE(stream.good());
    }
    write_obj(obj, mtl, "usemtl textured\nv 0 0 0\nv 10 0 0\nv 0 10 0\nf 1 2 3\n",
              "newmtl textured\nKd 1 0 0\nmap_Kd " + texture.path().filename().string() + "\n");
    TexturedMesh mesh;
    REQUIRE(load_assimp_textured_model(obj.string(), mesh));
    check_imported_texture(mesh);
}

TEST_CASE("Assimp decodes an embedded GLTF texture and bakes scene transforms", "[AssimpImport]")
{
    ScopedTemporaryFile scene(".gltf");
    {
        std::ofstream stream(scene.string());
        stream << embedded_gltf();
        REQUIRE(stream.good());
    }
    TexturedMesh mesh;
    std::string error;
    INFO(error);
    REQUIRE(load_assimp_textured_model(scene.string(), mesh, &error));
    check_imported_texture(mesh);
    REQUIRE(mesh.vertices.size() == 3);
    REQUIRE(mesh.indices.size() == 1);
    std::set<std::pair<float, float>> positions;
    for (const auto &vertex : mesh.vertices) {
        positions.emplace(vertex[0], vertex[1]);
        CHECK_THAT(vertex[2], WithinAbs(0., 1e-6));
    }
    CHECK(positions == std::set<std::pair<float, float>>{{5.f, 7.f}, {15.f, 7.f}, {5.f, 17.f}});
    REQUIRE(mesh.uvs.size() == mesh.vertices.size());
    for (size_t index = 0; index < mesh.vertices.size(); ++index) {
        CHECK_THAT(mesh.uvs[index][0], WithinAbs((mesh.vertices[index][0] - 5.f) / 10.f, 1e-6));
        CHECK_THAT(mesh.uvs[index][1], WithinAbs(1.f - (mesh.vertices[index][1] - 7.f) / 10.f, 1e-6));
    }
    const auto &color = mesh.material_colors.at(static_cast<size_t>(mesh.material_ids.front()));
    CHECK_THAT(color[0], WithinAbs(.2, 1e-5));
    CHECK_THAT(color[3], WithinAbs(.5, 1e-5));
}

TEST_CASE("Assimp applies the GLB texture-coordinate convention", "[AssimpImport]")
{
    std::string json = embedded_gltf();
    const std::string inline_buffer = R"fixture("uri":"data:application/octet-stream;base64,AAAAAAAAAAAAAAAAAAAgQQAAAAAAAAAAAAAAAAAAIEEAAAAAAAAAAAAAAAAAAIA/AAAAAAAAAAAAAIA/AAABAAIA",)fixture";
    const size_t uri = json.find(inline_buffer);
    REQUIRE(uri != std::string::npos);
    json.erase(uri, inline_buffer.size());
    const std::vector<unsigned char> geometry = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 32, 65, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 32, 65, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 128, 63, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 128, 63, 0, 0, 1, 0, 2, 0 };
    std::string binary(reinterpret_cast<const char *>(geometry.data()), geometry.size());
    while (binary.size() % 4 != 0)
        binary.push_back('\0');
    while (json.size() % 4 != 0)
        json.push_back(' ');
    std::string bytes = "glTF";
    append_u32(bytes, 2);
    append_u32(bytes, static_cast<uint32_t>(28 + json.size() + binary.size()));
    append_u32(bytes, static_cast<uint32_t>(json.size()));
    append_u32(bytes, 0x4e4f534a);
    bytes += json;
    append_u32(bytes, static_cast<uint32_t>(binary.size()));
    append_u32(bytes, 0x004e4942);
    bytes += binary;
    ScopedTemporaryFile scene(".glb");
    {
        std::ofstream stream(scene.string(), std::ios::binary);
        stream.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
        REQUIRE(stream.good());
    }
    TexturedMesh mesh;
    REQUIRE(load_assimp_textured_model(scene.string(), mesh));
    check_imported_texture(mesh);
    REQUIRE(mesh.uvs.size() == mesh.vertices.size());
    REQUIRE(mesh.uvs.size() == 3);
    // Assimp's GLTF reader inverts V during import. Orca applies FlipUVs to
    // GLB, returning these coordinates to the texture's original convention.
    for (size_t index = 0; index < mesh.vertices.size(); ++index) {
        CHECK_THAT(mesh.uvs[index][0], WithinAbs((mesh.vertices[index][0] - 5.f) / 10.f, 1e-6));
        CHECK_THAT(mesh.uvs[index][1], WithinAbs((mesh.vertices[index][1] - 7.f) / 10.f, 1e-6));
    }
}
