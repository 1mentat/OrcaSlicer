#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <ios>
#include <iterator>
#include <limits>
#include <map>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <boost/filesystem/path.hpp>
#include <boost/nowide/fstream.hpp>

#include <catch2/catch_all.hpp>
#include <catch2/catch_message.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/Format/OBJ.hpp"
#include "libslic3r/Format/objparser.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/TexturePainting.hpp"
#include "libslic3r/TriangleMesh.hpp"

#include "test_utils.hpp"

using namespace Slic3r;
using Catch::Matchers::WithinAbs;

namespace {

struct LoadedObj
{
    bool         ok{false};
    TriangleMesh mesh;
    ObjInfo      info;
    std::string  message;
};

// Loads an OBJ made of the given lines, with a material library that defines material "a".
LoadedObj load_textured_obj(const std::string &body)
{
    ScopedTemporaryFile obj(".obj");
    ScopedTemporaryFile mtl(".mtl");
    {
        boost::nowide::ofstream out(mtl.string());
        out << "newmtl a\nKd 1 0 0\n";
    }
    {
        boost::nowide::ofstream out(obj.string());
        out << "mtllib " << mtl.path().filename().string() << "\n" << body;
    }
    LoadedObj   loaded;
    loaded.ok = load_obj(obj.string().c_str(), &loaded.mesh, loaded.info, loaded.message);
    return loaded;
}

// A tetrahedron with a material and two texture coordinates, (0.25, 0.5) and (0.75, 1).
// Only the first face and the vt lines are varied; the other three faces reference vt 1.
LoadedObj load_textured_tetrahedron(const std::string &first_face, const std::string &vts = "vt 0.25 0.5\nvt 0.75 1\n")
{
    return load_textured_obj("v 0 0 0\nv 10 0 0\nv 0 10 0\nv 0 0 10\n" + vts + "usemtl a\n" + first_face + "\n" +
                             "f 1/1 2/1 4/1\nf 1/1 4/1 3/1\nf 2/1 3/1 4/1\n");
}

// Texture coordinate n is (n / 10, n / 20), so a UV identifies the vt it came from.
void check_uv_is_vt(const Vec2f &uv, int vt)
{
    CHECK_THAT(uv.x(), WithinAbs(vt / 10., 1e-6));
    CHECK_THAT(uv.y(), WithinAbs(vt / 20., 1e-6));
}

} // namespace

TEST_CASE("An out-of-range texture index falls back to a zero UV and keeps the geometry", "[OBJ][Regression]")
{
    const LoadedObj loaded = load_textured_tetrahedron("f 1/1000000000 3/1 2/1");

    REQUIRE(loaded.ok);
    CHECK(loaded.mesh.facets_count() == 4);
    REQUIRE(loaded.info.uvs.size() == 4);
    const std::array<Vec2f, 3> &uv = loaded.info.uvs.front();
    CHECK_THAT(uv[0].x(), WithinAbs(0., 1e-6));
    CHECK_THAT(uv[0].y(), WithinAbs(0., 1e-6));
    CHECK_THAT(uv[1].x(), WithinAbs(0.25, 1e-6));
    CHECK_THAT(uv[1].y(), WithinAbs(0.5, 1e-6));
}

TEST_CASE("A face without texture indices loads among faces that have them", "[OBJ][Regression]")
{
    const LoadedObj loaded = load_textured_tetrahedron("f 1 3 2");

    REQUIRE(loaded.ok);
    CHECK(loaded.mesh.facets_count() == 4);
    // One UV entry per face, so later faces keep their own coordinates.
    REQUIRE(loaded.info.uvs.size() == 4);
    for (const Vec2f &uv : loaded.info.uvs.front()) {
        CHECK_THAT(uv.x(), WithinAbs(0., 1e-6));
        CHECK_THAT(uv.y(), WithinAbs(0., 1e-6));
    }
    CHECK_THAT(loaded.info.uvs[1][0].x(), WithinAbs(0.25, 1e-6));
    CHECK_THAT(loaded.info.uvs[1][0].y(), WithinAbs(0.5, 1e-6));
}

TEST_CASE("A negative texture index counts back from the last texture coordinate", "[OBJ]")
{
    // -1 is the most recent vt (0.75, 1), -2 the one before it (0.25, 0.5).
    const LoadedObj loaded = load_textured_tetrahedron("f 1/-2 3/-1 2/-1");

    REQUIRE(loaded.ok);
    CHECK(loaded.mesh.facets_count() == 4);
    REQUIRE(loaded.info.uvs.size() == 4);
    const std::array<Vec2f, 3> &uv = loaded.info.uvs.front();
    CHECK_THAT(uv[0].x(), WithinAbs(0.25, 1e-6));
    CHECK_THAT(uv[0].y(), WithinAbs(0.5, 1e-6));
    CHECK_THAT(uv[1].x(), WithinAbs(0.75, 1e-6));
    CHECK_THAT(uv[1].y(), WithinAbs(1., 1e-6));
}

TEST_CASE("Texture coordinates with a w component are kept", "[OBJ][Regression]")
{
    const LoadedObj loaded = load_textured_tetrahedron("f 1/1 3/2 2/2", "vt 0.25 0.5 0\nvt 0.75 1 0\n");

    REQUIRE(loaded.ok);
    CHECK(loaded.mesh.facets_count() == 4);
    REQUIRE(loaded.info.uvs.size() == 4);
    const std::array<Vec2f, 3> &uv = loaded.info.uvs.front();
    CHECK_THAT(uv[0].x(), WithinAbs(0.25, 1e-6));
    CHECK_THAT(uv[0].y(), WithinAbs(0.5, 1e-6));
    CHECK_THAT(uv[1].x(), WithinAbs(0.75, 1e-6));
    CHECK_THAT(uv[1].y(), WithinAbs(1., 1e-6));
}

TEST_CASE("A texture coordinate with w does not shift the indices of the ones after it", "[OBJ][Regression]")
{
    // The w on the first vt used to drop that line, so vt 2 resolved to the third coordinate.
    const LoadedObj loaded = load_textured_tetrahedron("f 1/2 3/3 2/-1", "vt 0.1 0.2 0\nvt 0.25 0.5\nvt 0.75 1\n");

    REQUIRE(loaded.ok);
    REQUIRE(loaded.info.uvs.size() == 4);
    const std::array<Vec2f, 3> &uv = loaded.info.uvs.front();
    CHECK_THAT(uv[0].x(), WithinAbs(0.25, 1e-6));
    CHECK_THAT(uv[0].y(), WithinAbs(0.5, 1e-6));
    CHECK_THAT(uv[1].x(), WithinAbs(0.75, 1e-6));
    CHECK_THAT(uv[1].y(), WithinAbs(1., 1e-6));
    CHECK_THAT(uv[2].x(), WithinAbs(0.75, 1e-6));
    CHECK_THAT(uv[2].y(), WithinAbs(1., 1e-6));
}

TEST_CASE("Both triangles of a quad take the texture coordinates of their own corners", "[OBJ][Regression]")
{
    const LoadedObj loaded = load_textured_obj("v 0 0 0\nv 10 0 0\nv 10 10 0\nv 0 10 0\n"
                                               "vt 0.1 0.05\nvt 0.2 0.1\nvt 0.3 0.15\nvt 0.4 0.2\n"
                                               "usemtl a\n"
                                               "f 1/1 2/2 3/3 4/4\n");

    REQUIRE(loaded.ok);
    REQUIRE(loaded.mesh.facets_count() == 2);
    REQUIRE(loaded.info.uvs.size() == 2);
    check_uv_is_vt(loaded.info.uvs[0][0], 1);
    check_uv_is_vt(loaded.info.uvs[0][1], 2);
    check_uv_is_vt(loaded.info.uvs[0][2], 3);
    check_uv_is_vt(loaded.info.uvs[1][0], 1);
    check_uv_is_vt(loaded.info.uvs[1][1], 3);
    check_uv_is_vt(loaded.info.uvs[1][2], 4);
}

TEST_CASE("Texture coordinates follow the corners of a mesh that is flipped on load", "[OBJ][Regression]")
{
    // The faces wind inwards, so the loader flips them. Vertex n uses vt n.
    const LoadedObj loaded = load_textured_obj("v 0 0 0\nv 10 0 0\nv 0 10 0\nv 0 0 10\n"
                                               "vt 0.1 0.05\nvt 0.2 0.1\nvt 0.3 0.15\nvt 0.4 0.2\n"
                                               "usemtl a\n"
                                               "f 1/1 2/2 3/3\nf 1/1 4/4 2/2\nf 1/1 3/3 4/4\nf 2/2 4/4 3/3\n");

    REQUIRE(loaded.ok);
    const indexed_triangle_set &its = loaded.mesh.its;
    CHECK(its_volume(its) > 0.f);
    REQUIRE(its.indices.size() == 4);
    REQUIRE(loaded.info.uvs.size() == 4);
    for (size_t face = 0; face < its.indices.size(); ++face)
        for (int corner = 0; corner < 3; ++corner)
            check_uv_is_vt(loaded.info.uvs[face][corner], its.indices[face][corner] + 1);
}

namespace obj_parser_contracts {
using Catch::Matchers::WithinRel;
namespace {

void write_text(const ScopedTemporaryFile &file, const std::string &text)
{
    boost::nowide::ofstream output(file.string());
    output << text;
    REQUIRE(output.good());
}

ObjParser::ObjData parse_obj_stream(const std::string &text)
{
    ObjParser::ObjData data;
    std::istringstream input(text);
    REQUIRE(ObjParser::objparse(input, data));
    return data;
}

LoadedObj load_obj_text(const std::string &text)
{
    ScopedTemporaryFile file(".obj");
    write_text(file, text);
    LoadedObj loaded;
    loaded.ok = load_obj(file.string().c_str(), &loaded.mesh, loaded.info, loaded.message);
    return loaded;
}

std::string tetrahedron(bool vertex_colors = false)
{
    const std::string colors = vertex_colors ? " 1.2 -0.2 0.25 0.75" : "";
    return "v 0 0 0" + colors + "\n"
           "v 10 0 0" + colors + "\n"
           "v 0 10 0" + colors + "\n"
           "v 0 0 10" + colors + "\n"
           "f 1 2 3\n"
           "f 1 4 2\n"
           "f 1 3 4\n"
           "f 2 4 3\n";
}

} // namespace

TEST_CASE("OBJ parser records geometry, attributes, structure and materials", "[OBJParser][OBJ]")
{
    const ObjParser::ObjData data = parse_obj_stream(
        "# comments do not create records\n"
        "  mtllib ship.mtl\n"
        "o hull\n"
        "g shell\n"
        "s 7\n"
        "usemtl red\n"
        "v 0 0 0 1 0 0 0.25\n"
        "v 1 0 0 0 1 0 0.5\n"
        "v 0 1 0\n"
        "vt 0.1 0.2 0.4\n"
        "vn 0 0 1\n"
        "vp 0.5 0.25 0.75\n"
        "f 1/1/1 2/-1/-1 3//1\n");

    REQUIRE(data.coordinates.size() == 21);
    CHECK(data.has_vertex_color);
    CHECK_THAT(data.coordinates[3], WithinAbs(1., 1e-6));
    CHECK_THAT(data.coordinates[6], WithinAbs(.25, 1e-6));
    REQUIRE(data.textureCoordinates.size() == 2);
    CHECK_THAT(data.textureCoordinates[0], WithinAbs(.1, 1e-6));
    CHECK_THAT(data.textureCoordinates[1], WithinAbs(.2, 1e-6));
    REQUIRE(data.normals.size() == 3);
    CHECK_THAT(data.normals[2], WithinAbs(1., 1e-6));
    REQUIRE(data.parameters.size() == 3);
    CHECK_THAT(data.parameters[0], WithinAbs(.5, 1e-6));
    CHECK_THAT(data.parameters[2], WithinAbs(.75, 1e-6));
    REQUIRE(data.mtllibs.size() == 1);
    CHECK(data.mtllibs[0] == "ship.mtl");
    REQUIRE(data.objects.size() == 1);
    CHECK(data.objects[0].vertexIdxFirst == 0);
    REQUIRE(data.groups.size() == 1);
    CHECK(data.groups[0].vertexIdxFirst == 0);
    REQUIRE(data.smoothingGroups.size() == 1);
    CHECK(data.smoothingGroups[0].smoothingGroupID == 7);
    REQUIRE(data.usemtls.size() == 1);
    CHECK(data.usemtls[0].name == "red");
    CHECK(data.usemtls[0].face_start == 0);
    CHECK(data.usemtls[0].face_end == 0);
    REQUIRE(data.vertices.size() == 4);
    CHECK(data.vertices[0].coordIdx == 0);
    CHECK(data.vertices[1].coordIdx == 1);
    CHECK(data.vertices[1].textureCoordIdx == 0);
    CHECK(data.vertices[1].normalIdx == 0);
    CHECK(data.vertices[2].textureCoordIdx == -1);
    CHECK(data.vertices[3].coordIdx == -1);
}

TEST_CASE("OBJ parser resolves positive, negative and omitted face indices", "[OBJParser][OBJ]")
{
    const ObjParser::ObjData data = parse_obj_stream(
        "v 0 0 0\n v 1 0 0\n v 1 1 0\n"
        "vt 0 0\nvt 1 0\nvt 1 1\n"
        "vn 0 0 1\n"
        "f 1/1/1 2/2/1 3/3/1\n"
        "f -3/-3/-1 -2/-2/-1 -1/-1/-1\n"
        "f 1 2 3\n"
        "f 1//1 2//1 3//1\n");

    REQUIRE(data.vertices.size() == 16);
    CHECK(data.vertices[0] == ObjParser::ObjVertex{0, 0, 0});
    CHECK(data.vertices[1] == ObjParser::ObjVertex{1, 1, 0});
    CHECK(data.vertices[2] == ObjParser::ObjVertex{2, 2, 0});
    CHECK(data.vertices[4] == ObjParser::ObjVertex{0, 0, 0});
    CHECK(data.vertices[5] == ObjParser::ObjVertex{1, 1, 0});
    CHECK(data.vertices[6] == ObjParser::ObjVertex{2, 2, 0});
    CHECK(data.vertices[8] == ObjParser::ObjVertex{0, -1, -1});
    CHECK(data.vertices[12] == ObjParser::ObjVertex{0, -1, 0});
    CHECK(data.vertices[15] == ObjParser::ObjVertex{-1, -1, -1});
}

TEST_CASE("OBJ parser accepts CRLF, indentation and harmless unknown records", "[OBJParser]")
{
    const ObjParser::ObjData data = parse_obj_stream(
        "\t# header\r\n"
        "  vp 1 2\r\n"
        "  v 0 0 0\r\n"
        "  v 1 0 0\r\n"
        "  v 0 1 0\r\n"
        "  foo ignored extension\r\n"
        "  f 1 2 3\r\n");

    CHECK(data.parameters.size() == 3);
    CHECK(data.coordinates.size() == 21);
    REQUIRE(data.vertices.size() == 4);
    CHECK(data.vertices[0].coordIdx == 0);
    CHECK(data.vertices[2].coordIdx == 2);
}

TEST_CASE("OBJ MTL parser preserves ordered materials and extracts the texture filename past option flags", "[OBJParser][OBJ]")
{
    ScopedTemporaryFile mtl(".mtl");
    write_text(mtl,
        "# material comment\n"
        "newmtl coat\n"
        "Ns 12.5\nNi 1.45\nd 0.8\nTr 0.2\nillum 2\n"
        "Ka 0.1 0.2 0.3\nKd 0.4 0.5 0.6\nKs 0.7 0.8 0.9\nKe 0.05 0.06 0.07\nTf 0.2 0.3 0.4\n"
        "map_Kd -o 0.1 0.2 0.3 -s 1 2 3 -bm 0.7 coat.png\n"
        "newmtl plain\nKd 0.9 0.8 0.7\n");

    ObjParser::MtlData data;
    REQUIRE(ObjParser::mtlparse(mtl.string().c_str(), data));
    REQUIRE(data.mtl_orders.size() == 2);
    CHECK(data.mtl_orders[0] == "coat");
    CHECK(data.mtl_orders[1] == "plain");
    REQUIRE(data.new_mtl_unmap.count("coat") == 1);
    REQUIRE(data.new_mtl_unmap.at("coat") != nullptr);
    const auto &coat = *data.new_mtl_unmap.at("coat");
    CHECK_THAT(coat.Ns, WithinAbs(12.5, 1e-5));
    CHECK_THAT(coat.Ni, WithinAbs(1.45, 1e-5));
    CHECK_THAT(coat.d, WithinAbs(.8, 1e-5));
    CHECK_THAT(coat.Tr, WithinAbs(.2, 1e-5));
    CHECK_THAT(coat.illum, WithinAbs(2., 1e-5));
    CHECK_THAT(coat.Ka[1], WithinAbs(.2, 1e-5));
    CHECK_THAT(coat.Kd[2], WithinAbs(.6, 1e-5));
    CHECK_THAT(coat.Ks[0], WithinAbs(.7, 1e-5));
    CHECK_THAT(coat.Ke[2], WithinAbs(.07, 1e-5));
    CHECK_THAT(coat.Tf[0], WithinAbs(.2, 1e-5));
    // The -o, -s and -bm options are skipped, not stored; only the filename after them is kept.
    CHECK(coat.map_Kd == "coat.png");
}

TEST_CASE("OBJ parser reports missing input files", "[OBJParser]")
{
    ScopedTemporaryDir empty("orca_missing_obj");
    const auto obj = (empty.path() / "missing.obj").string();
    const auto material = (empty.path() / "missing.mtl").string();
    ObjParser::ObjData data;
    CHECK_FALSE(ObjParser::objparse(obj.c_str(), data));
    ObjParser::MtlData mtl;
    CHECK_FALSE(ObjParser::mtlparse(material.c_str(), mtl));
}

TEST_CASE("OBJ binary save writes the documented version header", "[OBJParser][OBJBinary]")
{
    ObjParser::ObjData data = parse_obj_stream(
        "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n");
    ScopedTemporaryFile cache(".objbin");
    REQUIRE(ObjParser::objbinsave(cache.string().c_str(), data));

    boost::nowide::ifstream input(cache.string(), std::ios::binary);
    REQUIRE(input.good());
    size_t version = 0;
    input.read(reinterpret_cast<char *>(&version), sizeof(version));
    REQUIRE(input.good());
    CHECK(version == 1);
    CHECK(input.peek() != std::ifstream::traits_type::eof());
}

TEST_CASE("OBJ binary loader rejects a file with an unsupported version", "[OBJParser][OBJBinary]")
{
    ScopedTemporaryFile cache(".objbin");
    boost::nowide::ofstream output(cache.string(), std::ios::binary);
    const size_t unsupported_version = 99;
    output.write(reinterpret_cast<const char *>(&unsupported_version), sizeof(unsupported_version));
    output.close();

    ObjParser::ObjData data;
    CHECK_FALSE(ObjParser::objbinload(cache.string().c_str(), data));
}

TEST_CASE("OBJ equality compares parsed semantic records", "[OBJParser]")
{
    const ObjParser::ObjData original = parse_obj_stream(
        "o model\ng group\ns 1\nv 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n");
    ObjParser::ObjData copy = original;
    CHECK(ObjParser::objequal(original, copy));
    copy.vertices.front().coordIdx = 2;
    CHECK_FALSE(ObjParser::objequal(original, copy));
    copy = original;
    copy.groups.front().name = "other";
    CHECK_FALSE(ObjParser::objequal(original, copy));
}

TEST_CASE("OBJ loader preserves analytic tetrahedron geometry and clamps vertex colors", "[OBJParser][OBJ]")
{
    LoadedObj loaded = load_obj_text(tetrahedron(true));
    REQUIRE(loaded.ok);
    REQUIRE(loaded.mesh.its.vertices.size() == 4);
    REQUIRE(loaded.mesh.its.indices.size() == 4);
    CHECK_THAT(loaded.mesh.volume(), WithinRel(1000. / 6., 1e-5));
    CHECK_THAT(loaded.mesh.bounding_box().size().x(), WithinAbs(10., 1e-5));
    CHECK_THAT(loaded.mesh.bounding_box().size().y(), WithinAbs(10., 1e-5));
    CHECK_THAT(loaded.mesh.bounding_box().size().z(), WithinAbs(10., 1e-5));
    REQUIRE(loaded.info.vertex_colors.size() == 4);
    CHECK_THAT(loaded.info.vertex_colors.front()[0], WithinAbs(1., 1e-6));
    CHECK_THAT(loaded.info.vertex_colors.front()[1], WithinAbs(0., 1e-6));
    CHECK_THAT(loaded.info.vertex_colors.front()[2], WithinAbs(.25, 1e-6));
    CHECK_THAT(loaded.info.vertex_colors.front()[3], WithinAbs(.75, 1e-6));
}

TEST_CASE("OBJ loader triangulates quads without overlapping boundary edges", "[OBJParser][OBJ]")
{
    LoadedObj loaded = load_obj_text(
        "v 0 0 0\nv 10 0 0\nv 10 10 0\nv 0 10 0\n"
        "f 1 2 3 4\n");
    REQUIRE(loaded.ok);
    REQUIRE(loaded.mesh.its.indices.size() == 2);
    std::map<std::pair<int, int>, int> edges;
    double area = 0.;
    for (const Vec3i32 &face : loaded.mesh.its.indices) {
        for (int edge = 0; edge < 3; ++edge) {
            const int a = face[edge], b = face[(edge + 1) % 3];
            REQUIRE(a >= 0);
            REQUIRE(a < 4);
            REQUIRE(b >= 0);
            REQUIRE(b < 4);
            ++edges[{std::min(a, b), std::max(a, b)}];
        }
        const auto &a = loaded.mesh.its.vertices[face[0]];
        const auto &b = loaded.mesh.its.vertices[face[1]];
        const auto &c = loaded.mesh.its.vertices[face[2]];
        area += 0.5 * (b - a).cross(c - a).norm();
    }
    CHECK(edges[std::make_pair(0, 1)] == 1);
    CHECK(edges[std::make_pair(1, 2)] == 1);
    CHECK(edges[std::make_pair(2, 3)] == 1);
    CHECK(edges[std::make_pair(0, 3)] == 1);
    CHECK(edges.size() == 5);
    CHECK_THAT(area, WithinAbs(100., 1e-6));
    CHECK_THAT(std::abs(loaded.mesh.volume()), WithinAbs(0., 1e-6));
    CHECK_THAT(loaded.mesh.bounding_box().size().x(), WithinAbs(10., 1e-5));
    CHECK_THAT(loaded.mesh.bounding_box().size().y(), WithinAbs(10., 1e-5));
}

TEST_CASE("OBJ loader rejects polygons outside the triangle and quad contract", "[OBJParser][OBJ]")
{
    const LoadedObj loaded = load_obj_text(
        "v 0 0 0\nv 1 0 0\nv 2 0 0\nv 2 1 0\nv 0 1 0\n"
        "f 1 2 3 4 5\n");
    CHECK_FALSE(loaded.ok);
    CHECK(loaded.message.find("more than 4") != std::string::npos);
}

TEST_CASE("OBJ loader rejects a face that references a missing position", "[OBJParser][OBJ]")
{
    const LoadedObj loaded = load_obj_text(
        "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 99\n");
    CHECK_FALSE(loaded.ok);
    CHECK(loaded.message.find("invalid vertex index") != std::string::npos);
}

TEST_CASE("OBJ loader rejects vertex-only and empty inputs", "[OBJParser][OBJ]")
{
    const LoadedObj vertex_only = load_obj_text("v 0 0 0\nv 1 0 0\nv 0 1 0\n");
    CHECK_FALSE(vertex_only.ok);
    const LoadedObj empty = load_obj_text("# no geometry\n\n");
    CHECK_FALSE(empty.ok);
}

TEST_CASE("OBJ model overload uses its caller-provided object name", "[OBJParser][OBJ]")
{
    ScopedTemporaryFile file(".obj");
    write_text(file, tetrahedron());
    Model model;
    ObjInfo info;
    std::string message;
    REQUIRE(load_obj(file.string().c_str(), &model, info, message, "contract-name"));
    REQUIRE(model.objects.size() == 1);
    REQUIRE(model.objects.front() != nullptr);
    CHECK(model.objects.front()->name == "contract-name");
    REQUIRE(model.objects.front()->volumes.size() == 1);
    CHECK(model.objects.front()->raw_mesh().facets_count() == 4);
}

TEST_CASE("OBJ export reload preserves mesh topology and bounds", "[OBJParser][OBJ]")
{
    LoadedObj initial = load_obj_text(tetrahedron());
    REQUIRE(initial.ok);
    ScopedTemporaryFile output(".obj");
    REQUIRE(store_obj(output.string().c_str(), &initial.mesh));

    ObjInfo info;
    std::string message;
    TriangleMesh reloaded;
    REQUIRE(load_obj(output.string().c_str(), &reloaded, info, message));
    REQUIRE(reloaded.its.vertices.size() == initial.mesh.its.vertices.size());
    REQUIRE(reloaded.its.indices.size() == initial.mesh.its.indices.size());
    CHECK_THAT(static_cast<double>(reloaded.volume()), WithinRel(static_cast<double>(initial.mesh.volume()), 1e-5));
    CHECK_THAT(reloaded.bounding_box().size().x(), WithinAbs(initial.mesh.bounding_box().size().x(), 1e-5));
    CHECK_THAT(reloaded.bounding_box().size().y(), WithinAbs(initial.mesh.bounding_box().size().y(), 1e-5));
    CHECK_THAT(reloaded.bounding_box().size().z(), WithinAbs(initial.mesh.bounding_box().size().z(), 1e-5));
    for (size_t i = 0; i < reloaded.its.indices.size(); ++i)
        CHECK(reloaded.its.indices[i] == initial.mesh.its.indices[i]);
}

TEST_CASE("OBJ textured import maps materials, texture bytes and flipped V coordinates", "[OBJParser][OBJ]")
{
    ScopedTemporaryFile obj(".obj");
    ScopedTemporaryFile mtl(".mtl");
    ScopedTemporaryFile texture(".png");
    write_text(texture, "not-an-image-but-a-stable-byte-payload");
    write_text(mtl, "newmtl painted\nKd 0.2 0.4 0.6\nTr 0.75\nmap_Kd " + texture.path().filename().string() + "\n");
    write_text(obj, "mtllib " + mtl.path().filename().string() + "\n"
                    "usemtl painted\n"
                    "v 0 0 0\nv 1 0 0\nv 0 1 0\n"
                    "vt 0.2 0.3\nvt 0.4 0.5\nvt 0.6 0.7\n"
                    "f 1/1 2/2 3/3\n");

    TriangleMesh mesh;
    ObjInfo info;
    ObjParser::MtlData mtl_data;
    std::string message;
    REQUIRE(load_obj(obj.string().c_str(), &mesh, info, message, &mtl_data));
    REQUIRE(info.has_uv_png);
    TexturedMesh textured;
    REQUIRE(obj_to_textured_mesh(info, mesh.its, mtl_data, obj.path().parent_path().string(), textured));
    REQUIRE(textured.textures.size() == 1);
    const auto &texture_bytes = textured.textures.front().data;
    CHECK(std::string(texture_bytes.begin(), texture_bytes.end()) == "not-an-image-but-a-stable-byte-payload");
    REQUIRE(textured.material_colors.size() == 1);
    CHECK_THAT(textured.material_colors[0][0], WithinAbs(.2, 1e-6));
    // Current behavior: the MTL Tr value is stored as the alpha channel unchanged (not as 1 - Tr).
    CHECK_THAT(textured.material_colors[0][3], WithinAbs(.75, 1e-6));
    REQUIRE(textured.material_texture_map.size() == 1);
    CHECK(textured.material_texture_map[0] == 0);
    REQUIRE(textured.material_ids.size() == 1);
    CHECK(textured.material_ids[0] == 0);
    REQUIRE(textured.uv_coords.size() == 3);
    CHECK_THAT(textured.uv_coords[0][0], WithinAbs(.2, 1e-6));
    CHECK_THAT(textured.uv_coords[0][1], WithinAbs(.7, 1e-6));
    CHECK_THAT(textured.uv_coords[2][1], WithinAbs(.3, 1e-6));
}

TEST_CASE("OBJ material lookup records an unknown material without dropping geometry", "[OBJParser][OBJ]")
{
    ScopedTemporaryFile obj(".obj");
    ScopedTemporaryFile mtl(".mtl");
    write_text(mtl, "newmtl known\nKd 1 0 0\n");
    write_text(obj, "mtllib " + mtl.path().filename().string() + "\nusemtl missing\n" + tetrahedron());
    TriangleMesh mesh;
    ObjInfo info;
    std::string message;
    REQUIRE(load_obj(obj.string().c_str(), &mesh, info, message));
    CHECK(mesh.facets_count() == 4);
    CHECK(info.lost_material_name == "missing");
    CHECK(info.face_colors.empty());
}

} // namespace obj_parser_contracts

namespace obj_binary_contracts {
namespace {

void write_bytes(const ScopedTemporaryFile &file, const std::vector<unsigned char> &bytes)
{
    boost::nowide::ofstream output(file.string(), std::ios::binary);
    output.write(reinterpret_cast<const char *>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
    REQUIRE(output.good());
}

template<typename T>
void append_raw(std::vector<unsigned char> &bytes, const T &value)
{
    const auto *begin = reinterpret_cast<const unsigned char *>(&value);
    bytes.insert(bytes.end(), begin, begin + sizeof(T));
}

ObjParser::ObjData fixture()
{
    ObjParser::ObjData data;
    data.version = 1;
    data.coordinates = {0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 1.f,
                        2.f, 0.f, 0.f, 0.f, 1.f, 0.f, .5f};
    data.has_vertex_color = true;
    data.textureCoordinates = {.1f, .2f, .8f, .9f};
    data.normals = {0.f, 0.f, 1.f};
    data.parameters = {.25f, .5f, .75f};
    data.mtllibs = {"materials/main.mtl", "materials/secondary.mtl"};

    ObjParser::ObjUseMtl material{};
    material.vertexIdxFirst = 4;
    material.vertexIdxEnd = 9;
    material.face_start = 2;
    material.face_end = 5;
    material.name = "painted material with a long name";
    data.usemtls.push_back(material);

    ObjParser::ObjObject object{};
    object.vertexIdxFirst = 4;
    object.name = "object name containing spaces";
    data.objects.push_back(object);

    ObjParser::ObjGroup group{};
    group.vertexIdxFirst = 4;
    group.name = "group name containing spaces";
    data.groups.push_back(group);

    data.smoothingGroups.push_back({4, 17});
    data.vertices = {{0, 0, 0}, {1, 1, 0}, {0, 0, 0}, {-1, -1, -1}};
    return data;
}

void save_fixture(const ScopedTemporaryFile &file)
{
    REQUIRE(ObjParser::objbinsave(file.string().c_str(), fixture()));
}

} // namespace

// Known bug: objbinload reads the size_t version into an int, so every later count is read 4 bytes off; fixed by the
// follow-up OBJ binary cache fix PR, which removes this [.]. Hidden because the misread count allocates gigabytes.
TEST_CASE("OBJ binary cache roundtrips geometry, names, groups, materials and smoothing", "[OBJBinary][OBJParser][.]")
{
    ScopedTemporaryFile cache(".objbin");
    const ObjParser::ObjData original = fixture();
    REQUIRE(ObjParser::objbinsave(cache.string().c_str(), original));

    ObjParser::ObjData loaded;
    REQUIRE(ObjParser::objbinload(cache.string().c_str(), loaded));
    CHECK(loaded.version == 1);
    CHECK(ObjParser::objequal(original, loaded));
    REQUIRE(loaded.coordinates.size() == original.coordinates.size());
    for (size_t i = 0; i < loaded.coordinates.size(); ++i)
        CHECK_THAT(loaded.coordinates[i], WithinAbs(original.coordinates[i], 1e-6));
    REQUIRE(loaded.textureCoordinates.size() == original.textureCoordinates.size());
    for (size_t i = 0; i < loaded.textureCoordinates.size(); ++i)
        CHECK_THAT(loaded.textureCoordinates[i], WithinAbs(original.textureCoordinates[i], 1e-6));
    REQUIRE(loaded.normals.size() == original.normals.size());
    for (size_t i = 0; i < loaded.normals.size(); ++i)
        CHECK_THAT(loaded.normals[i], WithinAbs(original.normals[i], 1e-6));
    REQUIRE(loaded.parameters.size() == original.parameters.size());
    for (size_t i = 0; i < loaded.parameters.size(); ++i)
        CHECK_THAT(loaded.parameters[i], WithinAbs(original.parameters[i], 1e-6));
    CHECK(loaded.mtllibs == original.mtllibs);
    REQUIRE(loaded.usemtls.size() == 1);
    CHECK(loaded.usemtls.front().vertexIdxFirst == original.usemtls.front().vertexIdxFirst);
    CHECK(loaded.usemtls.front().name == original.usemtls.front().name);
    REQUIRE(loaded.objects.size() == 1);
    CHECK(loaded.objects.front().name == original.objects.front().name);
    REQUIRE(loaded.groups.size() == 1);
    CHECK(loaded.groups.front().name == original.groups.front().name);
    REQUIRE(loaded.smoothingGroups.size() == 1);
    CHECK(loaded.smoothingGroups.front().smoothingGroupID == 17);
    CHECK(loaded.vertices == original.vertices);
}

// Known bug: objbinload reads the size_t version into an int, so every later count is read 4 bytes off; fixed by the
// follow-up OBJ binary cache fix PR, which removes this [.]. Hidden because the misread count allocates gigabytes.
TEST_CASE("OBJ binary cache supports an empty data set and clears prior records", "[OBJBinary][OBJParser][.]")
{
    ScopedTemporaryFile full(".objbin");
    ScopedTemporaryFile empty(".objbin");
    const ObjParser::ObjData original = fixture();
    ObjParser::ObjData no_records;
    REQUIRE(ObjParser::objbinsave(full.string().c_str(), original));
    REQUIRE(ObjParser::objbinsave(empty.string().c_str(), no_records));

    ObjParser::ObjData loaded;
    REQUIRE(ObjParser::objbinload(full.string().c_str(), loaded));
    REQUIRE(ObjParser::objbinload(empty.string().c_str(), loaded));
    CHECK(loaded.coordinates.empty());
    CHECK(loaded.textureCoordinates.empty());
    CHECK(loaded.normals.empty());
    CHECK(loaded.parameters.empty());
    CHECK(loaded.mtllibs.empty());
    CHECK(loaded.usemtls.empty());
    CHECK(loaded.objects.empty());
    CHECK(loaded.groups.empty());
    CHECK(loaded.smoothingGroups.empty());
    CHECK(loaded.vertices.empty());
}

TEST_CASE("OBJ binary cache rejects short and unsupported headers", "[OBJBinary][OBJBinarySafe][OBJParser]")
{
    ScopedTemporaryFile short_header(".objbin");
    // Three bytes are shorter than the version field of any build, so the version read itself fails.
    write_bytes(short_header, {1, 0, 0});
    ObjParser::ObjData loaded;
    CHECK_FALSE(ObjParser::objbinload(short_header.string().c_str(), loaded));

    ScopedTemporaryFile unsupported(".objbin");
    std::vector<unsigned char> bytes;
    const size_t version = 99;
    append_raw(bytes, version);
    write_bytes(unsupported, bytes);
    CHECK_FALSE(ObjParser::objbinload(unsupported.string().c_str(), loaded));
}

// Known bug: objbinload allocates whatever count the file claims, so this throws std::length_error instead of
// returning false; fixed by the follow-up OBJ binary cache fix PR
TEST_CASE("OBJ binary cache rejects a huge count before allocating", "[OBJBinary][OBJParser][!shouldfail]")
{
    ScopedTemporaryFile cache(".objbin");
    std::vector<unsigned char> bytes;
    const size_t version = 1;
    const size_t huge_count = std::numeric_limits<size_t>::max();
    append_raw(bytes, version);
    append_raw(bytes, huge_count);
    write_bytes(cache, bytes);

    ObjParser::ObjData loaded;
    CHECK_FALSE(ObjParser::objbinload(cache.string().c_str(), loaded));
}

// Known bug: objbinload reads the size_t version into an int, so every later count is read 4 bytes off; fixed by the
// follow-up OBJ binary cache fix PR, which removes this [.]. Hidden because the misread count allocates gigabytes.
TEST_CASE("OBJ binary cache rejects a truncated numeric vector", "[OBJBinary][OBJParser][.]")
{
    ScopedTemporaryFile cache(".objbin");
    std::vector<unsigned char> bytes;
    const size_t version = 1;
    const size_t coordinate_count = 1;
    append_raw(bytes, version);
    append_raw(bytes, coordinate_count);
    write_bytes(cache, bytes);

    ObjParser::ObjData loaded;
    CHECK_FALSE(ObjParser::objbinload(cache.string().c_str(), loaded));
}

// Known bug: objbinload reads the size_t version into an int, so every later count is read 4 bytes off; fixed by the
// follow-up OBJ binary cache fix PR, which removes this [.]. Hidden because the misread count allocates gigabytes.
TEST_CASE("OBJ binary cache rejects a truncated named record", "[OBJBinary][OBJParser][.]")
{
    ScopedTemporaryFile cache(".objbin");
    std::vector<unsigned char> bytes;
    const size_t version = 1;
    const size_t zero = 0;
    const size_t one = 1;
    const int vertex_index = 3;
    const size_t name_length = 8;
    append_raw(bytes, version);
    // coordinates, texture coordinates, normals, parameters, and mtllibs
    append_raw(bytes, zero);
    append_raw(bytes, zero);
    append_raw(bytes, zero);
    append_raw(bytes, zero);
    append_raw(bytes, zero);
    // one material name-index record with a name length but no payload
    append_raw(bytes, one);
    append_raw(bytes, vertex_index);
    append_raw(bytes, name_length);
    write_bytes(cache, bytes);

    ObjParser::ObjData loaded;
    CHECK_FALSE(ObjParser::objbinload(cache.string().c_str(), loaded));
}

// Known bug: objbinload reads the size_t version into an int, so every later count is read 4 bytes off; fixed by the
// follow-up OBJ binary cache fix PR, which removes this [.]. Hidden because the misread count allocates gigabytes.
TEST_CASE("OBJ binary cache rejects a truncated smoothing group vector", "[OBJBinary][OBJParser][.]")
{
    ScopedTemporaryFile cache(".objbin");
    std::vector<unsigned char> bytes;
    const size_t version = 1;
    const size_t zero = 0;
    const size_t one = 1;
    append_raw(bytes, version);
    // first five variable-length vectors and the three name-index vectors
    for (int i = 0; i < 8; ++i)
        append_raw(bytes, zero);
    append_raw(bytes, one); // smoothing group count, but no record bytes
    write_bytes(cache, bytes);

    ObjParser::ObjData loaded;
    CHECK_FALSE(ObjParser::objbinload(cache.string().c_str(), loaded));
}

// Known bug: objbinload reads the size_t version into an int, so every later count is read 4 bytes off; fixed by the
// follow-up OBJ binary cache fix PR, which removes this [.]. Hidden because the misread count allocates gigabytes.
TEST_CASE("OBJ binary cache rejects a truncated face vertex vector", "[OBJBinary][OBJParser][.]")
{
    ScopedTemporaryFile cache(".objbin");
    std::vector<unsigned char> bytes;
    const size_t version = 1;
    const size_t zero = 0;
    const size_t one = 1;
    append_raw(bytes, version);
    // every vector before the face vertices is empty, including the smoothing groups
    for (int i = 0; i < 9; ++i)
        append_raw(bytes, zero);
    append_raw(bytes, one); // face vertex count, but no record bytes
    write_bytes(cache, bytes);

    ObjParser::ObjData loaded;
    CHECK_FALSE(ObjParser::objbinload(cache.string().c_str(), loaded));
}

// Known bug: objbinload reads the size_t version into an int, so version 0x100000001 is accepted as version 1;
// fixed by the follow-up OBJ binary cache fix PR
TEST_CASE("OBJ binary cache rejects unsupported bits across its complete native version", "[OBJBinary][OBJBinaryWidthControl][OBJParser][!shouldfail]")
{
    ScopedTemporaryFile cache(".objbin");
    std::vector<unsigned char> bytes;
    size_t unsupported_version = 99;
    if constexpr (sizeof(size_t) > sizeof(int))
        unsupported_version = static_cast<size_t>(UINT64_C(0x100000001));
    append_raw(bytes, unsupported_version);
    // On 64-bit systems the old int reader sees version one, then interprets
    // the high word as one coordinate. The zero payload keeps that control
    // bounded: it allocates only one float rather than an oversized vector.
    for (int i = 0; i < 10; ++i)
        append_raw(bytes, size_t{0});
    write_bytes(cache, bytes);
    ObjParser::ObjData loaded;
    CHECK_FALSE(ObjParser::objbinload(cache.string().c_str(), loaded));
}

// Known bug: objbinload reads the size_t version into an int, so every later count is read 4 bytes off; fixed by the
// follow-up OBJ binary cache fix PR, which removes this [.]. Hidden because the misread count allocates gigabytes.
TEST_CASE("OBJ binary cache rejects every proper prefix of a valid cache", "[OBJBinary][OBJParser][.]")
{
    ScopedTemporaryFile full(".objbin");
    save_fixture(full);
    boost::nowide::ifstream input(full.string(), std::ios::binary);
    REQUIRE(input.good());
    const std::vector<unsigned char> complete{std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
    REQUIRE_FALSE(complete.empty());
    ScopedTemporaryFile truncated(".objbin");
    for (size_t length = 0; length < complete.size(); ++length) {
        const std::vector<unsigned char> prefix(complete.begin(), complete.begin() + length);
        write_bytes(truncated, prefix);
        ObjParser::ObjData loaded;
        INFO("prefix length " << length);
        CHECK_FALSE(ObjParser::objbinload(truncated.string().c_str(), loaded));
    }
}

} // namespace obj_binary_contracts
