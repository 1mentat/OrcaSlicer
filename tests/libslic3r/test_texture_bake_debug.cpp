#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <algorithm>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <cstddef>
#include <fstream>
#include <ios>
#include <iterator>
#include <sstream>
#include <string>
#include <vector>

#include "test_utils.hpp"
#include "libslic3r/Point.hpp"
#include <libslic3r/TextureBake/TextureBakeDebug.hpp>
#include <libslic3r/TextureBake/TextureBakeIndex.hpp>

using namespace Slic3r;
using namespace Slic3r::TextureBake;
using Catch::Matchers::WithinAbs;

namespace {

BakeStageMesh one_triangle()
{
    BakeStageMesh mesh;
    mesh.vertices = { Vec3f(0.f, 0.f, 0.f), Vec3f(1.f, 0.f, 0.f), Vec3f(0.f, 1.f, 0.f) };
    mesh.indices = { Vec3i32(0, 1, 2) };
    return mesh;
}

std::string read_text(const boost::filesystem::path &path)
{
    std::ifstream input(path.string(), std::ios::binary);
    REQUIRE(input.good());
    return std::string(std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>());
}

void require_unit_triangle_obj(const std::string &text)
{
    std::vector<Vec3d> vertices;
    std::vector<Vec3i32> faces;
    std::istringstream lines(text);
    std::string line;
    while (std::getline(lines, line)) {
        std::istringstream fields(line);
        std::string kind;
        fields >> kind;
        if (kind == "v") {
            Vec3d vertex;
            REQUIRE(static_cast<bool>(fields >> vertex.x() >> vertex.y() >> vertex.z()));
            vertices.push_back(vertex);
        } else if (kind == "f") {
            Vec3i32 face;
            REQUIRE(static_cast<bool>(fields >> face[0] >> face[1] >> face[2]));
            faces.push_back(face);
        }
    }
    REQUIRE(vertices.size() == 3);
    REQUIRE(faces.size() == 1);
    for (const Vec3f &expected : one_triangle().vertices) {
        const auto matches = std::count_if(vertices.begin(), vertices.end(), [&expected](const Vec3d &vertex) {
            return (vertex - expected.cast<double>()).norm() <= 1e-6;
        });
        CHECK(matches == 1);
    }
    for (int corner = 0; corner < 3; ++corner) {
        const int index = faces.front()[corner];
        REQUIRE(index >= 1);
        REQUIRE(static_cast<size_t>(index) <= vertices.size());
    }
    const Vec3i32 &face = faces.front();
    const Vec3d normal = (vertices[face[1] - 1] - vertices[face[0] - 1]).cross(
        vertices[face[2] - 1] - vertices[face[0] - 1]);
    CHECK_THAT(normal.z(), WithinAbs(1.0, 1e-6));
}

} // namespace

TEST_CASE("Bake stage recording stays inert until enabled and totals caller timings", "[TextureBakeDebug]")
{
    BakeStageRecorder recorder;
    const BakeStageMesh mesh = one_triangle();

    CHECK_FALSE(recorder.enabled());
    recorder.capture("ignored", mesh.vertices, mesh.indices, 9.0, "disabled");
    recorder.capture_note("also ignored", 3.0, "disabled");
    const TriSoup empty_soup;
    recorder.capture("ignored soup", empty_soup, 5.0);
    CHECK(recorder.empty());

    recorder.enable(true);
    recorder.set_check_topology(false);
    CHECK(recorder.enabled());
    CHECK_FALSE(recorder.check_topology());
    recorder.capture_note("skipped remesh", 1.25, "no changed faces");
    recorder.capture("subdivide stage", mesh.vertices, mesh.indices, 2.75, "one face retained");

    REQUIRE(recorder.stages().size() == 2);
    CHECK(recorder.stages()[0].name == "skipped remesh");
    CHECK(recorder.stages()[0].detail == "no changed faces");
    CHECK_FALSE(recorder.stages()[0].topology_checked);
    CHECK(recorder.stages()[1].name == "subdivide stage");
    CHECK(recorder.stages()[1].triangles == 1);
    CHECK(recorder.stages()[1].vertices == 3);
    CHECK_FALSE(recorder.stages()[1].topology_checked);
    CHECK_THAT(recorder.total_ms(), WithinAbs(4.0, 1e-12));
    const std::vector<BakeStageSnapshot> taken = recorder.take();
    REQUIRE(taken.size() == 2);
    CHECK(taken[1].name == "subdivide stage");
    recorder.clear();
    CHECK(recorder.empty());
    CHECK(recorder.mark() == 0);
    CHECK_THAT(recorder.total_ms(), WithinAbs(0.0, 1e-12));
}

TEST_CASE("Bake topology counts closed, open, non-manifold, and degenerate faces", "[TextureBakeDebug]")
{
    BakeStageMesh tetrahedron;
    tetrahedron.vertices = { Vec3f(0.f, 0.f, 0.f), Vec3f(1.f, 0.f, 0.f),
                             Vec3f(0.f, 1.f, 0.f), Vec3f(0.f, 0.f, 1.f) };
    tetrahedron.indices = { Vec3i32(0, 2, 1), Vec3i32(0, 1, 3),
                            Vec3i32(1, 2, 3), Vec3i32(2, 0, 3) };
    size_t open = 0, non_manifold = 0, degenerate = 0;
    bake_stage_topology(tetrahedron, open, non_manifold, degenerate);
    CHECK(open == 0);
    CHECK(non_manifold == 0);
    CHECK(degenerate == 0);

    const BakeStageMesh open_triangle = one_triangle();
    bake_stage_topology(open_triangle, open, non_manifold, degenerate);
    CHECK(open == 3);
    CHECK(non_manifold == 0);
    CHECK(degenerate == 0);

    BakeStageMesh shared_edge;
    shared_edge.vertices = { Vec3f(0.f, 0.f, 0.f), Vec3f(1.f, 0.f, 0.f),
                             Vec3f(0.f, 1.f, 0.f), Vec3f(0.f, -1.f, 0.f),
                             Vec3f(0.f, 0.f, 1.f) };
    shared_edge.indices = { Vec3i32(0, 1, 2), Vec3i32(1, 0, 3), Vec3i32(0, 1, 4) };
    bake_stage_topology(shared_edge, open, non_manifold, degenerate);
    CHECK(open == 6);
    CHECK(non_manifold == 1);
    CHECK(degenerate == 0);

    BakeStageMesh collinear;
    collinear.vertices = { Vec3f(0.f, 0.f, 0.f), Vec3f(1.f, 0.f, 0.f), Vec3f(2.f, 0.f, 0.f) };
    collinear.indices = { Vec3i32(0, 1, 2) };
    bake_stage_topology(collinear, open, non_manifold, degenerate);
    CHECK(open == 3);
    CHECK(non_manifold == 0);
    CHECK(degenerate == 1);

    BakeStageMesh collapsed;
    collapsed.vertices = { Vec3f(0.f, 0.f, 0.f), Vec3f(1.f, 0.f, 0.f) };
    collapsed.indices = { Vec3i32(0, 0, 1), Vec3i32(0, 1, 1), Vec3i32(0, 1, 0) };
    bake_stage_topology(collapsed, open, non_manifold, degenerate);
    CHECK(open == 0);
    CHECK(non_manifold == 0);
    CHECK(degenerate == 3);
}

TEST_CASE("TriSoup stage capture welds repeated corners and drops collapsed faces", "[TextureBakeDebug]")
{
    TriSoup soup;
    soup.pos = { Vec3f(0.f, 0.f, 0.f), Vec3f(1.f, 0.f, 0.f), Vec3f(0.f, 1.f, 0.f),
                 // Each pair of corners can collapse after welding, independently
                 // of which corner starts the triangle.
                 Vec3f(0.000002f, 0.f, 0.f), Vec3f(1.f, 0.f, 0.f), Vec3f(0.f, 0.000002f, 0.f),
                 Vec3f(0.f, 0.f, 0.f), Vec3f(0.000002f, 0.f, 0.f), Vec3f(1.f, 0.f, 0.f),
                 Vec3f(0.f, 0.f, 0.f), Vec3f(1.f, 0.f, 0.f), Vec3f(1.f, 0.000002f, 0.f) };

    BakeStageRecorder recorder;
    recorder.enable(true);
    recorder.capture("weld", soup, 3.5, "one valid face and three collapsed faces");
    REQUIRE(recorder.stages().size() == 1);
    const BakeStageSnapshot &stage = recorder.stages().front();
    CHECK(stage.triangles == 1);
    CHECK(stage.vertices == 3);
    REQUIRE(stage.mesh.vertices.size() == 3);
    REQUIRE(stage.mesh.indices.size() == 1);
    CHECK(stage.mesh.indices[0][0] == 0);
    CHECK(stage.mesh.indices[0][1] == 1);
    CHECK(stage.mesh.indices[0][2] == 2);
    CHECK_THAT((stage.mesh.vertices[0] - Vec3f(0.f, 0.f, 0.f)).norm(), WithinAbs(0.f, 1e-8));
    CHECK_THAT((stage.mesh.vertices[1] - Vec3f(1.f, 0.f, 0.f)).norm(), WithinAbs(0.f, 1e-8));
    CHECK_THAT((stage.mesh.vertices[2] - Vec3f(0.f, 1.f, 0.f)).norm(), WithinAbs(0.f, 1e-8));
    CHECK(stage.open_edges == 3);
    CHECK(stage.topology_checked);
    CHECK_FALSE(stage.mesh_dropped);
}

TEST_CASE("Bake stage cap retains counts and drops only meshes above the limit", "[TextureBakeDebug]")
{
    const BakeStageMesh mesh = one_triangle();
    BakeStageRecorder recorder;
    recorder.enable(true);
    recorder.set_mesh_cap(1);
    CHECK(recorder.mesh_cap() == 1);
    recorder.capture("at cap", mesh.vertices, mesh.indices, 1.0);
    recorder.set_mesh_cap(0);
    CHECK(recorder.mesh_cap() == 0);
    recorder.capture("over cap", mesh.vertices, mesh.indices, 2.0);

    REQUIRE(recorder.stages().size() == 2);
    const BakeStageSnapshot &retained = recorder.stages()[0];
    CHECK_FALSE(retained.mesh_dropped);
    CHECK(retained.triangles == 1);
    CHECK(retained.vertices == 3);
    CHECK(retained.mesh.indices.size() == 1);

    const BakeStageSnapshot &dropped = recorder.stages()[1];
    CHECK(dropped.mesh_dropped);
    CHECK(dropped.triangles == 1);
    CHECK(dropped.vertices == 3);
    CHECK(dropped.mesh.empty());
    CHECK(dropped.mesh.vertices.empty());
    CHECK(dropped.open_edges == 3);
    CHECK(dropped.topology_checked);
}

TEST_CASE("Bake stage rebasing transforms only the marked range and flips its winding", "[TextureBakeDebug]")
{
    const BakeStageMesh mesh = one_triangle();
    BakeStageRecorder recorder;
    recorder.enable(true);
    recorder.capture("before mark", mesh.vertices, mesh.indices, 0.5);
    recorder.rebase(0, nullptr, false);
    const size_t from = recorder.mark();

    const std::vector<Vec3f> world_vertices = { Vec3f(11.f, 2.f, 3.f), Vec3f(12.f, 2.f, 3.f),
                                                Vec3f(11.f, 3.f, 3.f) };
    recorder.capture("world stage", world_vertices, mesh.indices, 4.0);
    Transform3d to_local = Transform3d::Identity();
    to_local.translation() = Vec3d(-10.0, 3.0, -2.0);
    recorder.rebase(from, &to_local, true);

    REQUIRE(recorder.stages().size() == 2);
    REQUIRE(recorder.stages()[0].mesh.vertices.size() == 3);
    REQUIRE(recorder.stages()[0].mesh.indices.size() == 1);
    CHECK_THAT((recorder.stages()[0].mesh.vertices[0] - Vec3f(0.f, 0.f, 0.f)).norm(), WithinAbs(0.f, 1e-8));
    CHECK(recorder.stages()[0].mesh.indices[0][0] == 0);
    CHECK(recorder.stages()[0].mesh.indices[0][1] == 1);
    CHECK(recorder.stages()[0].mesh.indices[0][2] == 2);
    const BakeStageSnapshot &rebased = recorder.stages()[1];
    REQUIRE(rebased.mesh.vertices.size() == 3);
    REQUIRE(rebased.mesh.indices.size() == 1);
    CHECK_THAT((rebased.mesh.vertices[0] - Vec3f(1.f, 5.f, 1.f)).norm(), WithinAbs(0.f, 1e-6));
    CHECK_THAT((rebased.mesh.vertices[1] - Vec3f(2.f, 5.f, 1.f)).norm(), WithinAbs(0.f, 1e-6));
    CHECK_THAT((rebased.mesh.vertices[2] - Vec3f(1.f, 6.f, 1.f)).norm(), WithinAbs(0.f, 1e-6));
    const Vec3i32 &face = rebased.mesh.indices.front();
    for (int corner = 0; corner < 3; ++corner) {
        REQUIRE(face[corner] >= 0);
        REQUIRE(static_cast<size_t>(face[corner]) < rebased.mesh.vertices.size());
    }
    const Vec3f normal = (rebased.mesh.vertices[face[1]] - rebased.mesh.vertices[face[0]]).cross(
        rebased.mesh.vertices[face[2]] - rebased.mesh.vertices[face[0]]);
    CHECK_THAT(normal.z(), WithinAbs(-1.f, 1e-6));
    CHECK_THAT(recorder.total_ms(), WithinAbs(4.5, 1e-12));
}

TEST_CASE("Bake stage dumps write sanitized OBJ geometry and an honest summary", "[TextureBakeDebug]")
{
    BakeStageRecorder recorder;
    recorder.enable(true);
    const BakeStageMesh mesh = one_triangle();
    recorder.capture("Stage 1 / rough: A", mesh.vertices, mesh.indices, 12.5, "remesh detail");
    recorder.capture("Plain", mesh.vertices, mesh.indices, 0.25);
    recorder.capture_note("Skipped stage", 1.25, "no geometry");
    recorder.set_mesh_cap(0);
    recorder.capture("Over cap", mesh.vertices, mesh.indices, 0.25, "geometry retained as counts");

    ScopedTemporaryDir output("orca_texture_bake_debug");
    CHECK(recorder.dump_obj(output.string()) == 2);
    const boost::filesystem::path obj_path = output.path() / "00_Stage_1___rough__A.obj";
    REQUIRE(boost::filesystem::exists(obj_path));
    CHECK_FALSE(boost::filesystem::exists(output.path() / "02_Skipped_stage.obj"));
    const std::string obj = read_text(obj_path);
    CHECK(obj.find("Stage 1 / rough: A") != std::string::npos);
    CHECK(obj.find("remesh detail") != std::string::npos);
    require_unit_triangle_obj(obj);
    REQUIRE(boost::filesystem::exists(output.path() / "01_Plain.obj"));
    const std::string plain_obj = read_text(output.path() / "01_Plain.obj");
    require_unit_triangle_obj(plain_obj);
    CHECK(plain_obj.find("remesh detail") == std::string::npos);

    const std::string summary = read_text(output.path() / "stages.txt");
    CHECK(summary.find("Stage 1 / rough: A") != std::string::npos);
    CHECK(summary.find("remesh detail") != std::string::npos);
    CHECK(summary.find("Skipped stage") != std::string::npos);
    CHECK(summary.find("geometry retained as counts [mesh over cap, not written]") != std::string::npos);
    CHECK(summary.find("total 14.25 ms across 4 stages") != std::string::npos);

    // A regular file cannot become an output directory. Preserve its contents.
    CHECK(dump_bake_stages(recorder.stages(), obj_path.string()) == 0);
    CHECK(read_text(obj_path) == obj);

    // Existing directories at output-file paths cause deterministic write errors
    // on every platform, without depending on user permissions or a full disk.
    const boost::filesystem::path blocked = output.path() / "blocked";
    boost::filesystem::create_directories(blocked / obj_path.filename());
    boost::filesystem::create_directories(blocked / "stages.txt");
    CHECK(dump_bake_stages(recorder.stages(), blocked.string()) == 1);
    CHECK(boost::filesystem::is_directory(blocked / obj_path.filename()));
    CHECK(boost::filesystem::is_directory(blocked / "stages.txt"));
    require_unit_triangle_obj(read_text(blocked / "01_Plain.obj"));
}
