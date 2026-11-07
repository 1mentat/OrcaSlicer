#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <algorithm>
#include <cmath>
#include <vector>

#include <BRep_Builder.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Shell.hxx>
#include <gp_Pnt.hxx>

#include "libslic3r/CAD/GeometryEngine.hpp"

using namespace Slic3r;
using Catch::Matchers::WithinAbs;

TEST_CASE("A planar face reports bounds in the requested local axes", "[GeometryEngine][FacePlaneBounds]")
{
    BRepBuilderAPI_MakePolygon polygon;
    polygon.Add(gp_Pnt(0.0, 0.0, 30.0));
    polygon.Add(gp_Pnt(10.0, 0.0, 30.0));
    polygon.Add(gp_Pnt(10.0, 20.0, 30.0));
    polygon.Add(gp_Pnt(0.0, 20.0, 30.0));
    polygon.Close();
    REQUIRE(polygon.IsDone());

    BRepBuilderAPI_MakeFace face_builder(polygon.Wire());
    REQUIRE(face_builder.IsDone());

    double umin = 0.0, umax = 0.0, vmin = 0.0, vmax = 0.0;
    const bool has_bounds = GeometryEngine::face_plane_bounds(
        face_builder.Face(), Vec3d(2.0, 3.0, 30.0), Vec3d(0.0, 1.0, 0.0),
        Vec3d(-1.0, 0.0, 0.0), umin, umax, vmin, vmax);
    REQUIRE(has_bounds);
    CHECK_THAT(umin, WithinAbs(-3.0, 1e-9));
    CHECK_THAT(umax, WithinAbs(17.0, 1e-9));
    CHECK_THAT(vmin, WithinAbs(-8.0, 1e-9));
    CHECK_THAT(vmax, WithinAbs(2.0, 1e-9));

    CHECK_FALSE(GeometryEngine::face_plane_bounds(
        TopoDS_Face(), Vec3d::Zero(), Vec3d::UnitX(), Vec3d::UnitY(),
        umin, umax, vmin, vmax));
}

TEST_CASE("Circular edges expose their radius and rim center", "[GeometryEngine][CircleOfEdge]")
{
    const TopoDS_Shape cylinder = BRepPrimAPI_MakeCylinder(5.0, 8.0).Shape();
    const std::vector<TopoDS_Edge> edges = GeometryEngine::edges_of(cylinder);
    int circular_edges = 0;
    bool has_non_circular_edge = false;

    for (const TopoDS_Edge& edge : edges) {
        const GeometryEngine::CylinderFace circle = GeometryEngine::circle_of_edge(edge);
        if (!circle.ok) {
            has_non_circular_edge = true;
            continue;
        }

        ++circular_edges;
        CHECK_THAT(circle.radius, WithinAbs(5.0, 1e-9));
        CHECK_THAT(circle.height, WithinAbs(0.0, 1e-12));
        CHECK_FALSE(circle.internal);
        CHECK_THAT(std::abs(circle.axis.z()), WithinAbs(1.0, 1e-9));
        CHECK(std::min(std::abs(circle.base.z()), std::abs(circle.base.z() - 8.0)) < 1e-9);
    }

    CHECK(circular_edges == 2);
    CHECK(has_non_circular_edge);
    CHECK_FALSE(GeometryEngine::circle_of_edge(TopoDS_Edge()).ok);
}

TEST_CASE("An open sheet reports its area without claiming a volume", "[GeometryEngine][SheetMassProperties]")
{
    const TopoDS_Shape box = BRepPrimAPI_MakeCylinder(5.0, 8.0).Shape();
    const TopoDS_Face face = GeometryEngine::face_by_index(box, 0);
    REQUIRE_FALSE(face.IsNull());

    TopoDS_Shell sheet;
    BRep_Builder builder;
    builder.MakeShell(sheet);
    builder.Add(sheet, face);

    const GeometryEngine::MassProps props = GeometryEngine::mass_properties(sheet);
    REQUIRE(props.valid);
    CHECK_FALSE(props.is_solid);
    CHECK_THAT(props.volume, WithinAbs(0.0, 1e-12));
    CHECK(props.surface_area > 0.0);
    CHECK_THAT(props.center_of_mass.norm(), WithinAbs(0.0, 1e-12));

    const GeometryEngine::MassProps missing = GeometryEngine::mass_properties(TopoDS_Shape());
    CHECK_FALSE(missing.valid);
    CHECK_FALSE(missing.is_solid);
}
