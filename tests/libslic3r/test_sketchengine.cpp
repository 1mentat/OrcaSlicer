#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <BRepAdaptor_Surface.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepGProp.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <GProp_GProps.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Wire.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>
#include <cmath>
#include <stdexcept>

#include "libslic3r/CAD/SketchEngine.hpp"
#include "libslic3r/Point.hpp"

using namespace Slic3r;

TEST_CASE("SketchPlane projects forward rays in its local frame and rejects rays missing ahead", "[SketchEngine][SketchPlaneGap]")
{
    SketchPlane plane;
    plane.origin = Vec3d(5., 6., 4.);
    plane.normal = Vec3d(0., 0., 1.);
    plane.x_axis = Vec3d(0., 1., 0.);
    plane.y_axis = Vec3d(-1., 0., 0.);

    const Vec2d local = plane.project(Vec3d(8., 10., 12.), Vec3d(0., 0., -2.));
    CHECK_THAT(local.x(), Catch::Matchers::WithinAbs(4., 1e-12));
    CHECK_THAT(local.y(), Catch::Matchers::WithinAbs(-3., 1e-12));
    const Vec3d hit = plane.to_world(local);
    CHECK_THAT(hit.x(), Catch::Matchers::WithinAbs(8., 1e-12));
    CHECK_THAT(hit.y(), Catch::Matchers::WithinAbs(10., 1e-12));
    CHECK_THAT(hit.z(), Catch::Matchers::WithinAbs(4., 1e-12));

    const Vec2d parallel = plane.project(Vec3d(8., 10., 12.), Vec3d(1., 0., 0.));
    CHECK_THAT(parallel.x(), Catch::Matchers::WithinAbs(0., 1e-12));
    CHECK_THAT(parallel.y(), Catch::Matchers::WithinAbs(0., 1e-12));
    const Vec2d behind = plane.project(Vec3d(8., 10., 12.), Vec3d(0., 0., 2.));
    CHECK_THAT(behind.x(), Catch::Matchers::WithinAbs(0., 1e-12));
    CHECK_THAT(behind.y(), Catch::Matchers::WithinAbs(0., 1e-12));
}

TEST_CASE("SketchPlane derives a local frame from a curved face", "[SketchEngine][SketchPlaneGap]")
{
    const gp_Ax2 axis(gp_Pnt(3., -2., 7.), gp_Dir(0., 0., 1.));
    TopoDS_Shape cylinder = BRepPrimAPI_MakeCylinder(axis, 10., 20.).Shape();
    BRepMesh_IncrementalMesh mesh(cylinder, .1);
    (void)mesh;

    TopoDS_Face side;
    for (TopExp_Explorer faces(cylinder, TopAbs_FACE); faces.More(); faces.Next()) {
        TopoDS_Face face = TopoDS::Face(faces.Current());
        BRepAdaptor_Surface surface(face);
        if (surface.GetType() == GeomAbs_Cylinder) {
            side = face;
            break;
        }
    }
    REQUIRE_FALSE(side.IsNull());

    const SketchPlane plane = SketchPlane::from_face(side);
    CHECK_THAT(plane.normal.norm(), Catch::Matchers::WithinAbs(1., 1e-9));
    CHECK_THAT(plane.x_axis.norm(), Catch::Matchers::WithinAbs(1., 1e-9));
    CHECK_THAT(plane.y_axis.norm(), Catch::Matchers::WithinAbs(1., 1e-9));
    CHECK_THAT(plane.normal.dot(plane.x_axis), Catch::Matchers::WithinAbs(0., 1e-9));
    CHECK_THAT(plane.normal.dot(plane.y_axis), Catch::Matchers::WithinAbs(0., 1e-9));
    CHECK_THAT(plane.x_axis.dot(plane.y_axis), Catch::Matchers::WithinAbs(0., 1e-9));

    CHECK(plane.origin.z() >= 7. - 1e-6);
    CHECK(plane.origin.z() <= 27. + 1e-6);
    const double radial_x = plane.origin.x() - 3.;
    const double radial_y = plane.origin.y() + 2.;
    const double radial_length = std::hypot(radial_x, radial_y);
    REQUIRE_THAT(radial_length, Catch::Matchers::WithinAbs(10., .02));
    const Vec3d radial(radial_x / radial_length, radial_y / radial_length, 0.);
    CHECK_THAT(plane.normal.dot(radial), Catch::Matchers::WithinAbs(1., 1e-6));
    CHECK_THAT(plane.y_axis.dot(Vec3d(0., 0., 1.)), Catch::Matchers::WithinAbs(1., 1e-6));
}

TEST_CASE("SketchProfile rejects a profile with fewer than two points", "[SketchEngine][SketchPlaneGap]")
{
    SketchProfile profile;
    REQUIRE_THROWS_AS(profile.to_occt_wire(SketchPlane::XY()), std::runtime_error);
    profile.points.emplace_back(3., 4.);
    REQUIRE_THROWS_AS(profile.to_occt_wire(SketchPlane::XY()), std::runtime_error);
}

TEST_CASE("SketchEngine rejects a zero-length prism direction", "[SketchEngine][SketchPlaneGap]")
{
    SketchProfile profile;
    profile.points = {Vec2d(0., 0.), Vec2d(4., 0.), Vec2d(4., 3.), Vec2d(0., 3.)};
    const TopoDS_Wire wire = profile.to_occt_wire(SketchPlane::XY());
    const TopoDS_Face base = BRepBuilderAPI_MakeFace(wire).Face();
    REQUIRE_FALSE(base.IsNull());
    REQUIRE_THROWS_AS(SketchEngine::make_prism(base, gp_Vec(0., 0., 0.)), std::runtime_error);

    // The same base succeeds for positive depth and encloses rectangle area times depth.
    const TopoDS_Shape prism = SketchEngine::make_prism(base, gp_Vec(0., 0., 2.));
    REQUIRE_FALSE(prism.IsNull());
    GProp_GProps properties;
    BRepGProp::VolumeProperties(prism, properties);
    CHECK_THAT(properties.Mass(), Catch::Matchers::WithinAbs(4. * 3. * 2., 1e-8));
}
