#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "libslic3r/SurfaceCollection.hpp"
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/Surface.hpp"
#include "libslic3r/libslic3r.h"
#include "test_utils.hpp"

#include <algorithm>
#include <cstddef>
#include <fstream>
#include <initializer_list>
#include <ios>
#include <iterator>
#include <string>
#include <utility>
#include <vector>

using namespace Slic3r;
using Catch::Matchers::WithinAbs;

namespace {

ExPolygon rectangle(coord_t x0, coord_t y0, coord_t x1, coord_t y1)
{
    return ExPolygon(Polygon { { x0, y0 }, { x1, y0 }, { x1, y1 }, { x0, y1 } });
}

ExPolygon rectangle_with_hole(coord_t x0, coord_t y0, coord_t x1, coord_t y1)
{
    ExPolygon result = rectangle(x0, y0, x1, y1);
    Polygon hole = rectangle(x0 + 20, y0 + 20, x0 + 50, y0 + 50).contour;
    if (!hole.is_clockwise())
        hole.reverse();
    result.holes.emplace_back(std::move(hole));
    return result;
}

Surface tagged_surface(SurfaceType type, int id)
{
    Surface result(type, rectangle(coord_t(id * 20), 0, coord_t(id * 20 + 10), 10));
    result.thickness = double(id);
    result.thickness_layers = 1;
    result.bridge_angle = -1.0;
    result.extra_perimeters = static_cast<unsigned short>(id);
    return result;
}

int marker(const Surface &surface)
{
    return static_cast<int>(surface.thickness);
}

Surface grouped_surface(SurfaceType type, int id, double thickness,
                        unsigned short thickness_layers, double bridge_angle)
{
    Surface result = tagged_surface(type, id);
    result.thickness = thickness;
    result.thickness_layers = thickness_layers;
    result.bridge_angle = bridge_angle;
    return result;
}

std::vector<int> markers(const SurfaceCollection &collection)
{
    std::vector<int> result;
    result.reserve(collection.size());
    for (const Surface &surface : collection)
        result.push_back(marker(surface));
    return result;
}

std::string read_text(const std::string &path)
{
    std::ifstream input(path, std::ios::binary);
    return { std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>() };
}

size_t count_text(const std::string &text, const std::string &needle)
{
    size_t count = 0;
    for (size_t pos = text.find(needle); pos != std::string::npos; pos = text.find(needle, pos + needle.size()))
        ++count;
    return count;
}

std::vector<std::string> svg_paths(const std::string &text)
{
    std::vector<std::string> result;
    for (size_t pos = text.find("<path "); pos != std::string::npos; pos = text.find("<path ", pos + 6)) {
        const size_t end = text.find("/>", pos);
        if (end == std::string::npos)
            break;
        result.push_back(text.substr(pos, end + 2 - pos));
    }
    return result;
}

} // namespace

TEST_CASE("SurfaceCollection simplify preserves surface metadata and holes", "[SurfaceCollection]")
{
    ExPolygon outline(Polygon { { 0, 0 }, { 40, 0 }, { 100, 0 }, { 100, 100 }, { 0, 100 } });
    Polygon hole = rectangle(30, 30, 60, 60).contour;
    if (!hole.is_clockwise())
        hole.reverse();
    outline.holes.emplace_back(std::move(hole));
    REQUIRE(outline.is_valid());
    const double original_area = outline.area();

    Surface surface(stTop, std::move(outline));
    surface.thickness = 0.3;
    surface.thickness_layers = 2;
    surface.bridge_angle = 0.75;
    surface.extra_perimeters = 3;
    SurfaceCollection collection(Surfaces { surface });

    collection.simplify(1.0);

    REQUIRE(collection.size() == 1);
    const Surface &simplified = collection.surfaces.front();
    CHECK(simplified.surface_type == stTop);
    CHECK_THAT(simplified.expolygon.area(), WithinAbs(original_area, 1e-6));
    REQUIRE(simplified.expolygon.holes.size() == 1);
    // Only the collinear (40, 0) vertex is redundant; the four corners must survive.
    const Points &contour = simplified.expolygon.contour.points;
    CHECK(contour.size() == 4);
    for (const Point &corner : { Point { 0, 0 }, Point { 100, 0 }, Point { 100, 100 }, Point { 0, 100 } })
        CHECK(std::find(contour.begin(), contour.end(), corner) != contour.end());
    CHECK(simplified.expolygon.contains(Point { 10, 10 }));
    CHECK_FALSE(simplified.expolygon.contains(Point { 45, 45 }));
    CHECK_THAT(simplified.thickness, WithinAbs(0.3, 1e-12));
    CHECK(simplified.thickness_layers == 2);
    CHECK_THAT(simplified.bridge_angle, WithinAbs(0.75, 1e-12));
    CHECK(simplified.extra_perimeters == 3);
}

TEST_CASE("SurfaceCollection groups surfaces by every merge property", "[SurfaceCollection]")
{
    SurfaceCollection collection(Surfaces {
        grouped_surface(stTop, 1, 0.2, 1, -1.0),
        grouped_surface(stTop, 2, 0.2, 1, -1.0),
        grouped_surface(stTop, 3, 0.4, 1, -1.0),
        grouped_surface(stTop, 4, 0.2, 2, -1.0),
        grouped_surface(stTop, 5, 0.2, 1, 0.5),
        grouped_surface(stBottom, 6, 0.2, 1, -1.0),
    });
    // A caller may pass an existing, empty group; grouping leaves it intact
    // and appends groups for this collection.
    std::vector<SurfacesPtr> groups(1);

    collection.group(&groups);

    REQUIRE(groups.size() == 6);
    CHECK(groups.front().empty());
    REQUIRE(groups[1].size() == 2);
    CHECK(groups[1][0] == &collection.surfaces[0]);
    CHECK(groups[1][1] == &collection.surfaces[1]);
    CHECK(groups[2] == SurfacesPtr { &collection.surfaces[2] });
    CHECK(groups[3] == SurfacesPtr { &collection.surfaces[3] });
    CHECK(groups[4] == SurfacesPtr { &collection.surfaces[4] });
    CHECK(groups[5] == SurfacesPtr { &collection.surfaces[5] });
}

TEST_CASE("SurfaceCollection filters preserve source order and append contour polygons", "[SurfaceCollection]")
{
    Surface first_top = tagged_surface(stTop, 1);
    first_top.expolygon = rectangle_with_hole(20, 0, 100, 100);
    Surface second_top = tagged_surface(stTop, 3);
    SurfaceCollection collection(Surfaces {
        tagged_surface(stInternal, 0), first_top, tagged_surface(stBottom, 2),
        second_top, tagged_surface(stInternalSolid, 4),
    });

    const SurfacesPtr tops = collection.filter_by_type(stTop);
    REQUIRE(tops.size() == 2);
    CHECK(tops[0] == &collection.surfaces[1]);
    CHECK(tops[1] == &collection.surfaces[3]);

    const SurfacesPtr selected = collection.filter_by_types({ stBottom, stTop });
    REQUIRE(selected.size() == 3);
    CHECK(selected[0] == &collection.surfaces[1]);
    CHECK(selected[1] == &collection.surfaces[2]);
    CHECK(selected[2] == &collection.surfaces[3]);
    CHECK(collection.filter_by_type(stPerimeter).empty());

    Polygons polygons { rectangle(-20, -20, -10, -10).contour };
    collection.filter_by_type(stTop, &polygons);
    REQUIRE(polygons.size() == 4);
    CHECK(polygons[0].points == rectangle(-20, -20, -10, -10).contour.points);
    CHECK(polygons[1].points == collection.surfaces[1].expolygon.contour.points);
    CHECK(polygons[2].points == collection.surfaces[1].expolygon.holes.front().points);
    CHECK(polygons[3].points == collection.surfaces[3].expolygon.contour.points);
}

TEST_CASE("SurfaceCollection keep and remove operations retain selected order", "[SurfaceCollection]")
{
    const Surfaces input {
        tagged_surface(stTop, 0), tagged_surface(stInternal, 1),
        tagged_surface(stTop, 2), tagged_surface(stBottom, 3),
        tagged_surface(stInternalSolid, 4), tagged_surface(stBottom, 5),
    };

    SurfaceCollection keep_one(input);
    keep_one.keep_type(stTop);
    CHECK(markers(keep_one) == std::vector<int> { 0, 2 });

    SurfaceCollection keep_many(input);
    keep_many.keep_types({ stBottom, stInternal });
    CHECK(markers(keep_many) == std::vector<int> { 1, 3, 5 });

    SurfaceCollection remove_one(input);
    remove_one.remove_type(stTop);
    CHECK(markers(remove_one) == std::vector<int> { 1, 3, 4, 5 });

    SurfaceCollection remove_many(input);
    remove_many.remove_types({ stTop, stBottom });
    CHECK(markers(remove_many) == std::vector<int> { 1, 4 });
}

TEST_CASE("SurfaceCollection remove-by-type appends moved polygons", "[SurfaceCollection]")
{
    SurfaceCollection collection(Surfaces {
        tagged_surface(stInternal, 1), tagged_surface(stTop, 2),
        tagged_surface(stInternal, 3), tagged_surface(stBottom, 4),
    });
    ExPolygons removed { rectangle(-20, -20, -10, -10) };

    collection.remove_type(stInternal, &removed);

    REQUIRE(removed.size() == 3);
    CHECK(removed[0].contour.points == rectangle(-20, -20, -10, -10).contour.points);
    CHECK(removed[1].contour.points == tagged_surface(stInternal, 1).expolygon.contour.points);
    CHECK(removed[2].contour.points == tagged_surface(stInternal, 3).expolygon.contour.points);
    CHECK(markers(collection) == std::vector<int> { 2, 4 });
}

TEST_CASE("SurfaceCollection SVG export colors surfaces and optionally labels them", "[SurfaceCollection]")
{
    SurfaceCollection collection(Surfaces {
        tagged_surface(stTop, 1), tagged_surface(stBottomBridge, 2),
    });
    ScopedTemporaryFile labeled_file(".svg");

    collection.export_to_svg(labeled_file.string().c_str(), true);

    const std::string labeled = read_text(labeled_file.string());
    const std::vector<std::string> labeled_paths = svg_paths(labeled);
    REQUIRE(labeled_paths.size() == 2);
    CHECK(labeled_paths[0].find("rgb(255,0,0)") != std::string::npos);
    CHECK(labeled_paths[1].find("rgb(0,0,255)") != std::string::npos);
    const size_t opacity_attribute = labeled_paths[0].find("fill-opacity=");
    REQUIRE(opacity_attribute != std::string::npos);
    const size_t opacity_start = labeled_paths[0].find('"', opacity_attribute);
    REQUIRE(opacity_start != std::string::npos);
    const size_t opacity_end = labeled_paths[0].find('"', opacity_start + 1);
    REQUIRE(opacity_end != std::string::npos);
    const double opacity = std::stod(labeled_paths[0].substr(opacity_start + 1, opacity_end - opacity_start - 1));
    CHECK_THAT(opacity, WithinAbs(0.5, 1e-6));
    CHECK(count_text(labeled, ">0</text>") == 1);
    CHECK(count_text(labeled, ">1</text>") == 1);

    ScopedTemporaryFile plain_file(".svg");
    collection.export_to_svg(plain_file.string().c_str(), false);
    const std::string plain = read_text(plain_file.string());
    CHECK(count_text(plain, "<path ") == 2);
    CHECK(count_text(plain, ">0</text>") == 0);
    CHECK(count_text(plain, ">1</text>") == 0);
}

TEST_CASE("Surface conversions preserve contour and hole order across overloads", "[SurfaceCollection]")
{
    Surface holed(stTop, rectangle_with_hole(-100, -100, 0, 0));
    holed.thickness = 0.35;
    holed.thickness_layers = 2;
    holed.bridge_angle = 0.25;
    holed.extra_perimeters = 4;
    Surface plain(stBottom, rectangle(20, 30, 60, 50));
    Surfaces owned { holed, plain };

    CHECK_THAT(holed.area(), WithinAbs(9100., 1e-6));
    CHECK_FALSE(holed.empty());
    const Polygons from_one = to_polygons(holed);
    REQUIRE(from_one.size() == 2);
    CHECK(from_one[0].points == holed.expolygon.contour.points);
    CHECK(from_one[1].points == holed.expolygon.holes.front().points);

    CHECK(number_polygons(owned) == 3);
    const Polygons from_owned = to_polygons(owned);
    REQUIRE(from_owned.size() == 3);
    CHECK(from_owned[0].points == owned[0].expolygon.contour.points);
    CHECK(from_owned[1].points == owned[0].expolygon.holes.front().points);
    CHECK(from_owned[2].points == owned[1].expolygon.contour.points);

    const SurfacesPtr reordered { &owned[1], &owned[0] };
    CHECK(number_polygons(reordered) == 3);
    const Polygons from_pointers = to_polygons(reordered);
    REQUIRE(from_pointers.size() == 3);
    CHECK(from_pointers[0].points == owned[1].expolygon.contour.points);
    CHECK(from_pointers[1].points == owned[0].expolygon.contour.points);
    CHECK(from_pointers[2].points == owned[0].expolygon.holes.front().points);

    Polygons appended { rectangle(300, 300, 310, 310).contour };
    polygons_append(appended, owned);
    REQUIRE(appended.size() == 4);
    CHECK(appended[0].points == rectangle(300, 300, 310, 310).contour.points);
    CHECK(appended[1].points == owned[0].expolygon.contour.points);
    CHECK(appended[2].points == owned[0].expolygon.holes.front().points);
    CHECK(appended[3].points == owned[1].expolygon.contour.points);

    Polygons pointer_append;
    polygons_append(pointer_append, reordered);
    REQUIRE(pointer_append.size() == 3);
    CHECK(pointer_append[0].points == owned[1].expolygon.contour.points);
    CHECK(pointer_append[1].points == owned[0].expolygon.contour.points);
    CHECK(pointer_append[2].points == owned[0].expolygon.holes.front().points);

    Surface rvalue_surface(stInternal, rectangle_with_hole(100, 100, 200, 200));
    const Polygons from_rvalue = to_polygons(std::move(rvalue_surface));
    REQUIRE(from_rvalue.size() == 2);
    CHECK(from_rvalue[0].points == rectangle_with_hole(100, 100, 200, 200).contour.points);
    CHECK(from_rvalue[1].points == rectangle_with_hole(100, 100, 200, 200).holes.front().points);

    Surfaces move_append_source { holed, plain };
    Polygons move_appended;
    polygons_append(move_appended, std::move(move_append_source));
    REQUIRE(move_appended.size() == 3);
    CHECK(move_appended[0].points == holed.expolygon.contour.points);
    CHECK(move_appended[1].points == holed.expolygon.holes.front().points);
    CHECK(move_appended[2].points == plain.expolygon.contour.points);

    const ExPolygons owned_copy = to_expolygons(owned);
    const ExPolygons pointer_copy = to_expolygons(reordered);
    REQUIRE(owned_copy.size() == 2);
    REQUIRE(pointer_copy.size() == 2);
    CHECK(owned_copy[0].holes.size() == 1);
    CHECK(pointer_copy[0].contour.points == owned[1].expolygon.contour.points);
    const Point copied_first_point = owned_copy[0].contour.points.front();
    owned[0].clear();
    CHECK(owned_copy[0].contour.points.front() == copied_first_point);
    CHECK(owned_copy[0].holes.size() == 1);

    Surfaces move_expolygon_source { holed, plain };
    const ExPolygons moved_expolygons = to_expolygons(std::move(move_expolygon_source));
    REQUIRE(move_expolygon_source.empty());
    REQUIRE(moved_expolygons.size() == 2);
    CHECK(moved_expolygons[0].holes.size() == 1);
    CHECK(moved_expolygons[1].contour.points == plain.expolygon.contour.points);

    holed.clear();
    CHECK(holed.empty());
    CHECK_THAT(holed.area(), WithinAbs(0., 1e-9));
}

TEST_CASE("Surface collections preserve geometry and metadata across set append and retag", "[SurfaceCollection]")
{
    const Surface source_surface = grouped_surface(stInternal, 7, 0.4, 2, 0.3);
    const Surface second_surface = grouped_surface(stTop, 2, 0.25, 1, -1.0);
    SurfaceCollection source(Surfaces { source_surface, second_surface });
    SurfaceCollection target(Surfaces { tagged_surface(stPerimeter, 0) });

    target.set(source);
    REQUIRE(target.size() == 2);
    CHECK(target.surfaces[0].extra_perimeters == 7);
    CHECK(target.surfaces[1].extra_perimeters == 2);
    CHECK(target.surfaces[0].expolygon.contour.points == source_surface.expolygon.contour.points);
    CHECK(target.surfaces[1].expolygon.contour.points == second_surface.expolygon.contour.points);
    CHECK_THAT(target.surfaces[0].thickness, WithinAbs(0.4, 1e-12));
    CHECK(target.surfaces[0].thickness_layers == 2);
    CHECK_THAT(target.surfaces[0].bridge_angle, WithinAbs(0.3, 1e-12));

    SurfaceCollection from_surfaces;
    from_surfaces.set(source.surfaces);
    REQUIRE(from_surfaces.size() == 2);
    CHECK_THAT(from_surfaces.surfaces[0].thickness, WithinAbs(0.4, 1e-12));
    CHECK(from_surfaces.surfaces[0].expolygon.contour.points == source_surface.expolygon.contour.points);

    const ExPolygons templated_input { source_surface.expolygon };
    SurfaceCollection from_template;
    from_template.set(templated_input, source_surface);
    REQUIRE(from_template.size() == 1);
    CHECK_THAT(from_template.surfaces.front().thickness, WithinAbs(0.4, 1e-12));
    CHECK(from_template.surfaces.front().expolygon.contour.points == source_surface.expolygon.contour.points);

    SurfaceCollection moved_target;
    SurfaceCollection moved_source(Surfaces { source_surface });
    moved_target.set(std::move(moved_source));
    REQUIRE(moved_target.size() == 1);
    CHECK_THAT(moved_target.surfaces.front().thickness, WithinAbs(0.4, 1e-12));
    CHECK(moved_target.surfaces.front().expolygon.contour.points == source_surface.expolygon.contour.points);

    target.append(source);
    REQUIRE(target.size() == 4);
    CHECK(target.surfaces[0].extra_perimeters == 7);
    CHECK(target.surfaces[1].extra_perimeters == 2);
    CHECK(target.surfaces[2].extra_perimeters == 7);
    CHECK(target.surfaces[3].extra_perimeters == 2);
    target.append(SurfaceCollection(Surfaces { source_surface }));
    REQUIRE(target.size() == 5);
    CHECK(target.surfaces[0].extra_perimeters == 7);
    CHECK(target.surfaces[1].extra_perimeters == 2);
    CHECK(target.surfaces[2].extra_perimeters == 7);
    CHECK(target.surfaces[3].extra_perimeters == 2);
    CHECK(target.surfaces[4].extra_perimeters == 7);
    target.change_to_new_type(stInternal, stBottom);
    CHECK(target.has(stBottom));
    CHECK_FALSE(target.has(stInternal));
    CHECK(target.has(stTop));
    CHECK(target.surfaces[0].expolygon.contour.points == source_surface.expolygon.contour.points);
    target.set_type(stPerimeter);
    CHECK(target.has(stPerimeter));
    CHECK_FALSE(target.has(stTop));
    CHECK_FALSE(target.has(stBottom));
    CHECK_THAT(target.surfaces[0].thickness, WithinAbs(0.4, 1e-12));

    const ExPolygons additions { rectangle(-60, -40, -30, -20), rectangle(100, 110, 140, 150) };
    const Surface metadata = grouped_surface(stTop, 9, 0.6, 3, 0.5);
    target.append(additions, metadata);
    REQUIRE(target.size() == 7);
    CHECK(target.surfaces[5].surface_type == stTop);
    CHECK(target.surfaces[5].expolygon.contour.points == additions[0].contour.points);
    CHECK_THAT(target.surfaces[5].thickness, WithinAbs(0.6, 1e-12));
    CHECK(target.surfaces[5].thickness_layers == 3);
    CHECK_THAT(target.surfaces[5].bridge_angle, WithinAbs(0.5, 1e-12));
    CHECK(target.surfaces[5].extra_perimeters == metadata.extra_perimeters);
    CHECK(target.has(stTop));

    const SurfaceCollection &view = target;
    CHECK(std::distance(view.cbegin(), view.cend()) == 7);
    CHECK(std::distance(view.begin(), view.end()) == 7);
    CHECK(std::distance(target.begin(), target.end()) == 7);

    ExPolygons replacement { rectangle(-10, -20, 5, 15) };
    target.set(std::move(replacement), stBottom);
    REQUIRE(replacement.empty());
    REQUIRE(target.size() == 1);
    CHECK(target.has(stBottom));
    CHECK(target.surfaces.front().expolygon.contour.points == rectangle(-10, -20, 5, 15).contour.points);
    CHECK_FALSE(target.has(stPerimeter));
    target.clear();
    CHECK(target.empty());
}

TEST_CASE("Surface extents include negative and disjoint contours and define empty results", "[SurfaceCollection]")
{
    const Surfaces surfaces {
        Surface(stTop, rectangle(-60, -40, -30, -10)),
        Surface(stBottom, rectangle(50, 20, 80, 70)),
    };
    const BoundingBox owned_bounds = get_extents(surfaces);
    REQUIRE(owned_bounds.defined);
    CHECK(owned_bounds.min.x() == -60);
    CHECK(owned_bounds.min.y() == -40);
    CHECK(owned_bounds.max.x() == 80);
    CHECK(owned_bounds.max.y() == 70);

    const SurfacesPtr pointers { &surfaces[1], &surfaces[0] };
    const BoundingBox pointer_bounds = get_extents(pointers);
    REQUIRE(pointer_bounds.defined);
    CHECK(pointer_bounds.min.x() == -60);
    CHECK(pointer_bounds.min.y() == -40);
    CHECK(pointer_bounds.max.x() == 80);
    CHECK(pointer_bounds.max.y() == 70);

    const BoundingBox empty_owned = get_extents(Surfaces {});
    CHECK_FALSE(empty_owned.defined);
    const BoundingBox empty_pointers = get_extents(SurfacesPtr {});
    CHECK_FALSE(empty_pointers.defined);
}

TEST_CASE("Surface SVG export preserves holes colors and requested opacity", "[SurfaceCollection]")
{
    const Surfaces surfaces {
        Surface(stTop, rectangle_with_hole(-100, -50, 100, 50)),
        Surface(stInternalSolid, rectangle(200, 20, 260, 80)),
    };
    ScopedTemporaryFile output(".svg");

    REQUIRE(export_to_svg(output.string().c_str(), surfaces, 0.35f));

    const std::string text = read_text(output.string());
    const std::vector<std::string> paths = svg_paths(text);
    REQUIRE(paths.size() == 2);
    CHECK(paths[0].find("rgb(255,0,0)") != std::string::npos);
    CHECK(paths[1].find("rgb(255,0,255)") != std::string::npos);
    CHECK(count_text(paths[0], "M ") == 2);
    CHECK(count_text(paths[1], "M ") == 1);
    for (const std::string &path : paths) {
        const std::string attribute = "fill-opacity=\"";
        const size_t value_start = path.find(attribute);
        REQUIRE(value_start != std::string::npos);
        const size_t value_end = path.find('"', value_start + attribute.size());
        REQUIRE(value_end != std::string::npos);
        const double opacity = std::stod(path.substr(value_start + attribute.size(), value_end - value_start - attribute.size()));
        CHECK_THAT(opacity, WithinAbs(0.35, 1e-6));
    }
}
