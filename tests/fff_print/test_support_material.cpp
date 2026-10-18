#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/Surface.hpp"
#include "libslic3r/SurfaceCollection.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Geometry.hpp"
#include "libslic3r/Point.hpp"
#include <initializer_list>
#include <limits>
#include <algorithm>
#include <catch2/catch_all.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_message.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/generators/catch_generators.hpp>
#include "libslic3r/GCodeReader.hpp"
#include "libslic3r/GCode/ToolOrdering.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/Polyline.hpp"

#include <cmath>
#include <cstddef>
#include <cctype>
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/PrintBase.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/libslic3r.h"
#include "libslic3r/PrintConfig.hpp"
#include <map>
#include <math.h>
#include <mutex>
#include <set>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "test_helpers.hpp" // get access to init_print, etc

// Not self-contained: its inline constructor uses PrintObject, PrintRegion, SlicingParameters and
// Geometry, so it must follow the headers (pulled in via test_helpers.hpp) that define them.
#include "libslic3r/Support/SupportParameters.hpp"
#include "libslic3r/Config.hpp"
#include "libslic3r/Print.hpp"

using namespace Slic3r::Test;
using namespace Slic3r;

// Distinct layer Z heights carrying support interface extrusion.
static size_t support_interface_layer_count(const std::string &gcode)
{
    return layers_with_role(gcode, "support material interface").size();
}

// Distinct layer Z heights carrying support base extrusion. The base G-code label "support material"
// is a substring of "support material interface", so a base line is a support line that is not an
// interface line.
static size_t support_base_layer_count(const std::string &gcode)
{
    std::set<double> layers;
    GCodeReader parser;
    parser.parse_buffer(gcode, [&layers](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        if (! line.extruding(self)) return;
        const std::string_view comment = line.comment();
        if (comment.find("support material") != std::string_view::npos &&
            comment.find("interface") == std::string_view::npos)
            layers.insert(self.z());
    });
    return layers.size();
}

// Dominant support-interface fill direction per interface layer, in radians [0, pi). Uses the
// length-weighted axial mean (each segment angle doubled so a line and its reverse agree, then
// halved): the parallel infill lines reinforce while the surrounding perimeter cancels.
static std::map<double, double> interface_fill_angle_by_layer(const std::string &gcode)
{
    std::map<double, std::pair<double, double>> acc; // z -> summed length*(cos2a, sin2a)
    GCodeReader parser;
    parser.parse_buffer(gcode, [&acc](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        if (! line.extruding(self)) return;
        if (line.comment().find("support material interface") == std::string_view::npos) return;
        const double dx = line.dist_X(self), dy = line.dist_Y(self);
        const double len = std::hypot(dx, dy);
        if (len < 1e-6) return;
        const double a2 = 2.0 * std::atan2(dy, dx);
        auto &p = acc[self.z()];
        p.first  += len * std::cos(a2);
        p.second += len * std::sin(a2);
    });
    std::map<double, double> out;
    for (const auto &kv : acc) {
        double a = 0.5 * std::atan2(kv.second.second, kv.second.first);
        if (a < 0) a += M_PI;
        out[kv.first] = a;
    }
    return out;
}

// Acute angle (degrees) between two axial fill directions in [0, pi).
static double axial_angle_diff_deg(double a, double b)
{
    const double d = std::fmod(std::fabs(a - b), M_PI);
    return std::min(d, M_PI - d) * 180.0 / M_PI;
}

// Denser interface spacing yields more extruded length.
static double support_interface_extrusion_length(const std::string &gcode)
{
    double len = 0;
    GCodeReader parser;
    parser.parse_buffer(gcode, [&len](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        if (! line.extruding(self)) return;
        if (line.comment().find("support material interface") == std::string_view::npos) return;
        len += std::hypot(line.dist_X(self), line.dist_Y(self));
    });
    return len;
}

// A cap slab overhanging a base, joined by a central stem: the cap can only be supported by resting on the
// base, forcing a genuine bottom contact. A horizontal tunnel does not work here -- tree/organic can arch a
// branch in from the opening and avoid the floor entirely.
static TriangleMesh support_capital()
{
    TriangleMesh model = make_cube(40, 40, 2);                              // base  [0,40]x[0,40]x[0,2]
    TriangleMesh stem  = make_cube(8, 8, 12);   stem.translate(16, 16, 1);  // stem  centered, z 1..13
    TriangleMesh cap   = make_cube(40, 40, 2);  cap.translate(0, 0, 12);    // cap   z 12..14
    model.merge(stem);
    model.merge(cap);
    return model;
}


TEST_CASE("Three raft layers are created", "[SupportMaterial]")
{
	Slic3r::Print print;
	Slic3r::Test::init_and_process_print({ cube(20) }, print, {
        { "enable_support", 1 },
        { "raft_layers",    3 }
		});
    REQUIRE(print.objects().front()->support_layers().size() == 3);
}

TEST_CASE("Enforced support layers are generated", "[SupportMaterial]")
{
    // enforce_support_layers forces support on the first N layers even with support off.
    Slic3r::Print baseline;
    Slic3r::Test::init_and_process_print({ TestMesh::overhang }, baseline, {
        { "enable_support",         0 },
        { "enforce_support_layers", 0 }
    });
    REQUIRE(baseline.objects().front()->support_layers().empty());

    Slic3r::Print enforced;
    Slic3r::Test::init_and_process_print({ TestMesh::overhang }, enforced, {
        { "enable_support",         0 },
        { "enforce_support_layers", 100 }
    });
    REQUIRE(enforced.objects().front()->support_layers().size() > 0);
}

// Support-needed statuses raised while slicing support_capital() with support off. The CLI lists these
// in result.json and fails on them under --strict. Collected under a lock: generate_support_material()
// runs on TBB workers.
static std::vector<PrintBase::SlicingStatus> support_needed_statuses(bool no_check)
{
    Slic3r::Print print;
    Slic3r::Model model;
    Slic3r::Test::init_print({ support_capital() }, print, model, {
        { "enable_support",         0 },
        { "enforce_support_layers", 0 }
    });
    print.set_no_check_flag(no_check);

    std::mutex mutex;
    std::vector<PrintBase::SlicingStatus> statuses;
    print.set_status_callback([&mutex, &statuses](const PrintBase::SlicingStatus &status) {
        if (status.message_type != PrintStateBase::SlicingNeedSupportOn)
            return;
        std::lock_guard<std::mutex> lock(mutex);
        statuses.push_back(status);
    });
    print.process();
    return statuses;
}

TEST_CASE("An overhang sliced with support off reports that support is needed", "[SupportMaterial][Parallel]")
{
    // The 40mm cap reaches ~22mm past its 8mm stem, beyond the 6mm cantilever limit of
    // PrintObject::is_support_necessary().
    const std::vector<PrintBase::SlicingStatus> statuses = support_needed_statuses(false);
    REQUIRE(! statuses.empty());
    for (const PrintBase::SlicingStatus &status : statuses) {
        // The CLI only considers step warnings (warning_step != -1), and --strict only NON_CRITICAL ones.
        CHECK(status.warning_level == PrintStateBase::WarningLevel::NON_CRITICAL);
        CHECK(status.warning_step != -1);
    }
}

TEST_CASE("The no-check flag skips the support-needed check", "[SupportMaterial][Parallel]")
{
    CHECK(support_needed_statuses(true).empty());
}

SCENARIO("Support layer Z honors contact distance", "[SupportMaterial]")
{
    // Box h = 20mm, hole bottom at 5mm, hole height 10mm (top edge at 15mm).
    TriangleMesh mesh = Slic3r::Test::mesh(Slic3r::Test::TestMesh::cube_with_hole);
    mesh.rotate_x(float(M_PI / 2));

	auto check = [](Slic3r::Print &print, bool &first_support_layer_height_ok, bool &layer_height_minimum_ok, bool &layer_height_maximum_ok)
	{
        ConstSupportLayerPtrsAdaptor support_layers = print.objects().front()->support_layers();

		first_support_layer_height_ok = support_layers.front()->print_z == print.config().initial_layer_print_height.value;

		layer_height_minimum_ok = true;
		layer_height_maximum_ok = true;
		double min_layer_height = print.config().min_layer_height.values.front();
		double max_layer_height = print.config().nozzle_diameter.values.front();
		if (print.config().max_layer_height.values.front() > EPSILON)
			max_layer_height = std::min(max_layer_height, print.config().max_layer_height.values.front());
		for (size_t i = 1; i < support_layers.size(); ++ i) {
			if (support_layers[i]->print_z - support_layers[i - 1]->print_z < min_layer_height - EPSILON)
				layer_height_minimum_ok = false;
			if (support_layers[i]->print_z - support_layers[i - 1]->print_z > max_layer_height + EPSILON)
				layer_height_maximum_ok = false;
		}
	};

    GIVEN("A print object having one modelObject") {
        WHEN("Layer height = 0.2 and first layer height = 0.4") {
			Slic3r::Print print;
			Slic3r::Test::init_and_process_print({ mesh }, print, {
                { "enable_support",             1 },
                { "layer_height",               0.2 },
                { "initial_layer_print_height", 0.4 },
                { "dont_support_bridges",       false },
			});
			bool first_layer_ok, layer_min_ok, layer_max_ok;
            check(print, first_layer_ok, layer_min_ok, layer_max_ok);
            THEN("First layer height is honored")			{ REQUIRE(first_layer_ok == true); }
            THEN("No null or negative support layers")		{ REQUIRE(layer_min_ok == true); }
            THEN("No layers thicker than nozzle diameter")	{ REQUIRE(layer_max_ok == true); }
        }
        WHEN("Layer height = 0.2 and first layer height = 0.3") {
			Slic3r::Print print;
			Slic3r::Test::init_and_process_print({ mesh }, print, {
                { "enable_support",             1 },
                { "layer_height",               0.2 },
                { "initial_layer_print_height", 0.3 },
                { "dont_support_bridges",       false },
            });
            bool first_layer_ok, layer_min_ok, layer_max_ok;
            check(print, first_layer_ok, layer_min_ok, layer_max_ok);
            THEN("First layer height is honored")			{ REQUIRE(first_layer_ok == true); }
            THEN("No null or negative support layers")		{ REQUIRE(layer_min_ok == true); }
            THEN("No layers thicker than nozzle diameter")	{ REQUIRE(layer_max_ok == true); }
        }
    }
}

// extrude_support once held a `static` lambda capturing `this`, so a second export in the
// same process dereferenced a returned stack frame (ASan: stack-use-after-return).
TEST_CASE("Support G-code emission survives a second slice in the same process", "[SupportMaterial][Regression]")
{
    const std::string first = slice({ TestMesh::overhang }, { { "enable_support", 1 } });
    REQUIRE(! layers_with_role(first, "support").empty());

    const std::string second = slice({ TestMesh::overhang }, { { "enable_support", 1 } });
    REQUIRE(! layers_with_role(second, "support").empty());
}

// The contact layer counts toward the configured interface layer count, so N configured top
// interface layers produce exactly N interface layers, not N+1.
TEST_CASE("Support top interface layer count matches the configured value", "[SupportMaterial]")
{
    const int top = GENERATE(1, 2, 3, 4, 6);
    const std::string g = slice({ TestMesh::overhang }, {
        { "enable_support",                  1 },
        { "layer_height",                    0.2 },
        { "support_on_build_plate_only",     1 },
        { "support_interface_top_layers",    top },
        { "support_interface_bottom_layers", 0 },
    });
    CAPTURE(top);
    REQUIRE(support_base_layer_count(g)      > 0);          // support actually formed
    REQUIRE(support_interface_layer_count(g) == size_t(top));
}

// A rotated cube-with-hole is a horizontal tunnel whose ceiling and floor both receive support, so top
// and bottom interfaces can be exercised independently (the floor is the bottom contact).
static TriangleMesh support_tunnel()
{
    TriangleMesh tunnel = Slic3r::Test::mesh(TestMesh::cube_with_hole);
    tunnel.rotate_x(float(M_PI / 2));
    return tunnel;
}

static size_t tunnel_interface_layers(const TriangleMesh &tunnel, int top, int bottom)
{
    const std::string g = slice({ tunnel }, {
        { "enable_support",                  1 },
        { "layer_height",                    0.2 },
        { "support_on_build_plate_only",     0 },
        { "support_interface_top_layers",    top },
        { "support_interface_bottom_layers", bottom },
    });
    REQUIRE(support_base_layer_count(g) > 0); // support actually formed
    return support_interface_layer_count(g);
}

TEST_CASE("No support interface is generated when neither top nor bottom is configured", "[SupportMaterial]")
{
    REQUIRE(tunnel_interface_layers(support_tunnel(), 0, 0) == 0);
}

TEST_CASE("Bottom interface layer count matches its setting with top interface off", "[SupportMaterial]")
{
    const int bottom = GENERATE(1, 3, 6);
    CAPTURE(bottom);
    REQUIRE(tunnel_interface_layers(support_tunnel(), 0, bottom) == size_t(bottom));
}

// support_interface_bottom_layers = -1 means "same as top".
TEST_CASE("Support interface bottom layers default to the top layer count", "[SupportMaterial]")
{
    const TriangleMesh tunnel = support_tunnel();
    REQUIRE(tunnel_interface_layers(tunnel, 0, -1) == tunnel_interface_layers(tunnel, 0, 0));
    REQUIRE(tunnel_interface_layers(tunnel, 3, -1) == tunnel_interface_layers(tunnel, 3, 3));
}

TEST_CASE("Default support still emits base and interface material", "[SupportMaterial][Regression]")
{
    const std::string g = slice({ TestMesh::overhang }, { { "enable_support", 1 } });
    REQUIRE(support_base_layer_count(g)      > 0);
    REQUIRE(support_interface_layer_count(g) > 0);
}

// Organic runs TreeSupport3D + TreeModelVolumes, the others the classic TreeSupport.cpp path.
TEST_CASE("Every tree support style produces base and interface material", "[SupportMaterial]")
{
    const char *style = GENERATE("organic", "tree_slim", "tree_strong", "tree_hybrid");
    INFO("style=" << style);
    const std::string g = slice({ TestMesh::overhang }, {
        { "enable_support",               1 },
        { "layer_height",                 0.2 },
        { "support_type",                 "tree(auto)" },
        { "support_style",                style },
        { "support_interface_top_layers", 3 },
    });
    CHECK(support_base_layer_count(g)      > 0);
    CHECK(support_interface_layer_count(g) > 0);
}

TEST_CASE("Raft interface angle alternates by 45 degrees per interface id", "[SupportMaterial]")
{
    Slic3r::Print print;
    Slic3r::Test::init_and_process_print({ TestMesh::overhang }, print, { { "enable_support", 1 } });
    SupportParameters sp(*print.objects().front());
    sp.raft_angle_interface = 0.5f;
    REQUIRE_THAT(sp.raft_interface_angle(0), Catch::Matchers::WithinAbs(0.5 + M_PI / 4., 1e-6));
    REQUIRE_THAT(sp.raft_interface_angle(1), Catch::Matchers::WithinAbs(0.5 - M_PI / 4., 1e-6));
}

// The angle inputs are overwritten directly, so the pattern-to-angle mapping is checked
// independently of the sliced object's configuration.
TEST_CASE("Support interface fill angle follows the configured interface pattern", "[SupportMaterial]")
{
    Slic3r::Print print;
    Slic3r::Test::init_and_process_print({ TestMesh::overhang }, print, { { "enable_support", 1 } });
    SupportParameters sp(*print.objects().front());
    sp.interface_angle = 0.3f;
    sp.base_angle      = 1.1f;
    const double tol   = 1e-6;

    SECTION("Rectilinear shifts the interface angle by -45deg for snug support") {
        sp.support_interface_pattern = smipRectilinear;
        sp.support_style             = smsSnug;
        REQUIRE_THAT(sp.support_interface_angle(0), Catch::Matchers::WithinAbs(sp.interface_angle - M_PI_4, tol));
        REQUIRE_THAT(sp.support_interface_angle(3), Catch::Matchers::WithinAbs(sp.interface_angle - M_PI_4, tol));
    }
    SECTION("Rectilinear leaves the interface angle alone for the other styles") {
        sp.support_interface_pattern = smipRectilinear;
        sp.support_style             = smsGrid;
        REQUIRE_THAT(sp.support_interface_angle(0), Catch::Matchers::WithinAbs(sp.interface_angle, tol));
    }
    SECTION("Rectilinear interlaced alternates -/+45deg by interface id parity") {
        sp.support_interface_pattern = smipRectilinearInterlaced;
        REQUIRE_THAT(sp.support_interface_angle(0), Catch::Matchers::WithinAbs(sp.interface_angle - M_PI_4, tol));
        REQUIRE_THAT(sp.support_interface_angle(1), Catch::Matchers::WithinAbs(sp.interface_angle + M_PI_4, tol));
    }
    SECTION("Grid uses the base angle") {
        sp.support_interface_pattern = smipGrid;
        REQUIRE_THAT(sp.support_interface_angle(0), Catch::Matchers::WithinAbs(sp.base_angle, tol));
    }
    SECTION("Auto and concentric use the interface angle unchanged") {
        sp.support_interface_pattern = smipAuto;
        REQUIRE_THAT(sp.support_interface_angle(0), Catch::Matchers::WithinAbs(sp.interface_angle, tol));
        sp.support_interface_pattern = smipConcentric;
        REQUIRE_THAT(sp.support_interface_angle(0), Catch::Matchers::WithinAbs(sp.interface_angle, tol));
    }
}

// End-to-end that the pattern reaches the emitted fill, not just support_interface_angle().
TEST_CASE("Interlaced support interface alternates fill angle while rectilinear does not", "[SupportMaterial]")
{
    auto interface_angles = [](const char *pattern) {
        std::vector<double> a;
        for (const auto &kv : interface_fill_angle_by_layer(slice({ TestMesh::overhang }, {
                 { "enable_support",               1 },
                 { "layer_height",                 0.2 },
                 { "support_on_build_plate_only",  1 },
                 { "support_interface_top_layers", 6 },
                 { "support_interface_pattern",    pattern } })))
            a.push_back(kv.second);
        return a;
    };

    const std::vector<double> rectilinear = interface_angles("rectilinear");
    const std::vector<double> interlaced  = interface_angles("rectilinear_interlaced");
    REQUIRE(rectilinear.size() >= 3);
    REQUIRE(interlaced.size()  >= 3);

    for (size_t i = 1; i < rectilinear.size(); ++i)
        REQUIRE(axial_angle_diff_deg(rectilinear[i], rectilinear[0]) < 15.0);

    for (size_t i = 1; i < interlaced.size(); ++i)
        REQUIRE(axial_angle_diff_deg(interlaced[i], interlaced[i - 1]) > 60.0);
}

// Normal and non-organic tree support share the same interface angle logic: with a rectilinear interface
// pattern both emit their interface fill at the same angle (both go through support_interface_angle()).
TEST_CASE("Normal and tree support use the same interface fill angle", "[SupportMaterial]")
{
    auto mean_interface_angle = [](const char *type, const char *style) {
        const auto angles = interface_fill_angle_by_layer(slice({ TestMesh::overhang }, {
            { "enable_support", 1 }, { "layer_height", 0.2 }, { "support_on_build_plate_only", 1 },
            { "support_type", type }, { "support_style", style },
            { "support_interface_top_layers", 6 }, { "support_interface_pattern", "rectilinear" } }));
        REQUIRE(angles.size() >= 3);
        // Axial mean, as in interface_fill_angle_by_layer: a plain mean would split angles either
        // side of the [0, pi) wrap.
        double x = 0, y = 0;
        for (const auto &kv : angles) {
            x += std::cos(2.0 * kv.second);
            y += std::sin(2.0 * kv.second);
        }
        double mean = 0.5 * std::atan2(y, x);
        if (mean < 0) mean += M_PI;
        return mean;
    };
    REQUIRE(axial_angle_diff_deg(mean_interface_angle("normal(auto)", "default"),
                                 mean_interface_angle("tree(auto)", "tree_slim")) < 10.0);
}

// Every style, because the non-organic tree styles once emitted one more top interface layer than the rest.
TEST_CASE("Top interface layer count equals the configured value for every support style", "[SupportMaterial]")
{
    auto [type, style] = GENERATE(table<const char *, const char *>({
        { "normal(auto)", "grid" },        { "normal(auto)", "snug" },
        { "tree(auto)",   "organic" },     { "tree(auto)",   "tree_slim" },
        { "tree(auto)",   "tree_strong" }, { "tree(auto)",   "tree_hybrid" },
    }));
    CAPTURE(style);
    const std::string g = slice({ TestMesh::overhang }, {
        { "enable_support",               1 },
        { "layer_height",                 0.2 },
        { "support_type",                 type },
        { "support_style",                style },
        { "support_interface_top_layers", 4 },
    });
    REQUIRE(support_interface_layer_count(g) == 4u);
}

// The bottom interface was dropped in earlier versions when support started on the model rather
// than the plate.
TEST_CASE("Non-organic tree support generates a bottom interface on internal geometry", "[SupportMaterial]")
{
    const std::string g = slice({ support_tunnel() }, {
        { "enable_support",                  1 },
        { "layer_height",                    0.2 },
        { "support_on_build_plate_only",     0 },
        { "support_type",                    "tree(auto)" },
        { "support_style",                   "tree_slim" },
        { "support_interface_top_layers",    0 },
        { "support_interface_bottom_layers", 6 },
    });
    REQUIRE(support_base_layer_count(g)      > 0);
    REQUIRE(support_interface_layer_count(g) > 0);
}

// The capital forces the model contact; on a horizontal tunnel organic can arch a branch in and make none.
TEST_CASE("A bottom interface is produced for every support style on a forced model contact", "[SupportMaterial]")
{
    auto [type, style] = GENERATE(table<const char *, const char *>({
        { "normal(auto)", "default" },     { "tree(auto)", "tree_slim" },
        { "tree(auto)",   "tree_strong" }, { "tree(auto)", "tree_hybrid" },
        { "tree(auto)",   "organic" },
    }));
    CAPTURE(style);
    REQUIRE(support_interface_layer_count(slice({ support_capital() }, {
        { "enable_support", 1 }, { "layer_height", 0.2 }, { "support_on_build_plate_only", 0 },
        { "support_type", type }, { "support_style", style },
        { "support_interface_top_layers", 0 }, { "support_interface_bottom_layers", 6 } })) > 0);
}

TEST_CASE("Bottom interface spacing controls bottom interface density for every support style", "[SupportMaterial]")
{
    auto [type, style] = GENERATE(table<const char *, const char *>({
        { "normal(auto)", "default" },     { "tree(auto)", "tree_slim" },
        { "tree(auto)",   "tree_strong" }, { "tree(auto)", "tree_hybrid" },
        { "tree(auto)",   "organic" },
    }));
    CAPTURE(style);
    const TriangleMesh model = support_capital();
    auto len = [&model](const char *support_type, const char *support_style, double spacing) {
        return support_interface_extrusion_length(slice({ model }, {
            { "enable_support", 1 }, { "layer_height", 0.2 }, { "support_on_build_plate_only", 0 },
            { "support_type", support_type }, { "support_style", support_style }, { "support_interface_top_layers", 0 },
            { "support_interface_bottom_layers", 6 }, { "support_bottom_interface_spacing", spacing } }));
    };
    REQUIRE(len(type, style, 0.0) > len(type, style, 4.0) * 1.5);
}

// Interface and base flows are identical in width and rate unless a separate support-interface
// filament is used, so density is the observable here, not flow.
TEST_CASE("Bottom-only support interface keeps the dense interface density", "[SupportMaterial]")
{
    Slic3r::Print print;
    Slic3r::Test::init_and_process_print({ TestMesh::overhang }, print, {
        { "enable_support",                   1 },
        { "support_interface_top_layers",     0 },
        { "support_interface_bottom_layers",  6 },
        { "support_bottom_interface_spacing", 0.0 },  // solid: density resolves to 1.0
        { "support_base_pattern_spacing",     2.5 },  // sparse: density stays below 1.0
    });
    SupportParameters sp(*print.objects().front());
    REQUIRE(sp.bottom_interface_density > sp.support_density);
}

namespace support_settings_variance_contracts {

namespace {

TriangleMesh capital_fixture()
{
    // The 24 mm cap leaves a 9 mm overhang around the 6 mm stem. At 0.2 mm
    // layer height, the 7 mm fixture supplies 35 layers and a distinct contact region.
    TriangleMesh result = make_cube(24.0, 24.0, 1.0);
    TriangleMesh stem = make_cube(6.0, 6.0, 6.0);
    stem.translate(9.0, 9.0, 1.0);
    TriangleMesh cap = make_cube(24.0, 24.0, 1.0);
    cap.translate(0.0, 0.0, 6.0);
    result.merge(stem);
    result.merge(cap);
    return result;
}

TriangleMesh tunnel_fixture()
{
    TriangleMesh tunnel = mesh(TestMesh::cube_with_hole);
    tunnel.rotate_x(float(Geometry::deg2rad(90.0)));
    return tunnel;
}

struct SupportStats {
    std::size_t base_segments = 0;
    std::size_t interface_segments = 0;
    std::set<double> base_layers;
    std::set<double> interface_layers;
    std::set<int> support_tools;
    double base_length = 0.0;
    double interface_length = 0.0;
    double base_min_z = std::numeric_limits<double>::infinity();
    double base_max_z = -std::numeric_limits<double>::infinity();
    double interface_min_z = std::numeric_limits<double>::infinity();
    double interface_max_z = -std::numeric_limits<double>::infinity();
    std::set<int> base_direction_bins;
    std::map<double, double> base_direction_by_layer;
    bool finite = true;
};

constexpr std::size_t kMaxGCodeBytes = 16u * 1024u * 1024u;
constexpr std::size_t kMaxGCodeLines = 250000u;
constexpr std::size_t kMaxSupportSegments = 250000u;

std::size_t count_occurrences(const std::string &text, const std::string_view needle)
{
    std::size_t count = 0;
    std::size_t pos = 0;
    while ((pos = text.find(needle, pos)) != std::string::npos) {
        ++count;
        pos += needle.size();
    }
    return count;
}

SupportStats inspect_support(const std::string &gcode)
{
    // Guard the parser before it sees any caller-produced text. These limits are deliberately
    // generous for the small capital/tunnel fixtures, while bounding both parser input and the
    // support-role work performed below.
    REQUIRE(gcode.size() <= kMaxGCodeBytes);
    REQUIRE(std::count(gcode.begin(), gcode.end(), '\n') < kMaxGCodeLines);
    REQUIRE(count_occurrences(gcode, "support material") < kMaxSupportSegments);

    SupportStats out;
    std::map<double, std::pair<double, double>> base_direction_accumulator;
    GCodeReader parser;
    int active_tool = -1;
    parser.parse_buffer(gcode, [&out, &active_tool, &base_direction_accumulator](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        const std::string_view command = line.cmd();
        if (command.size() >= 2 && command.front() == 'T' && command[1] >= '0' && command[1] <= '9')
            active_tool = command[1] - '0';
        if (!line.extruding(self))
            return;
        const std::string_view comment = line.comment();
        const bool interface_line = comment.find("support material interface") != std::string_view::npos;
        const bool base_line = !interface_line && comment.find("support material") != std::string_view::npos;
        if (!interface_line && !base_line)
            return;
        const double dx = line.dist_X(self);
        const double dy = line.dist_Y(self);
        const double length = std::hypot(dx, dy);
        const double z = self.z();
        out.finite = out.finite && std::isfinite(length) && std::isfinite(z);
        if (!std::isfinite(length) || !std::isfinite(z))
            return;
        if (base_line && length > 1e-6) {
            double angle = std::fmod(std::atan2(dy, dx), M_PI);
            if (angle < 0.0)
                angle += M_PI;
            const int direction_bin = std::min(11, static_cast<int>(angle / M_PI * 12.0));
            out.base_direction_bins.insert(direction_bin);
            const double axial_angle = 2.0 * std::atan2(dy, dx);
            auto &accumulator = base_direction_accumulator[z];
            accumulator.first += length * std::cos(axial_angle);
            accumulator.second += length * std::sin(axial_angle);
        }
        if (active_tool >= 0)
            out.support_tools.insert(active_tool);
        if (interface_line) {
            ++out.interface_segments;
            out.interface_length += length;
            out.interface_layers.insert(z);
            out.interface_min_z = std::min(out.interface_min_z, z);
            out.interface_max_z = std::max(out.interface_max_z, z);
        } else {
            ++out.base_segments;
            out.base_length += length;
            out.base_layers.insert(z);
            out.base_min_z = std::min(out.base_min_z, z);
            out.base_max_z = std::max(out.base_max_z, z);
        }
    });
    for (const auto &entry : base_direction_accumulator) {
        double angle = 0.5 * std::atan2(entry.second.second, entry.second.first);
        if (angle < 0.0)
            angle += M_PI;
        out.base_direction_by_layer.emplace(entry.first, angle);
    }
    return out;
}

DynamicPrintConfig support_config()
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        {"enable_support", 1},
        {"layer_height", 0.2},
        {"support_on_build_plate_only", 0},
        {"support_interface_top_layers", 3},
        {"support_interface_bottom_layers", 0},
        {"support_base_pattern_spacing", 2.5},
        // "back" is the registered token for SeamPosition::spRear; it bypasses global
        // visibility raycasting while retaining the rear-seam path in SeamPlacer.
        {"seam_position", "back"},
        // init_print currently applies this too, but keep it explicit for the manual path.
        {"gcode_comments", 1},
        {"use_relative_e_distances", 1},
        // The validation message calls this layer_gcode; the registered config key is
        // layer_change_gcode and requires this exact uppercase command in relative-E mode.
        {"layer_change_gcode", "G92 E0\n"},
        {"skirt_loops", 0},
        {"brim_width", 0.0},
        {"raft_layers", 0}
    });
    return config;
}

std::string slice_checked(const TriangleMesh &mesh, const DynamicPrintConfig &config)
{
    Print print;
    Model model;
    init_print({mesh}, print, model, config);
    const StringObjectException validation = print.validate();
    INFO(validation.string);
    REQUIRE(validation.string.empty());
    return gcode(print);
}

std::string slice_capital(std::initializer_list<ConfigBase::SetDeserializeItem> items)
{
    DynamicPrintConfig config = support_config();
    config.set_deserialize_strict(items);
    return slice_checked(capital_fixture(), config);
}

std::string slice_tunnel(std::initializer_list<ConfigBase::SetDeserializeItem> items)
{
    DynamicPrintConfig config = support_config();
    config.set_deserialize_strict(items);
    return slice_checked(tunnel_fixture(), config);
}

enum class SupportFlowRole { none, base, interface, object };

struct ParsedSupportRoleMarker {
    bool recognized = false;
    SupportFlowRole role = SupportFlowRole::none;
};

// GCodeWriter emits role comments as either a direct canonical label or a TYPE:/FEATURE:
// marker. Only an anchored marker and an exact canonical payload can update state. This keeps
// arbitrary custom comments containing words such as "support material" from relabeling the next
// extrusion, and explicitly retires transition/ironing state before the next material role.
std::string_view trim_role_text(std::string_view text)
{
    while (!text.empty() && (text.front() == ' ' || text.front() == '\t'))
        text.remove_prefix(1);
    while (!text.empty() && (text.back() == ' ' || text.back() == '\t' || text.back() == '\r'))
        text.remove_suffix(1);
    return text;
}

ParsedSupportRoleMarker parse_support_role_marker(std::string_view comment)
{
    comment = trim_role_text(comment);
    bool tagged = false;
    for (const std::string_view prefix : {std::string_view("TYPE:"), std::string_view("FEATURE:")}) {
        if (comment.size() >= prefix.size() && comment.substr(0, prefix.size()) == prefix) {
            comment = trim_role_text(comment.substr(prefix.size()));
            tagged = true;
            break;
        }
    }

    SupportFlowRole role = SupportFlowRole::none;
    bool exact_role = false;
    if (comment == "support material" || comment == "Support" || comment == "Support material" || comment == "SupportMaterial") {
        role = SupportFlowRole::base;
        exact_role = true;
    } else if (comment == "support material interface" || comment == "Support material interface" ||
               comment == "SupportMaterialInterface" || comment == "Support interface") {
        role = SupportFlowRole::interface;
        exact_role = true;
    } else if (comment == "Outer wall" || comment == "External perimeter" || comment == "ExternalPerimeter") {
        role = SupportFlowRole::object;
        exact_role = true;
    } else if (comment == "support transition" || comment == "Support transition" ||
               comment == "SupportTransition" || comment == "support ironing" ||
               comment == "Support ironing" || comment == "SupportIroning") {
        // Explicitly recognized as none so a transition/ironing block cannot inherit support E.
        role = SupportFlowRole::none;
        exact_role = true;
    }
    return {tagged || exact_role, role};
}

struct SupportRoleFlowStats {
    double xy = 0.0;
    double e = 0.0;
    double max_pair_abs_e = 0.0;
    std::size_t moves = 0;
};

struct SupportFlowStats {
    SupportRoleFlowStats base;
    SupportRoleFlowStats interface;
    SupportRoleFlowStats object;
    bool finite = true;
};

SupportFlowStats inspect_support_flow(const std::string &gcode)
{
    REQUIRE(gcode.size() <= kMaxGCodeBytes);
    REQUIRE(std::count(gcode.begin(), gcode.end(), '\n') < kMaxGCodeLines);
    REQUIRE(count_occurrences(gcode, "support material") < kMaxSupportSegments);

    SupportFlowStats out;
    SupportFlowRole active_role = SupportFlowRole::none;
    GCodeReader parser;
    // Pin the reader to the same relative-E contract as support_config().
    parser.apply_config(support_config());
    parser.parse_buffer(gcode, [&out, &active_role](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        const ParsedSupportRoleMarker marker = parse_support_role_marker(line.comment());
        if (marker.recognized)
            active_role = marker.role;

        if (!line.extruding(self))
            return;
        const double xy = static_cast<double>(line.dist_XY(self));
        if (xy <= EPSILON)
            return;
        const double delta_e = static_cast<double>(line.dist_E(self));
        const double previous_e = static_cast<double>(self.e());
        const double current_e = static_cast<double>(line.e());
        const double max_pair_abs_e = std::max(std::abs(previous_e), std::abs(current_e));
        if (!std::isfinite(xy))
            out.finite = false;
        if (!std::isfinite(delta_e))
            out.finite = false;
        if (!std::isfinite(previous_e))
            out.finite = false;
        if (!std::isfinite(current_e))
            out.finite = false;
        if (!out.finite)
            return;

        SupportRoleFlowStats *role = nullptr;
        if (active_role == SupportFlowRole::base)
            role = &out.base;
        else if (active_role == SupportFlowRole::interface)
            role = &out.interface;
        else if (active_role == SupportFlowRole::object)
            role = &out.object;
        if (role == nullptr)
            return;
        role->xy += xy;
        role->e += delta_e;
        role->max_pair_abs_e = std::max(role->max_pair_abs_e, max_pair_abs_e);
        ++role->moves;
    });
    return out;
}

constexpr double kSupportEExportQuantum = 1e-5;

double support_e_pair_error_bound(double max_pair_abs_e)
{
    // E words are exported to five decimal places, then parsed as float; dist_E()
    // subtracts the current and previous parsed values.
    return kSupportEExportQuantum +
           4.0 * std::numeric_limits<float>::epsilon() * max_pair_abs_e;
}

double support_ratio_tolerance(const SupportRoleFlowStats &baseline,
                               const SupportRoleFlowStats &observed,
                               const double ratio)
{
    const double baseline_bound = static_cast<double>(baseline.moves) *
                                  support_e_pair_error_bound(baseline.max_pair_abs_e);
    const double observed_bound = static_cast<double>(observed.moves) *
                                  support_e_pair_error_bound(observed.max_pair_abs_e);
    const double expected = baseline.e * ratio;
    const double arithmetic_guard = 64.0 * std::numeric_limits<double>::epsilon() *
                                    std::max({1.0, std::abs(expected), std::abs(observed.e)});
    return observed_bound + std::abs(ratio) * baseline_bound + arithmetic_guard;
}

double support_equal_e_tolerance(const SupportRoleFlowStats &baseline,
                                  const SupportRoleFlowStats &observed)
{
    const double baseline_bound = static_cast<double>(baseline.moves) *
                                  support_e_pair_error_bound(baseline.max_pair_abs_e);
    const double observed_bound = static_cast<double>(observed.moves) *
                                  support_e_pair_error_bound(observed.max_pair_abs_e);
    const double arithmetic_guard = 64.0 * std::numeric_limits<double>::epsilon() *
                                    std::max({1.0, std::abs(baseline.e), std::abs(observed.e)});
    return baseline_bound + observed_bound + arithmetic_guard;
}

void require_support_geometry(const SupportRoleFlowStats &baseline,
                              const SupportRoleFlowStats &observed)
{
    REQUIRE(baseline.moves > 0);
    REQUIRE(observed.moves > 0);
    REQUIRE(observed.moves == baseline.moves);
    CHECK_THAT(observed.xy, Catch::Matchers::WithinAbs(baseline.xy, 1e-5));
}

void require_scaled_support_role(const SupportRoleFlowStats &baseline,
                                 const SupportRoleFlowStats &observed,
                                 const double ratio)
{
    require_support_geometry(baseline, observed);
    const double expected = baseline.e * ratio;
    const double tolerance = support_ratio_tolerance(baseline, observed, ratio);
    REQUIRE(std::isfinite(tolerance));
    REQUIRE(tolerance >= 0.0);
    const double signal = std::abs(expected - baseline.e);
    REQUIRE(signal > 0.0);
    // The analytic parser/export bound must not swallow the 25% signal.
    CHECK(tolerance < signal);
    CHECK_THAT(observed.e, Catch::Matchers::WithinAbs(expected, tolerance));
}

void require_unchanged_support_role(const SupportRoleFlowStats &baseline,
                                    const SupportRoleFlowStats &observed)
{
    require_support_geometry(baseline, observed);
    const double tolerance = support_equal_e_tolerance(baseline, observed);
    REQUIRE(std::isfinite(tolerance));
    REQUIRE(tolerance >= 0.0);
    CHECK_THAT(observed.e, Catch::Matchers::WithinAbs(baseline.e, tolerance));
}

void require_flow_option(const char *key, const ConfigOptionType expected_type)
{
    const ConfigOptionDef *definition = print_config_def.get(key);
    REQUIRE(definition != nullptr);
    REQUIRE(definition->type == expected_type);
    REQUIRE(definition->is_value_valid(1.0));
}

DynamicPrintConfig support_flow_ratio_config()
{
    // Fail before any slicing if a registered selector is renamed, changes type, or rejects the
    // neutral ratio. This prevents a silently ignored fixture setting from looking like a control.
    require_flow_option("support_flow_ratio", coFloat);
    require_flow_option("support_interface_flow_ratio", coFloat);
    require_flow_option("top_solid_infill_flow_ratio", coFloats);
    require_flow_option("bottom_solid_infill_flow_ratio", coFloat);
    require_flow_option("internal_bridge_flow", coFloat);

    DynamicPrintConfig config = support_config();
    config.set_deserialize_strict({
        {"gcode_flavor", "marlin"},
        {"support_type", "normal(auto)"},
        {"support_style", "default"},
        {"support_interface_top_layers", 2},
        {"support_interface_bottom_layers", 0},
        {"set_other_flow_ratios", true},
        {"first_layer_flow_ratio", 1.0},
        {"outer_wall_flow_ratio", 1.0},
        {"inner_wall_flow_ratio", 1.0},
        {"overhang_flow_ratio", 1.0},
        {"sparse_infill_flow_ratio", 1.0},
        {"internal_solid_infill_flow_ratio", 1.0},
        {"gap_fill_flow_ratio", 1.0},
        {"filament_flow_ratio", "1.0"},
        {"print_flow_ratio", 1.0},
        {"scarf_joint_flow_ratio", 1.0},
        // Pin the other production flow selectors explicitly; these are real registered keys,
        // including the top-surface vector and the internal-bridge scalar.
        {"top_solid_infill_flow_ratio", "1.0"},
        {"bottom_solid_infill_flow_ratio", 1.0},
        {"internal_bridge_flow", 1.0},
        {"support_flow_ratio", 1.0},
        {"support_interface_flow_ratio", 1.0}
    });
    return config;
}

std::string slice_manual(const char *support_type, bool with_enforcer)
{
    DynamicPrintConfig config = support_config();
    config.set_deserialize_strict({
        {"support_type", support_type},
        {"support_style", "default"},
        {"support_interface_top_layers", 2}
    });

    Model model;
    ModelObject *object = model.add_object();
    object->add_volume(capital_fixture());
    // Keep the volume types explicit while scaling their dimensions with the smaller capital.
    auto *blocker = object->add_volume(make_cube(3.0, 3.0, 3.0), ModelVolumeType::SUPPORT_BLOCKER);
    blocker->set_offset(Vec3d(4.0, 4.0, 0.0));
    if (with_enforcer) {
        auto *enforcer = object->add_volume(make_cube(4.0, 4.0, 2.0), ModelVolumeType::SUPPORT_ENFORCER);
        // The cap spans z=6..7; overlap it so enforcer polygons meet a sampled cap layer.
        enforcer->set_offset(Vec3d(0.0, 0.0, 5.5));
    }
    object->add_instance();
    object->ensure_on_bed();
    Print print;
    print.auto_assign_extruders(object);
    print.apply(model, config);
    const StringObjectException validation = print.validate();
    INFO(validation.string);
    REQUIRE(validation.string.empty());
    print.set_status_silent();
    return gcode(print);
}

std::size_t distinct_lengths(const std::vector<double> &values)
{
    std::vector<double> sorted = values;
    std::sort(sorted.begin(), sorted.end());
    sorted.erase(std::unique(sorted.begin(), sorted.end(), [](double a, double b) {
        return std::fabs(a - b) < 1e-3;
    }), sorted.end());
    return sorted.size();
}

// A single wall-supported cantilever keeps the unsupported contact footprint simply connected:
// the cap overhangs one 6 mm x 24 mm wall, leaving one 18 mm x 24 mm support rectangle. This is
// the fixture for the pattern topology test.
static TriangleMesh support_pattern_wall_capital()
{
    TriangleMesh result = make_cube(24.0, 24.0, 1.0);
    TriangleMesh wall = make_cube(6.0, 24.0, 6.0);
    wall.translate(0.0, 0.0, 1.0);
    TriangleMesh cap = make_cube(24.0, 24.0, 1.0);
    cap.translate(0.0, 0.0, 6.0);
    result.merge(wall);
    result.merge(cap);
    return result;
}

// Observe generated paths and use interface_id for the alternating-direction contract.
struct PublicInterfaceLayerObservation {
    size_t path_count = 0;
    double length = 0.0;
    double volume = 0.0;
    double long_horizontal_length = 0.0;
    double long_vertical_length = 0.0;
    std::pair<double, double> angle_accumulator{0.0, 0.0};
};

struct PublicInterfaceObservation {
    std::map<size_t, PublicInterfaceLayerObservation> by_interface_id;
    size_t interface_paths = 0;
    size_t open_paths = 0;
    size_t closed_paths = 0;
    double max_open_signed_turn = 0.0;
    bool finite = true;
};

static PublicInterfaceObservation inspect_public_support(const PrintObject &object)
{
    PublicInterfaceObservation out;
    for (const SupportLayer *layer : object.support_layers()) {
        PublicInterfaceLayerObservation &layer_out = out.by_interface_id[layer->interface_id()];
        for_each_extrusion_path(layer->support_fills, [&out, &layer_out](const ExtrusionPath &path) {
            if (path.role() != erSupportMaterialInterface)
                return;
            ++out.interface_paths;
            ++layer_out.path_count;
            const bool closed = path.is_closed();
            if (closed)
                ++out.closed_paths;
            else
                ++out.open_paths;

            double path_length = 0.0;
            double signed_turn = 0.0;
            double previous_angle = 0.0;
            bool have_previous_angle = false;
            for (size_t i = 1; i < path.polyline.points.size(); ++i) {
                const Point3 &a = path.polyline.points[i - 1];
                const Point3 &b = path.polyline.points[i];
                const double dx = unscale<double>(static_cast<double>(b.x()) - static_cast<double>(a.x()));
                const double dy = unscale<double>(static_cast<double>(b.y()) - static_cast<double>(a.y()));
                const double length = std::hypot(dx, dy);
                if (!std::isfinite(length) || !std::isfinite(dx) || !std::isfinite(dy)) {
                    out.finite = false;
                    continue;
                }
                path_length += length;
                if (!closed && length >= 1.0) {
                    double angle = std::fmod(std::atan2(dy, dx), M_PI);
                    if (angle < 0.0)
                        angle += M_PI;
                    if (axial_angle_diff_deg(angle, 0.0) < 10.0)
                        layer_out.long_horizontal_length += length;
                    if (axial_angle_diff_deg(angle, 0.5 * M_PI) < 10.0)
                        layer_out.long_vertical_length += length;
                    layer_out.angle_accumulator.first += length * std::cos(2.0 * angle);
                    layer_out.angle_accumulator.second += length * std::sin(2.0 * angle);
                }
                if (!closed && length > 1e-6) {
                    const double directed_angle = std::atan2(dy, dx);
                    if (have_previous_angle)
                        signed_turn += std::atan2(std::sin(directed_angle - previous_angle),
                                                  std::cos(directed_angle - previous_angle));
                    previous_angle = directed_angle;
                    have_previous_angle = true;
                }
            }
            layer_out.length += path_length;
            layer_out.volume += path_length * path.mm3_per_mm;
            if (!std::isfinite(path.mm3_per_mm) || path.mm3_per_mm <= 0.0)
                out.finite = false;
            out.max_open_signed_turn = std::max(out.max_open_signed_turn, std::abs(signed_turn));
            out.finite = out.finite && std::isfinite(path_length);
        });
    }
    return out;
}

static PublicInterfaceObservation process_pattern_fixture(const DynamicPrintConfig &config)
{
    Print print;
    Model model;
    init_print({support_pattern_wall_capital()}, print, model, config);
    const StringObjectException validation = print.validate();
    INFO(validation.string);
    REQUIRE(validation.string.empty());
    print.process();
    REQUIRE_FALSE(print.objects().empty());

    PublicInterfaceObservation out = inspect_public_support(*print.objects().front());
    REQUIRE(out.finite);
    REQUIRE(out.interface_paths > 0);
    return out;
}

} // namespace

TEST_CASE("Manual support modes respond only to an enforcer", "[SupportSettingsVariance][SupportMaterial]")
{
    for (const char *type : {"normal(manual)", "tree(manual)"}) {
        const SupportStats baseline = inspect_support(slice_manual(type, false));
        const SupportStats enforced = inspect_support(slice_manual(type, true));
        CAPTURE(type);
        REQUIRE(baseline.finite);
        REQUIRE(enforced.finite);
        CHECK(baseline.base_segments + baseline.interface_segments == 0);
        CHECK(enforced.base_segments + enforced.interface_segments > 0);
    }
}

TEST_CASE("Support base patterns emit finite material with distinct geometry", "[SupportSettingsVariance][SupportMaterial][TreeSupport]")
{
    const std::vector<const char *> patterns = {
        "default", "rectilinear", "rectilinear-grid", "honeycomb", "lightning", "hollow"
    };
    std::vector<double> base_lengths;
    for (const char *pattern : patterns) {
        const SupportStats stats = inspect_support(slice_capital({
            {"support_type", "tree(auto)"},
            {"support_style", "tree_hybrid"},
            {"support_interface_top_layers", 0},
            {"support_base_pattern", pattern}
        }));
        CAPTURE(pattern);
        REQUIRE(stats.finite);
        // `hollow` disables tree infill, but production still emits bounded perimeter/wall
        // support in this fixture; require support output while comparing signatures instead.
        CHECK(stats.base_segments + stats.interface_segments > 0);
        base_lengths.push_back(stats.base_length);
    }
    CHECK(distinct_lengths(base_lengths) >= 2);
}

TEST_CASE("Every support interface pattern emits its defining geometry", "[SupportInterfacePattern][SupportMaterial]")
{
    const std::vector<const char *> patterns = {
        "auto", "rectilinear", "concentric", "spiralinset", "rectilinear_interlaced", "grid"
    };
    std::map<std::string, PublicInterfaceObservation> observations;

    auto public_angles = [](const PublicInterfaceObservation &observation) {
        std::map<size_t, double> result;
        for (const auto &[interface_id, layer] : observation.by_interface_id) {
            const double magnitude = std::hypot(layer.angle_accumulator.first, layer.angle_accumulator.second);
            if (layer.path_count == 0 || magnitude <= 1e-9)
                continue;
            double angle = 0.5 * std::atan2(layer.angle_accumulator.second, layer.angle_accumulator.first);
            if (angle < 0.0)
                angle += M_PI;
            result.emplace(interface_id, angle);
        }
        return result;
    };

    // Only open, long paths participate in the direction-family witness.  Closed paths are the
    // nested-loop evidence for concentric and must not manufacture a second grid family.
    const auto has_orthogonal_open_families = [](const PublicInterfaceObservation &observation) {
        return std::any_of(observation.by_interface_id.begin(), observation.by_interface_id.end(),
            [](const auto &entry) {
                return entry.second.long_horizontal_length >= 2.0 && entry.second.long_vertical_length >= 2.0;
            });
    };

    for (const char *pattern : patterns) {
        CAPTURE(pattern);
        DynamicPrintConfig config = support_config();
        config.set_deserialize_strict({
            {"support_type",                 "normal(auto)"},
            {"support_style",                "default"},
            {"support_angle",                0.0},
            // Keep interface placement and contact-loop generation fixed while only the pattern
            // value changes.  These are registered production keys.
            {"support_top_z_distance",       0.2},
            {"support_interface_loop_pattern", 0},
            {"support_object_xy_distance",   0.0},
            {"support_interface_top_layers", 4},
            {"support_interface_spacing",    0.0},
            {"support_interface_pattern",    pattern}
        });
        PublicInterfaceObservation observation = process_pattern_fixture(config);
        size_t interface_ids_with_paths = 0;
        double deposited_length = 0.0;
        double deposited_volume = 0.0;
        for (const auto &[interface_id, layer] : observation.by_interface_id) {
            (void)interface_id;
            if (layer.path_count > 0)
                ++interface_ids_with_paths;
            deposited_length += layer.length;
            deposited_volume += layer.volume;
            REQUIRE(std::isfinite(layer.length));
        }
        REQUIRE(observation.finite);
        REQUIRE(observation.interface_paths > 0);
        REQUIRE(interface_ids_with_paths >= 3);
        REQUIRE(std::isfinite(deposited_length));
        REQUIRE(deposited_length > 0.0);
        REQUIRE(std::isfinite(deposited_volume));
        REQUIRE(deposited_volume > 0.0);
        observations.emplace(pattern, std::move(observation));
    }

    const auto auto_angles = public_angles(observations.at("auto"));
    const auto rect_angles = public_angles(observations.at("rectilinear"));
    REQUIRE(auto_angles.size() >= 3);
    REQUIRE(rect_angles.size() >= 3);
    std::size_t common_auto_ids = 0;
    for (const auto &[interface_id, angle] : auto_angles) {
        const auto rect_it = rect_angles.find(interface_id);
        if (rect_it == rect_angles.end())
            continue;
        ++common_auto_ids;
        CHECK(axial_angle_diff_deg(angle, rect_it->second) < 15.0);
    }
    REQUIRE(common_auto_ids >= 3);

    // Concentric is observed from the public extrusion topology rather than from every tiny
    // connector: its interface fill contains nested closed paths. The loop-pattern pin
    // keeps contact loops from becoming a second source of closed paths.
    REQUIRE(observations.at("concentric").closed_paths >= 2);
    CHECK(observations.at("concentric").open_paths == 0);

    // Rectilinear has one open long family; grid has two.  Restricting this witness to public
    // interface entities and open segments avoids counting the surrounding support boundary.
    CHECK_FALSE(has_orthogonal_open_families(observations.at("rectilinear")));
    REQUIRE(has_orthogonal_open_families(observations.at("grid")));
    CHECK(observations.at("rectilinear").max_open_signed_turn <= 3.0 * M_PI);

    // FillSpiralInset deliberately chains nested loops into an open extrusion path. Its signed
    // XY turning accumulates as the path winds through successive loops, while a rectilinear
    // serpentine's opposing turns cancel. The rectangular contact region keeps successive
    // spiral loops nested and reachable without crossing a hole.
    REQUIRE(observations.at("spiralinset").open_paths > 0);
    CHECK(observations.at("spiralinset").closed_paths == 0);
    REQUIRE(observations.at("spiralinset").max_open_signed_turn > 3.0 * M_PI);

    // The production contract alternates by SupportLayer::interface_id(), not by adjacent Z
    // values.  Compare the two actual id parities and require both to be present.
    const auto interlaced_angles = public_angles(observations.at("rectilinear_interlaced"));
    REQUIRE(interlaced_angles.size() >= 3);
    std::vector<double> even_angles;
    std::vector<double> odd_angles;
    for (const auto &[interface_id, angle] : interlaced_angles) {
        if ((interface_id & 1u) == 0)
            even_angles.push_back(angle);
        else
            odd_angles.push_back(angle);
    }
    REQUIRE(even_angles.size() > 0);
    REQUIRE(odd_angles.size() > 0);
    double even_x = 0.0, even_y = 0.0, odd_x = 0.0, odd_y = 0.0;
    for (double angle : even_angles) {
        even_x += std::cos(2.0 * angle);
        even_y += std::sin(2.0 * angle);
    }
    for (double angle : odd_angles) {
        odd_x += std::cos(2.0 * angle);
        odd_y += std::sin(2.0 * angle);
    }
    REQUIRE(std::hypot(even_x, even_y) > 0.8 * even_angles.size());
    REQUIRE(std::hypot(odd_x, odd_y) > 0.8 * odd_angles.size());
    const double even_angle = 0.5 * std::atan2(even_y, even_x);
    const double odd_angle = 0.5 * std::atan2(odd_y, odd_x);
    CHECK(axial_angle_diff_deg(even_angle, odd_angle) > 60.0);
    for (double angle : even_angles)
        CHECK(axial_angle_diff_deg(angle, even_angle) < 15.0);
    for (double angle : odd_angles)
        CHECK(axial_angle_diff_deg(angle, odd_angle) < 15.0);
}

namespace support_interface_loop_pattern_contracts {

struct LoopTopologyObservation
{
    std::set<size_t> interface_layers;
    size_t interface_entities = 0;
    double interface_length = 0.0;
    double interface_volume = 0.0;
    bool finite = true;
};

// Inspect public interface topology. The production loop route splits and clips each source
// polygon, then emits the resulting pieces as ExtrusionPath entities; the source loop's seam is
// therefore not an endpoint-closure contract at this layer. Count interface leaves and preserve
// their finite positive deposition as the observable public witness.
static void collect_interface_topology(const ExtrusionEntity &entity,
                                      const size_t layer_id,
                                      LoopTopologyObservation &out)
{
    if (const auto *collection = dynamic_cast<const ExtrusionEntityCollection *>(&entity)) {
        for (const ExtrusionEntity *child : collection->entities)
            if (child != nullptr)
                collect_interface_topology(*child, layer_id, out);
        return;
    }

    if (entity.role() != erSupportMaterialInterface)
        return;

    ++out.interface_entities;
    out.interface_layers.insert(layer_id);
    const double length = entity.length();
    const double volume = entity.total_volume();
    out.interface_length += length;
    out.interface_volume += volume;
    if (!std::isfinite(length) || !std::isfinite(volume) || length <= 0.0 || volume <= 0.0) {
        out.finite = false;
        return;
    }
}

static LoopTopologyObservation process_loop_fixture(const bool loop_pattern)
{
    DynamicPrintConfig config = support_config();
    config.set_deserialize_strict({
        {"support_type",                    "normal(auto)"},
        {"support_style",                   "default"},
        {"support_angle",                   0.0},
        {"support_top_z_distance",          0.2},
        {"support_interface_loop_pattern",  loop_pattern ? 1 : 0},
        {"support_interface_pattern",       "rectilinear"},
        {"support_object_xy_distance",      0.0},
        {"support_interface_top_layers",    4},
        {"support_interface_spacing",       0.0}
    });

    Model model;
    Print print;
    init_print({support_pattern_wall_capital()}, print, model, config);
    const StringObjectException validation = print.validate();
    INFO(validation.string);
    REQUIRE(validation.string.empty());
    print.process();
    REQUIRE_FALSE(print.objects().empty());
    REQUIRE(print.objects().size() == 1);

    LoopTopologyObservation out;
    for (const SupportLayer *layer : print.objects().front()->support_layers()) {
        for (const ExtrusionEntity *entity : layer->support_fills.entities)
            if (entity != nullptr)
                collect_interface_topology(*entity, layer->id(), out);
    }
    REQUIRE(out.interface_entities > 0);
    REQUIRE(out.interface_length > 0.0);
    REQUIRE(out.interface_volume > 0.0);
    REQUIRE(out.finite);
    return out;
}

} // namespace support_interface_loop_pattern_contracts

TEST_CASE("Support interface loop pattern adds contact interface paths", "[SupportInterfaceLoopPattern][SupportMaterial]")
{
    const auto without_loops = support_interface_loop_pattern_contracts::process_loop_fixture(false);
    const auto with_loops = support_interface_loop_pattern_contracts::process_loop_fixture(true);

    REQUIRE_FALSE(without_loops.interface_layers.empty());
    REQUIRE(without_loops.interface_layers == with_loops.interface_layers);
    // LoopInterfaceProcessor adds one or more public interface path leaves before the remaining
    // contact area is filled. The final leaves are intentionally open after seam splitting and
    // clipping, so compare the public path topology rather than an unavailable closed-endpoint
    // representation.
    CAPTURE(without_loops.interface_entities, with_loops.interface_entities);
    REQUIRE(with_loops.interface_entities > without_loops.interface_entities);
    CHECK(without_loops.interface_length > 0.0);
    CHECK(with_loops.interface_length > 0.0);
    CHECK(without_loops.interface_volume > 0.0);
    CHECK(with_loops.interface_volume > 0.0);
}

TEST_CASE("Spiral inset reaches support interface extrusion", "[SupportSettingsVariance][SupportMaterial]")
{
    std::vector<double> lengths;
    for (const char *pattern : {"rectilinear", "spiralinset"}) {
        const SupportStats stats = inspect_support(slice_capital({
            {"support_type", "normal(auto)"},
            {"support_style", "default"},
            {"support_interface_top_layers", 4},
            {"support_interface_pattern", pattern}
        }));
        CAPTURE(pattern);
        REQUIRE(stats.finite);
        REQUIRE(stats.interface_segments > 0);
        // Pattern-specific perimeter/base-interface handling may add a contact layer; require
        // the configured interface depth to be reached without pinning incidental extra layers.
        CHECK(stats.interface_layers.size() >= 3);
        lengths.push_back(stats.interface_length);
    }
    CHECK(distinct_lengths(lengths) == 2);
}

TEST_CASE("Top interface spacing changes emitted density", "[SupportSettingsVariance][SupportMaterial]")
{
    const SupportStats solid = inspect_support(slice_capital({
        {"support_type", "normal(auto)"}, {"support_interface_top_layers", 4},
        {"support_interface_spacing", 0.0}
    }));
    const SupportStats sparse = inspect_support(slice_capital({
        {"support_type", "normal(auto)"}, {"support_interface_top_layers", 4},
        {"support_interface_spacing", 4.0}
    }));
    REQUIRE(solid.finite);
    REQUIRE(sparse.finite);
    REQUIRE(solid.interface_segments > 0);
    REQUIRE(sparse.interface_segments > 0);
    CHECK(solid.interface_length > sparse.interface_length * 1.5);
}

TEST_CASE("Support XY gap changes clipped support geometry", "[SupportSettingsVariance][SupportMaterial]")
{
    const SupportStats tight = inspect_support(slice_capital({
        {"support_type", "normal(auto)"}, {"support_interface_top_layers", 3},
        {"support_object_xy_distance", 0.0}
    }));
    const SupportStats open = inspect_support(slice_capital({
        {"support_type", "normal(auto)"}, {"support_interface_top_layers", 3},
        {"support_object_xy_distance", 3.0}
    }));
    REQUIRE(tight.finite);
    REQUIRE(open.finite);
    REQUIRE(tight.base_segments + tight.interface_segments > 0);
    REQUIRE(open.base_segments + open.interface_segments > 0);
    const bool changed = std::fabs(tight.base_length - open.base_length) > 1e-3 ||
                         std::fabs(tight.interface_length - open.interface_length) > 1e-3;
    CHECK(changed);
}

TEST_CASE("Support top and bottom Z gaps move their interface contact", "[SupportSettingsVariance][SupportMaterial]")
{
    const SupportStats top_zero = inspect_support(slice_capital({
        {"support_type", "normal(auto)"}, {"support_interface_top_layers", 4},
        {"support_top_z_distance", 0.0}
    }));
    const SupportStats top_gap = inspect_support(slice_capital({
        {"support_type", "normal(auto)"}, {"support_interface_top_layers", 4},
        {"support_top_z_distance", 0.4}
    }));
    REQUIRE(top_zero.finite);
    REQUIRE(top_gap.finite);
    REQUIRE(top_zero.interface_segments > 0);
    REQUIRE(top_gap.interface_segments > 0);
    CHECK(std::fabs(top_zero.interface_max_z - top_gap.interface_max_z) > 0.1);

    const SupportStats bottom_zero = inspect_support(slice_tunnel({
        {"support_type", "normal(auto)"}, {"support_interface_top_layers", 0},
        {"support_interface_bottom_layers", 4}, {"support_bottom_z_distance", 0.0}
    }));
    const SupportStats bottom_gap = inspect_support(slice_tunnel({
        {"support_type", "normal(auto)"}, {"support_interface_top_layers", 0},
        {"support_interface_bottom_layers", 4}, {"support_bottom_z_distance", 0.4}
    }));
    REQUIRE(bottom_zero.finite);
    REQUIRE(bottom_gap.finite);
    REQUIRE(bottom_zero.interface_segments > 0);
    REQUIRE(bottom_gap.interface_segments > 0);
    CHECK(std::fabs(bottom_zero.interface_min_z - bottom_gap.interface_min_z) > 0.1);
}

TEST_CASE("Support and interface filament selections reach distinct tools", "[SupportSettingsVariance][SupportMaterial][GCode]")
{
    DynamicPrintConfig same = multifilament_config(2, {
        {"enable_support", 1}, {"layer_height", 0.2}, {"support_on_build_plate_only", 0},
        {"support_interface_top_layers", 3}, {"support_interface_bottom_layers", 0},
        {"support_base_pattern_spacing", 2.5}, {"seam_position", "back"}, {"gcode_comments", 1},
        {"use_relative_e_distances", 1}, {"layer_change_gcode", "G92 E0\n"},
        {"skirt_loops", 0}, {"brim_width", 0.0}, {"raft_layers", 0},
        {"support_filament", 0}, {"support_interface_filament", 0}
    });
    DynamicPrintConfig split = multifilament_config(2, {
        {"enable_support", 1}, {"layer_height", 0.2}, {"support_on_build_plate_only", 0},
        {"support_interface_top_layers", 3}, {"support_interface_bottom_layers", 0},
        {"support_base_pattern_spacing", 2.5}, {"seam_position", "back"}, {"gcode_comments", 1},
        {"use_relative_e_distances", 1}, {"layer_change_gcode", "G92 E0\n"},
        {"skirt_loops", 0}, {"brim_width", 0.0}, {"raft_layers", 0},
        {"support_filament", 1}, {"support_interface_filament", 2}
    });
    const SupportStats same_stats = inspect_support(slice_checked(capital_fixture(), same));
    const SupportStats split_stats = inspect_support(slice_checked(capital_fixture(), split));
    REQUIRE(same_stats.finite);
    REQUIRE(split_stats.finite);
    REQUIRE(same_stats.interface_segments > 0);
    REQUIRE(split_stats.interface_segments > 0);
    CHECK(same_stats.support_tools.size() == 1);
    CHECK(split_stats.support_tools.size() >= 2);
}


TEST_CASE("Support flow ratios scale only their labeled support roles",
          "[SupportSettingsVariance][SupportMaterial][SupportFlowRatio]")
{
    const DynamicPrintConfig baseline_config = support_flow_ratio_config();
    const std::string baseline_gcode = slice_checked(capital_fixture(), baseline_config);
    const SupportStats baseline_shape = inspect_support(baseline_gcode);
    const SupportFlowStats baseline = inspect_support_flow(baseline_gcode);
    REQUIRE(baseline_shape.finite);
    REQUIRE(baseline.finite);
    REQUIRE(baseline.base.moves > 0);
    REQUIRE(baseline.interface.moves > 0);
    // The capital fixture always has ordinary external-wall deposition. Every support-flow
    // variant must leave that positive object control unchanged.
    REQUIRE(baseline.object.moves > 0);

    struct RatioCase {
        const char *key;
        double ratio;
        bool base_target;
    };
    const std::vector<RatioCase> cases = {
        {"support_flow_ratio", 0.75, true},
        {"support_flow_ratio", 1.25, true},
        {"support_interface_flow_ratio", 0.75, false},
        {"support_interface_flow_ratio", 1.25, false}
    };

    for (const RatioCase &test_case : cases) {
        CAPTURE(test_case.key, test_case.ratio);
        DynamicPrintConfig changed_config = baseline_config;
        changed_config.set_deserialize_strict(test_case.key, std::to_string(test_case.ratio));
        const std::string changed_gcode = slice_checked(capital_fixture(), changed_config);
        const SupportStats changed_shape = inspect_support(changed_gcode);
        const SupportFlowStats changed = inspect_support_flow(changed_gcode);
        REQUIRE(changed_shape.finite);
        REQUIRE(changed.finite);
        REQUIRE(changed.base.moves > 0);
        REQUIRE(changed.interface.moves > 0);
        REQUIRE(changed.object.moves > 0);

        CHECK(changed_shape.base_segments == baseline_shape.base_segments);
        CHECK(changed_shape.interface_segments == baseline_shape.interface_segments);
        CHECK(changed_shape.base_layers == baseline_shape.base_layers);
        CHECK(changed_shape.interface_layers == baseline_shape.interface_layers);
        require_support_geometry(baseline.base, changed.base);
        require_support_geometry(baseline.interface, changed.interface);
        require_unchanged_support_role(baseline.object, changed.object);
        if (test_case.base_target) {
            require_scaled_support_role(baseline.base, changed.base, test_case.ratio);
            require_unchanged_support_role(baseline.interface, changed.interface);
        } else {
            require_unchanged_support_role(baseline.base, changed.base);
            require_scaled_support_role(baseline.interface, changed.interface, test_case.ratio);
        }
    }
}

struct BasePatternObservation {
    SupportStats stats;
    StringObjectException validation;
    std::vector<StringObjectException> warnings;
    InfillPattern base_fill_pattern = ipSupportBase;
};

BasePatternObservation slice_normal_base_pattern(const char *pattern)
{
    DynamicPrintConfig config = support_config();
    config.set_deserialize_strict({
        {"support_type", "normal(auto)"},
        {"support_style", "default"},
        {"support_interface_top_layers", 0},
        {"support_interface_bottom_layers", 0},
        {"support_base_pattern", pattern}
    });

    Print print;
    Model model;
    init_print({capital_fixture()}, print, model, config);
    std::vector<StringObjectException> warnings;
    const StringObjectException validation = print.validate(&warnings);
    INFO("Support validation: " << validation.string << " [" << validation.opt_key << "]");
    REQUIRE(validation.string.empty());
    REQUIRE_FALSE(print.objects().empty());
    BasePatternObservation out;
    out.validation = validation;
    out.warnings = std::move(warnings);
    const std::string output = gcode(print);
    REQUIRE_FALSE(output.empty());
    out.stats = inspect_support(output);
    out.base_fill_pattern = SupportParameters(*print.objects().front()).base_fill_pattern;
    return out;
}

void require_same_base_signature(const SupportStats &expected, const SupportStats &observed)
{
    REQUIRE(expected.finite);
    REQUIRE(observed.finite);
    REQUIRE(expected.base_segments > 0);
    REQUIRE(observed.base_segments > 0);
    CHECK(observed.base_segments == expected.base_segments);
    CHECK(observed.base_layers.size() == expected.base_layers.size());
    CHECK_THAT(observed.base_length, Catch::Matchers::WithinAbs(expected.base_length, 1e-5));
}

std::size_t interior_orientation_changes(const SupportStats &expected, const SupportStats &observed)
{
    REQUIRE_FALSE(expected.base_direction_by_layer.empty());
    REQUIRE_FALSE(observed.base_direction_by_layer.empty());
    const double first_z = expected.base_direction_by_layer.begin()->first;
    const double last_z = expected.base_direction_by_layer.rbegin()->first;
    std::size_t common_interior_layers = 0;
    std::size_t changed_layers = 0;
    for (const auto &entry : observed.base_direction_by_layer) {
        const auto expected_it = expected.base_direction_by_layer.find(entry.first);
        if (expected_it == expected.base_direction_by_layer.end())
            continue;
        if (entry.first <= first_z + 1e-6 || entry.first >= last_z - 1e-6)
            continue;
        ++common_interior_layers;
        double delta = std::fabs(entry.second - expected_it->second);
        delta = std::min(delta, M_PI - delta);
        if (delta > 0.2)
            ++changed_layers;
    }
    REQUIRE(common_interior_layers >= 3);
    return changed_layers;
}

TEST_CASE("Normal support base patterns honor supported and fallback partitions",
          "[SupportSettingsVariance][SupportMaterial][SupportBasePattern]")
{
    const BasePatternObservation rectilinear = slice_normal_base_pattern("rectilinear");
    REQUIRE(rectilinear.validation.string.empty());
    REQUIRE(rectilinear.stats.finite);
    REQUIRE(rectilinear.stats.base_segments > 0);

    for (const char *pattern : {"default", "rectilinear", "rectilinear-grid", "honeycomb", "lightning", "hollow"}) {
        CAPTURE(pattern);
        const BasePatternObservation observed = slice_normal_base_pattern(pattern);
        REQUIRE(observed.validation.string.empty());
        REQUIRE(observed.stats.finite);
        REQUIRE(observed.stats.base_segments > 0);

        const bool documented_fallback = std::string(pattern) == "lightning" || std::string(pattern) == "hollow";
        const bool documented_noop = std::string(pattern) == "default" ||
                                     std::string(pattern) == "rectilinear";
        if (documented_noop) {
            CHECK(observed.base_fill_pattern == rectilinear.base_fill_pattern);
            require_same_base_signature(rectilinear.stats, observed.stats);
        } else if (std::string(pattern) == "rectilinear-grid") {
            // The grid alternates base and interface angles. Weighted per-layer
            // orientation distinguishes this from the rectilinear control.
            CHECK(observed.base_fill_pattern == rectilinear.base_fill_pattern);
            const std::size_t changed_layers = interior_orientation_changes(rectilinear.stats, observed.stats);
            CHECK(changed_layers > 0);
        } else if (documented_fallback) {
            bool has_base_pattern_warning = false;
            for (const StringObjectException &warning : observed.warnings) {
                const bool is_base_pattern_warning = warning.opt_key == "support_base_pattern";
                const bool is_warning = warning.is_warning;
                const bool names_rectilinear_fallback = warning.string.find("Rectilinear") != std::string::npos;
                if (is_base_pattern_warning && is_warning && names_rectilinear_fallback)
                    has_base_pattern_warning = true;
            }
            CHECK(has_base_pattern_warning);
            CHECK(observed.base_fill_pattern == rectilinear.base_fill_pattern);
            require_same_base_signature(rectilinear.stats, observed.stats);
        } else {
            REQUIRE(std::string(pattern) == "honeycomb");
            CHECK(observed.base_fill_pattern == ipHoneycomb);
            CHECK(observed.stats.base_direction_bins != rectilinear.stats.base_direction_bins);
        }
    }
}


namespace support_ironing_settings_candidate {

enum class IroningMarkerRole { none, ironing, other };

struct IroningMarker {
    bool recognized = false;
    IroningMarkerRole role = IroningMarkerRole::none;
};

std::string_view ironing_trim(std::string_view text)
{
    while (!text.empty() && (text.front() == ' ' || text.front() == '\t'))
        text.remove_prefix(1);
    while (!text.empty() && (text.back() == ' ' || text.back() == '\t' || text.back() == '\r'))
        text.remove_suffix(1);
    return text;
}

IroningMarker parse_ironing_marker(std::string_view comment)
{
    comment = ironing_trim(comment);
    bool tagged = false;
    for (const std::string_view prefix : {std::string_view("TYPE:"), std::string_view("FEATURE:")}) {
        if (comment.size() >= prefix.size() && comment.substr(0, prefix.size()) == prefix) {
            comment = ironing_trim(comment.substr(prefix.size()));
            tagged = true;
            break;
        }
    }

    if (comment == "support ironing" || comment == "Support ironing" || comment == "SupportIroning")
        return {true, IroningMarkerRole::ironing};
    if (comment == "support material" || comment == "support material interface" ||
        comment == "support transition" || comment == "Support" ||
        comment == "Support material" || comment == "Support material interface" ||
        comment == "Support transition" || comment == "SupportMaterial" ||
        comment == "SupportMaterialInterface" || comment == "SupportTransition")
        return {true, IroningMarkerRole::other};
    return {tagged, IroningMarkerRole::other};
}

struct SupportIroningStats {
    std::size_t moves = 0;
    std::set<double> layers;
    double xy_length = 0.0;
    double e_length = 0.0;
    std::set<int> feedrates;
    double orientation_alignment = 0.0;
    double orientation_error_bound = 0.0;
    double xy_error_bound = 0.0;
    double e_error_bound = 0.0;
    double max_pair_abs_e = 0.0;
    bool finite = true;
};

constexpr std::size_t kIroningMaxGCodeBytes = 4u * 1024u * 1024u;
constexpr std::size_t kIroningMaxGCodeLines = 100000u;
constexpr std::size_t kIroningMaxMoves = 100000u;

void require_ironing_registry()
{
    const ConfigOptionDef *pattern = print_config_def.get("support_ironing_pattern");
    const ConfigOptionDef *flow = print_config_def.get("support_ironing_flow");
    const ConfigOptionDef *spacing = print_config_def.get("support_ironing_spacing");
    REQUIRE(pattern != nullptr);
    REQUIRE(flow != nullptr);
    REQUIRE(spacing != nullptr);
    REQUIRE(pattern->type == coEnum);
    REQUIRE(std::find(pattern->enum_values.begin(), pattern->enum_values.end(), "rectilinear") != pattern->enum_values.end());
    REQUIRE(std::find(pattern->enum_values.begin(), pattern->enum_values.end(), "concentric") != pattern->enum_values.end());
    CHECK(flow->min <= 10.0);
    CHECK(flow->max >= 25.0);
    CHECK(spacing->min <= 0.08);
    CHECK(spacing->max >= 0.30);
}

SupportIroningStats inspect_support_ironing(const std::string &gcode)
{
    REQUIRE(gcode.size() <= kIroningMaxGCodeBytes);
    REQUIRE(std::count(gcode.begin(), gcode.end(), '\n') <= kIroningMaxGCodeLines);

    SupportIroningStats out;
    IroningMarkerRole active_role = IroningMarkerRole::none;
    GCodeReader parser;
    parser.apply_config(support_config());
    parser.parse_buffer(gcode, [&out, &active_role](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        const IroningMarker marker = parse_ironing_marker(line.comment());
        if (marker.recognized)
            active_role = marker.role;

        // Validate coordinates and E before the extrusion predicate can hide malformed values.
        const double old_x = self.x();
        const double old_y = self.y();
        const double old_z = self.z();
        const double old_e = self.e();
        const double new_x = line.new_X(self);
        const double new_y = line.new_Y(self);
        const double new_z = line.new_Z(self);
        const double new_e = line.new_E(self);
        if (!std::isfinite(old_x) || !std::isfinite(old_y) || !std::isfinite(old_z) ||
            !std::isfinite(old_e) || !std::isfinite(new_x) || !std::isfinite(new_y) ||
            !std::isfinite(new_z) || !std::isfinite(new_e)) {
            out.finite = false;
            return;
        }
        const double xy = static_cast<double>(line.dist_XY(self));
        const double delta_e = static_cast<double>(line.dist_E(self));
        if (!std::isfinite(xy) || !std::isfinite(delta_e)) {
            out.finite = false;
            return;
        }
        if (active_role != IroningMarkerRole::ironing || !line.extruding(self) || xy <= EPSILON)
            return;
        REQUIRE(out.moves < kIroningMaxMoves);

        ++out.moves;
        out.layers.insert(self.z());
        out.xy_length += xy;
        out.e_length += std::abs(delta_e);
        const double max_coordinate = std::max(std::max(std::abs(old_x), std::abs(old_y)),
                                               std::max(std::abs(new_x), std::abs(new_y)));
        const double endpoint_error = 0.5e-3 +
            std::numeric_limits<float>::epsilon() * max_coordinate;
        out.xy_error_bound += 2.0 * std::sqrt(2.0) * endpoint_error;
        const double max_e = std::max(std::abs(old_e), std::abs(new_e));
        out.max_pair_abs_e = std::max(out.max_pair_abs_e, max_e);
        out.e_error_bound += 1e-5 + 4.0 * std::numeric_limits<float>::epsilon() * max_e;
        const double feedrate = static_cast<double>(line.new_F(self));
        REQUIRE(std::isfinite(feedrate));
        REQUIRE(feedrate > 0.0);
        out.feedrates.insert(static_cast<int>(std::lround(feedrate)));

        // support_angle=45 fixes the ironing angle at support_angle+90=135 degrees.
        // Compare length-weighted axial alignment to that actual fill direction. The
        // derivative bound below covers coordinate quantization and avoids a tuned angle bin.
        const double angle = std::atan2(line.dist_Y(self), line.dist_X(self));
        const double expected_angle = 0.75 * M_PI;
        out.orientation_alignment += xy * std::abs(std::cos(2.0 * (angle - expected_angle)));
        out.orientation_error_bound += 8.0 * endpoint_error;
    });
    REQUIRE(out.finite);
    return out;
}

DynamicPrintConfig support_ironing_config(const char *pattern, double flow, double spacing)
{
    DynamicPrintConfig config = support_config();
    config.set_deserialize_strict({
        {"support_type", "normal(auto)"},
        {"support_style", "default"},
        {"support_interface_top_layers", 3},
        {"support_interface_bottom_layers", 0},
        {"support_interface_pattern", "rectilinear"},
        {"support_interface_spacing", 0.5},
        {"support_angle", 45.0},
        {"support_ironing", 1},
        {"support_ironing_pattern", pattern},
        {"support_ironing_flow", flow},
        {"support_ironing_spacing", spacing},
        {"ironing_type", "no ironing"},
        {"enable_arc_fitting", 0},
        {"nozzle_diameter", "0.4"},
        {"support_flow_ratio", 1.0},
        {"support_interface_flow_ratio", 1.0},
        {"initial_layer_print_height", 0.2},
        {"support_line_width", 0.4}
    });
    return config;
}

struct SupportIroningExport {
    SupportIroningStats stats;
    SupportStats support;
    double interface_width = 0.0;
    double interface_height = 0.0;
};

SupportIroningExport slice_support_ironing(const char *pattern, double flow, double spacing, double ironing_speed = -1.0)
{
    DynamicPrintConfig config = support_ironing_config(pattern, flow, spacing);
    if (ironing_speed > 0.0) {
        // Keep the speed oracle independent of CoolingBuffer slowdown and the
        // filament volumetric cap. The production erIroning branch then uses
        // only the registered ironing_speed value for its F word.
        config.set_deserialize_strict({
            {"ironing_speed", ironing_speed},
            {"filament_ironing_speed", "nil"},
            {"slow_down_for_layer_cooling", false},
            {"filament_max_volumetric_speed", "200"},
            {"filament_adaptive_volumetric_speed", false}
        });
    }
    Print print;
    Model model;
    init_print({capital_fixture()}, print, model, config);
    const StringObjectException validation = print.validate();
    INFO(validation.string);
    REQUIRE(validation.string.empty());
    REQUIRE_FALSE(print.objects().empty());
    const std::string output = gcode(print);
    REQUIRE_FALSE(output.empty());
    const SupportParameters support_params(*print.objects().front());
    REQUIRE(output.size() <= kIroningMaxGCodeBytes);
    REQUIRE(std::count(output.begin(), output.end(), '\n') <= kIroningMaxGCodeLines);
    return {inspect_support_ironing(output), inspect_support(output),
            support_params.support_material_interface_flow.width(),
            support_params.support_material_interface_flow.height()};
}

double flow_area(double width, double height)
{
    return height * (width - height * (1.0 - 0.25 * M_PI));
}

void require_positive_support_ironing(const SupportIroningExport &exported)
{
    const SupportIroningStats &stats = exported.stats;
    REQUIRE(stats.finite);
    REQUIRE(stats.moves > 0);
    REQUIRE(stats.layers.size() > 0);
    REQUIRE(stats.xy_length > 0.0);
    REQUIRE(stats.e_length > 0.0);
    REQUIRE(exported.support.finite);
    REQUIRE(exported.support.base_segments > 0);
    REQUIRE(exported.support.interface_segments > 0);
}

void require_same_support_geometry(const SupportIroningExport &expected,
                                   const SupportIroningExport &observed)
{
    REQUIRE(expected.support.base_segments == observed.support.base_segments);
    REQUIRE(expected.support.interface_segments == observed.support.interface_segments);
    REQUIRE(expected.support.base_layers == observed.support.base_layers);
    REQUIRE(expected.support.interface_layers == observed.support.interface_layers);
    CHECK_THAT(observed.support.base_length, Catch::Matchers::WithinAbs(expected.support.base_length, 1e-5));
    CHECK_THAT(observed.support.interface_length, Catch::Matchers::WithinAbs(expected.support.interface_length, 1e-5));
}

TEST_CASE("Support ironing pattern selects the requested ironing path family",
          "[SupportIroningSettings][SupportSettingsVariance][SupportMaterial]")
{
    require_ironing_registry();
    const SupportIroningExport rectilinear = slice_support_ironing("rectilinear", 10.0, 0.10);
    const SupportIroningExport concentric = slice_support_ironing("concentric", 10.0, 0.10);
    require_positive_support_ironing(rectilinear);
    require_positive_support_ironing(concentric);
    require_same_support_geometry(rectilinear, concentric);
    REQUIRE(rectilinear.stats.layers == concentric.stats.layers);
    // With support_angle pinned at 45 degrees, the rectilinear family is aligned to
    // the actual 135-degree interface angle. Axis-aligned concentric square segments
    // have zero axial cosine at that diagonal angle. Compare length-weighted orientation,
    // with an error bound derived from the three-decimal coordinate export.
    const double alignment_gap = rectilinear.stats.orientation_alignment -
                                 concentric.stats.orientation_alignment;
    REQUIRE(alignment_gap > rectilinear.stats.orientation_error_bound +
            concentric.stats.orientation_error_bound);
}

TEST_CASE("Support ironing flow scales ironing material at fixed geometry",
          "[SupportIroningSettings][SupportSettingsVariance][SupportMaterial]")
{
    require_ironing_registry();
    const SupportIroningExport low = slice_support_ironing("rectilinear", 10.0, 0.10);
    const SupportIroningExport high = slice_support_ironing("rectilinear", 25.0, 0.10);
    require_positive_support_ironing(low);
    require_positive_support_ironing(high);
    require_same_support_geometry(low, high);
    REQUIRE(low.stats.layers == high.stats.layers);
    REQUIRE(low.stats.moves == high.stats.moves);
    CHECK_THAT(high.stats.xy_length, Catch::Matchers::WithinAbs(low.stats.xy_length,
        low.stats.xy_error_bound + high.stats.xy_error_bound));
    REQUIRE(low.interface_width > 0.0);
    REQUIRE(low.interface_height > 0.0);
    CHECK_THAT(high.interface_width, Catch::Matchers::WithinAbs(low.interface_width, 0.0));
    CHECK_THAT(high.interface_height, Catch::Matchers::WithinAbs(low.interface_height, 0.0));

    const double low_height = low.interface_height * 0.01 * 10.0;
    const double high_height = high.interface_height * 0.01 * 25.0;
    const double expected_ratio = flow_area(low.interface_width, high_height) /
                                  flow_area(low.interface_width, low_height);
    const double expected = low.stats.e_length * expected_ratio;
    const double tolerance = high.stats.e_error_bound + expected_ratio * low.stats.e_error_bound +
        64.0 * std::numeric_limits<double>::epsilon() *
            std::max({1.0, std::abs(expected), std::abs(high.stats.e_length)});
    REQUIRE(tolerance < std::abs(expected - low.stats.e_length));
    CHECK_THAT(high.stats.e_length, Catch::Matchers::WithinAbs(expected, tolerance));
}

TEST_CASE("Support ironing speed controls ironing feedrate without changing support material",
          "[SupportIroningSpeedSettings][SupportIroningSettings][SupportSettingsVariance][SupportMaterial]")
{
    require_ironing_registry();
    const SupportIroningExport slow = slice_support_ironing("rectilinear", 10.0, 0.10, 20.0);
    const SupportIroningExport fast = slice_support_ironing("rectilinear", 10.0, 0.10, 40.0);
    require_positive_support_ironing(slow);
    require_positive_support_ironing(fast);
    require_same_support_geometry(slow, fast);
    REQUIRE(slow.stats.layers == fast.stats.layers);
    REQUIRE(slow.stats.moves == fast.stats.moves);
    REQUIRE(slow.stats.feedrates == std::set<int>{1200});
    REQUIRE(fast.stats.feedrates == std::set<int>{2400});
    CHECK_THAT(fast.stats.xy_length, Catch::Matchers::WithinAbs(
        slow.stats.xy_length, slow.stats.xy_error_bound + fast.stats.xy_error_bound));
    CHECK_THAT(fast.stats.e_length, Catch::Matchers::WithinAbs(
        slow.stats.e_length, slow.stats.e_error_bound + fast.stats.e_error_bound));
    CHECK_THAT(fast.interface_width, Catch::Matchers::WithinAbs(slow.interface_width, 0.0));
    CHECK_THAT(fast.interface_height, Catch::Matchers::WithinAbs(slow.interface_height, 0.0));
}

TEST_CASE("Support ironing spacing changes ironing density",
          "[SupportIroningSettings][SupportSettingsVariance][SupportMaterial]")
{
    require_ironing_registry();
    const SupportIroningExport dense = slice_support_ironing("rectilinear", 10.0, 0.08);
    const SupportIroningExport wide = slice_support_ironing("rectilinear", 10.0, 0.30);
    require_positive_support_ironing(dense);
    require_positive_support_ironing(wide);
    require_same_support_geometry(dense, wide);
    REQUIRE(dense.stats.layers == wide.stats.layers);
    CHECK(dense.stats.moves > wide.stats.moves);
    CHECK(dense.stats.xy_length > wide.stats.xy_length);
    CHECK(dense.stats.e_length > wide.stats.e_length);
}

} // namespace support_ironing_settings_candidate

namespace independent_support_layer_height_contracts {

struct SupportHeightEvidence {
    std::size_t support_layers = 0;
    std::size_t interface_paths = 0;
    double interface_length = 0.0;
    double support_island_area = 0.0;
    double top_interface_z = -std::numeric_limits<double>::infinity();
    bool finite = true;
};

static SupportHeightEvidence inspect_support_height(const PrintObject &object)
{
    constexpr std::size_t max_support_layers = 250000;
    constexpr std::size_t max_interface_paths = 250000;

    SupportHeightEvidence out;
    out.support_layers = object.support_layers().size();
    if (out.support_layers > max_support_layers) {
        out.finite = false;
        return out;
    }

    for (const SupportLayer *layer : object.support_layers()) {
        double layer_length = 0.0;
        bool has_interface = false;
        for (const ExPolygon &island : layer->support_islands)
            out.support_island_area += std::max(0.0, island.area());

        for_each_extrusion_path(layer->support_fills, [&](const ExtrusionPath &path) {
            if (path.role() != erSupportMaterialInterface)
                return;
            has_interface = true;
            ++out.interface_paths;
            if (out.interface_paths > max_interface_paths) {
                out.finite = false;
                return;
            }
            for (std::size_t i = 1; i < path.polyline.points.size(); ++i) {
                const Point3 &a = path.polyline.points[i - 1];
                const Point3 &b = path.polyline.points[i];
                const double dx = unscale<double>(static_cast<double>(b.x()) - static_cast<double>(a.x()));
                const double dy = unscale<double>(static_cast<double>(b.y()) - static_cast<double>(a.y()));
                const double length = std::hypot(dx, dy);
                out.finite = out.finite && std::isfinite(dx) && std::isfinite(dy) && std::isfinite(length);
                if (std::isfinite(length))
                    layer_length += length;
            }
        });
        if (has_interface) {
            out.top_interface_z = std::max(out.top_interface_z, static_cast<double>(layer->print_z));
            out.interface_length += layer_length;
        }
        out.finite = out.finite && std::isfinite(static_cast<double>(layer->print_z)) &&
            std::isfinite(static_cast<double>(layer->height));
    }
    return out;
}

static SupportHeightEvidence process_support_height(bool independent)
{
    DynamicPrintConfig config = support_config();
    config.set_deserialize_strict({
        {"support_type", "normal(auto)"},
        {"support_style", "default"},
        {"support_angle", 0.0},
        {"thick_bridges", 0},
        {"bridge_no_support", 0},
        {"initial_layer_print_height", 0.2},
        {"support_top_z_distance", 0.3},
        {"support_bottom_z_distance", 0.2},
        {"independent_support_layer_height", independent},
        {"enable_prime_tower", 0},
        {"z_hop", 0.0}
    });

    Print print;
    Model model;
    init_print({capital_fixture()}, print, model, config);
    const StringObjectException validation = print.validate();
    INFO(validation.string);
    REQUIRE(validation.string.empty());
    REQUIRE(print.objects().size() == 1);
    print.process();
    return inspect_support_height(*print.objects().front());
}

TEST_CASE("Independent support layer height preserves a non-grid top Z gap",
          "[IndependentSupportLayerHeight][SupportSettingsVariance][SupportMaterial]")
{
    const SupportHeightEvidence independent = process_support_height(true);
    const SupportHeightEvidence synchronized = process_support_height(false);
    constexpr double cap_bottom_z = 6.0;
    constexpr double requested_gap = 0.3;
    constexpr double normalized_gap = 0.4; // 0.3 rounds to two 0.2mm object layers when independent=false.

    for (const SupportHeightEvidence &evidence : {independent, synchronized}) {
        REQUIRE(evidence.finite);
        REQUIRE(evidence.support_layers > 0);
        REQUIRE(evidence.interface_paths > 0);
        REQUIRE(std::isfinite(evidence.interface_length));
        REQUIRE(evidence.interface_length > 0.0);
        REQUIRE(std::isfinite(evidence.support_island_area));
        REQUIRE(evidence.support_island_area > 0.0);
        REQUIRE(std::isfinite(evidence.top_interface_z));
    }

    // capital_fixture() has a cap underside at z=6.0 and the object grid is pinned to 0.2mm.
    // The independent branch subtracts the requested 0.3mm directly; the synchronized branch
    // first rounds it to 0.4mm and selects an object-grid layer.
    CHECK_THAT(independent.top_interface_z,
               Catch::Matchers::WithinAbs(cap_bottom_z - requested_gap, 1e-5));
    CHECK_THAT(synchronized.top_interface_z,
               Catch::Matchers::WithinAbs(cap_bottom_z - normalized_gap, 1e-5));
    CHECK_THAT(independent.top_interface_z - synchronized.top_interface_z,
               Catch::Matchers::WithinAbs(normalized_gap - requested_gap, 1e-5));
}

} // namespace independent_support_layer_height_contracts

} // namespace support_settings_variance_contracts

namespace support_bridge_no_support_contracts {

struct BridgeSupportEvidence {
    double bridge_area_mm2 = 0.0;
    double cantilever_area_mm2 = 0.0;
    double total_area_mm2 = 0.0;
    double bridge_surface_area_mm2 = 0.0;
    double support_path_length_mm = 0.0;
    size_t support_layers = 0;
    size_t bridge_surface_count = 0;
    size_t support_path_count = 0;
    bool bridge_surface_angles_valid = true;
    bool finite = true;
};

// Two disconnected roof slabs keep the oracle local: the first spans two 4 mm pillars
// (x=4..16 is a genuine two-sided bridge), while the second extends past one 4 mm pillar
// and is therefore a non-bridge support control. Both slabs are at the same height.
static TriangleMesh bridge_and_cantilever_fixture()
{
    TriangleMesh result = make_cube(4.0, 8.0, 4.0); // left bridge pillar
    TriangleMesh right_pillar = make_cube(4.0, 8.0, 4.0);
    right_pillar.translate(16.0, 0.0, 0.0);
    result.merge(right_pillar);

    TriangleMesh bridge_roof = make_cube(20.0, 8.0, 0.4);
    bridge_roof.translate(0.0, 0.0, 4.0);
    result.merge(bridge_roof);

    TriangleMesh cantilever_pillar = make_cube(4.0, 8.0, 4.0);
    cantilever_pillar.translate(26.0, 0.0, 0.0);
    result.merge(cantilever_pillar);
    TriangleMesh cantilever_roof = make_cube(12.0, 8.0, 0.4);
    cantilever_roof.translate(26.0, 0.0, 4.0);
    result.merge(cantilever_roof);
    return result;
}

static ExPolygon model_rectangle(double x0, double y0, double x1, double y1, const Point &center_offset)
{
    return ExPolygon({ Point::new_scale(x0, y0) - center_offset,
                       Point::new_scale(x1, y0) - center_offset,
                       Point::new_scale(x1, y1) - center_offset,
                       Point::new_scale(x0, y1) - center_offset });
}

static double polygons_area_mm2(const ExPolygons &polygons)
{
    double area = 0.0;
    for (const ExPolygon &polygon : polygons)
        area += std::abs(polygon.area());
    return unscale<double>(unscale<double>(area));
}

static BridgeSupportEvidence process_bridge_fixture(bool bridge_no_support)
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        {"enable_support", 1},
        {"support_type", "normal(auto)"},
        {"support_style", "default"},
        {"layer_height", 0.2},
        {"initial_layer_print_height", 0.2},
        {"nozzle_diameter", "0.4"},
        {"line_width", 0.42},
        {"outer_wall_line_width", 0.42},
        {"inner_wall_line_width", 0.42},
        {"wall_generator", "classic"},
        {"detect_overhang_wall", 1},
        {"support_threshold_angle", 0},
        {"support_threshold_overlap", 0.0},
        {"support_on_build_plate_only", 1},
        {"support_object_xy_distance", 0.35},
        {"support_top_z_distance", 0.2},
        {"support_bottom_z_distance", 0.2},
        {"support_interface_top_layers", 0},
        {"support_interface_bottom_layers", 0},
        {"support_remove_small_overhang", 0},
        {"support_expansion", 0.0},
        {"support_critical_regions_only", 0},
        {"bridge_no_support", bridge_no_support},
        // max_bridge_length belongs to the tree-support path; keep it neutral here so this
        // normal-support test isolates bridge_no_support.
        {"max_bridge_length", 0.0},
        {"wall_loops", 2},
        {"top_shell_layers", 1},
        {"bottom_shell_layers", 1},
        {"sparse_infill_density", "0%"},
        {"skirt_loops", 0},
        {"brim_width", 0.0},
        {"raft_layers", 0},
        {"gcode_flavor", "marlin"},
        {"use_relative_e_distances", 1},
        {"layer_change_gcode", "G92 E0\n"},
        {"enable_prime_tower", 0},
        {"z_hop", 0.0}
    });

    Print print;
    Model model;
    init_print({bridge_and_cantilever_fixture()}, print, model, config, nullptr, false);
    const StringObjectException validation = print.validate();
    INFO(validation.string);
    REQUIRE(validation.string.empty());
    REQUIRE(print.objects().size() == 1);
    print.process();

    const PrintObject &object = *print.objects().front();
    BridgeSupportEvidence out;
    out.support_layers = object.support_layers().size();
    REQUIRE(out.support_layers > 0);

    // Slice coordinates are centered by PrintObject; convert the model-space bridge and
    // cantilever windows with the public center offset before clipping support islands.
    const Point &center_offset = object.center_offset();
    const ExPolygon bridge_window = model_rectangle(4.5, 1.0, 15.5, 7.0, center_offset);
    const ExPolygon cantilever_window = model_rectangle(30.5, 1.0, 37.5, 7.0, center_offset);

    // stBottomBridge belongs to model LayerRegion fill surfaces, not SupportLayer regions.
    // Scan the processed object layers independently from support-island evidence.
    for (const Layer *layer : object.layers()) {
        for (const LayerRegion *region : layer->regions()) {
            for (const Surface &surface : region->fill_surfaces.surfaces) {
                if (surface.surface_type != stBottomBridge)
                    continue;
                const double clipped = polygons_area_mm2(intersection_ex(surface.expolygon, bridge_window));
                if (clipped <= 0.0)
                    continue;
                ++out.bridge_surface_count;
                out.bridge_surface_area_mm2 += clipped;
                out.bridge_surface_angles_valid = out.bridge_surface_angles_valid &&
                    std::isfinite(surface.bridge_angle) && surface.bridge_angle >= 0.0;
            }
        }
    }

    for (const SupportLayer *layer : object.support_layers()) {
        const double total = polygons_area_mm2(layer->support_islands);
        const double bridge = polygons_area_mm2(intersection_ex(layer->support_islands, bridge_window));
        const double cantilever = polygons_area_mm2(intersection_ex(layer->support_islands, cantilever_window));
        out.total_area_mm2 += total;
        out.bridge_area_mm2 += bridge;
        out.cantilever_area_mm2 += cantilever;
        out.finite = out.finite && std::isfinite(total) && std::isfinite(bridge) && std::isfinite(cantilever);

        for_each_extrusion_path(layer->support_fills, [&out](const ExtrusionPath &path) {
            if (path.role() != erSupportMaterial && path.role() != erSupportMaterialInterface)
                return;
            ++out.support_path_count;
            out.finite = out.finite && std::isfinite(path.mm3_per_mm) && path.mm3_per_mm > 0.0;
            for (size_t i = 1; i < path.polyline.points.size(); ++i) {
                const Point3 &a = path.polyline.points[i - 1];
                const Point3 &b = path.polyline.points[i];
                const double dx = unscale<double>(static_cast<double>(b.x()) - static_cast<double>(a.x()));
                const double dy = unscale<double>(static_cast<double>(b.y()) - static_cast<double>(a.y()));
                const double length = std::hypot(dx, dy);
                out.finite = out.finite && std::isfinite(dx) && std::isfinite(dy) && std::isfinite(length);
                if (std::isfinite(length))
                    out.support_path_length_mm += length;
            }
        });
    }
    return out;
}

} // namespace support_bridge_no_support_contracts

TEST_CASE("Bridge suppression reduces bridge support while retaining cantilever support",
          "[BridgeNoSupport][SupportMaterial]")
{
    const auto with_bridge_support = support_bridge_no_support_contracts::process_bridge_fixture(false);
    const auto without_bridge_support = support_bridge_no_support_contracts::process_bridge_fixture(true);
    REQUIRE(with_bridge_support.finite);
    REQUIRE(without_bridge_support.finite);
    REQUIRE(with_bridge_support.total_area_mm2 > 0.0);
    REQUIRE(without_bridge_support.total_area_mm2 > 0.0);
    REQUIRE(with_bridge_support.bridge_surface_count > 0);
    REQUIRE(with_bridge_support.bridge_surface_area_mm2 > 1.0);
    REQUIRE(with_bridge_support.bridge_surface_angles_valid);
    REQUIRE(with_bridge_support.support_path_count > 0);
    REQUIRE(with_bridge_support.support_path_length_mm > 1.0);
    REQUIRE(without_bridge_support.support_path_count > 0);
    REQUIRE(without_bridge_support.support_path_length_mm > 1.0);
    REQUIRE(with_bridge_support.bridge_area_mm2 > 1.0);
    REQUIRE(with_bridge_support.cantilever_area_mm2 > 1.0);
    REQUIRE(without_bridge_support.cantilever_area_mm2 > 1.0);
    CHECK(with_bridge_support.bridge_area_mm2 > without_bridge_support.bridge_area_mm2 + 1.0);
}
namespace support_remove_small_overhang_followup {

struct SupportRemovalRun {
    std::size_t support_layers = 0;
    std::size_t support_paths = 0;
    double      support_area_mm2 = 0.0;
    double      support_path_length_mm = 0.0;
    bool        finite = true;
};

TriangleMesh symmetric_ledges(double overhang)
{
    TriangleMesh lower = make_cube(20.0, 20.0, 2.0);
    TriangleMesh upper = make_cube(20.0 + 2.0 * overhang, 20.0, 2.0);
    upper.translate(float(-overhang), 0.f, 2.f);
    lower.merge(upper);
    return lower;
}

DynamicPrintConfig small_overhang_config(bool remove_small_overhang)
{
    DynamicPrintConfig config = support_settings_variance_contracts::support_config();
    config.set_deserialize_strict({
        { "support_type",                 "normal(auto)" },
        { "support_style",                "default" },
        { "support_remove_small_overhang", remove_small_overhang },
        { "support_threshold_angle",      0.0 },
        { "support_threshold_overlap",    0.0 },
        { "support_on_build_plate_only",  true },
        { "support_object_xy_distance",   0.1 },
        { "support_top_z_distance",       0.2 },
        { "support_bottom_z_distance",    0.2 },
        { "support_interface_top_layers", 0 },
        { "support_interface_bottom_layers", 0 },
        { "support_expansion",            0.0 },
        { "outer_wall_line_width",         0.4 },
        { "inner_wall_line_width",         0.4 },
        { "support_line_width",             0.4 },
        { "wall_generator",                "classic" },
        { "bridge_no_support",              false },
        { "support_critical_regions_only", false },
        { "wall_loops",                   2 },
        { "top_shell_layers",             2 },
        { "bottom_shell_layers",          2 },
        { "skirt_loops",                  0 },
        { "brim_width",                   0.0 },
        { "raft_layers",                  0 },
        { "nozzle_diameter",              "0.4" },
        { "line_width",                   0.4 },
        { "layer_height",                 0.2 },
        { "initial_layer_print_height",   0.2 },
    });
    return config;
}

SupportRemovalRun run_symmetric_ledges(double overhang, bool remove_small_overhang)
{
    Print print;
    Model model;
    DynamicPrintConfig config = small_overhang_config(remove_small_overhang);
    init_print({symmetric_ledges(overhang)}, print, model, config, nullptr, false);
    const StringObjectException validation = print.validate();
    INFO("small-overhang validation: " << validation.string);
    REQUIRE(validation.string.empty());
    REQUIRE(print.objects().size() == 1);
    print.process();
    // Print::process() completes slicing/support steps; psGCodeExport is not
    // run by this helper. Check that processing produced object layers, then
    // retain the support-area/path assertions below as the setting oracle.
    REQUIRE_FALSE(print.objects().front()->layers().empty());

    const PrintObject *object = print.objects().front();
    SupportRemovalRun result;
    result.support_layers = object->support_layers().size();
    for (const SupportLayer *layer : object->support_layers()) {
        for (const ExPolygon &island : layer->support_islands) {
            result.support_area_mm2 += unscale<double>(unscale<double>(std::abs(island.area())));
        }
        for_each_extrusion_path(layer->support_fills, [&result](const ExtrusionPath &path) {
            if (path.role() != erSupportMaterial && path.role() != erSupportMaterialInterface)
                return;
            ++result.support_paths;
            result.finite = result.finite && std::isfinite(path.mm3_per_mm) && path.mm3_per_mm > 0.0;
            for (std::size_t i = 1; i < path.polyline.points.size(); ++i) {
                const Point3 &a = path.polyline.points[i - 1];
                const Point3 &b = path.polyline.points[i];
                const double dx = unscale<double>(static_cast<double>(b.x()) - static_cast<double>(a.x()));
                const double dy = unscale<double>(static_cast<double>(b.y()) - static_cast<double>(a.y()));
                const double length = std::hypot(dx, dy);
                result.finite = result.finite && std::isfinite(dx) && std::isfinite(dy) && std::isfinite(length);
                if (std::isfinite(length))
                    result.support_path_length_mm += length;
            }
        });
    }
    return result;
}

} // namespace support_remove_small_overhang_followup

TEST_CASE("Support removal setting drops only small non-cantilever overhangs",
          "[SupportMaterial][SupportRemoveSmallOverhang]")
{
    const auto kept_small = support_remove_small_overhang_followup::run_symmetric_ledges(0.6, false);
    const auto removed_small = support_remove_small_overhang_followup::run_symmetric_ledges(0.6, true);
    const auto kept_large = support_remove_small_overhang_followup::run_symmetric_ledges(1.5, true);

    REQUIRE(kept_small.finite);
    REQUIRE(removed_small.finite);
    REQUIRE(kept_large.finite);
    REQUIRE(kept_small.support_area_mm2 > 0.1);
    REQUIRE(kept_small.support_paths > 0);
    REQUIRE(kept_small.support_path_length_mm > 0.1);
    CHECK_THAT(removed_small.support_area_mm2, Catch::Matchers::WithinAbs(0.0, 1e-9));
    CHECK(removed_small.support_paths == 0);
    CHECK_THAT(removed_small.support_path_length_mm, Catch::Matchers::WithinAbs(0.0, 1e-9));
    REQUIRE(kept_large.support_area_mm2 > 0.1);
    REQUIRE(kept_large.support_paths > 0);
    REQUIRE(kept_large.support_path_length_mm > 0.1);
}


namespace gcode_process_layer_support_contracts {
namespace {

std::set<int> tools_for_support_role(const std::string &gcode, const DynamicPrintConfig &config,
                                     const bool interface_role,
                                     std::map<double, std::set<int>> *tools_by_layer = nullptr)
{
    std::set<int> tools;
    int current_tool = -1;
    double nominal_z = std::numeric_limits<double>::quiet_NaN();
    GCodeReader reader;
    reader.apply_config(config);
    reader.parse_buffer(gcode, [&tools, &current_tool, &nominal_z, interface_role, tools_by_layer](GCodeReader &self,
                                                                       const GCodeReader::GCodeLine &line) {
        std::string_view comment = line.comment();
        const auto comment_start = comment.find_first_not_of(" \t");
        if (comment_start != std::string_view::npos)
            comment.remove_prefix(comment_start);
        if (comment.rfind("LAYER_CHANGE", 0) == 0)
            nominal_z = std::numeric_limits<double>::quiet_NaN();
        else if (comment.rfind("Z:", 0) == 0)
            nominal_z = std::stod(std::string(comment.substr(2)));
        else if (comment.rfind("Z_HEIGHT:", 0) == 0)
            nominal_z = std::stod(std::string(comment.substr(9)));
        const std::string_view command = line.cmd();
        if (command.size() >= 2 && command.front() == 'T' && std::isdigit(static_cast<unsigned char>(command[1])))
            current_tool = std::stoi(std::string(command.substr(1)));
        if (!line.extruding(self) || current_tool < 0)
            return;
        const bool is_interface = comment.find("support material interface") != std::string_view::npos;
        const bool is_base = !is_interface && comment.find("support material") != std::string_view::npos;
        if ((interface_role ? is_interface : is_base)) {
            tools.insert(current_tool);
            if (tools_by_layer != nullptr) {
                // Motion Z is rounded and need not identify the exact nominal
                // layer used by ToolOrdering. The layer metadata does.
                REQUIRE(std::isfinite(nominal_z));
                (*tools_by_layer)[nominal_z].insert(current_tool);
            }
        }
    });
    return tools;
}

DynamicPrintConfig support_tool_selection_config(const int support_filament,
                                                const int interface_filament,
                                                const char *outer_filament,
                                                const char *infill_filament,
                                                const char *soluble)
{
    return multifilament_config(2, {
        {"enable_support", 1},
        {"layer_height", 0.2},
        {"initial_layer_print_height", 0.2},
        {"gcode_flavor", "marlin"},
        {"support_on_build_plate_only", 1},
        {"support_interface_top_layers", 3},
        {"support_interface_bottom_layers", 0},
        {"support_base_pattern_spacing", 2.5},
        {"support_filament", support_filament},
        {"support_interface_filament", interface_filament},
        {"outer_wall_filament_id", outer_filament},
        {"inner_wall_filament_id", outer_filament},
        {"sparse_infill_filament_id", infill_filament},
        {"internal_solid_filament_id", infill_filament},
        {"top_surface_filament_id", infill_filament},
        {"bottom_surface_filament_id", infill_filament},
        {"filament_soluble", soluble},
        {"filament_is_support", "0,0"},
        {"gcode_comments", 1},
        {"use_relative_e_distances", 1},
        {"layer_change_gcode", "G92 E0\n"},
        {"skirt_loops", 0},
        {"brim_type", "no_brim"},
        {"raft_layers", 0},
    });
}

} // namespace

TEST_CASE("Support dont-care base avoids an explicitly selected interface tool",
          "[SupportMaterial][GCode][GCodePipeline][GCodeSupportDontCareInterface]")
{
    const DynamicPrintConfig config = support_tool_selection_config(0, 1, "2", "2", "0,0");
    const std::string gcode = slice({TestMesh::overhang}, config);
    const std::set<int> base_tools = tools_for_support_role(gcode, config, false);
    const std::set<int> interface_tools = tools_for_support_role(gcode, config, true);

    REQUIRE(!gcode.empty());
    REQUIRE(!base_tools.empty());
    REQUIRE(!interface_tools.empty());
    CHECK(base_tools == std::set<int>{1});
    CHECK(interface_tools == std::set<int>{0});
}

TEST_CASE("Unspecified support roles avoid the currently soluble filament",
          "[SupportMaterial][GCode][GCodePipeline][GCodeSupportSolublePreference]")
{
    DynamicPrintConfig soluble_config = support_tool_selection_config(0, 0, "1", "2", "1,0");
    // Object layers offer both materials. Independent support-only layers may
    // inherit just the previous tool; selection cannot use an unavailable tool.
    soluble_config.set_deserialize_strict({
        {"wall_loops", 2},
        {"inner_wall_filament_id", "2"},
        {"print_sequence", "by layer"},
        {"independent_support_layer_height", true},
        {"enable_prime_tower", false},
        {"flush_into_support", false},
        {"flush_into_infill", false},
        {"flush_into_objects", false},
        {"z_offset", 0.0},
    });
    Print print;
    Model model;
    init_print({TestMesh::overhang}, print, model, soluble_config);
    const std::string soluble_gcode = gcode(print);
    std::map<double, std::set<int>> base_by_layer;
    std::map<double, std::set<int>> interface_by_layer;
    const std::set<int> soluble_base_tools = tools_for_support_role(soluble_gcode, soluble_config, false, &base_by_layer);
    const std::set<int> soluble_interface_tools = tools_for_support_role(soluble_gcode, soluble_config, true, &interface_by_layer);

    REQUIRE(!soluble_gcode.empty());
    REQUIRE(!soluble_base_tools.empty());
    REQUIRE(!soluble_interface_tools.empty());
    REQUIRE_FALSE(print.tool_ordering().empty());
    size_t soluble_first_with_choice = 0;
    for (const auto *layers : {&base_by_layer, &interface_by_layer}) {
        size_t non_soluble_choices = 0;
        for (const auto &[z, emitted_tools] : *layers) {
            const LayerTools &layer = print.tool_ordering().tools_for_layer(z);
            CAPTURE(z, layer.print_z, layer.extruders, emitted_tools);
            REQUIRE_THAT(layer.print_z, Catch::Matchers::WithinAbs(z, 0.001));
            REQUIRE_FALSE(layer.extruders.empty());
            REQUIRE(std::all_of(layer.extruders.begin(), layer.extruders.end(),
                                [](unsigned int tool) { return tool <= 1; }));
            const bool non_soluble_available =
                std::find(layer.extruders.begin(), layer.extruders.end(), 1u) != layer.extruders.end();
            // Tool 1 is the only non-soluble material in this pinned fixture.
            const std::set<int> expected{non_soluble_available ? 1 : 0};
            CHECK(emitted_tools == expected);
            if (non_soluble_available) {
                ++non_soluble_choices;
                if (layer.extruders.front() == 0)
                    ++soluble_first_with_choice;
            }
        }
        REQUIRE(non_soluble_choices > 0);
    }
    // Witness the branch's actual decision, not just layers that start on the
    // correct tool already. Removing the soluble preference must change output.
    REQUIRE(soluble_first_with_choice > 0);
}

} // namespace gcode_process_layer_support_contracts

namespace support_overlap_contracts {

struct OverlapEvidence {
    std::size_t support_layers = 0;
    std::size_t interface_paths = 0;
    std::size_t split_layers = 0;
    std::size_t reduced_paths = 0;
    std::size_t reduced_flow_paths = 0;
    double support_area_mm2 = 0.0;
    double interface_volume_mm3 = 0.0;
    double min_height = std::numeric_limits<double>::infinity();
    double max_height = -std::numeric_limits<double>::infinity();
    bool finite = true;
};

static OverlapEvidence process_fixture(bool zero_gap)
{
    // Reuse the existing support fixture baseline: it pins relative-E mode and the valid G92 E0 layer reset.
    DynamicPrintConfig config = support_settings_variance_contracts::support_config();
    config.set_deserialize_strict({
        {"enable_support",                    true},
        {"support_type",                      "normal(auto)"},
        {"support_style",                     "default"},
        {"support_on_build_plate_only",       false},
        {"support_angle",                     0.0},
        {"support_threshold_angle",            0.0},
        {"support_threshold_overlap",          0.0},
        {"support_object_xy_distance",         0.0},
        {"support_expansion",                  0.0},
        {"support_remove_small_overhang",      false},
        {"support_critical_regions_only",     false},
        {"support_interface_top_layers",       0},
        {"support_interface_bottom_layers",    6},
        {"support_interface_spacing",          0.0},
        {"support_bottom_interface_spacing",   0.0},
        {"support_top_z_distance",             0.2},
        {"support_bottom_z_distance",          zero_gap ? 0.0 : 0.2},
        {"independent_support_layer_height",   true},
        {"thick_bridges",                      true},
        {"bridge_flow",                        1.25},
        {"bridge_no_support",                  false},
        {"wall_generator",                     "classic"},
        {"wall_loops",                         2},
        {"top_shell_layers",                  2},
        {"bottom_shell_layers",               2},
        {"outer_wall_line_width",              0.4},
        {"inner_wall_line_width",              0.4},
        {"support_line_width",                 0.4},
        {"line_width",                         0.4},
        {"nozzle_diameter",                    "0.4"},
        {"layer_height",                       0.2},
        {"initial_layer_print_height",         0.2},
        {"sparse_infill_density",              "0%"},
        {"enable_prime_tower",                 false},
        {"z_hop",                              0.0},
        {"skirt_loops",                        0},
        {"brim_type",                          "no_brim"},
        {"raft_layers",                        0}
    });

    Print print;
    Model model;
    init_print({support_capital()}, print, model, config, nullptr, false);
    const StringObjectException validation = print.validate();
    INFO("support overlap validation: " << validation.string);
    REQUIRE(validation.string.empty());
    REQUIRE(print.objects().size() == 1);
    print.process();

    const PrintObject &object = *print.objects().front();
    OverlapEvidence out;
    out.support_layers = object.support_layers().size();
    for (const SupportLayer *layer : object.support_layers()) {
        for (const ExPolygon &island : layer->support_islands)
            out.support_area_mm2 += unscale<double>(unscale<double>(std::abs(island.area())));

        std::vector<std::pair<double, double>> interface_flows;
        for_each_extrusion_path(layer->support_fills, [&out, &interface_flows](const ExtrusionPath &path) {
            if (path.role() != erSupportMaterialInterface)
                return;
            ++out.interface_paths;
            const double length = path.length();
            const double volume = path.total_volume();
            const double height = static_cast<double>(path.height);
            const double flow = path.mm3_per_mm;
            out.finite = out.finite && std::isfinite(length) && length > 0.0;
            out.finite = out.finite && std::isfinite(volume) && volume > 0.0;
            out.finite = out.finite && std::isfinite(height) && height > 0.0;
            out.finite = out.finite && std::isfinite(flow) && flow > 0.0;
            out.interface_volume_mm3 += volume;
            out.min_height = std::min(out.min_height, height);
            out.max_height = std::max(out.max_height, height);
            interface_flows.emplace_back(height, flow);
        });

        if (interface_flows.empty())
            continue;
        const double layer_max_height = std::max_element(interface_flows.begin(), interface_flows.end(),
            [](const auto &a, const auto &b) { return a.first < b.first; })->first;
        const double layer_max_flow = std::max_element(interface_flows.begin(), interface_flows.end(),
            [](const auto &a, const auto &b) { return a.second < b.second; })->second;
        bool split = false;
        for (const auto &[height, flow] : interface_flows) {
            if (height + 0.01 >= layer_max_height)
                continue;
            split = true;
            ++out.reduced_paths;
            if (flow + 1e-6 < layer_max_flow)
                ++out.reduced_flow_paths;
        }
        if (split)
            ++out.split_layers;
    }
    return out;
}

} // namespace support_overlap_contracts

TEST_CASE("Bottom contact modulation distinguishes positive and zero bottom gaps",
          "[SupportMaterial][SupportOverlap]")
{
    const support_overlap_contracts::OverlapEvidence positive_gap =
        support_overlap_contracts::process_fixture(false);
    const support_overlap_contracts::OverlapEvidence zero_gap =
        support_overlap_contracts::process_fixture(true);

    for (const auto &evidence : {positive_gap, zero_gap}) {
        REQUIRE(evidence.support_layers > 0);
        REQUIRE(evidence.interface_paths > 0);
        REQUIRE(evidence.finite);
        REQUIRE(std::isfinite(evidence.support_area_mm2));
        REQUIRE(evidence.support_area_mm2 > 0.0);
        REQUIRE(std::isfinite(evidence.interface_volume_mm3));
        REQUIRE(evidence.interface_volume_mm3 > 0.0);
        REQUIRE(std::isfinite(evidence.min_height));
        REQUIRE(std::isfinite(evidence.max_height));
        REQUIRE(evidence.max_height >= evidence.min_height);
    }

    // A positive bottom gap makes the bottom contact a bridging layer. Its generated interface
    // path overlaps lower support material, so the public collection contains both the original
    // bridge-height path and a reduced-height fragment with a correspondingly lower flow.
    REQUIRE(positive_gap.split_layers > 0);
    REQUIRE(positive_gap.reduced_paths > 0);
    REQUIRE(positive_gap.reduced_flow_paths > 0);
    REQUIRE(positive_gap.max_height > positive_gap.min_height + 0.01);

    // With zero bottom gap, the source aligns the bottom contact to the object's layer and clears
    // the bridging flag; no reduced-height overlap fragment should be emitted.
    CHECK(zero_gap.split_layers == 0);
    CHECK(zero_gap.reduced_paths == 0);
    CHECK(zero_gap.reduced_flow_paths == 0);
    CHECK_THAT(zero_gap.max_height - zero_gap.min_height, Catch::Matchers::WithinAbs(0.0, 0.01));
}
