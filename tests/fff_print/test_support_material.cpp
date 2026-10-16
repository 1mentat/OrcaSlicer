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
#include "libslic3r/Layer.hpp"

#include <cmath>
#include <cstddef>
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

} // namespace support_settings_variance_contracts
