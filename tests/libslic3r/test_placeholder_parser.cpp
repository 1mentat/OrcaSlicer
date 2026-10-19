#include <catch2/catch_all.hpp>
#include <catch2/catch_message.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <initializer_list>
#include <random>
#include <utility>
#include <cstddef>
#include <vector>
#include <string>
#include <memory>
#include "libslic3r/CommonDefs.hpp"
#include "libslic3r/Config.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/libslic3r.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include "libslic3r/PlaceholderParser.hpp"
#include "libslic3r/PrintConfig.hpp"

using namespace Slic3r;

SCENARIO("Placeholder parser scripting", "[PlaceholderParser]") {
    PlaceholderParser parser;
    auto config = DynamicPrintConfig::full_print_config();

    config.set_deserialize_strict( {
	    { "printer_notes", "  PRINTER_VENDOR_PRUSA3D  PRINTER_MODEL_MK2  " },
	    { "nozzle_diameter", "0.6,0.6,0.6,0.6" },
	    { "nozzle_temperature", "357,359,363,378" }
	});
    // To test the "min_width_top_surface" over "inner_wall_line_width".
    config.option<ConfigOptionFloatOrPercent>("inner_wall_line_width")->value = 150.;
    config.option<ConfigOptionFloatOrPercent>("inner_wall_line_width")->percent = true;
    // To let the PlaceholderParser throw when referencing scarf_joint_speed if it is set to percent, as the PlaceholderParser does not know
    // a percent to what.
    config.option<ConfigOptionFloatOrPercent>("scarf_joint_speed")->value = 50.;
    config.option<ConfigOptionFloatOrPercent>("scarf_joint_speed")->percent = true;

    parser.apply_config(config);
    parser.set("foo", 0);
    parser.set("bar", 2);
    parser.set("num_extruders", 4);

    SECTION("nested config options (legacy syntax)") { REQUIRE(parser.process("[nozzle_temperature[foo]]") == "357"); }
    SECTION("array reference") { REQUIRE(parser.process("{nozzle_temperature[foo]}") == "357"); }
    SECTION("whitespaces and newlines are maintained") { REQUIRE(parser.process("test [ nozzle_temperature [foo] ] \n hu") == "test 357 \n hu"); }

    // Test the math expressions.
    SECTION("math: 2*3") { REQUIRE(parser.process("{2*3}") == "6"); }
    SECTION("math: 2*3/6") { REQUIRE(parser.process("{2*3/6}") == "1"); }
    SECTION("math: 2*3/12") { REQUIRE(parser.process("{2*3/12}") == "0"); }
    SECTION("math: 2.*3/12") { REQUIRE(std::stod(parser.process("{2.*3/12}")) == Catch::Approx(0.5)); }
    SECTION("math: 10 % 2.5") { REQUIRE(std::stod(parser.process("{10%2.5}")) == Catch::Approx(0.)); }
    SECTION("math: 11 % 2.5") { REQUIRE(std::stod(parser.process("{11%2.5}")) == Catch::Approx(1.)); }
    SECTION("math: 2*(3-12)") { REQUIRE(parser.process("{2*(3-12)}") == "-18"); }
    SECTION("math: 2*foo*(3-12)") { REQUIRE(parser.process("{2*foo*(3-12)}") == "0"); }
    SECTION("math: 2*bar*(3-12)") { REQUIRE(parser.process("{2*bar*(3-12)}") == "-36"); }
    SECTION("math: 2.5*bar*(3-12)") { REQUIRE(std::stod(parser.process("{2.5*bar*(3-12)}")) == Catch::Approx(-45)); }
    SECTION("math: min(12, 14)") { REQUIRE(parser.process("{min(12, 14)}") == "12"); }
    SECTION("math: max(12, 14)") { REQUIRE(parser.process("{max(12, 14)}") == "14"); }
    SECTION("math: min(13.4, -1238.1)") { REQUIRE(std::stod(parser.process("{min(13.4, -1238.1)}")) == Catch::Approx(-1238.1)); }
    SECTION("math: max(13.4, -1238.1)") { REQUIRE(std::stod(parser.process("{max(13.4, -1238.1)}")) == Catch::Approx(13.4)); }
    SECTION("math: int(13.4)") { REQUIRE(parser.process("{int(13.4)}") == "13"); }
    SECTION("math: int(-13.4)") { REQUIRE(parser.process("{int(-13.4)}") == "-13"); }
    SECTION("math: round(13.4)") { REQUIRE(parser.process("{round(13.4)}") == "13"); }
    SECTION("math: round(-13.4)") { REQUIRE(parser.process("{round(-13.4)}") == "-13"); }
    SECTION("math: round(13.6)") { REQUIRE(parser.process("{round(13.6)}") == "14"); }
    SECTION("math: round(-13.6)") { REQUIRE(parser.process("{round(-13.6)}") == "-14"); }
    SECTION("math: round(13.5)") { REQUIRE(parser.process("{round(13.5)}") == "14"); }
    SECTION("math: floor(13.9)") { REQUIRE(parser.process("{floor(13.9)}") == "13"); }
    SECTION("math: floor(-13.1)") { REQUIRE(parser.process("{floor(-13.1)}") == "-14"); }
    SECTION("math: ceil(13.1)") { REQUIRE(parser.process("{ceil(13.1)}") == "14"); }
    SECTION("math: ceil(-13.9)") { REQUIRE(parser.process("{ceil(-13.9)}") == "-13"); }
    SECTION("math: digits(5, 15)") { REQUIRE(parser.process("{digits(5, 15)}") == "              5"); }
    SECTION("math: digits(5., 15)") { REQUIRE(parser.process("{digits(5., 15)}") == "              5"); }
    SECTION("math: zdigits(5, 15)") { REQUIRE(parser.process("{zdigits(5, 15)}") == "000000000000005"); }
    SECTION("math: zdigits(5., 15)") { REQUIRE(parser.process("{zdigits(5., 15)}") == "000000000000005"); }
    SECTION("math: digits(5, 15, 8)") { REQUIRE(parser.process("{digits(5, 15, 8)}") == "     5.00000000"); }
    SECTION("math: digits(5., 15, 8)") { REQUIRE(parser.process("{digits(5, 15, 8)}") == "     5.00000000"); }
    SECTION("math: zdigits(5, 15, 8)") { REQUIRE(parser.process("{zdigits(5, 15, 8)}") == "000005.00000000"); }
    SECTION("math: zdigits(5., 15, 8)") { REQUIRE(parser.process("{zdigits(5, 15, 8)}") == "000005.00000000"); }
    SECTION("math: digits(13.84375892476, 15, 8)") { REQUIRE(parser.process("{digits(13.84375892476, 15, 8)}") == "    13.84375892"); }
    SECTION("math: zdigits(13.84375892476, 15, 8)") { REQUIRE(parser.process("{zdigits(13.84375892476, 15, 8)}") == "000013.84375892"); }
    SECTION("math: interpolate_table(13.84375892476, (0, 0), (20, 20))") { REQUIRE(std::stod(parser.process("{interpolate_table(13.84375892476, (0, 0), (20, 20))}")) == Catch::Approx(13.84375892476)); }
    SECTION("math: interpolate_table(13, (0, 0), (20, 20), (30, 20))") { REQUIRE(std::stod(parser.process("{interpolate_table(13, (0, 0), (20, 20), (30, 20))}")) == Catch::Approx(13.)); }
    SECTION("math: interpolate_table(25, (0, 0), (20, 20), (30, 20))") { REQUIRE(std::stod(parser.process("{interpolate_table(25, (0, 0), (20, 20), (30, 20))}")) == Catch::Approx(20.)); }
    // Only the grammar's built-in functions are callable; any other name is an undefined variable and throws.
    SECTION("math: a non-built-in function name throws") { REQUIRE_THROWS(parser.process("{sqrt(16)}")); }

    // regex_replace(subject, /pattern/, replacement): the string-transform primitive.
    SECTION("regex_replace: strips a file extension") { REQUIRE(parser.process("{regex_replace(\"part.stl\", /\\.[^.]*$/, \"\")}") == "part"); }
    SECTION("regex_replace: leaves a non-matching dot untouched") { REQUIRE(parser.process("{regex_replace(\"Bracket v2.1\", /\\.stl$/, \"\")}") == "Bracket v2.1"); }
    SECTION("regex_replace: replaces every match") { REQUIRE(parser.process("{regex_replace(\"a-b-c\", /-/, \"_\")}") == "a_b_c"); }
    SECTION("regex_replace: replacement may reference a capture group") { REQUIRE(parser.process("{regex_replace(\"v12\", /v(\\d+)/, \"$1\")}") == "12"); }
    // The result is an ordinary string, usable in further expressions (the real filename-template shape).
    SECTION("regex_replace: result composes with concatenation") { REQUIRE(parser.process("{regex_replace(\"part.stl\", /\\.[^.]*$/, \"\") + \".gcode\"}") == "part.gcode"); }
    // A malformed pattern and a non-string subject are both hard errors.
    SECTION("regex_replace: an invalid pattern throws") { REQUIRE_THROWS(parser.process("{regex_replace(\"x\", /[/, \"\")}")); }
    SECTION("regex_replace: a non-string subject throws") { REQUIRE_THROWS(parser.process("{regex_replace(123, /2/, \"\")}")); }
    // Inside a skipped branch the subject is TYPE_EMPTY and the call must no-op (exercises the guard).
    SECTION("regex_replace: is skipped inside a false if-branch") { REQUIRE(parser.process("{if false}{regex_replace(\"x\", /x/, \"y\")}{endif}done") == "done"); }

    // Test the "coFloatOrPercent" and "xxx_line_width" substitutions.
    // min_width_top_surface ratio_over inner_wall_line_width.
    SECTION("line_width") { REQUIRE(std::stod(parser.process("{line_width}")) == Catch::Approx(0.67500001192092896)); }
    SECTION("min_width_top_surface") { REQUIRE(std::stod(parser.process("{min_width_top_surface}")) == Catch::Approx(2.7)); }
    // Orca: this one is not coFloatOrPercent
    //SECTION("support_object_xy_distance") { REQUIRE(std::stod(parser.process("{support_object_xy_distance}")) == Catch::Approx(0.3375)); }
    // Orca: this one is not coFloatOrPercent
    // small_perimeter_speed over outer_wall_speed
    //SECTION("small_perimeter_speed") { REQUIRE(std::stod(parser.process("{small_perimeter_speed}")) == Catch::Approx(30.)); }
    // infill_anchor over sparse_infill_line_width
    SECTION("infill_anchor") { REQUIRE(std::stod(parser.process("{infill_anchor}")) == Catch::Approx(2.7)); }
    // If scarf_joint_speed is set to percent, then it is applied over respective extrusion types by overriding their respective speeds.
    // The PlaceholderParser has no way to know which extrusion type the caller has in mind, therefore it throws.
    SECTION("scarf_joint_speed") { REQUIRE_THROWS(parser.process("{scarf_joint_speed}")); }

    // Test the boolean expression parser.
    auto boolean_expression = [&parser](const std::string& templ) { return parser.evaluate_boolean_expression(templ, parser.config()); };

    SECTION("boolean expression parser: 12 == 12") { REQUIRE(boolean_expression("12 == 12")); }
    SECTION("boolean expression parser: 12 != 12") { REQUIRE(! boolean_expression("12 != 12")); }
    SECTION("boolean expression parser: regex matches") { REQUIRE(boolean_expression("\"has some PATTERN embedded\" =~ /.*PATTERN.*/")); }
    SECTION("boolean expression parser: regex does not match") { REQUIRE(! boolean_expression("\"has some PATTERN embedded\" =~ /.*PTRN.*/")); }
    SECTION("boolean expression parser: accessing variables, equal") { REQUIRE(boolean_expression("foo + 2 == bar")); }
    SECTION("boolean expression parser: accessing variables, not equal") { REQUIRE(! boolean_expression("foo + 3 == bar")); }
    SECTION("boolean expression parser: (12 == 12) and (13 != 14)") { REQUIRE(boolean_expression("(12 == 12) and (13 != 14)")); }
    SECTION("boolean expression parser: (12 == 12) && (13 != 14)") { REQUIRE(boolean_expression("(12 == 12) && (13 != 14)")); }
    SECTION("boolean expression parser: (12 == 12) or (13 == 14)") { REQUIRE(boolean_expression("(12 == 12) or (13 == 14)")); }
    SECTION("boolean expression parser: (12 == 12) || (13 == 14)") { REQUIRE(boolean_expression("(12 == 12) || (13 == 14)")); }
    SECTION("boolean expression parser: (12 == 12) and not (13 == 14)") { REQUIRE(boolean_expression("(12 == 12) and not (13 == 14)")); }
    SECTION("boolean expression parser: ternary true") { REQUIRE(boolean_expression("(12 == 12) ? (1 - 1 == 0) : (2 * 2 == 3)")); }
    SECTION("boolean expression parser: ternary false") { REQUIRE(! boolean_expression("(12 == 21/2) ? (1 - 1 == 0) : (2 * 2 == 3)")); }
    SECTION("boolean expression parser: ternary false 2") { REQUIRE(boolean_expression("(12 == 13) ? (1 - 1 == 3) : (2 * 2 == 4)")); }
    SECTION("boolean expression parser: ternary true 2") { REQUIRE(! boolean_expression("(12 == 2 * 6) ? (1 - 1 == 3) : (2 * 2 == 4)")); }
    SECTION("boolean expression parser: lower than - false") { REQUIRE(! boolean_expression("12 < 3")); }
    SECTION("boolean expression parser: lower than - true") { REQUIRE(boolean_expression("12 < 22")); }
    SECTION("boolean expression parser: greater than - true") { REQUIRE(boolean_expression("12 > 3")); }
    SECTION("boolean expression parser: greater than - false") { REQUIRE(! boolean_expression("12 > 22")); }
    SECTION("boolean expression parser: lower than or equal- false") { REQUIRE(! boolean_expression("12 <= 3")); }
    SECTION("boolean expression parser: lower than or equal - true") { REQUIRE(boolean_expression("12 <= 22")); }
    SECTION("boolean expression parser: greater than or equal - true") { REQUIRE(boolean_expression("12 >= 3")); }
    SECTION("boolean expression parser: greater than or equal - false") { REQUIRE(! boolean_expression("12 >= 22")); }
    SECTION("boolean expression parser: lower than or equal (same values) - true") { REQUIRE(boolean_expression("12 <= 12")); }
    SECTION("boolean expression parser: greater than or equal (same values) - true") { REQUIRE(boolean_expression("12 >= 12")); }
    SECTION("complex expression") { REQUIRE(boolean_expression("printer_notes=~/.*PRINTER_VENDOR_PRUSA3D.*/ and printer_notes=~/.*PRINTER_MODEL_MK2.*/ and nozzle_diameter[0]==0.6 and num_extruders>1")); }
    SECTION("complex expression2") { REQUIRE(boolean_expression("printer_notes=~/.*PRINTER_VEwerfNDOR_PRUSA3D.*/ or printer_notes=~/.*PRINTertER_MODEL_MK2.*/ or (nozzle_diameter[0]==0.6 and num_extruders>1)")); }
    SECTION("complex expression3") { REQUIRE(! boolean_expression("printer_notes=~/.*PRINTER_VEwerfNDOR_PRUSA3D.*/ or printer_notes=~/.*PRINTertER_MODEL_MK2.*/ or (nozzle_diameter[0]==0.3 and num_extruders>1)")); }
}

SCENARIO("Placeholder parser variables", "[PlaceholderParser]") {
    PlaceholderParser 	parser;
    auto 				config = DynamicPrintConfig::full_print_config();

    config.set_deserialize_strict({
        { "filament_notes", "testnotes" },
        { "enable_pressure_advance", "1" },
        { "nozzle_diameter", "0.6,0.6,0.6,0.6" },
        { "nozzle_temperature", "357,359,363,378" }
        });

    PlaceholderParser::ContextData context_with_global_dict;
    context_with_global_dict.global_config = std::make_unique<DynamicConfig>();

    SECTION("create an int local variable") { REQUIRE(parser.process("{local myint = 33+2}{myint}", 0, nullptr, nullptr, nullptr) == "35"); }
    SECTION("create a string local variable") { REQUIRE(parser.process("{local mystr = \"mine\" + \"only\" + \"mine\"}{mystr}", 0, nullptr, nullptr, nullptr) == "mineonlymine"); }
    SECTION("regex_replace transforms a string variable") { REQUIRE(parser.process("{local n = \"part.stl\"}{regex_replace(n, /\\.[^.]*$/, \"\")}", 0, nullptr, nullptr, nullptr) == "part"); }
    SECTION("create a bool local variable") { REQUIRE(parser.process("{local mybool = 1 + 1 == 2}{mybool}", 0, nullptr, nullptr, nullptr) == "true"); }
    SECTION("create an int global variable") { REQUIRE(parser.process("{global myint = 33+2}{myint}", 0, nullptr, nullptr, &context_with_global_dict) == "35"); }
    SECTION("create a string global variable") { REQUIRE(parser.process("{global mystr = \"mine\" + \"only\" + \"mine\"}{mystr}", 0, nullptr, nullptr, &context_with_global_dict) == "mineonlymine"); }
    SECTION("create a bool global variable") { REQUIRE(parser.process("{global mybool = 1 + 1 == 2}{mybool}", 0, nullptr, nullptr, &context_with_global_dict) == "true"); }

    SECTION("create an int local variable and overwrite it") { REQUIRE(parser.process("{local myint = 33+2}{myint = 12}{myint}", 0, nullptr, nullptr, nullptr) == "12"); }
    SECTION("create a string local variable and overwrite it") { REQUIRE(parser.process("{local mystr = \"mine\" + \"only\" + \"mine\"}{mystr = \"yours\"}{mystr}", 0, nullptr, nullptr, nullptr) == "yours"); }
    SECTION("create a bool local variable and overwrite it") { REQUIRE(parser.process("{local mybool = 1 + 1 == 2}{mybool = false}{mybool}", 0, nullptr, nullptr, nullptr) == "false"); }
    SECTION("create an int global variable and overwrite it") { REQUIRE(parser.process("{global myint = 33+2}{myint = 12}{myint}", 0, nullptr, nullptr, &context_with_global_dict) == "12"); }
    SECTION("create a string global variable and overwrite it") { REQUIRE(parser.process("{global mystr = \"mine\" + \"only\" + \"mine\"}{mystr = \"yours\"}{mystr}", 0, nullptr, nullptr, &context_with_global_dict) == "yours"); }
    SECTION("create a bool global variable and overwrite it") { REQUIRE(parser.process("{global mybool = 1 + 1 == 2}{mybool = false}{mybool}", 0, nullptr, nullptr, &context_with_global_dict) == "false"); }

    SECTION("create an int local variable and redefine it") { REQUIRE(parser.process("{local myint = 33+2}{local myint = 12}{myint}", 0, nullptr, nullptr, nullptr) == "12"); }
    SECTION("create a string local variable and redefine it") { REQUIRE(parser.process("{local mystr = \"mine\" + \"only\" + \"mine\"}{local mystr = \"yours\"}{mystr}", 0, nullptr, nullptr, nullptr) == "yours"); }
    SECTION("create a bool local variable and redefine it") { REQUIRE(parser.process("{local mybool = 1 + 1 == 2}{local mybool = false}{mybool}", 0, nullptr, nullptr, nullptr) == "false"); }
    SECTION("create an int global variable and redefine it") { REQUIRE(parser.process("{global myint = 33+2}{global myint = 12}{myint}", 0, nullptr, nullptr, &context_with_global_dict) == "12"); }
    SECTION("create a string global variable and redefine it") { REQUIRE(parser.process("{global mystr = \"mine\" + \"only\" + \"mine\"}{global mystr = \"yours\"}{mystr}", 0, nullptr, nullptr, &context_with_global_dict) == "yours"); }
    SECTION("create a bool global variable and redefine it") { REQUIRE(parser.process("{global mybool = 1 + 1 == 2}{global mybool = false}{mybool}", 0, nullptr, nullptr, &context_with_global_dict) == "false"); }

    SECTION("create an ints local variable with repeat()") { REQUIRE(parser.process("{local myint = repeat(2*3, 4*6)}{myint[5]}", 0, nullptr, nullptr, nullptr) == "24"); }
    SECTION("create a strings local variable with repeat()") { REQUIRE(parser.process("{local mystr = repeat(2*3, \"mine\" + \"only\" + \"mine\")}{mystr[5]}", 0, nullptr, nullptr, nullptr) == "mineonlymine"); }
    SECTION("create a bools local variable with repeat()") { REQUIRE(parser.process("{local mybool = repeat(5, 1 + 1 == 2)}{mybool[4]}", 0, nullptr, nullptr, nullptr) == "true"); }
    SECTION("create an ints global variable with repeat()") { REQUIRE(parser.process("{global myint = repeat(2*3, 4*6)}{myint[5]}", 0, nullptr, nullptr, &context_with_global_dict) == "24"); }
    SECTION("create a strings global variable with repeat()") { REQUIRE(parser.process("{global mystr = repeat(2*3, \"mine\" + \"only\" + \"mine\")}{mystr[5]}", 0, nullptr, nullptr, &context_with_global_dict) == "mineonlymine"); }
    SECTION("create a bools global variable with repeat()") { REQUIRE(parser.process("{global mybool = repeat(5, 1 + 1 == 2)}{mybool[4]}", 0, nullptr, nullptr, &context_with_global_dict) == "true"); }

    SECTION("create an ints local variable with initializer list") { REQUIRE(parser.process("{local myint = (2*3, 4*6, 5*5)}{myint[1]}", 0, nullptr, nullptr, nullptr) == "24"); }
    SECTION("create a strings local variable with initializer list") { REQUIRE(parser.process("{local mystr = (2*3, \"mine\" + \"only\" + \"mine\", 8)}{mystr[1]}", 0, nullptr, nullptr, nullptr) == "mineonlymine"); }
    SECTION("create a bools local variable with initializer list") { REQUIRE(parser.process("{local mybool = (3*3 == 8, 1 + 1 == 2)}{mybool[1]}", 0, nullptr, nullptr, nullptr) == "true"); }
    SECTION("create an ints global variable with initializer list") { REQUIRE(parser.process("{global myint = (2*3, 4*6, 5*5)}{myint[1]}", 0, nullptr, nullptr, &context_with_global_dict) == "24"); }
    SECTION("create a strings global variable with initializer list") { REQUIRE(parser.process("{global mystr = (2*3, \"mine\" + \"only\" + \"mine\", 8)}{mystr[1]}", 0, nullptr, nullptr, &context_with_global_dict) == "mineonlymine"); }
    SECTION("create a bools global variable with initializer list") { REQUIRE(parser.process("{global mybool = (2*3 == 8, 1 + 1 == 2, 5*5 != 33)}{mybool[1]}", 0, nullptr, nullptr, &context_with_global_dict) == "true"); }

    SECTION("create an ints local variable by a copy") { REQUIRE(parser.process("{local myint = nozzle_temperature}{myint[0]}", 0, &config, nullptr, nullptr) == "357"); }
    SECTION("create a strings local variable by a copy") { REQUIRE(parser.process("{local mystr = filament_notes}{mystr[0]}", 0, &config, nullptr, nullptr) == "testnotes"); }
    SECTION("create a bools local variable by a copy") { REQUIRE(parser.process("{local mybool = enable_pressure_advance}{mybool[0]}", 0, &config, nullptr, nullptr) == "true"); }
    SECTION("create an ints global variable by a copy") { REQUIRE(parser.process("{global myint = nozzle_temperature}{myint[0]}", 0, &config, nullptr, &context_with_global_dict) == "357"); }
    SECTION("create a strings global variable by a copy") { REQUIRE(parser.process("{global mystr = filament_notes}{mystr[0]}", 0, &config, nullptr, &context_with_global_dict) == "testnotes"); }
    SECTION("create a bools global variable by a copy") { REQUIRE(parser.process("{global mybool = enable_pressure_advance}{mybool[0]}", 0, &config, nullptr, &context_with_global_dict) == "true"); }

    SECTION("create an ints local variable by a copy and overwrite it") {
        REQUIRE(parser.process("{local myint = nozzle_temperature}{myint = repeat(2*3, 4*6)}{myint[5]}", 0, &config, nullptr, nullptr) == "24");
        REQUIRE(parser.process("{local myint = nozzle_temperature}{myint = (2*3, 4*6)}{myint[1]}", 0, &config, nullptr, nullptr) == "24");
        REQUIRE(parser.process("{local myint = nozzle_temperature}{myint = (1)}{myint = nozzle_temperature}{myint[0]}", 0, &config, nullptr, nullptr) == "357");
    }
    SECTION("create a strings local variable by a copy and overwrite it") {
        REQUIRE(parser.process("{local mystr = filament_notes}{mystr = repeat(2*3, \"mine\" + \"only\" + \"mine\")}{mystr[5]}", 0, &config, nullptr, nullptr) == "mineonlymine");
        REQUIRE(parser.process("{local mystr = filament_notes}{mystr = (2*3, \"mine\" + \"only\" + \"mine\")}{mystr[1]}", 0, &config, nullptr, nullptr) == "mineonlymine");
        REQUIRE(parser.process("{local mystr = filament_notes}{mystr = (2*3, \"mine\" + \"only\" + \"mine\")}{mystr = filament_notes}{mystr[0]}", 0, &config, nullptr, nullptr) == "testnotes");
    }
    SECTION("create a bools local variable by a copy and overwrite it") {
        REQUIRE(parser.process("{local mybool = enable_pressure_advance}{mybool = repeat(2*3, true)}{mybool[5]}", 0, &config, nullptr, nullptr) == "true");
        REQUIRE(parser.process("{local mybool = enable_pressure_advance}{mybool = (false, true)}{mybool[1]}", 0, &config, nullptr, nullptr) == "true");
        REQUIRE(parser.process("{local mybool = enable_pressure_advance}{mybool = (false, false)}{mybool = enable_pressure_advance}{mybool[0]}", 0, &config, nullptr, nullptr) == "true");
    }

    SECTION("size() of a non-empty vector returns the right size") { REQUIRE(parser.process("{local myint = (0, 1, 2, 3)}{size(myint)}", 0, nullptr, nullptr, nullptr) == "4"); }
    SECTION("size() of a an empty vector returns the right size") { REQUIRE(parser.process("{local myint = (0);myint=();size(myint)}", 0, nullptr, nullptr, nullptr) == "0"); }
    SECTION("empty() of a non-empty vector returns false") { REQUIRE(parser.process("{local myint = (0, 1, 2, 3)}{empty(myint)}", 0, nullptr, nullptr, nullptr) == "false"); }
    SECTION("empty() of a an empty vector returns true") { REQUIRE(parser.process("{local myint = (0);myint=();empty(myint)}", 0, nullptr, nullptr, nullptr) == "true"); }

    SECTION("nested if with new variables") {
        std::string script =
            "{if 1 == 1}{local myints = (5, 4, 3, 2, 1)}{else}{local myfloats = (1., 2., 3., 4., 5., 6., 7.)}{endif}"
            "{myints[1]},{size(myints)}";
        REQUIRE(parser.process(script, 0, nullptr, nullptr, nullptr) == "4,5");
    }
    SECTION("nested if with new variables 2") {
        std::string script =
            "{if 1 == 0}{local myints = (5, 4, 3, 2, 1)}{else}{local myfloats = (1., 2., 3., 4., 5., 6., 7.)}{endif}"
            "{size(myfloats)}";
        REQUIRE(parser.process(script, 0, nullptr, nullptr, nullptr) == "7");
    }
    SECTION("nested if with new variables 2, mixing }{ with ;") {
        std::string script =
            "{if 1 == 0 then local myints = (5, 4, 3, 2, 1);else;local myfloats = (1., 2., 3., 4., 5., 6., 7.);endif}"
            "{size(myfloats)}";
        REQUIRE(parser.process(script, 0, nullptr, nullptr, nullptr) == "7");
    }
    SECTION("nested if with new variables, two level") {
        std::string script =
            "{if 1 == 1}{if 2 == 3}{nejaka / haluz}{else}{local myints = (6, 5, 4, 3, 2, 1)}{endif}{else}{if zase * haluz}{else}{local myfloats = (1., 2., 3., 4., 5., 6., 7.)}{endif}{endif}"
            "{size(myints)}";
        REQUIRE(parser.process(script, 0, nullptr, nullptr, nullptr) == "6");
    }
    SECTION("if with empty block and ;") {
        std::string script =
            "{if false then else;local myfloats = (1., 2., 3., 4., 5., 6., 7.);endif}"
            "{size(myfloats)}";
        REQUIRE(parser.process(script, 0, nullptr, nullptr, nullptr) == "7");
    }
    SECTION("nested if with new variables, two level, mixing }{ with ;") {
        std::string script =
            "{if 1 == 1 then if 2 == 3}nejaka / haluz{else local myints = (6, 5, 4, 3, 2, 1) endif else if zase * haluz then else local myfloats = (1., 2., 3., 4., 5., 6., 7.) endif endif}"
            "{size(myints)}";
        REQUIRE(parser.process(script, 0, nullptr, nullptr, nullptr) == "6");
    }
    SECTION("nested if with new variables, two level, mixing }{ with ; 2") {
        std::string script =
            "{if 1 == 1 then if 2 == 3 then nejaka / haluz else}{local myints = (6, 5, 4, 3, 2, 1)}{endif else if zase * haluz then else local myfloats = (1., 2., 3., 4., 5., 6., 7.) endif endif}"
            "{size(myints)}";
        REQUIRE(parser.process(script, 0, nullptr, nullptr, nullptr) == "6");
    }
    SECTION("nested if with new variables, two level, mixing }{ with ; 3") {
        std::string script =
            "{if 1 == 1 then if 2 == 3 then nejaka / haluz else}{local myints = (6, 5, 4, 3, 2, 1)}{endif else}{if zase * haluz}{else local myfloats = (1., 2., 3., 4., 5., 6., 7.) endif}{endif}"
            "{size(myints)}";
        REQUIRE(parser.process(script, 0, nullptr, nullptr, nullptr) == "6");
    }
    SECTION("if else completely empty") { REQUIRE(parser.process("{if false then elsif false then else endif}", 0, nullptr, nullptr, nullptr) == ""); }
}

SCENARIO("Placeholder parser coFloatsOrPercents vector access", "[PlaceholderParser]") {
    PlaceholderParser parser;
    auto config = DynamicPrintConfig::full_print_config();

    // outer_wall_speed is the ratio_over target for small_perimeter_speed.
    // Different values per extruder to verify parent resolves at the same element index.
    config.set_deserialize_strict({
        { "outer_wall_speed", "60,70,80,90" },
        { "nozzle_diameter", "0.4,0.4,0.4,0.4" },
        { "pressure_advance", "1.5,2.0,3.0,4.0" }  // coFloats non-nullable
    });
    // small_perimeter_speed:
    //   [0] = 50% of outer_wall_speed[0] (= 60) → 30
    //   [1] = 80% of outer_wall_speed[1] (= 70) → 56
    //   [2] = 0 absolute
    //   [3] = 50% of outer_wall_speed[3] (= 90) → 45
    config.option<ConfigOptionFloatsOrPercentsNullable>("small_perimeter_speed")->values = {
        FloatOrPercent{50.0, true},    // 50% of outer_wall_speed[0] (60) = 30
        FloatOrPercent{80.0, true},    // 80% of outer_wall_speed[1] (70) = 56
        FloatOrPercent{0.0, false},    // absolute: 0
        FloatOrPercent{50.0, true},    // 50% of outer_wall_speed[3] (90) = 45
    };

    parser.apply_config(config);
    parser.set("foo", 0);
    parser.set("bar", 1);
    parser.set("baz", 3);
    parser.set("num_extruders", 4);

    SECTION("Indexed access - percent resolved against parent at same index [0]") {
        // 50% of outer_wall_speed[0] (60) = 30
        REQUIRE(std::stod(parser.process("{small_perimeter_speed[0]}")) == Catch::Approx(30.0));
    }

    SECTION("Indexed access - percent resolved against parent at same index [1]") {
        // 80% of outer_wall_speed[1] (70) = 56
        REQUIRE(std::stod(parser.process("{small_perimeter_speed[1]}")) == Catch::Approx(56.0));
    }

    SECTION("Indexed access - percent resolved against parent at same index [3]") {
        // 50% of outer_wall_speed[3] (90) = 45
        REQUIRE(std::stod(parser.process("{small_perimeter_speed[3]}")) == Catch::Approx(45.0));
    }

    SECTION("Variable-indexed access via foo (=0) - percent value") {
        // 50% of outer_wall_speed[0] (60) = 30
        REQUIRE(std::stod(parser.process("{small_perimeter_speed[foo]}")) == Catch::Approx(30.0));
    }

    SECTION("Variable-indexed access via bar (=1) - percent value") {
        // 80% of outer_wall_speed[1] (70) = 56
        REQUIRE(std::stod(parser.process("{small_perimeter_speed[bar]}")) == Catch::Approx(56.0));
    }

    SECTION("Variable-indexed access via baz (=3) - percent value") {
        // 50% of outer_wall_speed[3] (90) = 45
        REQUIRE(std::stod(parser.process("{small_perimeter_speed[baz]}")) == Catch::Approx(45.0));
    }

    SECTION("Literal-indexed access - absolute value") {
        REQUIRE(std::stod(parser.process("{small_perimeter_speed[2]}")) == Catch::Approx(0.0));
    }

    SECTION("No-index (extruder-based) access - percent resolved via current extruder") {
        // Extruder 0 = 50% of outer_wall_speed[0] (60) = 30
        REQUIRE(std::stod(parser.process("{small_perimeter_speed}")) == Catch::Approx(30.0));
    }

    SECTION("Out-of-range index clamps to index 0") {
        // Index 99 is out of range, clamps to 0: 50% of outer_wall_speed[0] (60) = 30
        REQUIRE(std::stod(parser.process("{small_perimeter_speed[99]}")) == Catch::Approx(30.0));
    }

    SECTION("coFloats no-index access - nullable (outer_wall_speed)") {
        // outer_wall_speed is ConfigOptionFloatsNullable, exercises the 'if' branch
        REQUIRE(std::stod(parser.process("{outer_wall_speed}")) == Catch::Approx(60.0));
    }

    SECTION("coFloats indexed access - nullable") {
        // outer_wall_speed[2] = 80
        REQUIRE(std::stod(parser.process("{outer_wall_speed[2]}")) == Catch::Approx(80.0));
    }

    SECTION("coFloats no-index access - non-nullable (pressure_advance)") {
        // pressure_advance is ConfigOptionFloats (non-nullable), exercises the 'else' branch
        REQUIRE(std::stod(parser.process("{pressure_advance}")) == Catch::Approx(1.5));
    }

    SECTION("coFloats indexed access - non-nullable") {
        // pressure_advance[2] = 3.0
        REQUIRE(std::stod(parser.process("{pressure_advance[2]}")) == Catch::Approx(3.0));
    }
}

SCENARIO("Placeholder parser names in branches that are not taken", "[PlaceholderParser]") {
    PlaceholderParser parser;
    auto config = DynamicPrintConfig::full_print_config();
    parser.apply_config(config);
    parser.set("idx", 0);
    PlaceholderParser::ContextData context;
    context.global_config = std::make_unique<DynamicConfig>();
    auto process = [&parser, &context](const std::string &templ) { return parser.process(templ, 0, nullptr, nullptr, &context); };

    SECTION("a declaration continuing after a variable reference parses when not taken") {
        REQUIRE(process("{if false}{local a = layer_height + 1}{endif}ok") == "ok");
    }
    SECTION("names are not checked by default") {
        REQUIRE(process("{if false}{no_such_var}[no_such_var]{endif}ok") == "ok");
    }
    SECTION("names must resolve when check_inactive_branches is set") {
        struct Restore { ~Restore() { PlaceholderParser::check_inactive_branches = false; } } restore;
        PlaceholderParser::check_inactive_branches = true;

        CHECK_THROWS_WITH(process("{if false}{no_such_var}{endif}"), Catch::Matchers::ContainsSubstring("Not a variable name (in an inactive branch)"));
        CHECK_THROWS_WITH(process("{if false}[no_such_var]{endif}"), Catch::Matchers::ContainsSubstring("Variable does not exist (in an inactive branch)"));
        CHECK_THROWS_WITH(process("{if false}[nozzle_temperature[no_such_var]]{endif}"), Catch::Matchers::ContainsSubstring("Variable does not exist (in an inactive branch)"));
        CHECK_THROWS(process("{if true}{else}{no_such_var}{endif}"));
        CHECK_THROWS(process("{if false}{local a = no_such_var + 1}{endif}"));

        CHECK(process("{if false}{layer_height}[layer_height][nozzle_temperature_0][nozzle_temperature[idx]]{endif}ok") == "ok");
        CHECK(process("{if false}{local a = 1}{a = a + 1}{a}{endif}{if false}{a}{endif}ok") == "ok");
        // A global declared in a branch that is not taken counts as defined for later expansions sharing the context.
        CHECK(process("{if false}{global g = 1}{endif}{if false}{g}{endif}ok") == "ok");
        CHECK(process("{if false}{g}{endif}ok") == "ok");
        // Boolean expressions are not checked, so compatibility conditions keep their behaviour.
        CHECK(PlaceholderParser::evaluate_boolean_expression("false ? no_such_var == 1 : true", config));
    }
}

TEST_CASE("Placeholder parser preserves nullable vector nil semantics", "[PlaceholderParser][PlaceholderParserExpressions]")
{
    auto config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        { "outer_wall_speed", "60,70" },
        { "nozzle_diameter", "0.4,0.4" }
    });
    auto *outer_wall_speed = config.option<ConfigOptionFloatsNullable>("outer_wall_speed");
    REQUIRE(outer_wall_speed != nullptr);
    REQUIRE(outer_wall_speed->values.size() == 2);
    outer_wall_speed->set_at_to_nil(1);

    PlaceholderParser parser;
    REQUIRE(parser.apply_config(config));
    CHECK(parser.process("{is_nil(outer_wall_speed[0])}") == "false");
    CHECK(parser.process("{is_nil(outer_wall_speed[1])}") == "true");
    // Indexed config access clamps out-of-range indexes to element zero; nil inspection follows that rule.
    CHECK(parser.process("{is_nil(outer_wall_speed[99])}") == "false");
    CHECK_THROWS_WITH(parser.process("{outer_wall_speed[1]}"),
                      Catch::Matchers::ContainsSubstring("undefined (nil)"));
    CHECK_THROWS_WITH(parser.process("{is_nil(outer_wall_speed)}"),
                      Catch::Matchers::ContainsSubstring("vector variable when scalar is expected"));
}

TEST_CASE("Placeholder parser validates one_of literal and regex operands", "[PlaceholderParser][PlaceholderParserExpressions]")
{
    PlaceholderParser parser;
    CHECK(parser.process("{one_of(\"ABS\", \"NO\", \"ABS\")}") == "true");
    CHECK(parser.process("{one_of(\"ABS\", \"NO\", \"OTHER\")}") == "false");
    CHECK(parser.process("{one_of(\"ABS-12\", /ABS-[0-9]+/)}") == "true");
    CHECK_THROWS_WITH(parser.process("{one_of(12, \"12\")}"),
                      Catch::Matchers::ContainsSubstring("First parameter"));
    CHECK_THROWS_WITH(parser.process("{one_of(\"12\", 12)}"),
                      Catch::Matchers::ContainsSubstring("Pattern has to be a string"));
}

TEST_CASE("Placeholder parser reports and applies only changed configuration", "[PlaceholderParser][PlaceholderParserConfig]")
{
    auto base = DynamicPrintConfig::full_print_config();
    base.set_deserialize_strict({
        { "layer_height", "0.2" },
        { "wall_loops", "2" }
    });
    auto changed = base;
    changed.set_deserialize_strict({
        { "layer_height", "0.3" },
        { "wall_loops", "3" }
    });

    PlaceholderParser parser;
    REQUIRE(parser.apply_config(base));
    CHECK_FALSE(parser.apply_config(base));
    const std::string original_wall_loops = parser.process("{wall_loops}");

    const std::vector<std::string> diff = parser.config_diff(changed);
    CHECK(diff.size() == 2);
    CHECK(std::find(diff.begin(), diff.end(), "layer_height") != diff.end());
    CHECK(std::find(diff.begin(), diff.end(), "wall_loops") != diff.end());

    parser.apply_only(changed, { "layer_height" });
    CHECK(parser.process("{layer_height}") == "0.3");
    CHECK(parser.process("{wall_loops}") == original_wall_loops);

    const std::vector<std::string> remaining_diff = parser.config_diff(changed);
    REQUIRE(remaining_diff.size() == 1);
    REQUIRE(remaining_diff.front() == "wall_loops");
    REQUIRE(parser.apply_config(changed));
    CHECK_FALSE(parser.apply_config(changed));
    CHECK(parser.config_diff(changed).empty());
}


TEST_CASE("Placeholder vector assignments update only the selected element",
          "[PlaceholderParser][PlaceholderParserVectors]")
{
    PlaceholderParser parser;

    // Each assignment targets a nonzero element and reads the complete vector
    // back, so an assignment cannot silently update element zero or be ignored.
    CHECK(parser.process("{local values=(10,20,30)}{values[1]=42}{values[0]},{values[1]},{values[2]}")
          == "10,42,30");
    CHECK(parser.process("{local values=(1.5,2.5,3.5)}{values[2]=4.75}{values[0]},{values[1]},{values[2]}")
          == "1.5,2.5,4.75");
    CHECK(parser.process("{local values=(\"alpha\",\"beta\",\"gamma\")}{values[1]=\"delta\"}{values[0]}|{values[1]}|{values[2]}")
          == "alpha|delta|gamma");
    CHECK(parser.process("{local values=(false,false,true)}{values[1]=true}{values[0]},{values[1]},{values[2]}")
          == "false,true,true");
}

TEST_CASE("Placeholder parser copies vectors and rejects incompatible vector types",
          "[PlaceholderParser][PlaceholderParserVectors]")
{
    PlaceholderParser parser;
    // Existing parser tests already cover related vector copies from config
    // variables; this keeps an explicit existing-target copy oracle here.
    CHECK(parser.process("{local source=(3,4,5)}{local target=(0,0,0)}{target=source}{target[0]},{target[1]},{target[2]}")
          == "3,4,5");
    CHECK_THROWS_WITH(
        parser.process("{local source=(3,4)}{local target=(false,false)}{target=source}"),
        Catch::Matchers::ContainsSubstring("Left hand side is a bool vector"));
}

TEST_CASE("Placeholder output vector writes preserve untouched elements and reject invalid assignments",
          "[PlaceholderParser][PlaceholderParserVectors]")
{
    PlaceholderParser parser;
    const auto check_percent_values = [](const DynamicConfig &config, const std::vector<double> &expected) {
        const auto *option = config.option<ConfigOptionPercents>("retract_before_wipe");
        REQUIRE(option != nullptr);
        REQUIRE(option->values.size() == expected.size());
        for (std::size_t i = 0; i < expected.size(); ++i)
            CHECK_THAT(option->values[i], Catch::Matchers::WithinAbs(expected[i], 1e-9));
    };

    DynamicConfig outputs;
    outputs.set_key_value("retract_before_wipe", new ConfigOptionPercents({ 10., 20., 30. }));
    CHECK(parser.process("{retract_before_wipe[1] = 42}{retract_before_wipe[0]},{retract_before_wipe[1]},{retract_before_wipe[2]}",
                         0, nullptr, &outputs, nullptr) == "10,42,30");
    check_percent_values(outputs, { 10., 42., 30. });

    DynamicConfig out_of_range;
    out_of_range.set_key_value("retract_before_wipe", new ConfigOptionPercents({ 10., 20., 30. }));
    CHECK_THROWS_WITH(
        parser.process("{retract_before_wipe[3] = 42}", 0, nullptr, &out_of_range, nullptr),
        Catch::Matchers::ContainsSubstring("Index out of range"));
    check_percent_values(out_of_range, { 10., 20., 30. });

    DynamicConfig empty;
    empty.set_key_value("retract_before_wipe", new ConfigOptionPercents({}));
    const auto *empty_option = empty.option<ConfigOptionPercents>("retract_before_wipe");
    REQUIRE(empty_option != nullptr);
    CHECK_THROWS_WITH(
        parser.process("{retract_before_wipe[0] = 42}", 0, nullptr, &empty, nullptr),
        Catch::Matchers::ContainsSubstring("Indexing an empty vector variable"));
    CHECK(empty_option->values.empty());

    DynamicConfig nonnumeric;
    nonnumeric.set_key_value("retract_before_wipe", new ConfigOptionPercents({ 10., 20., 30. }));
    CHECK_THROWS_WITH(
        parser.process("{retract_before_wipe[1] = true}", 0, nullptr, &nonnumeric, nullptr),
        Catch::Matchers::ContainsSubstring("Right side is not a numeric expression"));
    check_percent_values(nonnumeric, { 10., 20., 30. });
}

TEST_CASE("Placeholder parser uses the external filament map for vector access",
          "[PlaceholderParser][PlaceholderParserMapping]")
{
    DynamicPrintConfig stored = DynamicPrintConfig::full_print_config();
    auto *outer_wall_speed = stored.option<ConfigOptionFloatsNullable>("outer_wall_speed");
    REQUIRE(outer_wall_speed != nullptr);
    outer_wall_speed->values = { 60., 70., 80. };

    DynamicPrintConfig external = DynamicPrintConfig::full_print_config();
    auto *filament_map = external.option<ConfigOptionInts>("filament_map");
    REQUIRE(filament_map != nullptr);
    // Requests 0 and 1 resolve to distinct nonzero stored slots 1 and 2.
    // Thus both the mapping branch and its last-slot bound are observable;
    // the fallback 0 cannot accidentally satisfy either row.
    filament_map->values = { 1, 2 };

    PlaceholderParser parser(&external);
    REQUIRE(parser.apply_config(stored));
    const std::string mapped_first = parser.process("{outer_wall_speed}", 0);
    const std::string mapped_last = parser.process("{outer_wall_speed}", 1);
    CHECK(mapped_first == "70");
    CHECK(mapped_last == "80");
}

TEST_CASE("Placeholder random values respect numeric bounds and their supplied context",
          "[PlaceholderParser][PlaceholderParserRandom]")
{
    PlaceholderParser parser;
    PlaceholderParser::ContextData context;
    PlaceholderParser::ContextData replay;
    context.rng.seed(20261017u);
    replay.rng.seed(20261017u);

    struct Bounds {
        const char *expression;
        double lower;
        double upper;
        bool integral;
    };
    const std::array<Bounds, 4> bounds = {{
        {"{random(2,7)}", 2., 7., true},
        {"{random(1.25,2.75)}", 1.25, 2.75, false},
        {"{random(2.25,3)}", 2.25, 3., false},
        {"{random(2,2.75)}", 2., 2.75, false},
    }};
    for (const auto &row : bounds) {
        INFO(row.expression);
        for (int draw = 0; draw < 8; ++draw) {
            INFO(draw);
            const std::mt19937 before = context.rng;
            const std::string value_text = parser.process(row.expression, 0, nullptr, &context);
            CHECK(value_text == parser.process(row.expression, 0, nullptr, &replay));
            const double value = std::stod(value_text);
            REQUIRE(std::isfinite(value));
            CHECK(value >= row.lower);
            CHECK(value <= row.upper);
            if (row.integral)
                CHECK_THAT(value, Catch::Matchers::WithinAbs(std::floor(value), 1e-12));
            CHECK_FALSE(context.rng == before);
            CHECK(context.rng == replay.rng);
        }
    }

    CHECK(parser.process("{random(1,1)/2}", 0, nullptr, &context) == "0");
    CHECK_THAT(std::stod(parser.process("{random(1,1.0)/2}", 0, nullptr, &context)),
               Catch::Matchers::WithinAbs(0.5, 1e-12));
    CHECK_THAT(std::stod(parser.process("{random(1.0,1)/2}", 0, nullptr, &context)),
               Catch::Matchers::WithinAbs(0.5, 1e-12));
    CHECK_THAT(std::stod(parser.process("{random(1.25,1.25)}", 0, nullptr, &context)),
               Catch::Matchers::WithinAbs(1.25, 1e-12));
}

TEST_CASE("Skipped and rejected random expressions leave the supplied generator unchanged",
          "[PlaceholderParser][PlaceholderParserRandom]")
{
    PlaceholderParser parser;
    PlaceholderParser::ContextData context;
    context.rng.seed(173u);
    const std::mt19937 before = context.rng;
    CHECK(parser.process("{if false}{random(2,7)}{endif}ready", 0, nullptr, &context) == "ready");
    CHECK(context.rng == before);
    CHECK(parser.process("{if false}{random(2,7)}{endif}ready") == "ready");
    CHECK_THROWS_WITH(parser.process("{random(2,7)}"),
                      Catch::Matchers::ContainsSubstring("Random number generator not available"));

    const std::array<const char *, 4> rejected = {{
        "{random(\"invalid\",7)}", "{random(2,\"invalid\")}",
        "{random(true,7)}", "{random(2,false)}",
    }};
    for (const char *expression : rejected) {
        INFO(expression);
        CHECK_THROWS_WITH(parser.process(expression, 0, nullptr, &context),
                          Catch::Matchers::ContainsSubstring("Not a numeric type"));
        CHECK(context.rng == before);
    }
    CHECK(parser.process("{random(4,4)}", 0, nullptr, &context) == "4");
}

TEST_CASE("Placeholder interpolation clamps rounding error at both table endpoints",
          "[PlaceholderParser][PlaceholderParserInterpolation]")
{
    PlaceholderParser parser;
    const std::array<std::pair<double, double>, 5> rows = {{
        {-0.5 * EPSILON, 10.}, {0., 10.}, {1., 20.},
        {2., 30.}, {2. + 0.5 * EPSILON, 30.},
    }};
    for (const auto &row : rows) {
        INFO(row.first);
        parser.set("probe", row.first);
        CHECK_THAT(std::stod(parser.process("{interpolate_table(probe,(0,10),(2,30))}")),
                   Catch::Matchers::WithinAbs(row.second, 1e-9));
    }
    for (double outside : {-2. * EPSILON, 2. + 2. * EPSILON}) {
        INFO(outside);
        parser.set("probe", outside);
        CHECK_THROWS_WITH(parser.process("{interpolate_table(probe,(0,10),(2,30))}"),
                          Catch::Matchers::ContainsSubstring("outside the table range"));
    }
    parser.set("probe", 0.5);
    CHECK_THAT(std::stod(parser.process("{interpolate_table(probe,(0,10),(2,30))}")),
               Catch::Matchers::WithinAbs(15., 1e-9));
}

TEST_CASE("Placeholder interpolation rejects nonnumeric and decreasing table coordinates",
          "[PlaceholderParser][PlaceholderParserInterpolation]")
{
    PlaceholderParser parser;
    CHECK_THROWS_WITH(parser.process("{interpolate_table(\"invalid\",(0,10),(2,30))}"),
                      Catch::Matchers::ContainsSubstring("Interpolation value must be a number"));
    CHECK_THROWS_WITH(parser.process("{interpolate_table(1,(\"invalid\",10),(2,30))}"),
                      Catch::Matchers::ContainsSubstring("X value of a table point must be a number"));
    CHECK_THROWS_WITH(parser.process("{interpolate_table(1,(0,\"invalid\"),(2,30))}"),
                      Catch::Matchers::ContainsSubstring("Y value of a table point must be a number"));
    CHECK_THROWS_WITH(parser.process("{interpolate_table(1,(2,30),(0,10))}"),
                      Catch::Matchers::ContainsSubstring("X coordinates of the table must be increasing"));
    CHECK_THAT(std::stod(parser.process("{interpolate_table(1,(0,10),(2,30))}")),
               Catch::Matchers::WithinAbs(20., 1e-9));
}


TEST_CASE("Placeholder one_of evaluates supplied regex patterns as whole-string matches",
          "[PlaceholderParser][PlaceholderParserRegexBranches]")
{
    PlaceholderParser parser;
    parser.set("material_pattern", std::string("ABS-[0-9]+"));
    parser.set("invalid_pattern", std::string("["));

    CHECK(parser.process(R"({one_of("ABS-12", ~material_pattern)})") == "true");
    CHECK(parser.process(R"({one_of("PLA-12", ~material_pattern)})") == "false");
    CHECK(parser.process(R"({one_of("prefixABS-12", ~material_pattern)})") == "false");
    CHECK(parser.process(R"({one_of("ABS-12", "PLA", ~material_pattern)})") == "true");
    // A successful earlier alternative avoids compiling a later pattern.
    CHECK(parser.process(R"({one_of("ABS-12", "ABS-12", ~invalid_pattern)})") == "true");
    CHECK_THROWS_WITH(parser.process(R"({one_of("ABS-12", ~invalid_pattern)})"),
                      Catch::Matchers::ContainsSubstring("Regular expression compilation failed"));
    CHECK_THROWS_WITH(parser.process(R"({one_of(12, ~material_pattern)})"),
                      Catch::Matchers::ContainsSubstring("First parameter"));
    CHECK_THROWS_WITH(parser.process(R"({one_of("ABS-12", ~12)})"),
                      Catch::Matchers::ContainsSubstring("Pattern has to be a string"));
    CHECK(parser.process(R"({if false}{one_of("ABS-12", ~invalid_pattern)}{endif}ready)") == "ready");
    // Rejected and skipped expressions must not corrupt later evaluations.
    CHECK(parser.process(R"({one_of("ABS-12", ~material_pattern)})") == "true");
}

TEST_CASE("Placeholder regex negation complements a whole-string match",
          "[PlaceholderParser][PlaceholderParserRegexBranches]")
{
    PlaceholderParser parser;
    CHECK(parser.process(R"({"ABS-12" !~ /ABS-[0-9]+/})") == "false");
    CHECK(parser.process(R"({"PLA-12" !~ /ABS-[0-9]+/})") == "true");
    CHECK(parser.process(R"({"prefixABS-12" !~ /ABS-[0-9]+/})") == "true");
    CHECK_THROWS_WITH(parser.process(R"({12 !~ /ABS-[0-9]+/})"),
                      Catch::Matchers::ContainsSubstring("Left hand side of a regex match must be a string"));
    CHECK_THROWS_WITH(parser.process(R"({"ABS-12" !~ /[/})"),
                      Catch::Matchers::ContainsSubstring("Regular expression compilation failed"));
    CHECK(parser.process(R"({if false}{"ABS-12" !~ /[/}{endif}ready)") == "ready");
    CHECK(parser.process(R"({"ABS-12" !~ /ABS-[0-9]+/})") == "false");
}


TEST_CASE("Placeholder parser clones percent vectors as numeric local variables",
          "[PlaceholderParser][PlaceholderParserVectors]")
{
    auto config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({ { "retract_before_wipe", "10,20,30" } });

    PlaceholderParser parser;
    REQUIRE(parser.apply_config(config));
    // ConfigOptionPercents is cloned as a numeric local vector, preserving every element.
    CHECK(parser.process("{local copied = retract_before_wipe}{copied[0]},{copied[1]},{copied[2]}") == "10,20,30");
    CHECK(parser.process("{local copied = retract_before_wipe}{size(copied)}") == "3");
}

TEST_CASE("Placeholder parser rejects empty and mixed-type initializer vectors",
          "[PlaceholderParser][PlaceholderParserVectors]")
{
    PlaceholderParser parser;
    CHECK_THROWS_WITH(parser.process("{local empty_vec = ()}"),
                      Catch::Matchers::ContainsSubstring("Cannot create vector variable from an empty initializer list"));
    CHECK_THROWS_WITH(parser.process("{local mixed = (true, 1)}"),
                      Catch::Matchers::ContainsSubstring("Mixing numeric and boolean types"));
}

TEST_CASE("Placeholder parser preserves UTF-8 literals and validates escapes",
          "[PlaceholderParser][PlaceholderParserUnicode]")
{
    PlaceholderParser parser;
    std::string utf8 = "{\"caf";
    utf8.append("\xC3\xA9", 2);
    utf8 += " ";
    utf8.append("\xF0\x9F\x98\x80", 4);
    utf8 += "\"}";
    std::string expected = "caf";
    expected.append("\xC3\xA9", 2);
    expected += " ";
    expected.append("\xF0\x9F\x98\x80", 4);
    CHECK(parser.process(utf8) == expected);
    CHECK(parser.process(R"({"line\nnext"})") == "line\nnext");

    CHECK_THROWS_WITH(parser.process(R"({"bad\q"})"),
                      Catch::Matchers::ContainsSubstring("Invalid escape sequence"));
    std::string invalid = "{\"";
    invalid.append("\xE2\x28\xA1", 3);
    invalid += "\"}";
    CHECK_THROWS_WITH(parser.process(invalid),
                      Catch::Matchers::ContainsSubstring("Invalid utf8 sequence"));
}

TEST_CASE("Placeholder parser rejects non-ASCII code and reports malformed macro lines",
          "[PlaceholderParser][PlaceholderParserErrors]")
{
    PlaceholderParser parser;
    std::string non_ascii_code = "{";
    non_ascii_code.append("\xC3\xA9", 2);
    non_ascii_code += "}";
    CHECK_THROWS_WITH(parser.process(non_ascii_code),
                      Catch::Matchers::ContainsSubstring("Non-ASCII7 characters"));
    CHECK_THROWS_WITH(parser.process("prefix\n{if true}\nvalue"),
                      Catch::Matchers::ContainsSubstring("Parsing error at line"));
}

TEST_CASE("Placeholder parser reports negative legacy vector indexes",
          "[PlaceholderParser][PlaceholderParserErrors]")
{
    auto config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({ { "nozzle_temperature", "357,359" } });
    PlaceholderParser parser;
    REQUIRE(parser.apply_config(config));
    parser.set("idx", -1);
    CHECK_THROWS_WITH(parser.process("[nozzle_temperature[idx]]"),
                      Catch::Matchers::ContainsSubstring("Negative vector index"));
    parser.set("idx", 0);
    CHECK(parser.process("[nozzle_temperature[idx]]") == "357");
}

TEST_CASE("Placeholder parser merges an rvalue configuration",
          "[PlaceholderParser][PlaceholderParserConfig]")
{
    DynamicPrintConfig moved_config = DynamicPrintConfig::full_print_config();
    moved_config.set_deserialize_strict({
        { "layer_height", "0.25" },
        { "wall_loops", "3" }
    });

    PlaceholderParser parser;
    parser.set("already_present", "keep");
    parser.apply_config(std::move(moved_config));
    CHECK(parser.process("{layer_height}") == "0.25");
    CHECK(parser.process("{wall_loops}") == "3");
    CHECK(parser.process("{already_present}") == "keep");
}

TEST_CASE("Placeholder parser resolves configuration overrides before stored and external values",
          "[PlaceholderParser][PlaceholderParserLookup]")
{
    DynamicPrintConfig external = DynamicPrintConfig::full_print_config();
    external.set_deserialize_strict({
        { "layer_height", "0.1" },
        { "wall_loops", "7" }
    });
    DynamicPrintConfig stored;
    stored.set_deserialize_strict({ { "layer_height", "0.2" } });
    DynamicPrintConfig override;
    override.set_deserialize_strict({ { "layer_height", "0.3" } });

    PlaceholderParser parser(&external);
    REQUIRE(stored.option("wall_loops") == nullptr);
    REQUIRE(parser.apply_config(stored));
    CHECK(parser.process("{layer_height}") == "0.2");
    CHECK(parser.process("{layer_height}", 0, &override) == "0.3");
    CHECK(parser.process("{wall_loops}") == "7");
}

TEST_CASE("Placeholder parser writes scalar output values and reads them back",
          "[PlaceholderParser][PlaceholderParserOutput]")
{
    PlaceholderParser parser;
    DynamicConfig outputs;
    outputs.set_key_value("output_float", new ConfigOptionFloat(1.5));
    outputs.set_key_value("output_int", new ConfigOptionInt(2));
    outputs.set_key_value("output_string", new ConfigOptionString("before"));
    outputs.set_key_value("output_bool", new ConfigOptionBool(false));

    const std::string rendered = parser.process(
        "{output_float = 2.75}{output_int = 7}{output_string = \"ready\"}{output_bool = true}"
        "{output_float},{output_int},{output_string},{output_bool}",
        0, nullptr, &outputs, nullptr);
    CHECK(rendered == "2.75,7,ready,true");

    const auto *float_value = outputs.option<ConfigOptionFloat>("output_float");
    const auto *int_value = outputs.option<ConfigOptionInt>("output_int");
    const auto *string_value = outputs.option<ConfigOptionString>("output_string");
    const auto *bool_value = outputs.option<ConfigOptionBool>("output_bool");
    REQUIRE(float_value != nullptr);
    REQUIRE(int_value != nullptr);
    REQUIRE(string_value != nullptr);
    REQUIRE(bool_value != nullptr);
    CHECK_THAT(float_value->value, Catch::Matchers::WithinAbs(2.75, 1e-12));
    CHECK(int_value->value == 7);
    CHECK(string_value->value == "ready");
    CHECK(bool_value->value);
}

TEST_CASE("Placeholder parser assigns vectors by initializer list and repeat count",
          "[PlaceholderParser][PlaceholderParserVectors]")
{
    PlaceholderParser parser;
    DynamicConfig outputs;
    outputs.set_key_value("output_ints", new ConfigOptionInts({ 1, 2 }));
    outputs.set_key_value("output_floats", new ConfigOptionFloats({ 0.5, 1.5 }));
    outputs.set_key_value("output_strings", new ConfigOptionStrings({ "old" }));
    outputs.set_key_value("output_bools", new ConfigOptionBools({ false, false }));

    CHECK(parser.process(
        "{output_ints = (7,8,9)}{output_floats = repeat(3,1.25)}"
        "{output_strings = (\"new\",9)}{output_bools = (true,false)}",
        0, nullptr, &outputs, nullptr).empty());

    const auto *float_values = outputs.option<ConfigOptionFloats>("output_floats");
    REQUIRE(float_values != nullptr);
    REQUIRE(float_values->values.size() == 3);
    for (double value : float_values->values)
        CHECK_THAT(value, Catch::Matchers::WithinAbs(1.25, 1e-12));

    CHECK(parser.process(
        "{output_ints[0]},{output_ints[1]},{output_ints[2]}|"
        "{output_floats[0]},{output_floats[1]},{output_floats[2]},{size(output_floats)}|"
        "{output_strings[0]},{output_strings[1]}|"
        "{output_bools[0]},{output_bools[1]}",
        0, nullptr, &outputs, nullptr) == "7,8,9|1.25,1.25,1.25,3|new,9|true,false");
}

TEST_CASE("Placeholder parser rejects writes to read-only parser values",
          "[PlaceholderParser][PlaceholderParserOutput]")
{
    PlaceholderParser parser;
    parser.set("read_only", 4);
    DynamicConfig outputs;
    outputs.set_key_value("writable", new ConfigOptionInt(1));

    CHECK_THROWS_WITH(parser.process("{read_only = 5}", 0, nullptr, &outputs, nullptr),
                      Catch::Matchers::ContainsSubstring("Cannot modify a read-only variable"));
    CHECK(parser.process("{writable = 2}{writable}", 0, nullptr, &outputs, nullptr) == "2");
}

TEST_CASE("Placeholder parser changes the legacy vector index with filament_change",
          "[PlaceholderParser][PlaceholderParserLegacyIndex]")
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({ { "nozzle_temperature", "200,210,220" } });
    PlaceholderParser parser;
    REQUIRE(parser.apply_config(config));

    CHECK_THAT(parser.process("{filament_change(1)}temperature=[nozzle_temperature]"),
               Catch::Matchers::EndsWith("temperature=210"));
    CHECK_THAT(parser.process("{filament_change(2)}temperature=[nozzle_temperature]"),
               Catch::Matchers::EndsWith("temperature=220"));
    CHECK_THAT(parser.process("{filament_change(0)}temperature=[nozzle_temperature]"),
               Catch::Matchers::EndsWith("temperature=200"));
}

TEST_CASE("Placeholder parser unescapes carriage returns, quotes, and backslashes",
          "[PlaceholderParser][PlaceholderParserUnicode]")
{
    PlaceholderParser parser;
    const std::string actual = parser.process(R"({"quote:\" slash:\\ line:\n carriage:\r"})");
    CHECK(actual == std::string("quote:\" slash:\\ line:\n carriage:\r"));
}

TEST_CASE("Placeholder parser rejects truncated and standalone UTF-8 continuation bytes",
          "[PlaceholderParser][PlaceholderParserUnicode]")
{
    PlaceholderParser parser;
    std::string truncated = "truncated ";
    truncated.append("\xE2\x82", 2);
    CHECK_THROWS_WITH(parser.process(truncated),
                      Catch::Matchers::ContainsSubstring("Invalid utf8 sequence"));

    std::string continuation = "{\"";
    continuation.push_back(static_cast<char>(0x80));
    continuation += "\"}";
    CHECK_THROWS_WITH(parser.process(continuation),
                      Catch::Matchers::ContainsSubstring("Invalid utf8 sequence"));
}

TEST_CASE("Placeholder parser resolves dynamic legacy vector indices consistently",
          "[PlaceholderParser][PlaceholderParserLegacyIndex]")
{
    auto config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({ { "nozzle_temperature", "357,359" } });
    PlaceholderParser parser;
    REQUIRE(parser.apply_config(config));

    parser.set("index", 1);
    CHECK(parser.process("[nozzle_temperature[index]]") == "359");
    parser.set("index", 99);
    CHECK(parser.process("[nozzle_temperature[index]]") == "357");
    parser.set("index", "one");
    CHECK_THROWS_WITH(parser.process("[nozzle_temperature[index]]"),
                      Catch::Matchers::ContainsSubstring("Indexing variable has to be integer"));
    CHECK_THROWS_WITH(parser.process("[nozzle_temperature[missing_index]]"),
                      Catch::Matchers::ContainsSubstring("Variable does not exist"));
    parser.set("scalar", 1);
    parser.set("index", 0);
    CHECK_THROWS_WITH(parser.process("[scalar[index]]"),
                      Catch::Matchers::ContainsSubstring("Trying to index a scalar variable"));
}

TEST_CASE("Placeholder parser copies floating-point vectors independently",
          "[PlaceholderParser][PlaceholderParserVectors]")
{
    DynamicConfig external;
    external.set_key_value("source_values", new ConfigOptionFloats({ 1., 2., 3. }));
    PlaceholderParser parser(&external);

    CHECK(parser.process(
        "{local copied_values = source_values}{copied_values[1] = 1.5}"
        "{copied_values[0]},{copied_values[1]},{copied_values[2]}|"
        "{source_values[0]},{source_values[1]},{source_values[2]}") == "1,1.5,3|1,2,3");
}

TEST_CASE("Placeholder parser protects local and global variable scopes",
          "[PlaceholderParser][PlaceholderParserVariables]")
{
    PlaceholderParser parser;
    CHECK_THROWS_WITH(parser.process("{global shared = 1}"),
                      Catch::Matchers::ContainsSubstring("Global variables are not available"));

    PlaceholderParser::ContextData context;
    context.global_config = std::make_unique<DynamicConfig>();
    context.global_config->set_key_value("shared", new ConfigOptionInt(5));
    CHECK_THROWS_WITH(parser.process("{local shared = 2}", 0, nullptr, nullptr, &context),
                      Catch::Matchers::ContainsSubstring("Variable name already defined in global scope"));
    REQUIRE(context.global_config->option<ConfigOptionInt>("shared") != nullptr);
    CHECK(context.global_config->option<ConfigOptionInt>("shared")->value == 5);

    CHECK_THROWS_WITH(parser.process("{local scoped = 2}{global scoped = 3}", 0, nullptr, nullptr, &context),
                      Catch::Matchers::ContainsSubstring("Variable name already defined in local scope"));
    CHECK(context.global_config->option("scoped") == nullptr);
}

TEST_CASE("Placeholder parser preserves mixed numeric and string expression behavior",
          "[PlaceholderParser][PlaceholderParserExpressions]")
{
    PlaceholderParser parser;
    CHECK(parser.process(R"({17 % 5}|{5 + 0.25}|{6.5 - 1.25}|{2 + "mm"}|{"size=" + 2})") ==
          "2|5.25|5.25|2mm|size=2");
    CHECK_THROWS_WITH(parser.process("{2 / 0}"),
                      Catch::Matchers::ContainsSubstring("Division by zero"));
    CHECK_THROWS_WITH(parser.process("{2.0 % 0.0}"),
                      Catch::Matchers::ContainsSubstring("Division by zero"));
    CHECK_THROWS_WITH(parser.process("{1 - \"two\"}"),
                      Catch::Matchers::ContainsSubstring("Cannot subtract non-numeric types"));
    CHECK_THROWS_WITH(parser.process("{1 * \"two\"}"),
                      Catch::Matchers::ContainsSubstring("Cannot multiply with non-numeric type"));
}

TEST_CASE("Placeholder parser converts only numeric expressions",
          "[PlaceholderParser][PlaceholderParserExpressions]")
{
    PlaceholderParser parser;
    CHECK(parser.process("{-(7)}|{int(7)}|{round(7)}|{floor(7)}|{ceil(7)}") == "-7|7|7|7|7");
    CHECK_THROWS_WITH(parser.process("{-\"x\"}"),
                      Catch::Matchers::ContainsSubstring("Cannot apply unary minus operator"));
    CHECK_THROWS_WITH(parser.process("{int(\"x\")}"),
                      Catch::Matchers::ContainsSubstring("Cannot convert to integer"));
    CHECK_THROWS_WITH(parser.process("{round(\"x\")}"),
                      Catch::Matchers::ContainsSubstring("Cannot round a non-numeric value"));
    CHECK_THROWS_WITH(parser.process("{floor(\"x\")}"),
                      Catch::Matchers::ContainsSubstring("Cannot floor a non-numeric value"));
    CHECK_THROWS_WITH(parser.process("{ceil(\"x\")}"),
                      Catch::Matchers::ContainsSubstring("Cannot ceil a non-numeric value"));
    CHECK_THROWS_WITH(parser.process("{not \"x\"}"),
                      Catch::Matchers::ContainsSubstring("Cannot apply a not operator"));
}

TEST_CASE("Placeholder parser skips supported lazy expressions in inactive branches",
          "[PlaceholderParser][PlaceholderParserExpressions]")
{
    PlaceholderParser parser;
    const std::string skipped =
        R"({if false}{-"x"}{int("x")}{round("x")})"
        R"({not "x"}{1 + 2}{1 - 2}{1 * 2})"
        R"({1 / 0}{1 % 0}{true and false}{digits("x",2)}{min("x",2)})"
        R"({interpolate_table("x",("y",1))}{true}{endif}ready)";
    CHECK(parser.process(skipped) == "ready");
}

TEST_CASE("Placeholder boolean expressions compare strings and reject incompatible operands",
          "[PlaceholderParser][PlaceholderParserExpressions]")
{
    PlaceholderParser parser;
    const auto evaluate = [&parser](const std::string &expression) {
        return PlaceholderParser::evaluate_boolean_expression(expression, parser.config());
    };
    CHECK(evaluate(R"("alpha" < "beta")"));
    CHECK(evaluate(R"("same" == "same")"));
    CHECK(evaluate(R"("1" == 1)"));
    CHECK(evaluate("true == true"));
    CHECK_FALSE(evaluate("false == true"));
    CHECK_THROWS_WITH(evaluate("true < false"),
                      Catch::Matchers::ContainsSubstring("Cannot compare the types"));
    CHECK_THROWS_WITH(evaluate("true == 1"),
                      Catch::Matchers::ContainsSubstring("Cannot compare the types"));
    CHECK_THROWS_WITH(evaluate("1 and 2"),
                      Catch::Matchers::ContainsSubstring("Cannot apply logical operation to non-boolean operators"));
    CHECK_THROWS_WITH(evaluate("1"),
                      Catch::Matchers::ContainsSubstring("Not a boolean expression"));
    CHECK_THROWS_WITH(parser.process("{if 1}never{endif}"),
                      Catch::Matchers::ContainsSubstring("Not a boolean expression"));
}

TEST_CASE("Placeholder digit formatting rounds values and validates width arguments",
          "[PlaceholderParser][PlaceholderParserExpressions]")
{
    PlaceholderParser parser;
    CHECK(parser.process("{digits(12.6,2)}|{zdigits(3.4,2)}") == "13|03");
    CHECK_THROWS_WITH(parser.process("{digits(1,2.5)}"),
                      Catch::Matchers::ContainsSubstring("digits: second parameter must be integer"));
    CHECK_THROWS_WITH(parser.process("{digits(1,2,0.5)}"),
                      Catch::Matchers::ContainsSubstring("digits: third parameter must be integer"));
    CHECK_THROWS_WITH(parser.process("{digits(\"one\",2)}"),
                      Catch::Matchers::ContainsSubstring("Not a numeric type"));
}

TEST_CASE("Placeholder interpolation averages nearly coincident table coordinates",
          "[PlaceholderParser][PlaceholderParserInterpolation]")
{
    PlaceholderParser parser;
    parser.set("probe", 0.0000025);
    CHECK_THAT(std::stod(parser.process("{interpolate_table(probe,(0,10),(0.00001,30))}")),
               Catch::Matchers::WithinAbs(20., 1e-9));
}

TEST_CASE("Placeholder parser validates legacy indexes and optional vector elements",
          "[PlaceholderParser][PlaceholderParserAssignmentGaps]")
{
    auto config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({ { "nozzle_temperature", "200,210" } });
    PlaceholderParser parser;
    REQUIRE(parser.apply_config(config));
    parser.set("scalar", 1);
    parser.set("index", 1);
    CHECK_THROWS_WITH(parser.process("[scalar_0]"),
                      Catch::Matchers::ContainsSubstring("Trying to index a scalar variable"));
    CHECK_THROWS_WITH(parser.process("[nozzle_temperature_bad]"),
                      Catch::Matchers::ContainsSubstring("Invalid vector index"));
    CHECK(parser.process("[nozzle_temperature_[index]]") == "210");

    auto empty_config = DynamicPrintConfig::full_print_config();
    auto *empty_speed = empty_config.option<ConfigOptionFloatsNullable>("outer_wall_speed");
    REQUIRE(empty_speed != nullptr);
    empty_speed->values.clear();
    PlaceholderParser empty_parser;
    REQUIRE(empty_parser.apply_config(empty_config));
    CHECK_THROWS_WITH(empty_parser.process("[outer_wall_speed_0]"),
                      Catch::Matchers::ContainsSubstring("Indexing an empty vector variable"));

    auto nil_config = DynamicPrintConfig::full_print_config();
    auto *nil_speed = nil_config.option<ConfigOptionFloatsNullable>("outer_wall_speed");
    REQUIRE(nil_speed != nullptr);
    nil_speed->values = { 60., 70. };
    nil_speed->set_at_to_nil(1);
    PlaceholderParser nil_parser;
    REQUIRE(nil_parser.apply_config(nil_config));
    CHECK_THROWS_WITH(nil_parser.process("[outer_wall_speed_1]"),
                      Catch::Matchers::ContainsSubstring("undefined (nil)"));
}

TEST_CASE("Placeholder parser resolves point and enum vector elements",
          "[PlaceholderParser][PlaceholderParserAssignmentGaps]")
{
    auto config = DynamicPrintConfig::full_print_config();
    auto *area = config.option<ConfigOptionPoints>("printable_area");
    REQUIRE(area != nullptr);
    area->values = { Vec2d(3., 4.), Vec2d(5., 6.) };
    auto *nozzle_type = config.option<ConfigOptionEnumsGenericNullable>("nozzle_type");
    REQUIRE(nozzle_type != nullptr);
    nozzle_type->values = { ntHardenedSteel };

    PlaceholderParser parser;
    REQUIRE(parser.apply_config(config));
    CHECK(parser.process("{printable_area[0]}") == "[3, 4]");
    CHECK(parser.process("{nozzle_type[0]}") == std::to_string(static_cast<int>(ntHardenedSteel)));
    CHECK_THROWS_WITH(parser.process("{printable_area}"),
                      Catch::Matchers::ContainsSubstring("vector variable when scalar is expected"));
}

TEST_CASE("Placeholder parser preserves scalar assignments and rejects vector-shaped values",
          "[PlaceholderParser][PlaceholderParserAssignmentGaps]")
{
    PlaceholderParser parser;
    DynamicConfig outputs;
    outputs.set_key_value("percent", new ConfigOptionPercent(10.));
    outputs.set_key_value("flag", new ConfigOptionBool(false));
    outputs.set_key_value("number", new ConfigOptionInt(7));
    auto run = [&parser, &outputs](const std::string &script) {
        return parser.process(script, 0, nullptr, &outputs, nullptr);
    };

    CHECK(run("{percent = 25}{percent}") == "25");
    CHECK_THROWS_WITH(run("{flag = 1}"),
                      Catch::Matchers::ContainsSubstring("Right side is not a boolean expression"));
    CHECK_THROWS_WITH(run("{number = (1,2)}"),
                      Catch::Matchers::ContainsSubstring("Cannot assign a vector value to a scalar variable"));
    CHECK_THROWS_WITH(run("{number = repeat(2,1)}"),
                      Catch::Matchers::ContainsSubstring("Cannot assign a vector value to a scalar variable"));
    CHECK_THROWS_WITH(parser.process("{local scalar = 1}{local vector = (1,2)}{local vector = scalar}"),
                      Catch::Matchers::ContainsSubstring("Cannot assign scalar to a vector"));
    CHECK_THROWS_WITH(parser.process("{local vector = (1,2)}{local vector = 1}"),
                      Catch::Matchers::ContainsSubstring("Cannot assign a scalar value to a vector variable."));
    CHECK(outputs.option<ConfigOptionBool>("flag")->value == false);
    CHECK(outputs.option<ConfigOptionInt>("number")->value == 7);
}

TEST_CASE("Placeholder parser converts vector elements and preserves unsupported output vectors",
          "[PlaceholderParser][PlaceholderParserAssignmentGaps]")
{
    PlaceholderParser parser;
    DynamicConfig outputs;
    outputs.set_key_value("floats", new ConfigOptionFloats({ 1., 2. }));
    outputs.set_key_value("strings", new ConfigOptionStrings({ "old" }));
    outputs.set_key_value("flags", new ConfigOptionBools({ false, true }));
    outputs.set_key_value("points", new ConfigOptionPoints({ Vec2d(1., 2.) }));
    auto run = [&parser, &outputs](const std::string &script) {
        return parser.process(script, 0, nullptr, &outputs, nullptr);
    };

    CHECK(run("{floats[1] = 3}{strings[0] = 9}").empty());
    CHECK_THAT(outputs.option<ConfigOptionFloats>("floats")->values[1], Catch::Matchers::WithinAbs(3., 1e-12));
    CHECK(outputs.option<ConfigOptionStrings>("strings")->values[0] == "9");
    CHECK_THROWS_WITH(run("{flags[1] = 1}"),
                      Catch::Matchers::ContainsSubstring("Right side is not a boolean expression"));
    CHECK_THROWS_WITH(run("{points[0] = 1}"),
                      Catch::Matchers::ContainsSubstring("Unsupported output vector variable type"));
    CHECK_THAT(outputs.option<ConfigOptionPoints>("points")->values[0].x(),
               Catch::Matchers::WithinAbs(1., 1e-12));
    CHECK_THAT(outputs.option<ConfigOptionPoints>("points")->values[0].y(),
               Catch::Matchers::WithinAbs(2., 1e-12));
    CHECK(outputs.option<ConfigOptionBools>("flags")->values[1]);
}

TEST_CASE("Placeholder parser validates repeat counts and preserves vector values on errors",
          "[PlaceholderParser][PlaceholderParserAssignmentGaps]")
{
    PlaceholderParser parser;
    DynamicConfig outputs;
    outputs.set_key_value("numbers", new ConfigOptionInts({ 1, 2 }));
    outputs.set_key_value("flags", new ConfigOptionBools({ false }));
    auto run = [&parser, &outputs](const std::string &script) {
        return parser.process(script, 0, nullptr, &outputs, nullptr);
    };

    CHECK_THROWS_WITH(run("{numbers = repeat(1.5, 9)}"),
                      Catch::Matchers::ContainsSubstring("Expected number of elements"));
    CHECK_THROWS_WITH(run("{numbers = repeat(-1, 9)}"),
                      Catch::Matchers::ContainsSubstring("Negative number of elements"));
    CHECK(outputs.option<ConfigOptionInts>("numbers")->values == std::vector<int>{ 1, 2 });
    CHECK(run("{numbers = repeat(0, 4)}").empty());
    CHECK(outputs.option<ConfigOptionInts>("numbers")->values.empty());
    CHECK_THROWS_WITH(run("{flags = repeat(2, 1)}"),
                      Catch::Matchers::ContainsSubstring("Right side is not a boolean expression"));
    CHECK(run("{flags = repeat(2, true)}").empty());
    CHECK(outputs.option<ConfigOptionBools>("flags")->values == std::vector<unsigned char>{ 1, 1 });
    CHECK(run("{numbers = repeat(3, 4)}").empty());
    CHECK(outputs.option<ConfigOptionInts>("numbers")->values == std::vector<int>{ 4, 4, 4 });
}

TEST_CASE("Placeholder parser converts initializer lists and rejects incompatible booleans",
          "[PlaceholderParser][PlaceholderParserAssignmentGaps]")
{
    PlaceholderParser parser;
    DynamicConfig outputs;
    outputs.set_key_value("numbers", new ConfigOptionInts({ 1 }));
    outputs.set_key_value("strings", new ConfigOptionStrings({ "old" }));
    outputs.set_key_value("flags", new ConfigOptionBools({ false, false }));
    auto run = [&parser, &outputs](const std::string &script) {
        return parser.process(script, 0, nullptr, &outputs, nullptr);
    };

    CHECK(run("{numbers = (4,5)}{strings = (\"new\", 6)}").empty());
    CHECK(outputs.option<ConfigOptionInts>("numbers")->values == std::vector<int>{ 4, 5 });
    CHECK(outputs.option<ConfigOptionStrings>("strings")->values == std::vector<std::string>{ "new", "6" });
    CHECK_THROWS_WITH(run("{flags = (true, 1)}"),
                      Catch::Matchers::ContainsSubstring("Right side is not a boolean expression"));
    CHECK(outputs.option<ConfigOptionBools>("flags")->values == std::vector<unsigned char>{ 0, 0 });
}

TEST_CASE("Placeholder parser rejects incompatible vector copies with preserved targets",
          "[PlaceholderParser][PlaceholderParserAssignmentGaps]")
{
    CHECK(PlaceholderParser{}.process("{local ints = (1,2)}{local copy = ints}{copy[0]},{copy[1]}") == "1,2");

    DynamicConfig outputs;
    outputs.set_key_value("floats", new ConfigOptionFloats({ 9., 8. }));
    outputs.set_key_value("ints", new ConfigOptionInts({ 7, 6 }));
    outputs.set_key_value("strings", new ConfigOptionStrings({ "old", "keep" }));
    DynamicConfig inputs;
    inputs.set_key_value("source_ints", new ConfigOptionInts({ 1, 2 }));
    inputs.set_key_value("source_strings", new ConfigOptionStrings({ "x", "y" }));
    PlaceholderParser parser;
    auto run = [&parser, &inputs, &outputs](const std::string &script) {
        return parser.process(script, 0, &inputs, &outputs, nullptr);
    };

    CHECK_THROWS_WITH(run("{floats = source_ints}"),
                      Catch::Matchers::ContainsSubstring("Left hand side is a float vector"));
    const auto &floats = outputs.option<ConfigOptionFloats>("floats")->values;
    REQUIRE(floats.size() == 2);
    CHECK_THAT(floats[0], Catch::Matchers::WithinAbs(9., 1e-12));
    CHECK_THAT(floats[1], Catch::Matchers::WithinAbs(8., 1e-12));
    CHECK_THROWS_WITH(run("{ints = source_strings}"),
                      Catch::Matchers::ContainsSubstring("Left hand side is an int vector"));
    CHECK(outputs.option<ConfigOptionInts>("ints")->values == std::vector<int>{ 7, 6 });
    CHECK_THROWS_WITH(run("{strings = source_ints}"),
                      Catch::Matchers::ContainsSubstring("Left hand side is a string vector"));
    CHECK(outputs.option<ConfigOptionStrings>("strings")->values == std::vector<std::string>{ "old", "keep" });
}

TEST_CASE("Placeholder parser rejects nil and unsupported vector copies",
          "[PlaceholderParser][PlaceholderParserAssignmentGaps]")
{
    auto config = DynamicPrintConfig::full_print_config();
    auto *speed = config.option<ConfigOptionFloatsNullable>("outer_wall_speed");
    REQUIRE(speed != nullptr);
    speed->values = { 60., 70. };
    speed->set_at_to_nil(0);
    speed->set_at_to_nil(1);
    PlaceholderParser nil_parser;
    REQUIRE(nil_parser.apply_config(config));
    CHECK_THROWS_WITH(nil_parser.process("{local copied = outer_wall_speed}"),
                      Catch::Matchers::ContainsSubstring("Some elements of the right hand side vector variable of optional values are undefined (nil)"));

    auto points_config = DynamicPrintConfig::full_print_config();
    auto *area = points_config.option<ConfigOptionPoints>("printable_area");
    REQUIRE(area != nullptr);
    area->values = { Vec2d(1., 2.) };
    PlaceholderParser points_parser;
    REQUIRE(points_parser.apply_config(points_config));
    CHECK_THROWS_WITH(points_parser.process("{local copied = printable_area}"),
                      Catch::Matchers::ContainsSubstring("Duplicating this type of vector variable"));
}

TEST_CASE("Placeholder parser reports empty and size misuse without changing outputs",
          "[PlaceholderParser][PlaceholderParserAssignmentGaps]")
{
    PlaceholderParser parser;
    DynamicConfig outputs;
    outputs.set_key_value("scalar", new ConfigOptionInt(4));
    outputs.set_key_value("empty_values", new ConfigOptionInts({}));
    auto run = [&parser, &outputs](const std::string &script) {
        return parser.process(script, 0, nullptr, &outputs, nullptr);
    };

    CHECK_THROWS_WITH(run("{empty(scalar)}"), Catch::Matchers::ContainsSubstring("not a vector variable"));
    CHECK_THROWS_WITH(run("{size(scalar)}"), Catch::Matchers::ContainsSubstring("not a vector variable"));
    CHECK_THROWS_WITH(run("{empty(empty_values[0])}"), Catch::Matchers::ContainsSubstring("not a vector variable"));
    CHECK_THROWS_WITH(run("{size(empty_values[0])}"), Catch::Matchers::ContainsSubstring("not a vector variable"));
    CHECK_THROWS_WITH(run("{is_nil(empty_values[0])}"), Catch::Matchers::ContainsSubstring("Indexing an empty vector variable"));
    CHECK(outputs.option<ConfigOptionInt>("scalar")->value == 4);
}

TEST_CASE("Placeholder parser rejects local names that shadow system and output symbols",
          "[PlaceholderParser][PlaceholderParserAssignmentGaps]")
{
    auto config = DynamicPrintConfig::full_print_config();
    PlaceholderParser parser;
    REQUIRE(parser.apply_config(config));
    CHECK_THROWS_WITH(parser.process("{local layer_height = 2}"),
                      Catch::Matchers::ContainsSubstring("read-only system dictionary"));

    DynamicConfig outputs;
    outputs.set_key_value("output_value", new ConfigOptionInt(1));
    CHECK_THROWS_WITH(parser.process("{local output_value = 2}", 0, nullptr, &outputs, nullptr),
                      Catch::Matchers::ContainsSubstring("system output variable"));
}

TEST_CASE("Placeholder parser resolves vector percentages against the selected parent element",
          "[PlaceholderParser][PlaceholderParserPercentParents]")
{
    auto config = DynamicPrintConfig::full_print_config();
    config.set_key_value("bridge_speed", new ConfigOptionFloats({ 20., 40. }));
    config.set_key_value("internal_bridge_speed", new ConfigOptionFloatsOrPercents({
        FloatOrPercent{ 150., true }, FloatOrPercent{ 50., true }
    }));
    PlaceholderParser parser;
    REQUIRE(parser.apply_config(config));

    CHECK_THAT(std::stod(parser.process("{internal_bridge_speed}", 0)), Catch::Matchers::WithinAbs(30., 1e-12));
    CHECK_THAT(std::stod(parser.process("{internal_bridge_speed}", 1)), Catch::Matchers::WithinAbs(20., 1e-12));
    CHECK_THAT(std::stod(parser.process("{internal_bridge_speed[0]}")), Catch::Matchers::WithinAbs(30., 1e-12));
    CHECK_THAT(std::stod(parser.process("{internal_bridge_speed[1]}")), Catch::Matchers::WithinAbs(20., 1e-12));

    // Single-element child and parent vectors use element zero for any extruder.
    auto single_config = DynamicPrintConfig::full_print_config();
    single_config.set_key_value("bridge_speed", new ConfigOptionFloats({ 25. }));
    single_config.set_key_value("internal_bridge_speed",
                                new ConfigOptionFloatsOrPercents({ FloatOrPercent{ 120., true } }));
    PlaceholderParser single_parser;
    REQUIRE(single_parser.apply_config(single_config));
    CHECK_THAT(std::stod(single_parser.process("{bridge_speed}", 7)), Catch::Matchers::WithinAbs(25., 1e-12));
    CHECK_THAT(std::stod(single_parser.process("{internal_bridge_speed}", 7)), Catch::Matchers::WithinAbs(30., 1e-12));
}
