#include <catch2/catch_all.hpp>
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
#include "libslic3r/Config.hpp"
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
    CHECK_THROWS_WITH(parser.process("{local empty = ()}"),
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
