#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include "libslic3r/GCode/AdaptivePAProcessor.hpp"

#include <string>

namespace adaptive_pa_validation_contracts {
using Slic3r::AdaptivePAProcessor;
using Catch::Matchers::ContainsSubstring;

TEST_CASE("Adaptive PA accepts empty models and blank physical lines", "[GCode][AdaptivePAValidation]")
{
    const std::string input = GENERATE("", "\n", " \t\r\n\n \t");
    CHECK(AdaptivePAProcessor::validate_adaptive_pa_model(input).empty());
}

TEST_CASE("Adaptive PA accepts complete decimal triples and edge whitespace", "[GCode][AdaptivePAValidation]")
{
    const std::string input = GENERATE("0,1,2", ".1,10.,1000", "1.999,2,2.001", " \t0.04,12.5,3000\t\r\n");
    CHECK(AdaptivePAProcessor::validate_adaptive_pa_model(input).empty());
}

TEST_CASE("Adaptive PA accepts multiple independently valid model rows", "[GCode][AdaptivePAValidation]")
{
    CHECK(AdaptivePAProcessor::validate_adaptive_pa_model("0.04,10,1000\n\n0.05,20,2000\n0.06,30,3000\n").empty());
}

TEST_CASE("Adaptive PA rejects an incorrect number of comma separated fields", "[GCode][AdaptivePAValidation]")
{
    const std::string input = GENERATE("0.1", "0.1,10", "0.1,10,1000,2000", "0.1,10,1000,");
    CHECK_THAT(AdaptivePAProcessor::validate_adaptive_pa_model(input), ContainsSubstring("exactly 3"));
}

TEST_CASE("Adaptive PA rejects missing numeric values in every column", "[GCode][AdaptivePAValidation]")
{
    const std::string input = GENERATE(",10,1000", "0.1,,1000", "0.1,10,", ",,");
    CHECK_FALSE(AdaptivePAProcessor::validate_adaptive_pa_model(input).empty());
}

TEST_CASE("Adaptive PA rejects partial decimal parses in every column", "[GCode][AdaptivePAValidation]")
{
    const std::string input = GENERATE("0.1.2,10,1000", "0.1,10.20.30,1000", "0.1,10,1000.2.3");
    CHECK_THAT(AdaptivePAProcessor::validate_adaptive_pa_model(input), ContainsSubstring("invalid numeric"));
}

TEST_CASE("Adaptive PA rejects decimal punctuation without digits", "[GCode][AdaptivePAValidation]")
{
    const std::string input = GENERATE(".,10,1000", "0.1,.,1000", "0.1,10,.", "..,10,1000");
    CHECK_THAT(AdaptivePAProcessor::validate_adaptive_pa_model(input), ContainsSubstring("invalid numeric"));
}

TEST_CASE("Adaptive PA rejects signs exponents and embedded whitespace", "[GCode][AdaptivePAValidation]")
{
    const std::string input = GENERATE("-0.1,10,1000", "+0.1,10,1000", "0.1,1e1,1000", "0.1,10,10 00", "0.1, 10,1000", "0.1,10,NaN", "0.1,10,inf");
    CHECK_THAT(AdaptivePAProcessor::validate_adaptive_pa_model(input), ContainsSubstring("only numbers"));
}

TEST_CASE("Adaptive PA rejects out of range decimal magnitudes", "[GCode][AdaptivePAValidation]")
{
    const std::string enormous(400, '9');
    const int column = GENERATE(0, 1, 2);
    std::string input = column == 0 ? enormous + ",10,1000" : column == 1 ? "0.1," + enormous + ",1000" : "0.1,10," + enormous;
    CHECK_THAT(AdaptivePAProcessor::validate_adaptive_pa_model(input), ContainsSubstring("invalid numeric"));
}

TEST_CASE("Adaptive PA enforces its pressure advance upper boundary", "[GCode][AdaptivePAValidation]")
{
    const std::string input = GENERATE("2,10,1000", "2.001,10,1000");
    CHECK_THAT(AdaptivePAProcessor::validate_adaptive_pa_model(input), ContainsSubstring("PA value must be less than 2"));
}

TEST_CASE("Adaptive PA requires flow and acceleration to increase strictly", "[GCode][AdaptivePAValidation]")
{
    const std::string input = GENERATE("0.1,0.1,1000", "0.1,0.09,1000", "0.1,10,10", "0.1,10,9.9");
    CHECK_THAT(AdaptivePAProcessor::validate_adaptive_pa_model(input), ContainsSubstring("must be greater than"));
}

TEST_CASE("Adaptive PA errors identify the first invalid physical line", "[GCode][AdaptivePAValidation]")
{
    const std::string result = AdaptivePAProcessor::validate_adaptive_pa_model("\n0.1,10,1000\n\t\r\n0.2,20,20\n2,10,1000");
    CHECK_THAT(result, ContainsSubstring("Line 4:"));
    CHECK_THAT(result, ContainsSubstring("acceleration"));
}
} // namespace adaptive_pa_validation_contracts
