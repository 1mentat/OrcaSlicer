#include <catch2/catch_all.hpp>

#include <catch2/catch_test_macros.hpp>
#include "libslic3r/Time.hpp"

#include <ctime>
#include <sstream>
#include <iomanip>
#include <locale>
#include <string>

static void test_time_fmt(Slic3r::Utils::TimeFormat fmt) {
    using namespace Slic3r::Utils;
    time_t t = get_current_time_utc();
    
    std::string tstr = time2str(t, TimeZone::local, fmt);
    time_t parsedtime = str2time(tstr, TimeZone::local, fmt);
    REQUIRE(t == parsedtime);
    
    tstr = time2str(t, TimeZone::utc, fmt);
    parsedtime = str2time(tstr, TimeZone::utc, fmt);
    REQUIRE(t == parsedtime);
    
    parsedtime = str2time("not valid string", TimeZone::local, fmt);
    REQUIRE(parsedtime == time_t(-1));
    
    parsedtime = str2time("not valid string", TimeZone::utc, fmt);
    REQUIRE(parsedtime == time_t(-1));
}

TEST_CASE("ISO8601Z", "[Timeutils]") {
    test_time_fmt(Slic3r::Utils::TimeFormat::iso8601Z);
    
    std::string mydate = "20190710T085000Z";
    time_t t = Slic3r::Utils::parse_iso_utc_timestamp(mydate);
    std::string date = Slic3r::Utils::iso_utc_timestamp(t);
    
    REQUIRE(date == mydate);
}

TEST_CASE("Slic3r_UTC_Time_Format", "[Timeutils]") {
    using namespace Slic3r::Utils;
    test_time_fmt(TimeFormat::gcode);
    
    std::string mydate = "2019-07-10 at 08:50:00 UTC";
    time_t t = Slic3r::Utils::str2time(mydate, TimeZone::utc, TimeFormat::gcode);
    std::string date = Slic3r::Utils::utc_timestamp(t);
    
    REQUIRE(date == mydate);
}

TEST_CASE("ISO 8601 millisecond parsing honors the UTC epoch and leap-day boundary", "[Timeutils][Iso8601MillisGap]") {
    using Slic3r::Utils::iso8601_to_millis;

    // These constants are Unix milliseconds computed from the Gregorian UTC dates,
    // independently of the production parser.
    CHECK(iso8601_to_millis("1970-01-01T00:00:00Z") == 0LL);
    CHECK(iso8601_to_millis("2000-02-29T00:00:00.000Z") == 951782400000LL);
    CHECK(iso8601_to_millis("2000-03-01T00:00:00.000Z") == 951868800000LL);
}

TEST_CASE("ISO 8601 millisecond fractions normalize to three digits", "[Timeutils][Iso8601MillisGap]") {
    using Slic3r::Utils::iso8601_to_millis;
    constexpr long long leap_day_midnight = 1709164800000LL;

    CHECK(iso8601_to_millis("2024-02-29T00:00:00Z") == leap_day_midnight);
    CHECK(iso8601_to_millis("2024-02-29T00:00:00.1Z") == leap_day_midnight + 100LL);
    CHECK(iso8601_to_millis("2024-02-29T00:00:00.12Z") == leap_day_midnight + 120LL);
    CHECK(iso8601_to_millis("2024-02-29T00:00:00.123Z") == leap_day_midnight + 123LL);
    CHECK(iso8601_to_millis("2024-02-29T00:00:00.1234Z") == leap_day_midnight + 123LL);
    CHECK(iso8601_to_millis("2024-02-29T00:00:00.123456Z") == leap_day_midnight + 123LL);
}

TEST_CASE("ISO 8601 millisecond formatting round-trips and parse errors use the documented sentinel", "[Timeutils][Iso8601MillisGap]") {
    using Slic3r::Utils::iso8601_to_millis;
    using Slic3r::Utils::millis_to_iso8601;

    constexpr long long leap_day_instant = 1709164800123LL;
    CHECK(millis_to_iso8601(0LL) == "1970-01-01T00:00:00.000Z");
    CHECK(millis_to_iso8601(leap_day_instant) == "2024-02-29T00:00:00.123Z");
    CHECK(iso8601_to_millis(millis_to_iso8601(leap_day_instant)) == leap_day_instant);
    CHECK(iso8601_to_millis("") == -1LL);
    CHECK(iso8601_to_millis("not an ISO 8601 timestamp") == -1LL);
    CHECK(iso8601_to_millis("2024-02-29") == -1LL);
}
