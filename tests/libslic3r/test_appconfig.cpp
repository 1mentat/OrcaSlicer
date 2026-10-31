#include <catch2/catch_all.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include "libslic3r/AppConfig.hpp"

#include <string>
#include <utility>

using namespace Slic3r;

TEST_CASE("AppConfig network version helpers", "[AppConfig]") {
    AppConfig config;

    SECTION("skipped versions starts empty") {
        auto skipped = config.get_skipped_network_versions();
        REQUIRE(skipped.empty());
    }

    SECTION("add and check skipped version") {
        config.add_skipped_network_version("02.01.01.52");
        REQUIRE(config.is_network_version_skipped("02.01.01.52"));
        REQUIRE_FALSE(config.is_network_version_skipped("02.03.00.62"));
    }

    SECTION("multiple skipped versions") {
        config.add_skipped_network_version("02.01.01.52");
        config.add_skipped_network_version("02.00.02.50");

        auto skipped = config.get_skipped_network_versions();
        REQUIRE(skipped.size() == 2);
        REQUIRE(config.is_network_version_skipped("02.01.01.52"));
        REQUIRE(config.is_network_version_skipped("02.00.02.50"));
    }

    SECTION("clear skipped versions") {
        config.add_skipped_network_version("02.01.01.52");
        config.clear_skipped_network_versions();
        REQUIRE_FALSE(config.is_network_version_skipped("02.01.01.52"));
    }

    SECTION("duplicate add is idempotent") {
        config.add_skipped_network_version("02.01.01.52");
        config.add_skipped_network_version("02.01.01.52");

        auto skipped = config.get_skipped_network_versions();
        REQUIRE(skipped.size() == 1);
        REQUIRE(config.is_network_version_skipped("02.01.01.52"));
    }
}

TEST_CASE("Remembered checkbox settings retain both selections", "[AppConfig][Regression]") {
    AppConfig config;
    const bool checked = GENERATE(false, true);

    config.set("recent", "checkbox", checked ? "1" : "0");
    CHECK(config.get("recent", "checkbox") == (checked ? "1" : "0"));
}

TEST_CASE("Boolean setters retain their established encoding", "[AppConfig][Regression]") {
    AppConfig config;
    const bool value = GENERATE(false, true);

    config.set("recent", "flag", value);
    CHECK(config.get("recent", "flag") == (value ? "true" : "false"));
}

TEST_CASE("Boolean reads use only the requested section", "[AppConfig][Regression]") {
    AppConfig config;
    const bool value = GENERATE(false, true);
    config.set("recent", "flag", std::string(value ? "1" : "0"));
    config.set("app", "flag", std::string(value ? "0" : "1"));

    CHECK(config.get_bool("recent", "flag") == value);
}

TEST_CASE("AppConfig Speed Dial recent count defaults, clamps and parses", "[AppConfig]") {
    AppConfig config;

    SECTION("unset falls back to the default") {
        REQUIRE(config.get_speed_dial_recent_count() == SPEED_DIAL_RECENT_COUNT_DEFAULT);
    }

    SECTION("zero disables recents") {
        config.set(SETTING_SPEED_DIAL_RECENT_COUNT, "0");
        REQUIRE(config.get_speed_dial_recent_count() == 0);
    }

    SECTION("a value in range is returned as-is") {
        config.set(SETTING_SPEED_DIAL_RECENT_COUNT, "7");
        REQUIRE(config.get_speed_dial_recent_count() == 7);
    }

    SECTION("the maximum is kept") {
        config.set(SETTING_SPEED_DIAL_RECENT_COUNT, "10");
        REQUIRE(config.get_speed_dial_recent_count() == SPEED_DIAL_RECENT_COUNT_MAX);
    }

    SECTION("values above the maximum clamp down") {
        config.set(SETTING_SPEED_DIAL_RECENT_COUNT, "42");
        REQUIRE(config.get_speed_dial_recent_count() == SPEED_DIAL_RECENT_COUNT_MAX);
    }

    SECTION("negative values clamp up to 0") {
        config.set(SETTING_SPEED_DIAL_RECENT_COUNT, "-3");
        REQUIRE(config.get_speed_dial_recent_count() == 0);
    }

    SECTION("garbage falls back to the default") {
        config.set(SETTING_SPEED_DIAL_RECENT_COUNT, "abc");
        REQUIRE(config.get_speed_dial_recent_count() == SPEED_DIAL_RECENT_COUNT_DEFAULT);
    }
}

TEST_CASE("AppConfig normalizes language codes", "[AppConfig][AppConfigPreferences][AppConfigLanguage]")
{
    AppConfig config;
    const auto [language, expected] = GENERATE(
        std::make_pair(std::string("zh_CN"), std::string("zh-cn")),
        std::make_pair(std::string("en_US"), std::string("en")),
        std::make_pair(std::string("x"), std::string("x")),
        std::make_pair(std::string(), std::string()));
    config.set("language", language);
    REQUIRE(config.get_language_code() == expected);
}

TEST_CASE("AppConfig uses the stable HMS host", "[AppConfig][AppConfigPreferences]")
{
    AppConfig config;
    const std::string environment = GENERATE(std::string(ENV_DEV_HOST), std::string(ENV_QAT_HOST),
                                             std::string(ENV_PRE_HOST), std::string(ENV_PRODUCT_HOST));
    config.set("iot_environment", environment);
    REQUIRE(config.get_hms_host() == "e.bambulab.com");
}

TEST_CASE("AppConfig gates stealth mode on setup completion", "[AppConfig][AppConfigPreferences][AppConfigStealth]")
{
    AppConfig config;
    config.set_bool("stealth_mode", false);
    REQUIRE(config.get_stealth_mode());

    config.set("firstguide", "finish", true);
    REQUIRE_FALSE(config.get_stealth_mode());

    config.set_bool("stealth_mode", true);
    REQUIRE(config.get_stealth_mode());
}

TEST_CASE("AppConfig reads login panel visibility", "[AppConfig][AppConfigPreferences]")
{
    AppConfig config;
    const bool hidden = GENERATE(false, true);
    config.set_bool("hide_login_side_panel", hidden);
    REQUIRE(config.get_hide_login_side_panel() == hidden);
}

TEST_CASE("AppConfig maps region names to country codes", "[AppConfig][AppConfigPreferences][AppConfigRegion]")
{
    AppConfig config;
    const auto [region, country] = GENERATE(
        std::make_pair(std::string("CHN"), std::string("CN")),
        std::make_pair(std::string("China"), std::string("CN")),
        std::make_pair(std::string("USA"), std::string("US")),
        std::make_pair(std::string("Asia-Pacific"), std::string("Others")),
        std::make_pair(std::string("Europe"), std::string("US")),
        std::make_pair(std::string("North America"), std::string("US")),
        std::make_pair(std::string("Unknown"), std::string("Others")),
        std::make_pair(std::string(), std::string("Others")));
    config.set("region", region);
    REQUIRE(config.get_region() == region);
    REQUIRE(config.get_country_code() == country);
}

TEST_CASE("AppConfig identifies engineering environments", "[AppConfig][AppConfigPreferences]")
{
    AppConfig config;
    const auto [environment, engineering] = GENERATE(
        std::make_pair(std::string(ENV_DEV_HOST), true),
        std::make_pair(std::string(ENV_QAT_HOST), true),
        std::make_pair(std::string(ENV_PRE_HOST), true),
        std::make_pair(std::string(ENV_PRODUCT_HOST), false),
        std::make_pair(std::string("unknown"), false));
    config.set("iot_environment", environment);
    REQUIRE(config.is_engineering_region() == engineering);
}

TEST_CASE("AppConfig round-trips network update state", "[AppConfig][AppConfigPreferences]")
{
    AppConfig config;
    REQUIRE(config.get_network_plugin_version().empty());
    config.set_network_plugin_version("02.01.03.04");
    REQUIRE(config.get_network_plugin_version() == "02.01.03.04");

    const bool prompts_disabled = GENERATE(false, true);
    config.set_network_update_prompt_disabled(prompts_disabled);
    REQUIRE(config.is_network_update_prompt_disabled() == prompts_disabled);

    config.set_remind_network_update_later(true);
    REQUIRE(config.should_remind_network_update_later());
    config.clear_remind_network_update_later();
    REQUIRE_FALSE(config.should_remind_network_update_later());
}

TEST_CASE("AppConfig resolves endpoint overrides", "[AppConfig][AppConfigPreferences]")
{
    AppConfig config;
    REQUIRE(config.version_check_url() == "https://check-version.orcaslicer.com/latest");
    REQUIRE(config.profile_update_url() == "https://check-version.orcaslicer.com/profile");

    config.set("version_check_url", "https://example.invalid/version");
    config.set("orca_updater_url", "https://example.invalid/profile");
    REQUIRE(config.version_check_url() == "https://example.invalid/version");
    REQUIRE(config.profile_update_url() == "https://example.invalid/profile");
}
