#include <iostream>
#include <string>
#include <utility>

#include <nlohmann/json.hpp>

#include "libslic3r/Config.hpp"
#include "libslic3r/PrintConfig.hpp"

// A runtime source of truth for auditing the static settings inventory. This
// reports registration metadata; it makes no claim about behavioral coverage.
int main()
{
    nlohmann::json options = nlohmann::json::array();
    for (const auto &[key, definition] : Slic3r::print_config_def.options) {
        nlohmann::json canonical_values = nlohmann::json::array();
        if (definition.enum_keys_map != nullptr)
            for (const auto &[token, value] : *definition.enum_keys_map) {
                (void) value;
                canonical_values.push_back(token);
            }
        options.push_back({
            {"key", key},
            {"type_id", static_cast<int>(definition.type)},
            {"nullable", definition.nullable},
            {"has_default", definition.default_value.get() != nullptr},
            {"enum_values", definition.enum_values},
            {"canonical_enum_values", canonical_values},
        });
    }
    const nlohmann::json report = {
        {"schema_version", 1},
        {"source", "PrintConfigDef::options"},
        {"settings_count", options.size()},
        {"options", std::move(options)},
    };
    std::cout << report.dump(2) << '\n';
    return std::cout.good() ? 0 : 1;
}
