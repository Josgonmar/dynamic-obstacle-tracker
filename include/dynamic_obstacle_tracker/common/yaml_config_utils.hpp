#pragma once

#include <yaml-cpp/yaml.h>

#include <stdexcept>
#include <string>

namespace dynamic_obstacle_tracker::config_loader_detail {

template <typename T>
T readValue(const YAML::Node& root, const char* key, const T& fallback)
{
    const YAML::Node value = root[key];
    if (!value)
        return fallback;

    try {
        return value.as<T>();
    } catch (const YAML::Exception& exception) {
        throw std::runtime_error("Invalid value for '" + std::string(key) + "': " + exception.what());
    }
}

inline YAML::Node readSection(const YAML::Node& root, const char* key)
{
    const YAML::Node section = root[key];
    if (!section)
        return {};

    if (!section.IsMap())
        throw std::runtime_error("Config section '" + std::string(key) + "' must contain a YAML map");

    return section;
}

} // namespace dynamic_obstacle_tracker::config_loader_detail
