#pragma once

#include <yaml-cpp/yaml.h>
#include <ros/ros.h>

// C++ includes

namespace rviz_plugins
{

std::tuple<bool , std::string, int32_t, std::vector<std::string>> parsePresetYAML(const std::string &filepath);
}