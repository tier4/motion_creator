
#include "read_yaml.hpp"


namespace rviz_plugins
{

std::tuple<bool, std::string, int32_t, std::vector<std::string>> parsePresetYAML(const std::string &filepath)
{
  ROS_INFO("read: %s", filepath.c_str());
  try
  {
    YAML::Node node = YAML::LoadFile(filepath);
    // for(const auto &n : node)
    //  ROS_INFO_STREAM(n.first << ": "  << n.second);

    std::string topic_name = node["topic_name"].as<std::string>();
    int32_t density = node["density"].as<int32_t>();

    std::vector<std::string> file_list;
    for (const auto &e : node["file_list"])
      file_list.push_back(e.as<std::string>());

    return std::make_tuple(true, topic_name, density, file_list);
  }
  catch (YAML::Exception &e)
  {
    ROS_ERROR("%s", e.what());
    return std::make_tuple(false, "", -1, std::vector<std::string>());
  }
  catch (std::out_of_range &e)
  {
    ROS_ERROR("%s", e.what());
    return std::make_tuple(false, "", -1, std::vector<std::string>());
  }
}
}