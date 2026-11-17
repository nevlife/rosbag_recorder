#include "selection.hpp"

#include "config.hpp"

#include <set>
#include <string>
#include <vector>

namespace rosbag_recorder
{

std::string bag_name(const std::string & name, const std::string & transport)
{
  return transport == "flux" ? name + "/flux" : name;
}

void check_selection(
  const std::vector<rosbag_recorder_msgs::msg::TopicSelection> & topics,
  const std::set<std::string> & ros_available, const std::set<std::string> & flux_available,
  bool with_flux)
{
  std::set<std::string> bag_names;
  for (const auto & topic : topics) {
    if (topic.transport != "ros" && topic.transport != "flux") {
      throw ConfigError("unknown transport: " + topic.transport);
    }
    if (topic.transport == "flux" && !with_flux) {
      throw ConfigError(
        "transport flux is not available: rosbag_recorder was built without Flux "
        "(ENABLE_FLUX=1 at build time enables it)");
    }
    const auto & available = topic.transport == "flux" ? flux_available : ros_available;
    if (!available.count(topic.name)) {
      throw ConfigError("topic not available: " + topic.name);
    }
    if (!bag_names.insert(bag_name(topic.name, topic.transport)).second) {
      throw ConfigError("duplicate bag topic: " + bag_name(topic.name, topic.transport));
    }
  }
}

}  // namespace rosbag_recorder
