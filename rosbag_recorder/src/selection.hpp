#ifndef SELECTION_HPP_
#define SELECTION_HPP_

#include "rosbag_recorder_msgs/msg/topic_selection.hpp"

#include <set>
#include <string>
#include <vector>

namespace rosbag_recorder
{

// Flux input NAME is recorded on bag topic NAME/flux.
std::string bag_name(const std::string & name, const std::string & transport);

// Throws ConfigError on an unknown transport, a missing input or a bag topic used twice.
void check_selection(
  const std::vector<rosbag_recorder_msgs::msg::TopicSelection> & topics,
  const std::set<std::string> & ros_available, const std::set<std::string> & flux_available);

}  // namespace rosbag_recorder

#endif  // SELECTION_HPP_
