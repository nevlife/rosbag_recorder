#ifndef SUMMARY_HPP_
#define SUMMARY_HPP_

#include "rosbag_recorder_msgs/msg/check_result.hpp"
#include "rosbag_recorder_msgs/msg/marker.hpp"
#include "rosbag_recorder_msgs/msg/session.hpp"
#include "rosbag_recorder_msgs/msg/topic_health.hpp"

#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace rosbag_recorder
{

struct Summary
{
  rosbag_recorder_msgs::msg::Session end;
  std::string host;
  double duration_s{0.0};
  std::uint64_t bag_bytes{0};
  std::vector<rosbag_recorder_msgs::msg::CheckResult> preflight;
  std::vector<rosbag_recorder_msgs::msg::TopicHealth> topics;
  std::vector<rosbag_recorder_msgs::msg::Marker> markers;
  // Message count per topic in the closed bag's metadata.
  std::map<std::string, std::uint64_t> stored;
};

// summary.yaml, written next to the bag at session end.
std::string summary_yaml(const Summary & s);

// Messages handed to the writer that are not in the bag. The writer cache reports a drop only in
// a log line, so it shows up as this gap.
std::uint64_t dropped(const Summary & s);

}  // namespace rosbag_recorder

#endif  // SUMMARY_HPP_
