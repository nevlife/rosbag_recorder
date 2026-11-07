#ifndef PREFLIGHT_HPP_
#define PREFLIGHT_HPP_

#include "rosbag_recorder_msgs/msg/check_result.hpp"
#include "rosbag_recorder_msgs/msg/topic_health.hpp"

#include <set>
#include <string>
#include <vector>

namespace rosbag_recorder
{

using rosbag_recorder_msgs::msg::CheckResult;
using rosbag_recorder_msgs::msg::TopicHealth;

struct PreflightInput
{
  std::string output_dir;
  bool writable{false};
  double disk_free_gb{0.0};
  double disk_warn_gb{0.0};
  double disk_stop_gb{0.0};
  std::vector<std::string> unknown_topics;
  std::vector<TopicHealth> topics;
  std::vector<std::string> required_frames;
  std::set<std::string> frames;
};

std::vector<CheckResult> preflight(const PreflightInput & in);
std::vector<std::string> required_missing(const std::vector<TopicHealth> & topics);

}  // namespace rosbag_recorder

#endif  // PREFLIGHT_HPP_
