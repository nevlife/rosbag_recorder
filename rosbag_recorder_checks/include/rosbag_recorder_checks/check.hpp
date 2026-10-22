// Copyright 2026 pgw
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef ROSBAG_RECORDER_CHECKS__CHECK_HPP_
#define ROSBAG_RECORDER_CHECKS__CHECK_HPP_

#include <cstdint>
#include <iomanip>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "rclcpp/serialized_message.hpp"

namespace rosbag_recorder_checks
{

using Thresholds = std::map<std::string, double>;

struct TopicInfo
{
  std::string name;
  std::string type;
  bool latched{false};
};

struct Context
{
  double wall{0.0};
  bool sim_time{false};
  std::uint32_t publishers{0};
};

struct Result
{
  std::vector<std::string> issues;
  std::vector<std::string> values;
};

class Check
{
public:
  virtual ~Check() = default;

  virtual bool accepts(const TopicInfo & topic) const = 0;
  virtual void configure(const TopicInfo & topic, const Thresholds & thresholds) = 0;
  // wall: receive time in seconds since the epoch.
  virtual void on_message(const rclcpp::SerializedMessage & message, double wall) = 0;
  // Judges the messages received since the last call; false when there were none.
  virtual bool evaluate(const Context & context, Result & result) = 0;
};

inline double threshold(const Thresholds & thresholds, const std::string & key, double fallback)
{
  const auto it = thresholds.find(key);
  return it == thresholds.end() ? fallback : it->second;
}

inline std::string fixed(double value, int precision)
{
  std::ostringstream os;
  os << std::fixed << std::setprecision(precision) << value;
  return os.str();
}

}  // namespace rosbag_recorder_checks

#endif  // ROSBAG_RECORDER_CHECKS__CHECK_HPP_
