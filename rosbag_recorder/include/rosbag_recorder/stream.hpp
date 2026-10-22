// Copyright 2026 pgw
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef ROSBAG_RECORDER__STREAM_HPP_
#define ROSBAG_RECORDER__STREAM_HPP_

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "rclcpp/generic_subscription.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/serialization.hpp"
#include "rosbag_recorder/config.hpp"
#include "rosbag_recorder_checks/check.hpp"
#include "rosbag_recorder_checks/message.hpp"
#include "rosbag_recorder_msgs/msg/topic_health.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

namespace rosbag_recorder
{

using rosbag_recorder_msgs::msg::TopicHealth;

struct Rate
{
  double hz{0.0};
  std::optional<double> age;
  std::uint8_t status{TopicHealth::WAITING};
};

// Rate and liveness from arrival times (steady clock, seconds), as SCV's monitor judged them.
Rate evaluate_rate(
  std::optional<double> expected, const std::deque<double> & arrivals, std::uint64_t count,
  double now, double tolerance, std::uint32_t publishers, bool latched, bool type_error);

struct StreamSpec
{
  std::string name;
  std::string type;
  std::string source;
  std::optional<double> hz;
  bool required{false};
  bool latched{false};
  bool heavy{false};
  bool record{true};
};

// One recorded topic or source channel: its subscription, rate statistics and value checks.
class Stream
{
public:
  explicit Stream(StreamSpec spec);

  void set_checks(std::vector<std::shared_ptr<rosbag_recorder_checks::Check>> checks);
  // Throws std::runtime_error when the type cannot be introspected.
  void set_auto_marker(const AutoMarker & marker);
  void collect_frames();

  // Returns the auto marker text when the watched value changed.
  std::optional<std::string> receive(
    const rclcpp::SerializedMessage & message, double mono, double wall);
  void count_written() {++written_;}
  // Latched messages arrive once, when subscribing; each session's bag starts with them.
  void keep_latched(std::shared_ptr<rclcpp::SerializedMessage> message);
  std::vector<std::shared_ptr<rclcpp::SerializedMessage>> latched() const;

  TopicHealth report(double mono, double wall, double tolerance, bool sim_time);
  std::set<std::string> frames() const;
  std::uint64_t received() const;
  std::uint64_t written() const {return written_;}
  void reset_counts();

  StreamSpec spec;
  std::string serialization_format{"cdr"};
  std::string definition;
  std::string definition_encoding;
  std::vector<rclcpp::QoS> offers;
  std::string auto_kind;
  rclcpp::GenericSubscription::SharedPtr subscription;
  std::string error;
  std::uint32_t publishers{0};
  bool marker_open{false};
  double last_marker{-1e18};
  std::vector<std::string> held_issues;

private:
  mutable std::mutex mutex_;
  std::deque<double> arrivals_;
  std::deque<std::size_t> sizes_;
  std::uint64_t received_{0};
  std::atomic<std::uint64_t> written_{0};
  std::vector<std::shared_ptr<rosbag_recorder_checks::Check>> checks_;
  rosbag_recorder_checks::Result result_;
  double last_result_{-1e18};
  double held_until_{-1e18};
  std::optional<AutoMarker> auto_marker_;
  std::unique_ptr<rosbag_recorder_checks::TypeInfo> type_info_;
  std::optional<std::string> last_value_;
  std::string auto_marker_error_;
  bool frames_on_{false};
  std::set<std::string> frames_;
  std::deque<std::shared_ptr<rclcpp::SerializedMessage>> latched_;
  rclcpp::Serialization<tf2_msgs::msg::TFMessage> tf_serialization_;
};

}  // namespace rosbag_recorder

#endif  // ROSBAG_RECORDER__STREAM_HPP_
