#ifndef STREAM_HPP_
#define STREAM_HPP_

#include "rclcpp/generic_subscription.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/serialization.hpp"

#include "rosbag_recorder_msgs/msg/topic_health.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <vector>

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
};

// One recorded ROS topic or Flux input: its subscription and rate statistics.
class Stream
{
public:
  explicit Stream(StreamSpec spec);

  void collect_frames() { frames_on_ = true; }
  void receive(const rclcpp::SerializedMessage & message, double mono);
  void count_written() { ++written_; }
  // Latched messages arrive once, when subscribing; each session's bag starts with them.
  void keep_latched(std::shared_ptr<rclcpp::SerializedMessage> message);
  const std::deque<std::shared_ptr<rclcpp::SerializedMessage>> & latched() const
  {
    return latched_;
  }

  TopicHealth report(double mono, double tolerance) const;
  const std::set<std::string> & frames() const { return frames_; }
  std::uint64_t written() const { return written_; }
  void reset_counts() { written_ = 0; }

  StreamSpec spec;
  std::vector<rclcpp::QoS> offers;
  rclcpp::GenericSubscription::SharedPtr subscription;
  std::optional<rclcpp::QoS> subscription_qos;
  std::string error;
  std::uint32_t publishers{0};

private:
  std::deque<double> arrivals_;
  std::deque<std::size_t> sizes_;
  std::uint64_t received_{0};
  std::uint64_t written_{0};
  bool frames_on_{false};
  std::set<std::string> frames_;
  std::deque<std::shared_ptr<rclcpp::SerializedMessage>> latched_;
  rclcpp::Serialization<tf2_msgs::msg::TFMessage> tf_serialization_;
};

}  // namespace rosbag_recorder

#endif  // STREAM_HPP_
