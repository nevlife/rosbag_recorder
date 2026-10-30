#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rosbag_recorder_source/source.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "yaml-cpp/yaml.h"

namespace rosbag_recorder_test
{

// A source without ROS topics: a counter and an IMU-shaped stamped channel, written from its own
// thread, with a marker every `mark_every` counts.
class CounterSource : public rosbag_recorder_source::Source
{
public:
  ~CounterSource() override {stop();}

  void initialize(rclcpp::Node & node, const std::string & name, const std::string & params)
  override
  {
    clock_ = node.get_clock();
    name_ = name;
    const YAML::Node p = YAML::Load(params);
    rate_hz_ = p["rate_hz"] ? p["rate_hz"].as<double>() : 20.0;
    mark_every_ = p["mark_every"] ? p["mark_every"].as<std::uint64_t>() : 0;
  }

  std::vector<rosbag_recorder_source::Channel> channels() override
  {
    rosbag_recorder_source::Channel value;
    value.name = "/" + name_ + "/value";
    value.type = "std_msgs/msg/UInt64";
    value.definition = "uint64 data\n";
    value.definition_encoding = "ros2msg";
    value.hz = rate_hz_;
    value.required = true;
    rosbag_recorder_source::Channel imu;
    imu.name = "/" + name_ + "/imu";
    imu.type = "sensor_msgs/msg/Imu";
    imu.hz = rate_hz_;
    return {value, imu};
  }

  void start(rosbag_recorder_source::Sink & sink) override
  {
    running_ = true;
    thread_ = std::thread([this, &sink] {run(sink);});
  }

  void stop() override
  {
    running_ = false;
    if (thread_.joinable()) {
      thread_.join();
    }
  }

private:
  void run(rosbag_recorder_source::Sink & sink)
  {
    const auto period = std::chrono::duration<double>(1.0 / rate_hz_);
    auto next = std::chrono::steady_clock::now();
    for (std::uint64_t count = 1; running_; ++count) {
      const rclcpp::Time now = clock_->now();
      std_msgs::msg::UInt64 value;
      value.data = count;
      auto bytes = std::make_shared<rclcpp::SerializedMessage>();
      rclcpp::Serialization<std_msgs::msg::UInt64>().serialize_message(&value, bytes.get());
      sink.write(0, bytes, now.nanoseconds());
      sensor_msgs::msg::Imu imu;
      imu.header.stamp = now;
      imu.header.frame_id = name_;
      imu.linear_acceleration.z = 9.8;
      imu.orientation.w = 1.0;
      bytes = std::make_shared<rclcpp::SerializedMessage>();
      rclcpp::Serialization<sensor_msgs::msg::Imu>().serialize_message(&imu, bytes.get());
      sink.write(1, bytes, now.nanoseconds());
      if (mark_every_ > 0 && count % mark_every_ == 0) {
        sink.mark("count", "count " + std::to_string(count), "/" + name_ + "/value");
      }
      next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
      std::this_thread::sleep_until(next);
    }
  }

  rclcpp::Clock::SharedPtr clock_;
  std::string name_;
  double rate_hz_{20.0};
  std::uint64_t mark_every_{0};
  std::atomic<bool> running_{false};
  std::thread thread_;
};

}  // namespace rosbag_recorder_test

PLUGINLIB_EXPORT_CLASS(rosbag_recorder_test::CounterSource, rosbag_recorder_source::Source)
