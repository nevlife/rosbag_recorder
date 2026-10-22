// Copyright 2026 nevlife
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

// Publishes stamped IMU messages on <prefix><i> for i < count, all at `rate` Hz.
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("many_publishers");
  const auto count = node->declare_parameter("count", 500);
  const auto rate = node->declare_parameter("rate", 10.0);
  const auto prefix = node->declare_parameter("prefix", std::string("/scale/t"));
  std::vector<rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr> publishers;
  for (int64_t i = 0; i < count; ++i) {
    publishers.push_back(
      node->create_publisher<sensor_msgs::msg::Imu>(prefix + std::to_string(i), 10));
  }
  sensor_msgs::msg::Imu imu;
  imu.header.frame_id = "imu";
  imu.linear_acceleration.z = 9.8;
  imu.orientation.w = 1.0;
  auto timer = node->create_wall_timer(
    std::chrono::duration<double>(1.0 / rate), [&]() {
      imu.header.stamp = node->now();
      for (auto & p : publishers) {
        p->publish(imu);
      }
    });
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
