// Copyright 2026 nevlife
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef BUILTIN_CHECKS_HPP_
#define BUILTIN_CHECKS_HPP_

#include <cstdint>
#include <set>
#include <string>

#include "rclcpp/serialization.hpp"
#include "rosbag_recorder_checks/check.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

namespace rosbag_recorder_checks
{

// header.stamp read from the CDR bytes: delay, future, zero, backwards and repeated stamps.
class HeaderStamp : public Check
{
public:
  bool accepts(const TopicInfo & topic) const override;
  void configure(const TopicInfo & topic, const Thresholds & thresholds) override;
  void on_message(const rclcpp::SerializedMessage & message, double wall) override;
  bool evaluate(const Context & context, Result & result) override;

private:
  double delay_max_{0.5};
  double future_max_{0.2};
  std::int64_t previous_{-1};
  std::uint64_t count_{0}, zero_{0}, backwards_{0}, same_run_{0}, same_max_{0};
  double delay_sum_{0.0};
  double delay_max_seen_{-1e18};
};

// Deserializes small messages of one type with a reused message object.
template<class MessageT>
class Decoding : public Check
{
protected:
  bool decode(const rclcpp::SerializedMessage & message)
  {
    try {
      serialization_.deserialize_message(&message, &message_);
    } catch (const std::exception &) {
      ++bad_decode_;
      return false;
    }
    return true;
  }

  void report_decode(Result & result)
  {
    if (bad_decode_ > 0) {
      result.issues.push_back("deserialization failed " + std::to_string(bad_decode_) + " times");
    }
    bad_decode_ = 0;
  }

  rclcpp::Serialization<MessageT> serialization_;
  MessageT message_;
  std::uint64_t bad_decode_{0};
};

// All-zero or non-finite IMU values.
class Imu : public Decoding<sensor_msgs::msg::Imu>
{
public:
  bool accepts(const TopicInfo & topic) const override;
  void configure(const TopicInfo & topic, const Thresholds & thresholds) override;
  void on_message(const rclcpp::SerializedMessage & message, double wall) override;
  bool evaluate(const Context & context, Result & result) override;

private:
  std::uint64_t count_{0}, valid_{0}, nonfinite_{0}, zero_{0};
  double acceleration_sum_{0.0};
};

// GNSS horizontal position accuracy from the covariance.
class NavSatFix : public Decoding<sensor_msgs::msg::NavSatFix>
{
public:
  bool accepts(const TopicInfo & topic) const override;
  void configure(const TopicInfo & topic, const Thresholds & thresholds) override;
  void on_message(const rclcpp::SerializedMessage & message, double wall) override;
  bool evaluate(const Context & context, Result & result) override;

private:
  double max_std_{10.0};
  int status_{-1};
  int covariance_type_{0};
  double std_m_{0.0};
  std::uint64_t count_{0}, bad_covariance_{0};
};

// Non-finite transforms and quaternions that are not unit length.
class TfMessage : public Decoding<tf2_msgs::msg::TFMessage>
{
public:
  bool accepts(const TopicInfo & topic) const override;
  void configure(const TopicInfo & topic, const Thresholds & thresholds) override;
  void on_message(const rclcpp::SerializedMessage & message, double wall) override;
  bool evaluate(const Context & context, Result & result) override;

private:
  double quaternion_tolerance_{0.01};
  std::set<std::string> bad_, frames_;
  std::uint64_t count_{0};
};

// Point clouds without points, read from the CDR bytes.
class PointCloud2 : public Check
{
public:
  bool accepts(const TopicInfo & topic) const override;
  void configure(const TopicInfo & topic, const Thresholds & thresholds) override;
  void on_message(const rclcpp::SerializedMessage & message, double wall) override;
  bool evaluate(const Context & context, Result & result) override;

private:
  std::uint64_t count_{0}, empty_{0}, points_{0};
};

}  // namespace rosbag_recorder_checks

#endif  // BUILTIN_CHECKS_HPP_
