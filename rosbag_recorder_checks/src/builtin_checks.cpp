#include "builtin_checks.hpp"

#include <algorithm>
#include <cmath>
#include <string>

#include "pluginlib/class_list_macros.hpp"
#include "rosbag_recorder_checks/message.hpp"

namespace rosbag_recorder_checks
{
namespace
{

bool finite3(double a, double b, double c)
{
  return std::isfinite(a) && std::isfinite(b) && std::isfinite(c);
}

}  // namespace

bool HeaderStamp::accepts(const TopicInfo & topic) const
{
  return !topic.latched && TypeInfo(topic.type).starts_with_header();
}

void HeaderStamp::configure(const TopicInfo &, const Thresholds & thresholds)
{
  delay_max_ = threshold(thresholds, "stamp_delay_max_s", delay_max_);
  future_max_ = threshold(thresholds, "stamp_future_max_s", future_max_);
}

void HeaderStamp::on_message(const rclcpp::SerializedMessage & message, double wall)
{
  const auto & raw = message.get_rcl_serialized_message();
  const auto stamp = read_stamp(raw.buffer, raw.buffer_length);
  if (!stamp) {
    return;
  }
  ++count_;
  const std::int64_t ns = stamp->nanoseconds();
  if (ns == 0) {
    ++zero_;
    previous_ = 0;
    return;
  }
  if (previous_ > 0) {
    if (ns < previous_) {
      ++backwards_;
    }
    same_run_ = ns == previous_ ? same_run_ + 1 : 0;
    same_max_ = std::max(same_max_, same_run_);
  }
  previous_ = ns;
  const double delay = wall - static_cast<double>(ns) * 1e-9;
  delay_sum_ += delay;
  delay_max_seen_ = std::max(delay_max_seen_, delay);
}

bool HeaderStamp::evaluate(const Context & context, Result & result)
{
  if (count_ == 0) {
    return false;
  }
  if (zero_ == count_) {
    result.values.push_back("stamp 0");
  } else {
    const std::uint64_t stamped = count_ - zero_;
    if (!context.sim_time) {
      const double mean = delay_sum_ / static_cast<double>(stamped);
      if (mean > delay_max_) {
        result.issues.push_back(
          "stamp delay mean " + fixed(mean * 1000, 0) + " ms (max " +
          fixed(delay_max_seen_ * 1000, 0) + ")");
      } else if (mean < -future_max_) {
        result.issues.push_back("stamp " + fixed(-mean, 2) + " s ahead of receive time");
      }
      result.values.push_back("delay " + fixed(mean * 1000, 0) + " ms");
    }
    if (context.publishers == 1) {
      if (backwards_ > 0) {
        result.issues.push_back("stamp went backwards " + std::to_string(backwards_) + " times");
      }
      if (same_max_ >= 2) {
        result.issues.push_back("same stamp " + std::to_string(same_max_ + 1) + " times in a row");
      }
    }
  }
  count_ = zero_ = backwards_ = same_max_ = 0;
  delay_sum_ = 0.0;
  delay_max_seen_ = -1e18;
  return true;
}

bool Imu::accepts(const TopicInfo & topic) const
{
  return topic.type == "sensor_msgs/msg/Imu";
}

void Imu::configure(const TopicInfo &, const Thresholds &) {}

void Imu::on_message(const rclcpp::SerializedMessage & message, double)
{
  if (!decode(message)) {
    return;
  }
  ++count_;
  const auto & a = message_.linear_acceleration;
  const auto & g = message_.angular_velocity;
  const auto & q = message_.orientation;
  if (!finite3(a.x, a.y, a.z) || !finite3(g.x, g.y, g.z) || !finite3(q.x, q.y, q.z) ||
    !std::isfinite(q.w))
  {
    ++nonfinite_;
    return;
  }
  if (a.x == 0 && a.y == 0 && a.z == 0 && g.x == 0 && g.y == 0 && g.z == 0) {
    ++zero_;
    return;
  }
  ++valid_;
  acceleration_sum_ += std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
}

bool Imu::evaluate(const Context &, Result & result)
{
  if (count_ == 0 && bad_decode_ == 0) {
    return false;
  }
  report_decode(result);
  if (nonfinite_ > 0) {
    result.issues.push_back("NaN/Inf values in " + std::to_string(nonfinite_) + " messages");
  }
  if (count_ > 0 && zero_ == count_) {
    result.issues.push_back("acceleration and angular velocity all zero");
  } else if (zero_ > 0) {
    result.issues.push_back("all-zero values in " + std::to_string(zero_) + " messages");
  }
  if (valid_ > 0) {
    result.values.push_back("|a| " + fixed(acceleration_sum_ / valid_, 2) + " m/s^2");
  }
  count_ = valid_ = nonfinite_ = zero_ = 0;
  acceleration_sum_ = 0.0;
  return true;
}

bool NavSatFix::accepts(const TopicInfo & topic) const
{
  return topic.type == "sensor_msgs/msg/NavSatFix";
}

void NavSatFix::configure(const TopicInfo &, const Thresholds & thresholds)
{
  max_std_ = threshold(thresholds, "gnss_max_std_m", max_std_);
}

void NavSatFix::on_message(const rclcpp::SerializedMessage & message, double)
{
  if (!decode(message)) {
    return;
  }
  ++count_;
  status_ = message_.status.status;
  covariance_type_ = message_.position_covariance_type;
  const double cx = message_.position_covariance[0];
  const double cy = message_.position_covariance[4];
  if (!std::isfinite(cx) || !std::isfinite(cy) || cx < 0 || cy < 0) {
    ++bad_covariance_;
    return;
  }
  std_m_ = std::sqrt(std::max(cx, cy));
}

bool NavSatFix::evaluate(const Context &, Result & result)
{
  if (count_ == 0 && bad_decode_ == 0) {
    return false;
  }
  report_decode(result);
  if (bad_covariance_ > 0) {
    result.issues.push_back(
      "covariance NaN or negative in " + std::to_string(bad_covariance_) + " messages");
  } else if (covariance_type_ > 0 && std_m_ > max_std_) {
    result.issues.push_back("low accuracy +-" + fixed(std_m_, 1) + " m");
  }
  static const char * const kStatus[] = {"FIX", "SBAS", "GBAS"};
  result.values.push_back(
    status_ < 0 ? "NO_FIX" : status_ <= 2 ? kStatus[status_] : "status " +
    std::to_string(status_));
  result.values.push_back(covariance_type_ > 0 ? "+-" + fixed(std_m_, 2) + " m" : "no covariance");
  count_ = bad_covariance_ = 0;
  return true;
}

bool TfMessage::accepts(const TopicInfo & topic) const
{
  return topic.type == "tf2_msgs/msg/TFMessage";
}

void TfMessage::configure(const TopicInfo &, const Thresholds & thresholds)
{
  quaternion_tolerance_ = threshold(thresholds, "tf_quaternion_tolerance", quaternion_tolerance_);
}

void TfMessage::on_message(const rclcpp::SerializedMessage & message, double)
{
  if (!decode(message)) {
    return;
  }
  ++count_;
  for (const auto & t : message_.transforms) {
    const auto & p = t.transform.translation;
    const auto & q = t.transform.rotation;
    const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    if (!finite3(p.x, p.y, p.z) || !std::isfinite(norm) ||
      std::fabs(norm - 1.0) > quaternion_tolerance_)
    {
      bad_.insert(t.header.frame_id + "->" + t.child_frame_id);
    }
    frames_.insert(t.child_frame_id);
  }
}

bool TfMessage::evaluate(const Context &, Result & result)
{
  if (count_ == 0 && bad_decode_ == 0) {
    return false;
  }
  report_decode(result);
  if (!bad_.empty()) {
    std::string list;
    std::size_t i = 0;
    for (const auto & b : bad_) {
      if (i++ == 3) {
        list += " and " + std::to_string(bad_.size() - 3) + " more";
        break;
      }
      list += (i > 1 ? ", " : "") + b;
    }
    result.issues.push_back("invalid transform (NaN or quaternion norm): " + list);
  }
  result.values.push_back(std::to_string(frames_.size()) + " frames");
  count_ = 0;
  bad_.clear();
  return true;
}

bool PointCloud2::accepts(const TopicInfo & topic) const
{
  return topic.type == "sensor_msgs/msg/PointCloud2";
}

void PointCloud2::configure(const TopicInfo &, const Thresholds &) {}

void PointCloud2::on_message(const rclcpp::SerializedMessage & message, double)
{
  const auto & raw = message.get_rcl_serialized_message();
  const auto body = after_header(raw.buffer, raw.buffer_length, 4);
  if (!body || kEncapsulation + *body + 8 > raw.buffer_length) {
    return;
  }
  const std::uint8_t * p = raw.buffer + kEncapsulation + *body;
  const std::uint64_t points = static_cast<std::uint64_t>(read_u32(p)) * read_u32(p + 4);
  ++count_;
  points_ += points;
  if (points == 0) {
    ++empty_;
  }
}

bool PointCloud2::evaluate(const Context &, Result & result)
{
  if (count_ == 0) {
    return false;
  }
  if (empty_ == count_) {
    result.issues.push_back("all point clouds empty");
  } else if (empty_ > 0) {
    result.issues.push_back("empty point cloud in " + std::to_string(empty_) + " messages");
  }
  result.values.push_back(std::to_string(points_ / count_) + " points");
  count_ = empty_ = points_ = 0;
  return true;
}

}  // namespace rosbag_recorder_checks

PLUGINLIB_EXPORT_CLASS(rosbag_recorder_checks::HeaderStamp, rosbag_recorder_checks::Check)
PLUGINLIB_EXPORT_CLASS(rosbag_recorder_checks::Imu, rosbag_recorder_checks::Check)
PLUGINLIB_EXPORT_CLASS(rosbag_recorder_checks::NavSatFix, rosbag_recorder_checks::Check)
PLUGINLIB_EXPORT_CLASS(rosbag_recorder_checks::TfMessage, rosbag_recorder_checks::Check)
PLUGINLIB_EXPORT_CLASS(rosbag_recorder_checks::PointCloud2, rosbag_recorder_checks::Check)
