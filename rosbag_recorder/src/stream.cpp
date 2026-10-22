// Copyright 2026 pgw
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "rosbag_recorder/stream.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace rosbag_recorder
{
namespace
{

constexpr std::size_t kArrivals = 4000;
constexpr std::size_t kSizes = 50;
constexpr double kIssueHold = 3.0;
constexpr std::size_t kLatched = 100;

double window_for(std::optional<double> expected)
{
  return expected ? std::max(3.0, 4.0 / *expected) : 5.0;
}

}  // namespace

Rate evaluate_rate(
  std::optional<double> expected, const std::deque<double> & arrivals, std::uint64_t count,
  double now, double tolerance, std::uint32_t publishers, bool latched, bool type_error)
{
  Rate r;
  if (!arrivals.empty()) {
    r.age = now - arrivals.back();
    const auto first = std::lower_bound(arrivals.begin(), arrivals.end(),
        now - window_for(expected));
    const auto n = std::distance(first, arrivals.end());
    if (n >= 2 && arrivals.back() > *first) {
      r.hz = static_cast<double>(n - 1) / (arrivals.back() - *first);
    }
  }
  if (type_error) {
    r.status = TopicHealth::TYPE_UNAVAILABLE;
  } else if (latched) {
    r.status = count > 0 || publishers > 0 ? TopicHealth::OK : TopicHealth::LATCHED_MISSING;
  } else if (!r.age) {
    r.status = publishers > 0 ? TopicHealth::WAITING : TopicHealth::NO_PUBLISHER;
  } else if (*r.age > (expected ? std::max(2.0, 3.0 / *expected) : 5.0)) {
    r.status = publishers > 0 ? TopicHealth::STALE : TopicHealth::NO_PUBLISHER;
  } else if (expected && r.hz < *expected * tolerance) {
    r.status = TopicHealth::LOW;
  } else {
    r.status = TopicHealth::OK;
  }
  return r;
}

Stream::Stream(StreamSpec s)
: spec(std::move(s)) {}

void Stream::set_checks(std::vector<std::shared_ptr<rosbag_recorder_checks::Check>> checks)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  checks_ = std::move(checks);
}

void Stream::set_auto_marker(const AutoMarker & marker)
{
  auto info = std::make_unique<rosbag_recorder_checks::TypeInfo>(spec.type);
  const std::lock_guard<std::mutex> lock(mutex_);
  auto_kind = marker.kind;
  auto_marker_ = marker;
  type_info_ = std::move(info);
}

void Stream::collect_frames()
{
  const std::lock_guard<std::mutex> lock(mutex_);
  frames_on_ = true;
}

std::optional<std::string> Stream::receive(
  const rclcpp::SerializedMessage & message, double mono, double wall)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  arrivals_.push_back(mono);
  if (arrivals_.size() > kArrivals) {
    arrivals_.pop_front();
  }
  sizes_.push_back(message.size());
  if (sizes_.size() > kSizes) {
    sizes_.pop_front();
  }
  ++received_;
  for (const auto & check : checks_) {
    check->on_message(message, wall);
  }
  if (frames_on_) {
    tf2_msgs::msg::TFMessage tf;
    tf_serialization_.deserialize_message(&message, &tf);
    for (const auto & t : tf.transforms) {
      frames_.insert(t.header.frame_id);
      frames_.insert(t.child_frame_id);
    }
  }
  if (!type_info_) {
    return std::nullopt;
  }
  std::string value;
  try {
    value = type_info_->text(message, auto_marker_->field);
  } catch (const std::invalid_argument & e) {
    type_info_.reset();
    auto_marker_error_ = std::string("auto marker off: ") + e.what();
    return std::nullopt;
  }
  std::optional<std::string> change;
  if (last_value_ && *last_value_ != value) {
    const std::string label = auto_marker_->label.empty() ? spec.name : auto_marker_->label;
    change = label + ": " + *last_value_ + " -> " + value;
  }
  last_value_ = value;
  return change;
}

TopicHealth Stream::report(double mono, double wall, double tolerance, bool sim_time)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  const Rate rate = evaluate_rate(
    spec.hz, arrivals_, received_, mono, tolerance, publishers, spec.latched, !error.empty());
  TopicHealth h;
  h.name = spec.name;
  h.type = spec.type;
  h.source = spec.source;
  h.status = rate.status;
  h.required = spec.required;
  h.latched = spec.latched;
  h.expected_hz = spec.hz.value_or(0.0);
  h.hz = std::round(rate.hz * 100) / 100;
  h.age_s = rate.age ? std::round(*rate.age * 100) / 100 : std::numeric_limits<double>::quiet_NaN();
  double bytes = 0;
  for (const auto s : sizes_) {
    bytes += static_cast<double>(s);
  }
  h.size_kb = sizes_.empty() ? 0.0 : std::round(bytes / sizes_.size() / 1024.0 * 100) / 100;
  h.received = received_;
  h.written = written_;
  h.publishers = publishers;
  if (!checks_.empty()) {
    rosbag_recorder_checks::Result r;
    bool got = false;
    const rosbag_recorder_checks::Context context{wall, sim_time, publishers};
    for (const auto & check : checks_) {
      got = check->evaluate(context, r) || got;
    }
    if (got) {
      result_ = std::move(r);
      last_result_ = wall;
    } else if (wall - last_result_ > kIssueHold) {
      result_ = {};
    }
    if (!result_.issues.empty()) {
      held_issues = result_.issues;
      held_until_ = wall + kIssueHold;
    } else if (wall > held_until_) {
      held_issues.clear();
    }
    h.values = result_.values;
    if (h.status == TopicHealth::OK) {
      h.issues = held_issues;
      if (!held_issues.empty()) {
        h.status = TopicHealth::ABNORMAL;
      }
    }
  }
  if (!error.empty()) {
    h.issues.push_back(error);
  }
  if (!auto_marker_error_.empty()) {
    h.values.push_back(auto_marker_error_);
  }
  return h;
}

void Stream::keep_latched(std::shared_ptr<rclcpp::SerializedMessage> message)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  latched_.push_back(std::move(message));
  if (latched_.size() > kLatched) {
    latched_.pop_front();
  }
}

std::vector<std::shared_ptr<rclcpp::SerializedMessage>> Stream::latched() const
{
  const std::lock_guard<std::mutex> lock(mutex_);
  return {latched_.begin(), latched_.end()};
}

std::set<std::string> Stream::frames() const
{
  const std::lock_guard<std::mutex> lock(mutex_);
  return frames_;
}

std::uint64_t Stream::received() const
{
  const std::lock_guard<std::mutex> lock(mutex_);
  return received_;
}

void Stream::reset_counts()
{
  written_ = 0;
}

}  // namespace rosbag_recorder
