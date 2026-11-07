#include "stream.hpp"

#include <algorithm>
#include <cmath>
#include <deque>
#include <iterator>
#include <limits>
#include <memory>
#include <string>
#include <utility>

namespace rosbag_recorder
{
namespace
{

constexpr std::size_t size_window = 50;
constexpr std::size_t latched_kept = 100;

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
    const auto first =
      std::lower_bound(arrivals.begin(), arrivals.end(), now - window_for(expected));
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

Stream::Stream(StreamSpec s) : spec(std::move(s))
{
}

void Stream::receive(const rclcpp::SerializedMessage & message, double mono)
{
  arrivals_.push_back(mono);
  const double keep = window_for(spec.hz) + 1.0;
  while (arrivals_.front() < mono - keep) {
    arrivals_.pop_front();
  }
  sizes_.push_back(message.size());
  if (sizes_.size() > size_window) {
    sizes_.pop_front();
  }
  ++received_;
  if (frames_on_) {
    tf2_msgs::msg::TFMessage tf;
    tf_serialization_.deserialize_message(&message, &tf);
    for (const auto & t : tf.transforms) {
      frames_.insert(t.header.frame_id);
      frames_.insert(t.child_frame_id);
    }
  }
}

TopicHealth Stream::report(double mono, double tolerance) const
{
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
  if (!error.empty()) {
    h.issues.push_back(error);
  }
  return h;
}

void Stream::keep_latched(std::shared_ptr<rclcpp::SerializedMessage> message)
{
  latched_.push_back(std::move(message));
  if (latched_.size() > latched_kept) {
    latched_.pop_front();
  }
}

}  // namespace rosbag_recorder
