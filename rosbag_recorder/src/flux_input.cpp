#include "flux_input.hpp"

#include <chrono>
#include <cstring>
#include <utility>

#include "flux_msgs/msg/flux_frame.hpp"
#include "rclcpp/serialization.hpp"

namespace rosbag_recorder
{

FluxInput::FluxInput(
  rclcpp::Node & node, const std::string & topic, std::uint64_t fingerprint, Callback cb)
: clock_(node.get_clock()), topic_(topic), fingerprint_(fingerprint), callback_(std::move(cb)),
  subscription_(node, topic, fingerprint), thread_([this] {run();}) {}

FluxInput::~FluxInput()
{
  running_ = false;
  thread_.join();
}

std::string FluxInput::error() const
{
  const std::lock_guard<std::mutex> lock(mutex_);
  return error_;
}

void FluxInput::run()
{
  try {
    rclcpp::Serialization<flux_msgs::msg::FluxFrame> serialization;
    while (running_) {
      auto view = subscription_.take_blocking(100'000'000);
      if (!view) {
        continue;
      }
      const auto mono = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
      const auto stamp = clock_->now().nanoseconds() -
        (mono - static_cast<std::int64_t>(view.publish_ns()));
      flux_msgs::msg::FluxFrame frame;
      frame.header.stamp = rclcpp::Time(stamp);
      frame.flux_topic = topic_;
      frame.fingerprint = fingerprint_;
      frame.codec = "raw";
      const auto * channel = subscription_.channel();
      const std::uint32_t slots[] = {channel->slot_size(), channel->slot_count()};
      const auto & meta = view.meta();
      frame.meta.resize(10 + 8 * meta.ndim);
      std::memcpy(frame.meta.data(), slots, sizeof(slots));
      frame.meta[8] = meta.dtype;
      frame.meta[9] = meta.ndim;
      std::memcpy(frame.meta.data() + 10, meta.shape, 8 * meta.ndim);
      const auto * data = static_cast<const std::uint8_t *>(view.data());
      frame.data.assign(data, data + view.size());
      view.release();
      auto bytes = std::make_shared<rclcpp::SerializedMessage>();
      serialization.serialize_message(&frame, bytes.get());
      callback_(std::move(bytes), stamp);
    }
  } catch (const std::exception & e) {
    const std::lock_guard<std::mutex> lock(mutex_);
    error_ = e.what();
  }
}

}  // namespace rosbag_recorder
