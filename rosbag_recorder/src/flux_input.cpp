#include "flux_input.hpp"

#include <chrono>
#include <cstring>
#include <memory>
#include <string>
#include <utility>

namespace rosbag_recorder
{

FluxInput::FluxInput(rclcpp::Node & node, const std::string & topic, std::uint64_t fingerprint)
: clock_(node.get_clock()),
  topic_(topic),
  fingerprint_(fingerprint),
  subscription_(
    node, topic, fingerprint, flux::QoS{}, [this](const flux::FrameView & view) { on_frame(view); })
{
  subscription_.add_to_callback_group(node);
}

void FluxInput::on_frame(const flux::FrameView & view)
{
  if (!callback_) {
    return;
  }
  const auto mono = std::chrono::duration_cast<std::chrono::nanoseconds>(
                      std::chrono::steady_clock::now().time_since_epoch())
                      .count();
  const auto stamp =
    clock_->now().nanoseconds() - (mono - static_cast<std::int64_t>(view.publish_ns()));
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
  auto bytes = std::make_shared<rclcpp::SerializedMessage>();
  serialization_.serialize_message(&frame, bytes.get());
  callback_(std::move(bytes), stamp);
}

}  // namespace rosbag_recorder
