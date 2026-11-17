#include "flux_input.hpp"

#include <chrono>
#include <cstddef>
#include <memory>
#include <string>
#include <utility>

namespace rosbag_recorder
{

FluxInput::FluxInput(rclcpp::Node & node, const std::string & topic, std::uint64_t fingerprint)
: clock_(node.get_clock()),
  logger_(node.get_logger()),
  topic_(topic),
  fingerprint_(fingerprint),
  encoder_(topic, fingerprint),
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
  flux_bag::FluxFrameFields fields;
  if (const char * why = encoder_.encode(view, subscription_, stamp, fields)) {
    if (left_out_++ == 0) {
      RCLCPP_WARN(
        logger_, "%s: a frame with fields in its publisher's arena is not recorded: %s",
        topic_.c_str(), why);
    }
    return;
  }
  const std::size_t n = flux_bag::FluxFrameCdr::size(fields);
  auto bytes = std::make_shared<rclcpp::SerializedMessage>(n);
  auto & out = bytes->get_rcl_serialized_message();
  out.buffer_length = flux_bag::FluxFrameCdr::write(fields, out.buffer);
  callback_(std::move(bytes), stamp);
}

}  // namespace rosbag_recorder
