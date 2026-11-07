#ifndef FLUX_INPUT_HPP_
#define FLUX_INPUT_HPP_

#include "flux/subscription.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"

#include "flux_msgs/msg/flux_frame.hpp"

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <utility>

namespace rosbag_recorder
{

// A Flux subscription in the node's default callback group that hands each frame on as a
// serialized flux_msgs/msg/FluxFrame. Keep it as long as the node: a flux executor has no way
// to drop a source, so destroying it while the node spins leaves the executor a dangling one.
class FluxInput
{
public:
  using Callback = std::function<void(std::shared_ptr<rclcpp::SerializedMessage>, std::int64_t)>;

  FluxInput(rclcpp::Node & node, const std::string & topic, std::uint64_t fingerprint);
  // Frames are dropped while the callback is empty.
  void set_callback(Callback cb) { callback_ = std::move(cb); }
  std::uint64_t fingerprint() const { return fingerprint_; }

private:
  void on_frame(const flux::FrameView & view);

  rclcpp::Clock::SharedPtr clock_;
  std::string topic_;
  std::uint64_t fingerprint_;
  Callback callback_;
  rclcpp::Serialization<flux_msgs::msg::FluxFrame> serialization_;
  flux::Subscription subscription_;
};

}  // namespace rosbag_recorder

#endif  // FLUX_INPUT_HPP_
