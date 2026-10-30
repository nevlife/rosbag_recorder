#ifndef FLUX_INPUT_HPP_
#define FLUX_INPUT_HPP_

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "flux/subscription.hpp"
#include "rclcpp/rclcpp.hpp"

namespace rosbag_recorder
{

class FluxInput
{
public:
  using Callback = std::function<void(std::shared_ptr<rclcpp::SerializedMessage>, std::int64_t)>;

  FluxInput(rclcpp::Node & node, const std::string & topic, std::uint64_t fingerprint, Callback cb);
  ~FluxInput();
  std::string error() const;
  std::uint64_t fingerprint() const {return fingerprint_;}

private:
  void run();

  rclcpp::Clock::SharedPtr clock_;
  std::string topic_;
  std::uint64_t fingerprint_;
  Callback callback_;
  flux::Subscription subscription_;
  mutable std::mutex mutex_;
  std::string error_;
  std::atomic<bool> running_{true};
  std::thread thread_;
};

}  // namespace rosbag_recorder

#endif  // FLUX_INPUT_HPP_
