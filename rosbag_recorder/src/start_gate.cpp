#include "start_gate.hpp"

#include "text.hpp"

#include <string>
#include <vector>

namespace rosbag_recorder
{

bool StartGate::request(
  const ResolvedProfile & profile, std::size_t streams,
  const std::vector<rosbag_recorder_msgs::msg::CheckResult> & preflight, bool force, double now)
{
  if (streams == 0 && profile.name == "default") {
    throw ConfigError("select at least one topic");
  }
  if (profile.abstract) {
    throw ConfigError(profile.name + " is abstract");
  }
  for (const auto & c : preflight) {
    if (c.level == rosbag_recorder_msgs::msg::CheckResult::ERROR) {
      throw ConfigError("preflight: " + c.text);
    }
  }
  const auto & o = profile.options;
  message_.clear();
  if (force || o.start_mode == "immediate") {
    return true;
  }
  const bool ready = o.start_mode == "wait_ready";
  deadline_ = now + static_cast<double>(ready ? o.wait_ready_sec : o.start_delay_sec);
  message_ = ready ? "waiting up to " + std::to_string(o.wait_ready_sec) + " s for required topics"
                   : "starting in " + std::to_string(o.start_delay_sec) + " s";
  return false;
}

StartGate::Step StartGate::poll(
  const RecordOptions & options, double now, const std::vector<std::string> & missing)
{
  const bool due = now >= deadline_;
  message_.clear();
  if (options.start_mode == "delay" ? !due : (!missing.empty() && !due)) {
    return Step::Wait;
  }
  if (options.start_mode == "wait_ready" && !missing.empty()) {
    if (options.on_not_ready == "abort") {
      message_ = "required topics not ready after " + std::to_string(options.wait_ready_sec) +
                 " s: " + listed(missing);
      return Step::Abort;
    }
    message_ = "started without required topics: " + listed(missing);
  }
  return Step::Begin;
}

}  // namespace rosbag_recorder
