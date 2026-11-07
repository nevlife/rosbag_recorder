#ifndef START_GATE_HPP_
#define START_GATE_HPP_

#include "config.hpp"

#include "rosbag_recorder_msgs/msg/check_result.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace rosbag_recorder
{

// When a requested session begins: at once, after start_delay_sec, or once the required topics
// are ready (start_mode wait_ready).
class StartGate
{
public:
  enum class Step { Wait, Abort, Begin };

  // Throws ConfigError when the profile cannot record. False: wait, as message() says.
  bool request(
    const ResolvedProfile & profile, std::size_t streams,
    const std::vector<rosbag_recorder_msgs::msg::CheckResult> & preflight, bool force, double now);
  // Abort and Begin set message(): why it gave up, or a note for a marker (may be empty).
  Step poll(const RecordOptions & options, double now, const std::vector<std::string> & missing);
  const std::string & message() const { return message_; }

private:
  double deadline_{0.0};
  std::string message_;
};

}  // namespace rosbag_recorder

#endif  // START_GATE_HPP_
