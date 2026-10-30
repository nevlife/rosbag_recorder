#ifndef ROSBAG_RECORDER_SOURCE__SOURCE_HPP_
#define ROSBAG_RECORDER_SOURCE__SOURCE_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/node.hpp"
#include "rclcpp/serialized_message.hpp"

namespace rosbag_recorder_source
{

struct Channel
{
  std::string name;
  std::string type;
  std::string serialization_format{"cdr"};
  // Schema text and its encoding (ros2msg, ros2idl). Empty: the recorder looks the type up.
  std::string definition;
  std::string definition_encoding;
  double hz{0.0};
  bool required{false};
  bool latched{false};
};

class Sink
{
public:
  virtual ~Sink() = default;

  // channel indexes Source::channels(); time_ns is on the clock of the node given to initialize().
  virtual void write(
    std::size_t channel, std::shared_ptr<rclcpp::SerializedMessage> message,
    std::int64_t time_ns) = 0;

  virtual void mark(
    const std::string & kind, const std::string & text,
    const std::string & topic) = 0;
};

class Source
{
public:
  virtual ~Source() = default;

  // params_yaml: the source's `params` mapping in the catalog, as YAML text.
  virtual void initialize(
    rclcpp::Node & node, const std::string & name,
    const std::string & params_yaml) = 0;

  virtual std::vector<Channel> channels() = 0;

  // The source may call the sink from any thread from start() until stop() returns.
  virtual void start(Sink & sink) = 0;
  virtual void stop() = 0;

  virtual std::uint32_t publishers(std::size_t channel) const
  {
    (void)channel;
    return 1;
  }
};

}  // namespace rosbag_recorder_source

#endif  // ROSBAG_RECORDER_SOURCE__SOURCE_HPP_
