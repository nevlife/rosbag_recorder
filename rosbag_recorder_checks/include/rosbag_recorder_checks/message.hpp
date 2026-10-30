#ifndef ROSBAG_RECORDER_CHECKS__MESSAGE_HPP_
#define ROSBAG_RECORDER_CHECKS__MESSAGE_HPP_

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <optional>
#include <string>

#include "rclcpp/serialized_message.hpp"
#include "rcpputils/shared_library.hpp"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"

namespace rosbag_recorder_checks
{

struct Stamp
{
  std::int32_t sec{0};
  std::uint32_t nanosec{0};

  std::int64_t nanoseconds() const
  {
    return static_cast<std::int64_t>(sec) * 1000000000LL + nanosec;
  }
};

// Little-endian CDR: 4-byte encapsulation, then the body; field offsets are relative to the body.
constexpr std::size_t kEncapsulation = 4;

inline bool little_endian_cdr(const std::uint8_t * data, std::size_t length)
{
  return length >= kEncapsulation && data[0] == 0x00 && data[1] == 0x01;
}

inline std::uint32_t read_u32(const std::uint8_t * data)
{
  std::uint32_t v = 0;
  std::memcpy(&v, data, sizeof(v));
  return v;
}

// header.stamp of a message whose first field is std_msgs/Header.
inline std::optional<Stamp> read_stamp(const std::uint8_t * data, std::size_t length)
{
  if (!little_endian_cdr(data, length) || length < kEncapsulation + 8) {
    return std::nullopt;
  }
  Stamp stamp;
  std::memcpy(&stamp.sec, data + kEncapsulation, sizeof(stamp.sec));
  stamp.nanosec = read_u32(data + kEncapsulation + 4);
  if (stamp.nanosec >= 1000000000U) {
    return std::nullopt;
  }
  return stamp;
}

// Body offset of the field after std_msgs/Header, aligned to `alignment`.
inline std::optional<std::size_t> after_header(
  const std::uint8_t * data, std::size_t length, std::size_t alignment)
{
  if (!little_endian_cdr(data, length) || length < kEncapsulation + 12) {
    return std::nullopt;
  }
  const std::size_t end = 12 + read_u32(data + kEncapsulation + 8);
  const std::size_t aligned = (end + alignment - 1) / alignment * alignment;
  if (kEncapsulation + aligned > length) {
    return std::nullopt;
  }
  return aligned;
}

// A message type known only by name (pkg/msg/Type), through its introspection type support.
class TypeInfo
{
public:
  explicit TypeInfo(const std::string & type);

  bool starts_with_header() const;

  // `field` (a.b.c) as text; empty: the whole message but its header.
  std::string text(const rclcpp::SerializedMessage & message, const std::string & field) const;

private:
  std::shared_ptr<rcpputils::SharedLibrary> introspection_library_;
  std::shared_ptr<rcpputils::SharedLibrary> cpp_library_;
  const rosidl_typesupport_introspection_cpp::MessageMembers * members_{nullptr};
  const rosidl_message_type_support_t * cpp_support_{nullptr};
};

}  // namespace rosbag_recorder_checks

#endif  // ROSBAG_RECORDER_CHECKS__MESSAGE_HPP_
