#include "rosbag_recorder_checks/message.hpp"

#include <cstdint>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/serialization.hpp"
#include "rclcpp/typesupport_helpers.hpp"
#include "rclcpp/version.h"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"

namespace rosbag_recorder_checks
{
namespace
{

using rosidl_typesupport_introspection_cpp::MessageMember;
using rosidl_typesupport_introspection_cpp::MessageMembers;

const rosidl_message_type_support_t * handle(
  const std::string & type, const std::string & identifier, rcpputils::SharedLibrary & library)
{
#if RCLCPP_VERSION_GTE(28, 0, 0)
  return rclcpp::get_message_typesupport_handle(type, identifier, library);
#else
  return rclcpp::get_typesupport_handle(type, identifier, library);
#endif
}

const MessageMembers * nested(const MessageMember & member)
{
  return static_cast<const MessageMembers *>(member.members_->data);
}

constexpr std::size_t kArrayShown = 8;

std::string message_text(
  const MessageMembers & members, const std::uint8_t * data,
  bool skip_header);

template<class T>
std::string number(const void * p)
{
  std::ostringstream os;
  os << +*static_cast<const T *>(p);
  return os.str();
}

std::string scalar_text(const MessageMember & member, const void * p)
{
  namespace ft = rosidl_typesupport_introspection_cpp;
  switch (member.type_id_) {
    case ft::ROS_TYPE_FLOAT: return number<float>(p);
    case ft::ROS_TYPE_DOUBLE: return number<double>(p);
    case ft::ROS_TYPE_LONG_DOUBLE: return number<long double>(p);
    case ft::ROS_TYPE_CHAR: return number<char>(p);
    case ft::ROS_TYPE_BOOLEAN: return *static_cast<const bool *>(p) ? "true" : "false";
    case ft::ROS_TYPE_OCTET:
    case ft::ROS_TYPE_UINT8: return number<std::uint8_t>(p);
    case ft::ROS_TYPE_INT8: return number<std::int8_t>(p);
    case ft::ROS_TYPE_UINT16: return number<std::uint16_t>(p);
    case ft::ROS_TYPE_INT16: return number<std::int16_t>(p);
    case ft::ROS_TYPE_UINT32: return number<std::uint32_t>(p);
    case ft::ROS_TYPE_INT32: return number<std::int32_t>(p);
    case ft::ROS_TYPE_UINT64: return number<std::uint64_t>(p);
    case ft::ROS_TYPE_INT64: return number<std::int64_t>(p);
    case ft::ROS_TYPE_STRING: return *static_cast<const std::string *>(p);
    case ft::ROS_TYPE_MESSAGE:
      return message_text(*nested(member), static_cast<const std::uint8_t *>(p), false);
    default: return "?";
  }
}

std::string member_text(const MessageMember & member, const std::uint8_t * data)
{
  const void * field = data + member.offset_;
  if (!member.is_array_) {
    return scalar_text(member, field);
  }
  const std::size_t n = member.size_function(field);
  std::string out = "[";
  const bool by_reference =
    member.type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE ||
    member.type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING;
  for (std::size_t i = 0; i < n && i < kArrayShown; ++i) {
    alignas(long double) std::uint8_t value[sizeof(long double)];
    const void * element = value;
    if (by_reference) {
      element = member.get_const_function(field, i);
    } else {
      member.fetch_function(field, i, value);
    }
    out += (i ? ", " : "") + scalar_text(member, element);
  }
  return out + (n > kArrayShown ? ", ... (" + std::to_string(n) + ")]" : "]");
}

std::string message_text(
  const MessageMembers & members, const std::uint8_t * data,
  bool skip_header)
{
  std::string out = "{";
  bool first = true;
  for (std::uint32_t i = 0; i < members.member_count_; ++i) {
    const MessageMember & m = members.members_[i];
    if (skip_header && std::string(m.name_) == "header") {
      continue;
    }
    out += (first ? "" : ", ") + std::string(m.name_) + ": " + member_text(m, data);
    first = false;
  }
  return out + "}";
}

struct Buffer
{
  explicit Buffer(const MessageMembers & m)
  : members(m), storage(m.size_of_ / sizeof(std::max_align_t) + 1)
  {
    members.init_function(storage.data(), rosidl_runtime_cpp::MessageInitialization::ALL);
  }
  ~Buffer() {members.fini_function(storage.data());}
  Buffer(const Buffer &) = delete;
  Buffer & operator=(const Buffer &) = delete;

  const MessageMembers & members;
  std::vector<std::max_align_t> storage;
};

}  // namespace

TypeInfo::TypeInfo(const std::string & type)
: introspection_library_(
    rclcpp::get_typesupport_library(type, "rosidl_typesupport_introspection_cpp")),
  cpp_library_(rclcpp::get_typesupport_library(type, "rosidl_typesupport_cpp"))
{
  members_ = static_cast<const MessageMembers *>(
    handle(type, "rosidl_typesupport_introspection_cpp", *introspection_library_)->data);
  cpp_support_ = handle(type, "rosidl_typesupport_cpp", *cpp_library_);
}

bool TypeInfo::starts_with_header() const
{
  if (members_->member_count_ == 0) {
    return false;
  }
  const MessageMember & first = members_->members_[0];
  if (first.type_id_ != rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE || first.is_array_) {
    return false;
  }
  const MessageMembers * header = nested(first);
  return std::string(header->message_namespace_) == "std_msgs::msg" &&
         std::string(header->message_name_) == "Header";
}

std::string TypeInfo::text(
  const rclcpp::SerializedMessage & message, const std::string & field) const
{
  Buffer buffer(*members_);
  rclcpp::SerializationBase(cpp_support_).deserialize_message(&message, buffer.storage.data());
  const auto * data = reinterpret_cast<const std::uint8_t *>(buffer.storage.data());
  if (field.empty()) {
    return message_text(*members_, data, true);
  }
  const MessageMembers * members = members_;
  std::istringstream path(field);
  std::string part;
  const MessageMember * found = nullptr;
  while (std::getline(path, part, '.')) {
    if (found) {
      if (found->type_id_ != rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE ||
        found->is_array_)
      {
        throw std::invalid_argument("field " + field + ": " + found->name_ + " is not a message");
      }
      data += found->offset_;
      members = nested(*found);
      found = nullptr;
    }
    for (std::uint32_t i = 0; i < members->member_count_; ++i) {
      if (part == members->members_[i].name_) {
        found = &members->members_[i];
      }
    }
    if (!found) {
      throw std::invalid_argument("no field " + field);
    }
  }
  return member_text(*found, data);
}

}  // namespace rosbag_recorder_checks
