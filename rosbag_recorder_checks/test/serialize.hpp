// Copyright 2026 pgw
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef SERIALIZE_HPP_
#define SERIALIZE_HPP_

#include <cstdint>

#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"

template<class MessageT>
rclcpp::SerializedMessage serialize(const MessageT & message)
{
  rclcpp::SerializedMessage out;
  rclcpp::Serialization<MessageT>().serialize_message(&message, &out);
  return out;
}

template<class MessageT>
MessageT stamped(std::int32_t sec, std::uint32_t nanosec)
{
  MessageT message;
  message.header.stamp.sec = sec;
  message.header.stamp.nanosec = nanosec;
  message.header.frame_id = "base_link";
  return message;
}

inline double seconds(std::int32_t sec, std::uint32_t nanosec)
{
  return sec + nanosec * 1e-9;
}

#endif  // SERIALIZE_HPP_
