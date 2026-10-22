// Copyright 2026 pgw
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef COMPAT_HPP_
#define COMPAT_HPP_

#include <map>
#include <string>
#include <vector>

#include "rclcpp/qos.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_storage/storage_options.hpp"
#include "rosbag2_storage/topic_metadata.hpp"

// ROSBAG_RECORDER_ROSBAG2_MINOR comes from CMake: 15 on Humble, 26 on Jazzy.
#if ROSBAG_RECORDER_ROSBAG2_MINOR >= 22
#include "rosbag2_storage/message_definition.hpp"
#include "rosbag2_storage/qos.hpp"
#else
#include "rosbag2_transport/qos.hpp"
#include "yaml-cpp/yaml.h"
#endif

namespace rosbag_recorder
{

#if ROSBAG_RECORDER_ROSBAG2_MINOR >= 22
using Rosbag2QoS = rosbag2_storage::Rosbag2QoS;
#else
using Rosbag2QoS = rosbag2_transport::Rosbag2QoS;
#endif

inline void set_offered_qos(
  rosbag2_storage::TopicMetadata & topic, const std::vector<rclcpp::QoS> & offers)
{
#if ROSBAG_RECORDER_ROSBAG2_MINOR >= 22
  topic.offered_qos_profiles = offers;
#else
  YAML::Node list;
  for (const auto & qos : offers) {
    list.push_back(Rosbag2QoS(qos));
  }
  topic.offered_qos_profiles = YAML::Dump(list);
#endif
}

inline void set_custom_data(
  rosbag2_storage::StorageOptions & options, const std::map<std::string, std::string> & data)
{
#if ROSBAG_RECORDER_ROSBAG2_MINOR >= 22
  options.custom_data.insert(data.begin(), data.end());
#else
  (void)options;
  (void)data;
#endif
}

// A source's own schema goes into the bag where rosbag2 takes one; elsewhere the storage
// plugin looks the type up.
inline void create_topic(
  rosbag2_cpp::Writer & writer, const rosbag2_storage::TopicMetadata & topic,
  const std::string & definition, const std::string & encoding)
{
#if ROSBAG_RECORDER_ROSBAG2_MINOR >= 22
  if (!definition.empty()) {
    writer.create_topic(topic, {topic.type, encoding, definition, ""});
    return;
  }
#else
  (void)definition;
  (void)encoding;
#endif
  writer.create_topic(topic);
}

}  // namespace rosbag_recorder

#endif  // COMPAT_HPP_
