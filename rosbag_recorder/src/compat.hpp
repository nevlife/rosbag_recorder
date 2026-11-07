#ifndef COMPAT_HPP_
#define COMPAT_HPP_

#include "rclcpp/qos.hpp"
#include "rclcpp/version.h"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_storage/storage_options.hpp"
#include "rosbag2_storage/topic_metadata.hpp"

#include <map>
#include <string>
#include <vector>

// ROSBAG_RECORDER_ROSBAG2_MINOR comes from CMake: 15 on Humble, 26 on Jazzy.
#if ROSBAG_RECORDER_ROSBAG2_MINOR >= 22
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

inline auto service_qos()
{
#if RCLCPP_VERSION_GTE(21, 0, 0)
  return rclcpp::ServicesQoS();
#else
  return rmw_qos_profile_services_default;
#endif
}

// Humble's Writer cannot split on request; its SequentialWriter keeps split_bagfile protected.
inline bool split(rosbag2_cpp::Writer & writer)
{
#if ROSBAG_RECORDER_ROSBAG2_MINOR >= 22
  writer.split_bagfile();
  return true;
#else
  (void)writer;
  return false;
#endif
}

}  // namespace rosbag_recorder

#endif  // COMPAT_HPP_
