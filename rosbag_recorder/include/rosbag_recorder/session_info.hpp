#ifndef ROSBAG_RECORDER__SESSION_INFO_HPP_
#define ROSBAG_RECORDER__SESSION_INFO_HPP_

#include <cstdint>
#include <ctime>
#include <optional>
#include <string>
#include <vector>

#include "rosbag_recorder_msgs/msg/file_snapshot.hpp"

namespace rosbag_recorder
{

std::optional<double> disk_free_gb(const std::string & path);
std::uint64_t directory_bytes(const std::string & path);
bool writable(const std::string & directory);
std::string expand_user(const std::string & path);

std::string session_id(const std::string & profile, const std::string & label, std::time_t now);

std::string host_name();
std::string user_name();
std::string env(const char * name);

std::string git_snapshot(const std::string & workspace);

bool glob_match(const std::string & pattern, const std::string & path);

std::vector<rosbag_recorder_msgs::msg::FileSnapshot> snapshot_files(
  const std::string & workspace, const std::vector<std::string> & globs, std::uint64_t max_bytes);

std::string sha256(const std::string & data);

}  // namespace rosbag_recorder

#endif  // ROSBAG_RECORDER__SESSION_INFO_HPP_
