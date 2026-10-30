#ifndef ROSBAG_RECORDER__CONFIG_HPP_
#define ROSBAG_RECORDER__CONFIG_HPP_

#include <cstdint>
#include <map>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include "yaml-cpp/yaml.h"

namespace rosbag_recorder
{

class ConfigError : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

struct TopicSpec
{
  std::string name;
  std::string group;
  std::optional<std::string> type;
  std::optional<double> hz;
  bool required{false};
  bool heavy{false};
  std::optional<double> size_kb;
  // Read for compatibility; every recorded topic is measured on its own subscription.
  std::optional<std::string> monitor_via;
  bool latched{false};
  std::string description;
};

struct Group
{
  std::string name;
  std::string description;
  std::vector<std::string> topics;
};

struct AutoMarker
{
  std::string topic;
  std::string label;
  std::string field;
  std::string kind{"auto"};
};

struct SourceSpec
{
  std::string name;
  std::string plugin;
  std::string description;
  std::string params_yaml;
  std::map<std::string, TopicSpec> channels;
};

struct Catalog
{
  static Catalog load(const std::string & path);
  static Catalog parse(const YAML::Node & root, const std::string & where);

  const TopicSpec * topic(const std::string & name) const;
  const Group & group(const std::string & name) const;
  std::vector<std::string> sort_topics(const std::set<std::string> & names) const;

  int version{1};
  double heavy_threshold_kb{500.0};
  std::vector<Group> groups;
  std::map<std::string, TopicSpec> topics;
  std::vector<AutoMarker> auto_markers;
  std::map<std::string, double> value_checks;
  std::map<std::string, SourceSpec> sources;
};

struct RecordOptions
{
  std::string storage{"mcap"};
  std::string compression{"none"};
  std::int64_t split_duration_sec{300};
  std::int64_t split_size_mb{0};
  std::string heavy_qos{"keep"};
  std::string start_mode{"wait_ready"};
  std::int64_t wait_ready_sec{30};
  std::int64_t start_delay_sec{5};
  double disk_warn_gb{30.0};
  double disk_stop_gb{10.0};
  std::int64_t max_cache_mb{100};
  std::string storage_preset;
  std::string storage_config_uri;
  std::string on_not_ready{"abort"};
  std::vector<std::string> required_frames;

  static RecordOptions parse(
    const std::map<std::string, YAML::Node> & values,
    const std::string & where);
  YAML::Node to_yaml() const;
  // MCAP preset for the compression unless storage_preset names one.
  std::string preset() const;
};

struct Profile
{
  static Profile parse(const YAML::Node & root, const std::string & where);

  std::string name;
  std::string description;
  std::string extends;
  bool abstract{false};
  std::optional<std::vector<std::string>> groups;
  std::vector<std::string> add_groups;
  std::vector<std::string> exclude_groups;
  std::vector<std::string> topics;
  std::vector<std::string> exclude;
  std::vector<std::string> sources;
  std::vector<std::string> exclude_sources;
  std::map<std::string, YAML::Node> options;
};

struct ResolvedProfile
{
  std::string name;
  std::string description;
  std::vector<std::string> chain;
  std::vector<std::string> topics;
  std::vector<std::string> unknown_topics;
  std::vector<std::string> sources;
  RecordOptions options;
  bool abstract{false};

  std::string to_yaml() const;
};

class ProfileStore
{
public:
  explicit ProfileStore(std::string directory);

  std::vector<std::string> names() const;
  Profile load(const std::string & name) const;
  ResolvedProfile resolve(const std::string & name, const Catalog & catalog) const;

private:
  ResolvedProfile resolve(
    const Profile & profile, const Catalog & catalog,
    std::vector<std::string> & stack) const;

  std::string directory_;
};

}  // namespace rosbag_recorder

#endif  // ROSBAG_RECORDER__CONFIG_HPP_
