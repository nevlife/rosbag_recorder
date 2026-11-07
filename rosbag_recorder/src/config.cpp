#include "config.hpp"

#include <algorithm>
#include <filesystem>
#include <map>
#include <regex>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace rosbag_recorder
{
namespace
{

namespace fs = std::filesystem;

const std::regex topic_pattern("^(/[A-Za-z_][A-Za-z0-9_]*)+$");
const std::regex group_pattern("^[a-z][a-z0-9_]*$");
const std::regex profile_pattern("^[a-z0-9][a-z0-9_]*$");
constexpr std::size_t max_depth = 8;

std::string join(const std::vector<std::string> & items, const std::string & separator)
{
  std::string out;
  for (const auto & item : items) {
    out += (out.empty() ? "" : separator) + item;
  }
  return out;
}

void check_keys(
  const YAML::Node & node, const std::set<std::string> & allowed, const std::string & where)
{
  std::vector<std::string> unknown;
  for (const auto & kv : node) {
    if (!allowed.count(kv.first.as<std::string>())) {
      unknown.push_back(kv.first.as<std::string>());
    }
  }
  if (!unknown.empty()) {
    throw ConfigError(where + ": unknown keys [" + join(unknown, ", ") + "]");
  }
}

void require_map(const YAML::Node & node, const std::string & where)
{
  if (!node.IsMap()) {
    throw ConfigError(where + ": must be a mapping");
  }
}

std::string topic_name(const YAML::Node & node, const std::string & where)
{
  const std::string name = node && node.IsScalar() ? node.as<std::string>() : "";
  if (!std::regex_match(name, topic_pattern)) {
    throw ConfigError(where + ": bad topic name '" + name + "' (e.g. /velodyne_points)");
  }
  return name;
}

bool is_bool(const YAML::Node & node)
{
  bool b;
  return node.IsScalar() && YAML::convert<bool>::decode(node, b);
}

bool is_number(const YAML::Node & node)
{
  double d;
  return node.IsScalar() && !is_bool(node) && node.Tag() != "!" &&
         YAML::convert<double>::decode(node, d);
}

bool is_integer(const YAML::Node & node)
{
  std::int64_t i;
  return is_number(node) && YAML::convert<std::int64_t>::decode(node, i);
}

std::optional<double> positive(const YAML::Node & node, const std::string & where)
{
  if (!node || node.IsNull()) {
    return std::nullopt;
  }
  if (!is_number(node) || node.as<double>() <= 0) {
    throw ConfigError(where + ": must be a positive number (got '" + YAML::Dump(node) + "')");
  }
  return node.as<double>();
}

bool flag(const YAML::Node & node, const std::string & where)
{
  if (!node || node.IsNull()) {
    return false;
  }
  if (!is_bool(node)) {
    throw ConfigError(where + ": must be true or false (got '" + YAML::Dump(node) + "')");
  }
  return node.as<bool>();
}

std::string text(const YAML::Node & node)
{
  return node && !node.IsNull() ? node.as<std::string>() : "";
}

std::vector<std::string> strings(const YAML::Node & node, const std::string & where)
{
  std::vector<std::string> out;
  if (!node || node.IsNull()) {
    return out;
  }
  if (!node.IsSequence()) {
    throw ConfigError(where + ": must be a list of strings");
  }
  for (const auto & item : node) {
    if (!item.IsScalar()) {
      throw ConfigError(where + ": must be a list of strings");
    }
    out.push_back(item.as<std::string>());
  }
  return out;
}

TopicSpec topic_spec(
  const std::string & name, const std::string & group, const YAML::Node & data,
  const std::string & where)
{
  TopicSpec t;
  t.name = name;
  t.group = group;
  if (!data || data.IsNull()) {
    return t;
  }
  require_map(data, where);
  check_keys(
    data, {"type", "hz", "required", "heavy", "latched", "description", "size_kb", "monitor_via"},
    where);
  if (data["type"] && !data["type"].IsNull()) {
    const std::string type = data["type"].as<std::string>();
    if (std::count(type.begin(), type.end(), '/') != 2) {
      throw ConfigError(where + ".type: must be pkg/msg/Type (got '" + type + "')");
    }
    t.type = type;
  }
  t.hz = positive(data["hz"], where + ".hz");
  t.required = flag(data["required"], where + ".required");
  t.heavy = flag(data["heavy"], where + ".heavy");
  t.latched = flag(data["latched"], where + ".latched");
  t.description = text(data["description"]);
  return t;
}

const std::map<std::string, std::vector<std::string>> choices_of{
  {"storage", {"mcap", "sqlite3"}},       {"compression", {"none", "zstd"}},
  {"heavy_qos", {"best_effort", "keep"}}, {"start_mode", {"wait_ready", "delay", "immediate"}},
  {"on_not_ready", {"abort", "start"}},
};

void read_choice(
  const std::map<std::string, YAML::Node> & values, const std::string & key, std::string & out,
  const std::string & where)
{
  const auto it = values.find(key);
  if (it == values.end()) {
    return;
  }
  const auto & choices = choices_of.at(key);
  const std::string v = it->second.IsScalar() ? it->second.as<std::string>() : "";
  if (std::find(choices.begin(), choices.end(), v) == choices.end()) {
    throw ConfigError(where + "." + key + ": '" + v + "' (choices: " + join(choices, ", ") + ")");
  }
  out = v;
}

void read_count(
  const std::map<std::string, YAML::Node> & values, const std::string & key, std::int64_t & out,
  const std::string & where)
{
  const auto it = values.find(key);
  if (it == values.end()) {
    return;
  }
  if (!is_integer(it->second) || it->second.as<std::int64_t>() < 0) {
    throw ConfigError(where + "." + key + ": must be an integer >= 0");
  }
  out = it->second.as<std::int64_t>();
}

void read_amount(
  const std::map<std::string, YAML::Node> & values, const std::string & key, double & out,
  const std::string & where)
{
  const auto it = values.find(key);
  if (it == values.end()) {
    return;
  }
  if (!is_number(it->second) || it->second.as<double>() < 0) {
    throw ConfigError(where + "." + key + ": must be a number >= 0");
  }
  out = it->second.as<double>();
}

const std::set<std::string> option_keys{
  "storage",      "compression",    "split_duration_sec", "split_size_mb", "heavy_qos",
  "start_mode",   "wait_ready_sec", "start_delay_sec",    "disk_warn_gb",  "disk_stop_gb",
  "max_cache_mb", "storage_preset", "storage_config_uri", "on_not_ready",  "required_frames"};

}  // namespace

Catalog Catalog::load(const std::string & path)
{
  if (!fs::exists(path)) {
    throw ConfigError("catalog not found: " + path);
  }
  try {
    return parse(YAML::LoadFile(path), fs::path(path).filename().string());
  } catch (const YAML::Exception & e) {
    throw ConfigError(path + ": " + e.what());
  }
}

Catalog Catalog::parse(const YAML::Node & root, const std::string & where)
{
  require_map(root, where);
  // SCV writes heavy_threshold_kb, auto_markers, value_checks, size_kb and monitor_via; this
  // recorder accepts them so an SCV folder loads unchanged, and does not use them.
  check_keys(
    root, {"version", "groups", "heavy_threshold_kb", "auto_markers", "value_checks"}, where);
  Catalog c;
  if (root["version"]) {
    c.version = root["version"].as<int>();
  }
  std::map<std::string, std::string> seen;
  if (const auto groups = root["groups"]; groups && !groups.IsNull()) {
    require_map(groups, where + ".groups");
    for (const auto & g : groups) {
      const std::string gname = g.first.as<std::string>();
      const std::string gwhere = where + ".groups." + gname;
      if (!std::regex_match(gname, group_pattern)) {
        throw ConfigError(gwhere + ": group names use lowercase letters, digits and _");
      }
      Group group{gname, "", {}};
      if (g.second && !g.second.IsNull()) {
        require_map(g.second, gwhere);
        check_keys(g.second, {"description", "topics"}, gwhere);
        group.description = text(g.second["description"]);
        if (const auto topics = g.second["topics"]; topics && !topics.IsNull()) {
          require_map(topics, gwhere + ".topics");
          for (const auto & t : topics) {
            const std::string twhere = gwhere + "." + t.first.as<std::string>();
            const std::string name = topic_name(t.first, twhere);
            if (seen.count(name)) {
              throw ConfigError(twhere + ": topic is also in group " + seen[name]);
            }
            seen[name] = gname;
            c.topics[name] = topic_spec(name, gname, t.second, twhere);
            group.topics.push_back(name);
          }
        }
      }
      c.groups.push_back(group);
    }
  }
  return c;
}

const TopicSpec * Catalog::topic(const std::string & name) const
{
  const auto it = topics.find(name);
  return it == topics.end() ? nullptr : &it->second;
}

const Group & Catalog::group(const std::string & name) const
{
  for (const auto & g : groups) {
    if (g.name == name) {
      return g;
    }
  }
  throw ConfigError("no group " + name + " in the catalog");
}

std::vector<std::string> Catalog::sort_topics(const std::set<std::string> & names) const
{
  std::vector<std::string> known, unknown;
  for (const auto & g : groups) {
    for (const auto & t : g.topics) {
      if (names.count(t)) {
        known.push_back(t);
      }
    }
  }
  for (const auto & n : names) {
    if (!topic(n)) {
      unknown.push_back(n);
    }
  }
  known.insert(known.end(), unknown.begin(), unknown.end());
  return known;
}

RecordOptions RecordOptions::parse(
  const std::map<std::string, YAML::Node> & values, const std::string & where)
{
  RecordOptions o;
  for (const auto & kv : values) {
    if (!option_keys.count(kv.first)) {
      throw ConfigError(where + ": unknown option " + kv.first);
    }
  }
  read_choice(values, "storage", o.storage, where);
  read_choice(values, "compression", o.compression, where);
  read_choice(values, "heavy_qos", o.heavy_qos, where);
  read_choice(values, "start_mode", o.start_mode, where);
  read_choice(values, "on_not_ready", o.on_not_ready, where);
  read_count(values, "split_duration_sec", o.split_duration_sec, where);
  read_count(values, "split_size_mb", o.split_size_mb, where);
  read_count(values, "wait_ready_sec", o.wait_ready_sec, where);
  read_count(values, "start_delay_sec", o.start_delay_sec, where);
  read_count(values, "max_cache_mb", o.max_cache_mb, where);
  read_amount(values, "disk_warn_gb", o.disk_warn_gb, where);
  read_amount(values, "disk_stop_gb", o.disk_stop_gb, where);
  if (values.count("storage_preset")) {
    o.storage_preset = text(values.at("storage_preset"));
  }
  if (values.count("storage_config_uri")) {
    o.storage_config_uri = text(values.at("storage_config_uri"));
  }
  if (values.count("required_frames")) {
    o.required_frames = strings(values.at("required_frames"), where + ".required_frames");
  }
  if (o.disk_stop_gb > o.disk_warn_gb) {
    throw ConfigError(where + ": disk_stop_gb is larger than disk_warn_gb");
  }
  if (o.max_cache_mb < 1) {
    throw ConfigError(where + ".max_cache_mb: must be at least 1");
  }
  if (o.start_mode == "wait_ready" && o.wait_ready_sec < 1) {
    throw ConfigError(where + ".wait_ready_sec: must be at least 1");
  }
  return o;
}

std::string RecordOptions::preset() const
{
  if (!storage_preset.empty() || storage != "mcap") {
    return storage_preset;
  }
  return compression == "zstd" ? "zstd_fast" : "fastwrite";
}

YAML::Node RecordOptions::to_yaml() const
{
  YAML::Node n;
  n["storage"] = storage;
  n["compression"] = compression;
  n["split_duration_sec"] = split_duration_sec;
  n["split_size_mb"] = split_size_mb;
  n["heavy_qos"] = heavy_qos;
  n["start_mode"] = start_mode;
  n["wait_ready_sec"] = wait_ready_sec;
  n["start_delay_sec"] = start_delay_sec;
  n["disk_warn_gb"] = disk_warn_gb;
  n["disk_stop_gb"] = disk_stop_gb;
  n["max_cache_mb"] = max_cache_mb;
  n["storage_preset"] = storage_preset;
  n["storage_config_uri"] = storage_config_uri;
  n["on_not_ready"] = on_not_ready;
  n["required_frames"] = required_frames;
  return n;
}

Profile Profile::parse(const YAML::Node & root, const std::string & where)
{
  require_map(root, where);
  check_keys(
    root,
    {"name", "description", "extends", "abstract", "groups", "add_groups", "exclude_groups",
     "topics", "exclude", "options"},
    where);
  Profile p;
  p.name = text(root["name"]);
  if (!std::regex_match(p.name, profile_pattern)) {
    throw ConfigError(where + ": name uses lowercase letters, digits and _ ('" + p.name + "')");
  }
  p.description = text(root["description"]);
  p.extends = text(root["extends"]);
  p.abstract = flag(root["abstract"], where + ".abstract");
  if (root["groups"] && !root["groups"].IsNull()) {
    p.groups = strings(root["groups"], where + ".groups");
  }
  p.add_groups = strings(root["add_groups"], where + ".add_groups");
  p.exclude_groups = strings(root["exclude_groups"], where + ".exclude_groups");
  p.topics = strings(root["topics"], where + ".topics");
  p.exclude = strings(root["exclude"], where + ".exclude");
  for (const auto * list : {&p.topics, &p.exclude}) {
    for (const auto & t : *list) {
      topic_name(YAML::Node(t), where + (list == &p.topics ? ".topics" : ".exclude"));
    }
  }
  if (const auto options = root["options"]; options && !options.IsNull()) {
    require_map(options, where + ".options");
    for (const auto & kv : options) {
      const std::string key = kv.first.as<std::string>();
      if (!option_keys.count(key)) {
        throw ConfigError(where + ".options: unknown option " + key);
      }
      p.options[key] = kv.second;
    }
  }
  return p;
}

std::string ResolvedProfile::to_yaml() const
{
  YAML::Node n;
  n["name"] = name;
  n["description"] = description;
  n["chain"] = chain;
  n["topics"] = topics;
  n["unknown_topics"] = unknown_topics;
  n["options"] = options.to_yaml();
  return YAML::Dump(n);
}

ProfileStore::ProfileStore(std::string directory) : directory_(std::move(directory))
{
}

std::vector<std::string> ProfileStore::names() const
{
  std::vector<std::string> out;
  if (!fs::is_directory(directory_)) {
    return out;
  }
  for (const auto & entry : fs::directory_iterator(directory_)) {
    if (entry.is_regular_file() && entry.path().extension() == ".yaml") {
      out.push_back(entry.path().stem().string());
    }
  }
  std::sort(out.begin(), out.end());
  return out;
}

Profile ProfileStore::load(const std::string & name) const
{
  const fs::path path = fs::path(directory_) / (name + ".yaml");
  if (!fs::exists(path)) {
    throw ConfigError("no profile " + name + " (" + path.string() + ")");
  }
  Profile p;
  try {
    p = Profile::parse(YAML::LoadFile(path.string()), path.filename().string());
  } catch (const YAML::Exception & e) {
    throw ConfigError(path.string() + ": " + e.what());
  }
  if (p.name != name) {
    throw ConfigError(path.filename().string() + ": file name and name (" + p.name + ") differ");
  }
  return p;
}

ResolvedProfile ProfileStore::resolve(const std::string & name, const Catalog & catalog) const
{
  std::vector<std::string> stack{name};
  return resolve(load(name), catalog, stack);
}

ResolvedProfile ProfileStore::resolve(
  const Profile & profile, const Catalog & catalog, std::vector<std::string> & stack) const
{
  if (stack.size() > max_depth) {
    throw ConfigError("inheritance too deep: " + join(stack, " -> "));
  }
  std::set<std::string> topics;
  std::map<std::string, YAML::Node> options;
  std::vector<std::string> chain{profile.name};
  if (!profile.extends.empty()) {
    if (std::find(stack.begin(), stack.end(), profile.extends) != stack.end()) {
      stack.push_back(profile.extends);
      throw ConfigError("inheritance cycle: " + join(stack, " -> "));
    }
    stack.push_back(profile.extends);
    const ResolvedProfile parent = resolve(load(profile.extends), catalog, stack);
    topics.insert(parent.topics.begin(), parent.topics.end());
    topics.insert(parent.unknown_topics.begin(), parent.unknown_topics.end());
    const YAML::Node parent_options = parent.options.to_yaml();
    for (const auto & kv : parent_options) {
      options[kv.first.as<std::string>()] = kv.second;
    }
    chain.insert(chain.end(), parent.chain.begin(), parent.chain.end());
  }
  auto group_set = [&](const std::vector<std::string> & names, const std::string & key) {
    std::set<std::string> out;
    for (const auto & g : names) {
      try {
        const auto & members = catalog.group(g).topics;
        out.insert(members.begin(), members.end());
      } catch (const ConfigError &) {
        throw ConfigError(profile.name + "." + key + ": no group '" + g + "' in the catalog");
      }
    }
    return out;
  };
  if (profile.groups) {
    topics = group_set(*profile.groups, "groups");
  }
  for (const auto & t : group_set(profile.add_groups, "add_groups")) {
    topics.insert(t);
  }
  for (const auto & t : group_set(profile.exclude_groups, "exclude_groups")) {
    topics.erase(t);
  }
  topics.insert(profile.topics.begin(), profile.topics.end());
  for (const auto & t : profile.exclude) {
    topics.erase(t);
  }
  for (const auto & kv : profile.options) {
    options[kv.first] = kv.second;
  }
  ResolvedProfile r;
  r.name = profile.name;
  r.description = profile.description;
  r.chain = chain;
  r.abstract = profile.abstract;
  r.options = RecordOptions::parse(options, profile.name + ".options");
  for (const auto & t : catalog.sort_topics(topics)) {
    (catalog.topic(t) ? r.topics : r.unknown_topics).push_back(t);
  }
  return r;
}

}  // namespace rosbag_recorder
