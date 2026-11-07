#include "summary.hpp"

#include "text.hpp"
#include "yaml-cpp/yaml.h"

#include <ctime>
#include <string>

namespace rosbag_recorder
{
namespace
{

const char * const level_names[] = {"ok", "info", "warn", "error"};
const char * const status_names[] = {
  "ok",      "low", "stale", "waiting", "no_publisher", "latched_missing", "type_unavailable",
  "abnormal"};

std::string local_time(const builtin_interfaces::msg::Time & t)
{
  const std::time_t seconds = t.sec;
  std::tm local{};
  localtime_r(&seconds, &local);
  char text[32];
  std::strftime(text, sizeof(text), "%Y-%m-%d %H:%M:%S", &local);
  return text;
}

double seconds(const builtin_interfaces::msg::Time & t)
{
  return static_cast<double>(t.sec) + static_cast<double>(t.nanosec) * 1e-9;
}

}  // namespace

std::string summary_yaml(const Summary & s)
{
  YAML::Emitter out;
  out << YAML::BeginMap;
  out << YAML::Key << "session_id" << YAML::Value << s.end.session_id;
  out << YAML::Key << "profile" << YAML::Value << s.end.profile;
  out << YAML::Key << "label" << YAML::Value << s.end.label;
  out << YAML::Key << "notes" << YAML::Value << s.end.notes;
  out << YAML::Key << "host" << YAML::Value << s.host;
  out << YAML::Key << "start" << YAML::Value << local_time(s.end.start_time);
  out << YAML::Key << "end" << YAML::Value << local_time(s.end.end_time);
  out << YAML::Key << "duration_s" << YAML::Value << fixed(s.duration_s, 1);
  out << YAML::Key << "end_reason" << YAML::Value << s.end.end_reason;
  out << YAML::Key << "bag_bytes" << YAML::Value << s.bag_bytes;
  out << YAML::Key << "preflight" << YAML::Value << YAML::BeginSeq;
  for (const auto & c : s.preflight) {
    out << std::string(level_names[c.level]) + ": " + c.text;
  }
  out << YAML::EndSeq << YAML::Key << "topics" << YAML::Value << YAML::BeginMap;
  for (const auto & t : s.topics) {
    out << YAML::Key << t.name << YAML::Value << YAML::Flow << YAML::BeginMap;
    out << YAML::Key << "type" << YAML::Value << t.type;
    out << YAML::Key << "written" << YAML::Value << t.written;
    out << YAML::Key << "last_status" << YAML::Value << status_names[t.status];
    out << YAML::EndMap;
  }
  out << YAML::EndMap << YAML::Key << "markers" << YAML::Value << YAML::BeginSeq;
  for (const auto & m : s.markers) {
    const double t = seconds(m.stamp) - seconds(s.end.start_time);
    out << fixed(t, 1) + " s [" + m.kind + "] " + m.text;
  }
  out << YAML::EndSeq << YAML::EndMap;
  return std::string(out.c_str()) + "\n";
}

}  // namespace rosbag_recorder
