#include "preflight.hpp"

#include "text.hpp"

#include <cstddef>
#include <set>
#include <string>
#include <vector>

namespace rosbag_recorder
{
namespace
{

CheckResult result(std::uint8_t level, const std::string & text)
{
  CheckResult r;
  r.level = level;
  r.text = text;
  return r;
}

CheckResult disk(const PreflightInput & in)
{
  const std::string free = "disk free " + fixed(in.disk_free_gb, 0) + " GB";
  if (in.disk_free_gb < in.disk_stop_gb) {
    return result(
      CheckResult::ERROR, free + ", below the stop limit " + fixed(in.disk_stop_gb, 0) + " GB");
  }
  if (in.disk_free_gb < in.disk_warn_gb) {
    return result(
      CheckResult::WARN, free + ", below the warning limit " + fixed(in.disk_warn_gb, 0) + " GB");
  }
  return result(CheckResult::OK, free + " (" + in.output_dir + ")");
}

void topics(const PreflightInput & in, std::vector<CheckResult> & out)
{
  std::size_t required = 0;
  std::vector<std::string> no_type, silent;
  for (const auto & t : in.topics) {
    required += t.required;
    if (t.status == TopicHealth::TYPE_UNAVAILABLE) {
      no_type.push_back(t.name);
    } else if (
      t.status == TopicHealth::NO_PUBLISHER || t.status == TopicHealth::WAITING ||
      t.status == TopicHealth::STALE) {
      silent.push_back(t.name);
    }
  }
  const auto missing = required_missing(in.topics);
  if (required > 0) {
    const std::string count =
      std::to_string(required - missing.size()) + "/" + std::to_string(required);
    out.push_back(
      missing.empty()
        ? result(CheckResult::OK, "required topics " + count + " ok")
        : result(
            CheckResult::WARN, "required topics " + count + " ok, not ready: " + listed(missing)));
  }
  if (!no_type.empty()) {
    out.push_back(result(
      CheckResult::WARN,
      "message type not available (source the workspace that defines it): " + listed(no_type)));
  }
  if (!silent.empty()) {
    out.push_back(result(
      CheckResult::INFO, std::to_string(silent.size()) + "/" + std::to_string(in.topics.size()) +
                           " topics without data: " + listed(silent)));
  }
}

}  // namespace

std::vector<std::string> required_missing(const std::vector<TopicHealth> & topics)
{
  std::vector<std::string> missing;
  for (const auto & t : topics) {
    if (t.required && t.status != TopicHealth::OK) {
      missing.push_back(t.name);
    }
  }
  return missing;
}

std::vector<CheckResult> preflight(const PreflightInput & in)
{
  std::vector<CheckResult> out;
  if (!in.writable) {
    out.push_back(result(CheckResult::ERROR, "cannot write to " + in.output_dir));
  }
  out.push_back(disk(in));
  if (!in.unknown_topics.empty()) {
    out.push_back(result(
      CheckResult::WARN,
      std::to_string(in.unknown_topics.size()) +
        " topics not in the catalog, no reference rate: " + listed(in.unknown_topics)));
  }
  topics(in, out);
  if (!in.required_frames.empty()) {
    std::vector<std::string> absent;
    for (const auto & f : in.required_frames) {
      if (!in.frames.count(f)) {
        absent.push_back(f);
      }
    }
    out.push_back(
      absent.empty() ? result(CheckResult::OK, "TF frames present: " + listed(in.required_frames))
                     : result(CheckResult::WARN, "TF frames missing: " + listed(absent)));
  }
  return out;
}

}  // namespace rosbag_recorder
