#include "preflight.hpp"

#include <gtest/gtest.h>

#include <string>
#include <vector>

using rosbag_recorder::CheckResult;
using rosbag_recorder::PreflightInput;
using rosbag_recorder::TopicHealth;

namespace
{

TopicHealth topic(const std::string & name, std::uint8_t status, bool required = false)
{
  TopicHealth t;
  t.name = name;
  t.status = status;
  t.required = required;
  return t;
}

PreflightInput ready()
{
  PreflightInput in;
  in.output_dir = "/bags";
  in.writable = true;
  in.disk_free_gb = 100.0;
  in.disk_warn_gb = 30.0;
  in.disk_stop_gb = 10.0;
  return in;
}

std::vector<std::string> texts(const std::vector<CheckResult> & checks, std::uint8_t level)
{
  std::vector<std::string> out;
  for (const auto & c : checks) {
    if (c.level == level) {
      out.push_back(c.text);
    }
  }
  return out;
}

}  // namespace

TEST(Preflight, Disk)
{
  auto in = ready();
  EXPECT_EQ(
    texts(rosbag_recorder::preflight(in), CheckResult::OK).front(), "disk free 100 GB (/bags)");
  in.disk_free_gb = 20.0;
  EXPECT_EQ(
    texts(rosbag_recorder::preflight(in), CheckResult::WARN).front(),
    "disk free 20 GB, below the warning limit 30 GB");
  in.disk_free_gb = 5.0;
  in.writable = false;
  EXPECT_EQ(
    texts(rosbag_recorder::preflight(in), CheckResult::ERROR),
    (std::vector<std::string>{
      "cannot write to /bags", "disk free 5 GB, below the stop limit 10 GB"}));
}

TEST(Preflight, Topics)
{
  auto in = ready();
  in.unknown_topics = {"/u1", "/u2", "/u3", "/u4", "/u5"};
  in.topics = {
    topic("/a", TopicHealth::OK, true), topic("/b", TopicHealth::WAITING, true),
    topic("/c", TopicHealth::TYPE_UNAVAILABLE), topic("/d", TopicHealth::STALE)};
  const auto checks = rosbag_recorder::preflight(in);
  EXPECT_EQ(
    texts(checks, CheckResult::WARN),
    (std::vector<std::string>{
      "5 topics not in the catalog, no reference rate: /u1, /u2, /u3, /u4 and 1 more",
      "required topics 1/2 ok, not ready: /b",
      "message type not available (source the workspace that defines it): /c"}));
  EXPECT_EQ(
    texts(checks, CheckResult::INFO), std::vector<std::string>{"2/4 topics without data: /b, /d"});
  EXPECT_EQ(rosbag_recorder::required_missing(in.topics), std::vector<std::string>{"/b"});
}

TEST(Preflight, Frames)
{
  auto in = ready();
  in.required_frames = {"base_link", "map"};
  in.frames = {"map"};
  EXPECT_EQ(
    texts(rosbag_recorder::preflight(in), CheckResult::WARN),
    std::vector<std::string>{"TF frames missing: base_link"});
  in.frames.insert("base_link");
  EXPECT_EQ(
    texts(rosbag_recorder::preflight(in), CheckResult::OK).back(),
    "TF frames present: base_link, map");
}
