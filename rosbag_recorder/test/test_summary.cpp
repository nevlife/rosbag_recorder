#include "summary.hpp"
#include "yaml-cpp/yaml.h"

#include <gtest/gtest.h>

#include <string>
#include <vector>

TEST(Summary, Yaml)
{
  rosbag_recorder::Summary s;
  s.end.session_id = "20250101_000000_p";
  s.end.profile = "p";
  s.end.end_reason = "user";
  s.end.start_time.sec = 1000;
  s.end.end_time.sec = 1010;
  s.host = "robot";
  s.duration_s = 10.04;
  s.bag_bytes = 42;
  s.preflight.resize(1);
  s.preflight[0].level = rosbag_recorder_msgs::msg::CheckResult::WARN;
  s.preflight[0].text = "disk";
  s.topics.resize(2);
  s.topics[0].name = "/a";
  s.topics[0].type = "std_msgs/msg/String";
  s.topics[0].written = 7;
  s.topics[0].status = rosbag_recorder_msgs::msg::TopicHealth::STALE;
  s.topics[1].name = "/b";
  s.topics[1].written = 3;
  s.stored = {{"/a", 5}, {"/b", 3}};
  s.markers.resize(1);
  s.markers[0].stamp.sec = 1002;
  s.markers[0].stamp.nanosec = 500000000;
  s.markers[0].kind = "note";
  s.markers[0].text = "hello";
  const YAML::Node y = YAML::Load(rosbag_recorder::summary_yaml(s));
  EXPECT_EQ(y["session_id"].as<std::string>(), "20250101_000000_p");
  EXPECT_EQ(y["host"].as<std::string>(), "robot");
  EXPECT_EQ(y["duration_s"].as<std::string>(), "10.0");
  EXPECT_EQ(y["bag_bytes"].as<int>(), 42);
  EXPECT_EQ(y["end_reason"].as<std::string>(), "user");
  EXPECT_EQ(y["preflight"].as<std::vector<std::string>>(), std::vector<std::string>{"warn: disk"});
  EXPECT_EQ(y["topics"]["/a"]["written"].as<int>(), 5);
  EXPECT_EQ(y["topics"]["/a"]["dropped"].as<int>(), 2);
  EXPECT_EQ(y["topics"]["/b"]["dropped"].as<int>(), 0);
  EXPECT_EQ(rosbag_recorder::dropped(s), 2U);
  EXPECT_EQ(y["topics"]["/a"]["last_status"].as<std::string>(), "stale");
  EXPECT_EQ(
    y["markers"].as<std::vector<std::string>>(), std::vector<std::string>{"2.5 s [note] hello"});
  EXPECT_EQ(y["start"].as<std::string>().size(), 19U);
}
