// Copyright 2026 pgw
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include <cmath>
#include <deque>
#include <memory>
#include <string>

#include "rclcpp/serialization.hpp"
#include "rosbag_recorder/session_info.hpp"
#include "rosbag_recorder/stream.hpp"
#include "std_msgs/msg/string.hpp"

using rosbag_recorder::evaluate_rate;
using rosbag_recorder::Stream;
using rosbag_recorder::StreamSpec;
using rosbag_recorder::TopicHealth;

namespace
{

std::deque<double> ticks(double start, double period, int n)
{
  std::deque<double> d;
  for (int i = 0; i < n; ++i) {
    d.push_back(start + i * period);
  }
  return d;
}

rclcpp::SerializedMessage text(const std::string & s)
{
  std_msgs::msg::String m;
  m.data = s;
  rclcpp::SerializedMessage out;
  rclcpp::Serialization<std_msgs::msg::String>().serialize_message(&m, &out);
  return out;
}

class Flag : public rosbag_recorder_checks::Check
{
public:
  bool accepts(const rosbag_recorder_checks::TopicInfo &) const override {return true;}
  void configure(
    const rosbag_recorder_checks::TopicInfo &, const rosbag_recorder_checks::Thresholds &)
  override {}
  void on_message(const rclcpp::SerializedMessage &, double) override {++n;}
  bool evaluate(const rosbag_recorder_checks::Context &, rosbag_recorder_checks::Result & r)
  override
  {
    if (n == 0) {
      return false;
    }
    if (bad) {
      r.issues.push_back("bad");
    }
    n = 0;
    return true;
  }
  int n{0};
  bool bad{false};
};

}  // namespace

TEST(Rate, Statuses)
{
  const auto ok = evaluate_rate(10.0, ticks(0, 0.1, 50), 50, 5.0, 0.8, 1, false, false);
  EXPECT_EQ(ok.status, TopicHealth::OK);
  EXPECT_NEAR(ok.hz, 10.0, 0.01);
  EXPECT_EQ(
    evaluate_rate(10.0, ticks(0, 0.2, 25), 25, 5.0, 0.8, 1, false, false).status,
    TopicHealth::LOW);
  EXPECT_EQ(
    evaluate_rate(10.0, ticks(0, 0.1, 10), 10, 5.0, 0.8, 1, false, false).status,
    TopicHealth::STALE);
  EXPECT_EQ(
    evaluate_rate(10.0, ticks(0, 0.1, 10), 10, 5.0, 0.8, 0, false, false).status,
    TopicHealth::NO_PUBLISHER);
  EXPECT_EQ(evaluate_rate(10.0, {}, 0, 5.0, 0.8, 1, false, false).status, TopicHealth::WAITING);
  EXPECT_EQ(
    evaluate_rate(10.0, {}, 0, 5.0, 0.8, 0, false, false).status,
    TopicHealth::NO_PUBLISHER);
  EXPECT_EQ(
    evaluate_rate(std::nullopt, {}, 0, 5.0, 0.8, 0, true, false).status,
    TopicHealth::LATCHED_MISSING);
  EXPECT_EQ(
    evaluate_rate(std::nullopt, {}, 0, 5.0, 0.8, 1, true, false).status,
    TopicHealth::OK);
  EXPECT_EQ(
    evaluate_rate(std::nullopt, {}, 0, 5.0, 0.8, 1, false, true).status,
    TopicHealth::TYPE_UNAVAILABLE);
  EXPECT_EQ(
    evaluate_rate(std::nullopt, ticks(0, 1.0, 3), 3, 2.5, 0.8, 1, false, false).status,
    TopicHealth::OK);
}

TEST(Stream, ReportAndHeldIssues)
{
  Stream s(StreamSpec{"/s", "std_msgs/msg/String", "", 10.0, true, false, false, true});
  auto flag = std::make_shared<Flag>();
  s.set_checks({flag});
  s.publishers = 1;
  for (int i = 0; i < 20; ++i) {
    s.receive(text("x"), 100.0 + i * 0.1, 0);
  }
  auto h = s.report(102.0, 10.0, 0.8, false);
  EXPECT_EQ(h.status, TopicHealth::OK);
  EXPECT_EQ(h.received, 20U);
  EXPECT_GT(h.size_kb, 0.0);
  flag->bad = true;
  s.receive(text("x"), 102.0, 0);
  h = s.report(102.1, 11.0, 0.8, false);
  EXPECT_EQ(h.status, TopicHealth::ABNORMAL);
  EXPECT_EQ(h.issues.front(), "bad");
  flag->bad = false;
  s.receive(text("x"), 102.2, 0);
  EXPECT_EQ(s.report(102.3, 12.0, 0.8, false).status, TopicHealth::ABNORMAL);
  s.receive(text("x"), 102.4, 0);
  EXPECT_EQ(s.report(102.5, 15.0, 0.8, false).status, TopicHealth::OK);
  EXPECT_TRUE(
    std::isnan(
      Stream(
        StreamSpec{"/n", "", "", std::nullopt, false, false, false,
          true}).report(0, 0, 0.8, false).age_s));
}

TEST(Stream, AutoMarkerOnValueChange)
{
  Stream s(StreamSpec{"/mode", "std_msgs/msg/String", "", std::nullopt, false, false, false, true});
  s.set_auto_marker({"/mode", "drive mode", "data", "mode"});
  EXPECT_EQ(s.auto_kind, "mode");
  EXPECT_FALSE(s.receive(text("MANUAL"), 1, 0));
  EXPECT_FALSE(s.receive(text("MANUAL"), 2, 0));
  EXPECT_EQ(*s.receive(text("AUTO"), 3, 0), "drive mode: MANUAL -> AUTO");

  Stream bad(StreamSpec{"/mode", "std_msgs/msg/String", "", std::nullopt, false, false, false,
      true});
  bad.set_auto_marker({"/mode", "", "nope", "auto"});
  EXPECT_FALSE(bad.receive(text("A"), 1, 0));
  EXPECT_FALSE(bad.receive(text("B"), 2, 0));
  EXPECT_EQ(bad.report(2, 2, 0.8, false).values.back().rfind("auto marker off", 0), 0U);
}

TEST(SessionInfo, GlobAndIds)
{
  using rosbag_recorder::glob_match;
  EXPECT_TRUE(glob_match("src/**/config/*.yaml", "src/a/b/config/x.yaml"));
  EXPECT_TRUE(glob_match("src/**/config/*.yaml", "src/config/x.yaml"));
  EXPECT_FALSE(glob_match("src/**/config/*.yaml", "src/a/config/sub/x.yaml"));
  EXPECT_TRUE(glob_match("src/bring_up/launch/*.py", "src/bring_up/launch/a.py"));
  EXPECT_FALSE(glob_match("src/*.py", "src/a/b.py"));
  EXPECT_TRUE(glob_match("params/scv?.yaml", "params/scv1.yaml"));
  const std::time_t t = 0;
  const std::string id = rosbag_recorder::session_id("drive", " test run!", t);
  EXPECT_EQ(id.substr(15), "_drive_test_run");
  EXPECT_EQ(rosbag_recorder::session_id("drive", "", t).substr(15), "_drive");
  EXPECT_EQ(
    rosbag_recorder::sha256("abc"),
    "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
  EXPECT_EQ(rosbag_recorder::expand_user("/x"), "/x");
}
