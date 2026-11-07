#include "rclcpp/serialization.hpp"
#include "stream.hpp"

#include "tf2_msgs/msg/tf_message.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <deque>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <vector>

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

rclcpp::SerializedMessage tf_message(const std::string & parent, const std::string & child)
{
  tf2_msgs::msg::TFMessage m;
  m.transforms.resize(1);
  m.transforms[0].header.frame_id = parent;
  m.transforms[0].child_frame_id = child;
  rclcpp::SerializedMessage out;
  rclcpp::Serialization<tf2_msgs::msg::TFMessage>().serialize_message(&m, &out);
  return out;
}

}  // namespace

TEST(Rate, Statuses)
{
  const auto ok = evaluate_rate(10.0, ticks(0, 0.1, 50), 50, 5.0, 0.8, 1, false, false);
  EXPECT_EQ(ok.status, TopicHealth::OK);
  EXPECT_NEAR(ok.hz, 10.0, 0.01);
  EXPECT_EQ(
    evaluate_rate(10.0, ticks(0, 0.2, 25), 25, 5.0, 0.8, 1, false, false).status, TopicHealth::LOW);
  EXPECT_EQ(
    evaluate_rate(10.0, ticks(0, 0.1, 10), 10, 5.0, 0.8, 1, false, false).status,
    TopicHealth::STALE);
  EXPECT_EQ(
    evaluate_rate(10.0, ticks(0, 0.1, 10), 10, 5.0, 0.8, 0, false, false).status,
    TopicHealth::NO_PUBLISHER);
  EXPECT_EQ(evaluate_rate(10.0, {}, 0, 5.0, 0.8, 1, false, false).status, TopicHealth::WAITING);
  EXPECT_EQ(
    evaluate_rate(10.0, {}, 0, 5.0, 0.8, 0, false, false).status, TopicHealth::NO_PUBLISHER);
  EXPECT_EQ(
    evaluate_rate(std::nullopt, {}, 0, 5.0, 0.8, 0, true, false).status,
    TopicHealth::LATCHED_MISSING);
  EXPECT_EQ(evaluate_rate(std::nullopt, {}, 0, 5.0, 0.8, 1, true, false).status, TopicHealth::OK);
  EXPECT_EQ(
    evaluate_rate(std::nullopt, {}, 0, 5.0, 0.8, 1, false, true).status,
    TopicHealth::TYPE_UNAVAILABLE);
  EXPECT_EQ(
    evaluate_rate(std::nullopt, ticks(0, 1.0, 3), 3, 2.5, 0.8, 1, false, false).status,
    TopicHealth::OK);
}

TEST(Stream, Report)
{
  Stream s(StreamSpec{"/tf", "tf2_msgs/msg/TFMessage", "", 10.0, true, false, false});
  s.publishers = 1;
  for (int i = 0; i < 20; ++i) {
    s.receive(tf_message("map", "odom"), 100.0 + i * 0.1);
  }
  s.count_written();
  auto h = s.report(102.0, 0.8);
  EXPECT_EQ(h.status, TopicHealth::OK);
  EXPECT_EQ(h.received, 20U);
  EXPECT_EQ(h.written, 1U);
  EXPECT_GT(h.size_kb, 0.0);
  EXPECT_TRUE(h.issues.empty());
  s.reset_counts();
  s.error = "type unavailable: x";
  h = s.report(102.0, 0.8);
  EXPECT_EQ(h.status, TopicHealth::TYPE_UNAVAILABLE);
  EXPECT_EQ(h.issues, std::vector<std::string>{"type unavailable: x"});
  EXPECT_EQ(h.written, 0U);
  EXPECT_TRUE(std::isnan(
    Stream(StreamSpec{"/n", "", "", std::nullopt, false, false, false}).report(0, 0.8).age_s));
}

TEST(Stream, FramesAndLatched)
{
  Stream s(
    StreamSpec{"/tf_static", "tf2_msgs/msg/TFMessage", "", std::nullopt, false, true, false});
  s.receive(tf_message("map", "odom"), 1.0);
  EXPECT_TRUE(s.frames().empty());
  s.collect_frames();
  s.receive(tf_message("odom", "base_link"), 2.0);
  EXPECT_EQ(s.frames(), (std::set<std::string>{"odom", "base_link"}));
  for (int i = 0; i < 101; ++i) {
    s.keep_latched(std::make_shared<rclcpp::SerializedMessage>(tf_message("a", std::to_string(i))));
  }
  EXPECT_EQ(s.latched().size(), 100U);
}
