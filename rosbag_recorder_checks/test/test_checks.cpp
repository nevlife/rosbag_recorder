// Copyright 2026 nevlife
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include "builtin_checks.hpp"
#include "pluginlib/class_loader.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "serialize.hpp"

namespace rc = rosbag_recorder_checks;

namespace
{

bool has_issue(const rc::Result & r, const std::string & part)
{
  for (const auto & i : r.issues) {
    if (i.find(part) != std::string::npos) {
      return true;
    }
  }
  return false;
}

rc::Result run(rc::Check & check, std::uint32_t publishers = 1, bool sim_time = false)
{
  rc::Result r;
  EXPECT_TRUE(check.evaluate(rc::Context{0.0, sim_time, publishers}, r));
  return r;
}

void feed_imu(rc::Check & check, std::int32_t sec, std::uint32_t nanosec, double wall)
{
  check.on_message(serialize(stamped<sensor_msgs::msg::Imu>(sec, nanosec)), wall);
}

}  // namespace

TEST(HeaderStamp, AcceptsStampedTypesOnly)
{
  rc::HeaderStamp c;
  EXPECT_TRUE(c.accepts({"/imu", "sensor_msgs/msg/Imu", false}));
  EXPECT_FALSE(c.accepts({"/imu", "sensor_msgs/msg/Imu", true}));
  EXPECT_FALSE(c.accepts({"/tf", "tf2_msgs/msg/TFMessage", false}));
}

TEST(HeaderStamp, NothingReceivedKeepsPreviousResult)
{
  rc::HeaderStamp c;
  rc::Result r;
  EXPECT_FALSE(c.evaluate(rc::Context{}, r));
}

TEST(HeaderStamp, DelayAndFuture)
{
  rc::HeaderStamp c;
  c.configure({"/imu", "sensor_msgs/msg/Imu", false}, {{"stamp_delay_max_s", 0.5}});
  feed_imu(c, 100, 0, 100.1);
  feed_imu(c, 100, 100000000, 100.2);
  EXPECT_TRUE(run(c).issues.empty());

  feed_imu(c, 100, 0, 101.0);
  feed_imu(c, 100, 100000000, 101.1);
  EXPECT_TRUE(has_issue(run(c), "stamp delay mean 1000 ms"));

  feed_imu(c, 200, 0, 199.0);
  feed_imu(c, 200, 100000000, 199.1);
  EXPECT_TRUE(has_issue(run(c), "ahead of receive time"));
  feed_imu(c, 300, 0, 299.0);
  EXPECT_TRUE(run(c, 1, true).issues.empty());
}

TEST(HeaderStamp, BackwardsAndRepeat)
{
  rc::HeaderStamp c;
  feed_imu(c, 10, 0, 10.0);
  feed_imu(c, 9, 0, 10.0);
  EXPECT_TRUE(has_issue(run(c), "backwards 1 times"));

  feed_imu(c, 20, 0, 20.0);
  feed_imu(c, 20, 0, 20.0);
  feed_imu(c, 20, 0, 20.0);
  EXPECT_TRUE(has_issue(run(c), "same stamp 3 times in a row"));

  feed_imu(c, 30, 0, 30.0);
  feed_imu(c, 29, 0, 30.0);
  EXPECT_TRUE(run(c, 2).issues.empty());
}

TEST(HeaderStamp, ZeroStampIsNoted)
{
  rc::HeaderStamp c;
  feed_imu(c, 0, 0, 10.0);
  const auto r = run(c);
  EXPECT_TRUE(r.issues.empty());
  EXPECT_EQ(r.values.front(), "stamp 0");
}

TEST(Imu, ZeroAndNonFinite)
{
  rc::Imu c;
  sensor_msgs::msg::Imu imu;
  c.on_message(serialize(imu), 0);
  EXPECT_TRUE(has_issue(run(c), "all zero"));

  imu.linear_acceleration.z = 9.8;
  c.on_message(serialize(imu), 0);
  auto r = run(c);
  EXPECT_TRUE(r.issues.empty());
  EXPECT_EQ(r.values.front(), "|a| 9.80 m/s^2");

  imu.angular_velocity.x = std::numeric_limits<double>::quiet_NaN();
  c.on_message(serialize(imu), 0);
  EXPECT_TRUE(has_issue(run(c), "NaN/Inf values in 1 messages"));
}

TEST(NavSatFix, Covariance)
{
  rc::NavSatFix c;
  c.configure({"/fix", "sensor_msgs/msg/NavSatFix", false}, {{"gnss_max_std_m", 2.0}});
  sensor_msgs::msg::NavSatFix fix;
  fix.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
  fix.position_covariance[0] = 1.0;
  fix.position_covariance[4] = 1.0;
  c.on_message(serialize(fix), 0);
  EXPECT_TRUE(run(c).issues.empty());

  fix.position_covariance[4] = 9.0;
  c.on_message(serialize(fix), 0);
  EXPECT_TRUE(has_issue(run(c), "low accuracy +-3.0 m"));

  fix.position_covariance[0] = -1.0;
  c.on_message(serialize(fix), 0);
  EXPECT_TRUE(has_issue(run(c), "covariance NaN or negative"));
}

TEST(TfMessage, NonFiniteAndQuaternionNorm)
{
  rc::TfMessage c;
  tf2_msgs::msg::TFMessage tf;
  tf.transforms.resize(2);
  tf.transforms[0].header.frame_id = "map";
  tf.transforms[0].child_frame_id = "odom";
  tf.transforms[0].transform.rotation.w = 1.0;
  tf.transforms[1].header.frame_id = "odom";
  tf.transforms[1].child_frame_id = "base_link";
  tf.transforms[1].transform.rotation.w = 1.0;
  c.on_message(serialize(tf), 0);
  auto r = run(c);
  EXPECT_TRUE(r.issues.empty());
  EXPECT_EQ(r.values.front(), "2 frames");

  tf.transforms[0].transform.rotation.w = 0.5;
  tf.transforms[1].transform.translation.x = std::numeric_limits<double>::infinity();
  c.on_message(serialize(tf), 0);
  r = run(c);
  EXPECT_TRUE(has_issue(r, "map->odom"));
  EXPECT_TRUE(has_issue(r, "odom->base_link"));
}

TEST(PointCloud2, Empty)
{
  rc::PointCloud2 c;
  auto cloud = stamped<sensor_msgs::msg::PointCloud2>(1, 0);
  c.on_message(serialize(cloud), 0);
  EXPECT_TRUE(has_issue(run(c), "all point clouds empty"));

  cloud.height = 1;
  cloud.width = 500;
  c.on_message(serialize(cloud), 0);
  cloud.width = 0;
  c.on_message(serialize(cloud), 0);
  const auto r = run(c);
  EXPECT_TRUE(has_issue(r, "empty point cloud in 1 messages"));
  EXPECT_EQ(r.values.front(), "250 points");
}

TEST(Plugins, AllBuiltinChecksLoad)
{
  pluginlib::ClassLoader<rc::Check> loader("rosbag_recorder_checks",
    "rosbag_recorder_checks::Check");
  const auto classes = loader.getDeclaredClasses();
  EXPECT_EQ(classes.size(), 5U);
  for (const auto & name : classes) {
    EXPECT_TRUE(loader.createSharedInstance(name)) << name;
  }
  EXPECT_TRUE(
    loader.createSharedInstance("rosbag_recorder_checks/PointCloud2")->accepts(
      {"/points", "sensor_msgs/msg/PointCloud2", false}));
}
