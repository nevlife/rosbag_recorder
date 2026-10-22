// Copyright 2026 pgw
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include <stdexcept>
#include <string>

#include "rosbag_recorder_checks/message.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "serialize.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

using rosbag_recorder_checks::TypeInfo;

TEST(TypeInfo, DetectsLeadingHeader)
{
  EXPECT_TRUE(TypeInfo("sensor_msgs/msg/Imu").starts_with_header());
  EXPECT_TRUE(TypeInfo("sensor_msgs/msg/PointCloud2").starts_with_header());
  EXPECT_FALSE(TypeInfo("tf2_msgs/msg/TFMessage").starts_with_header());
  EXPECT_FALSE(TypeInfo("std_msgs/msg/String").starts_with_header());
}

TEST(TypeInfo, MissingTypeThrows)
{
  EXPECT_THROW(TypeInfo("no_such_pkg/msg/Nothing"), std::runtime_error);
}

TEST(TypeInfo, TextOfFieldAndWholeMessage)
{
  std_msgs::msg::String s;
  s.data = "AUTO";
  EXPECT_EQ(TypeInfo("std_msgs/msg/String").text(serialize(s), "data"), "AUTO");
  EXPECT_EQ(TypeInfo("std_msgs/msg/String").text(serialize(s), ""), "{data: AUTO}");

  auto imu = stamped<sensor_msgs::msg::Imu>(1, 2);
  imu.linear_acceleration.z = 9.5;
  const TypeInfo info("sensor_msgs/msg/Imu");
  EXPECT_EQ(info.text(serialize(imu), "header.frame_id"), "base_link");
  EXPECT_EQ(info.text(serialize(imu), "linear_acceleration.z"), "9.5");
  const std::string whole = info.text(serialize(imu), "");
  EXPECT_EQ(whole.find("header"), std::string::npos);
  EXPECT_NE(whole.find("linear_acceleration: {x: 0, y: 0, z: 9.5}"), std::string::npos);
  EXPECT_THROW(info.text(serialize(imu), "nope"), std::invalid_argument);
  EXPECT_THROW(info.text(serialize(imu), "header.frame_id.x"), std::invalid_argument);
}

TEST(TypeInfo, ArraysAreShortened)
{
  std_msgs::msg::UInt8MultiArray a;
  a.data = {1, 2, 3};
  const TypeInfo info("std_msgs/msg/UInt8MultiArray");
  EXPECT_EQ(info.text(serialize(a), "data"), "[1, 2, 3]");
  a.data.assign(20, 7);
  EXPECT_EQ(info.text(serialize(a), "data"), "[7, 7, 7, 7, 7, 7, 7, 7, ... (20)]");
}
