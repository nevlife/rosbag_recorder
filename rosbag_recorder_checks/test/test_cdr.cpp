// Copyright 2026 pgw
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include <vector>

#include "rosbag_recorder_checks/message.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "serialize.hpp"

using rosbag_recorder_checks::after_header;
using rosbag_recorder_checks::kEncapsulation;
using rosbag_recorder_checks::read_stamp;
using rosbag_recorder_checks::read_u32;

TEST(Cdr, ReadsHeaderStamp)
{
  const auto bytes = serialize(stamped<sensor_msgs::msg::Imu>(1700000000, 123456789));
  const auto & raw = bytes.get_rcl_serialized_message();
  const auto stamp = read_stamp(raw.buffer, raw.buffer_length);
  ASSERT_TRUE(stamp);
  EXPECT_EQ(stamp->sec, 1700000000);
  EXPECT_EQ(stamp->nanosec, 123456789U);
  EXPECT_EQ(stamp->nanoseconds(), 1700000000123456789LL);
}

TEST(Cdr, ReadsNegativeSeconds)
{
  const auto bytes = serialize(stamped<sensor_msgs::msg::Imu>(-5, 1));
  const auto & raw = bytes.get_rcl_serialized_message();
  EXPECT_EQ(read_stamp(raw.buffer, raw.buffer_length)->sec, -5);
}

TEST(Cdr, RejectsShortBigEndianAndBadNanoseconds)
{
  std::vector<std::uint8_t> b{0x00, 0x01, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0};
  EXPECT_TRUE(read_stamp(b.data(), b.size()));
  EXPECT_FALSE(read_stamp(b.data(), b.size() - 1));
  b[1] = 0x00;
  EXPECT_FALSE(read_stamp(b.data(), b.size()));
  b[1] = 0x01;
  b[8] = 0xff;
  b[9] = 0xff;
  b[10] = 0xff;
  b[11] = 0xff;
  EXPECT_FALSE(read_stamp(b.data(), b.size()));
}

TEST(Cdr, FindsFieldAfterHeader)
{
  for (const char * frame : {"", "a", "abc", "velodyne"}) {
    auto cloud = stamped<sensor_msgs::msg::PointCloud2>(1, 2);
    cloud.header.frame_id = frame;
    cloud.height = 3;
    cloud.width = 1000;
    const auto bytes = serialize(cloud);
    const auto & raw = bytes.get_rcl_serialized_message();
    const auto body = after_header(raw.buffer, raw.buffer_length, 4);
    ASSERT_TRUE(body) << frame;
    EXPECT_EQ(read_u32(raw.buffer + kEncapsulation + *body), 3U) << frame;
    EXPECT_EQ(read_u32(raw.buffer + kEncapsulation + *body + 4), 1000U) << frame;
  }
}

TEST(Cdr, RejectsFrameIdPastTheEnd)
{
  std::vector<std::uint8_t> b{0x00, 0x01, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 100, 0, 0, 0, 'a'};
  EXPECT_FALSE(after_header(b.data(), b.size(), 4));
}
