#include "disk_watch.hpp"

#include <gtest/gtest.h>

#include <cmath>

using rosbag_recorder::DiskWatch;

TEST(DiskWatch, RateAndRemaining)
{
  DiskWatch watch;
  auto r = watch.update(0.0, 0, 100.0, 10.0);
  EXPECT_DOUBLE_EQ(r.write_mbps, 0.0);
  EXPECT_TRUE(std::isnan(r.remaining_min));
  r = watch.update(2.0, 2 * 1048576, 100.0, 10.0);
  EXPECT_DOUBLE_EQ(r.write_mbps, 1.0);
  EXPECT_DOUBLE_EQ(r.remaining_min, 90.0 * 1024.0 / 60.0);
  EXPECT_FALSE(r.stop);
  for (int i = 3; i < 20; ++i) {
    r = watch.update(i, 2 * 1048576, 100.0, 10.0);
  }
  EXPECT_DOUBLE_EQ(r.write_mbps, 0.0);
}

TEST(DiskWatch, StopsAfterThreeLowReports)
{
  DiskWatch watch;
  EXPECT_FALSE(watch.update(0.0, 0, 5.0, 10.0).stop);
  EXPECT_FALSE(watch.update(1.0, 0, 50.0, 10.0).stop);
  EXPECT_FALSE(watch.update(2.0, 0, 5.0, 10.0).stop);
  EXPECT_FALSE(watch.update(3.0, 0, 5.0, 10.0).stop);
  EXPECT_TRUE(watch.update(4.0, 0, 5.0, 10.0).stop);
  watch.reset();
  EXPECT_FALSE(watch.update(5.0, 0, 5.0, 10.0).stop);
}
