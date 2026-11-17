#include "config.hpp"
#include "selection.hpp"

#include <gtest/gtest.h>

#include <set>
#include <string>
#include <vector>

using rosbag_recorder_msgs::msg::TopicSelection;

namespace
{

TopicSelection topic(const std::string & name, const std::string & transport)
{
  TopicSelection t;
  t.name = name;
  t.transport = transport;
  return t;
}

std::string error(const std::vector<TopicSelection> & topics, bool with_flux = true)
{
  try {
    rosbag_recorder::check_selection(topics, {"/a", "/b/flux"}, {"/a", "/b"}, with_flux);
  } catch (const rosbag_recorder::ConfigError & e) {
    return e.what();
  }
  return "";
}

}  // namespace

TEST(Selection, BagNames)
{
  EXPECT_EQ(rosbag_recorder::bag_name("/a", "ros"), "/a");
  EXPECT_EQ(rosbag_recorder::bag_name("/a", "flux"), "/a/flux");
}

TEST(Selection, Checks)
{
  EXPECT_EQ(error({topic("/a", "ros"), topic("/a", "flux"), topic("/b", "flux")}), "");
  EXPECT_EQ(error({topic("/a", "dds")}), "unknown transport: dds");
  EXPECT_EQ(error({topic("/b", "ros")}), "topic not available: /b");
  EXPECT_EQ(error({topic("/x", "flux")}), "topic not available: /x");
  EXPECT_EQ(error({topic("/b/flux", "ros"), topic("/b", "flux")}), "duplicate bag topic: /b/flux");
  EXPECT_EQ(error({topic("/a", "ros")}, false), "");
  EXPECT_EQ(
    error({topic("/a", "ros"), topic("/b", "flux")}, false),
    "transport flux is not available: rosbag_recorder was built without Flux "
    "(ENABLE_FLUX=1 at build time enables it)");
}
