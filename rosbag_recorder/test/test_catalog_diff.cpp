#include "catalog_diff.hpp"

#include <gtest/gtest.h>

#include <map>
#include <string>
#include <vector>

using rosbag_recorder_msgs::msg::CatalogDiffItem;

TEST(CatalogDiff, NewMissingAndTypeChanged)
{
  const auto catalog = rosbag_recorder::Catalog::parse(
    YAML::Load("groups: {a: {topics: {/same: {type: std_msgs/msg/String}, /typed: {type: "
               "std_msgs/msg/String},"
               " /gone: {type: std_msgs/msg/Bool}, /untyped: {}}}}"),
    "catalog");
  const std::map<std::string, std::vector<std::string>> graph{
    {"/same", {"std_msgs/msg/String"}},
    {"/typed", {"std_msgs/msg/Int32"}},
    {"/untyped", {"std_msgs/msg/Int32"}},
    {"/new", {"std_msgs/msg/Empty"}}};
  const auto items = rosbag_recorder::catalog_diff(catalog, graph);
  ASSERT_EQ(items.size(), 3U);
  EXPECT_EQ(items[0].topic, "/new");
  EXPECT_EQ(items[0].kind, CatalogDiffItem::NEW);
  EXPECT_EQ(items[0].graph_type, "std_msgs/msg/Empty");
  EXPECT_EQ(items[1].topic, "/typed");
  EXPECT_EQ(items[1].kind, CatalogDiffItem::TYPE_CHANGED);
  EXPECT_EQ(items[1].catalog_type, "std_msgs/msg/String");
  EXPECT_EQ(items[2].topic, "/gone");
  EXPECT_EQ(items[2].kind, CatalogDiffItem::MISSING);
  EXPECT_EQ(items[2].catalog_type, "std_msgs/msg/Bool");
}
