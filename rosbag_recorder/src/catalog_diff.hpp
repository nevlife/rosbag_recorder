#ifndef CATALOG_DIFF_HPP_
#define CATALOG_DIFF_HPP_

#include "config.hpp"

#include "rosbag_recorder_msgs/msg/catalog_diff_item.hpp"

#include <map>
#include <string>
#include <vector>

namespace rosbag_recorder
{

// Topics of `graph` (name to types) that are new or typed differently, and catalog topics that
// are missing from it. Publisher counts are left at zero.
std::vector<rosbag_recorder_msgs::msg::CatalogDiffItem> catalog_diff(
  const Catalog & catalog, const std::map<std::string, std::vector<std::string>> & graph);

}  // namespace rosbag_recorder

#endif  // CATALOG_DIFF_HPP_
