#include "catalog_diff.hpp"

#include <map>
#include <string>
#include <vector>

namespace rosbag_recorder
{

using rosbag_recorder_msgs::msg::CatalogDiffItem;

std::vector<CatalogDiffItem> catalog_diff(
  const Catalog & catalog, const std::map<std::string, std::vector<std::string>> & graph)
{
  std::vector<CatalogDiffItem> items;
  for (const auto & [topic, types] : graph) {
    const TopicSpec * spec = catalog.topic(topic);
    CatalogDiffItem item;
    item.topic = topic;
    item.graph_type = types.front();
    if (!spec) {
      item.kind = CatalogDiffItem::NEW;
      items.push_back(item);
    } else if (spec->type && *spec->type != item.graph_type) {
      item.kind = CatalogDiffItem::TYPE_CHANGED;
      item.catalog_type = *spec->type;
      items.push_back(item);
    }
  }
  for (const auto & [topic, spec] : catalog.topics) {
    if (!graph.count(topic)) {
      CatalogDiffItem item;
      item.kind = CatalogDiffItem::MISSING;
      item.topic = topic;
      item.catalog_type = spec.type.value_or("");
      items.push_back(item);
    }
  }
  return items;
}

}  // namespace rosbag_recorder
