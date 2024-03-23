#pragma once

#include <json/json.h>

#include "tuw_object_map_msgs/geo_json_feature_json.hpp"
#include "tuw_object_map_msgs/msg/geo_json.hpp"

namespace tuw_json
{
inline tuw_object_map_msgs::msg::GeoJSON & fromJson(
  const Json::Value & json, tuw_object_map_msgs::msg::GeoJSON & des)
{
  des.type = json.get("type", "").asCString();

  if (json.isMember("features") && json["features"].isArray()) {
    const Json::Value & featuresArray = json["features"];
    for (auto & featureJson : featuresArray) {
      tuw_object_map_msgs::msg::GeoJSONFeature feature;
      des.features.push_back(std::move(tuw_json::fromJson(featureJson, feature)));
    }
  }

  return des;
}
}  // namespace tuw_json
