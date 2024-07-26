#ifndef TUW_GEO_MSGS_MSGS__GEO_JSON_JSON_HPP_
#define TUW_GEO_MSGS_MSGS__GEO_JSON_JSON_HPP_

#include <json/json.h>

#include "tuw_geo_msgs/geo_json_feature_json.hpp"
#include "tuw_geo_msgs/msg/geo_json.hpp"

namespace tuw_json
{
inline tuw_geo_msgs::msg::GeoJSON & fromJson(
  const Json::Value & json, tuw_geo_msgs::msg::GeoJSON & des)
{
  des.type = json.get("type", "").asCString();

  if (json.isMember("features") && json["features"].isArray()) {
    const Json::Value & featuresArray = json["features"];
    for (auto & featureJson : featuresArray) {
      tuw_geo_msgs::msg::GeoJSONFeature feature;
      des.features.push_back(std::move(tuw_json::fromJson(featureJson, feature)));
    }
  }

  return des;
}
}  // namespace tuw_json
#endif  // TUW_GEO_MSGS_MSGS__GEO_JSON_JSON_HPP_
