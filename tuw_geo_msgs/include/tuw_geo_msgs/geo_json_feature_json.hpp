#ifndef TUW_GEO_MSGS_MSGS__GEO_JSON_FEATURE_JSON_HPP_
#define TUW_GEO_MSGS_MSGS__GEO_JSON_FEATURE_JSON_HPP_

#include <json/json.h>

#include "tuw_geo_msgs/geo_json_geometry_json.hpp"
#include "tuw_geo_msgs/msg/geo_json_feature.hpp"

namespace tuw_json
{
inline tuw_geo_msgs::msg::GeoJSONFeature & fromJson(
  const Json::Value & json, tuw_geo_msgs::msg::GeoJSONFeature & des)
{
  des.type = json.get("type", "").asCString();

  const Json::Value & geometryJson = json["geometry"];
  tuw_geo_msgs::msg::GeoJSONGeometry geometry;
  des.geometry = std::move(tuw_json::fromJson(geometryJson, geometry));

  return des;
}
}  // namespace tuw_json
#endif  // #define TUW_GEO_MSGS_MSGS__GEO_JSON_FEATURE_JSON_HPP_
