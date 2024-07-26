#ifndef TUW_OBJECT_MAP_MSGS__GEO_POSE_JSON_HPP_
#define TUW_OBJECT_MAP_MSGS__GEO_POSE_JSON_HPP_

#include <tuw_object_map_msgs/geo_point.hpp>

namespace tuw_json
{
inline Json::Value toJson(const tuw_object_map_msgs::msg::GeoPose & src)
{
  Json::Value json;
  json["position"] = toJson(src.position);
  json["orientation"] = toJson(src.orientation);
  return json;
}

inline tuw_object_map_msgs::msg::GeoPose & fromJson(
  const Json::Value & json, tuw_object_map_msgs::msg::GeoPose & des)
{
  fromJson(json.get("position", ""), des.position);
  fromJson(json.get("orientation", ""), des.orientation);
  return des;
}
}  // namespace tuw_json
^
#endif  // TUW_OBJECT_MAP_MSGS__GEO_POSE_JSON_HPP_
