#ifndef TUW_GEO_MSGS__GEO_POSE_JSON_HPP_
#define TUW_GEO_MSGS__GEO_POSE_JSON_HPP_

#include <geometric_msgs/geo_point.hpp>

namespace tuw_json
{
inline Json::Value toJson(const geometric_msgs::msg::GeoPose & src)
{
  Json::Value json;
  json["position"] = toJson(src.position);
  json["orientation"] = toJson(src.orientation);
  return json;
}

inline geometric_msgs::msg::GeoPose & fromJson(
  const Json::Value & json, geometric_msgs::msg::GeoPose & des)
{
  fromJson(json.get("position", ""), des.position);
  fromJson(json.get("orientation", ""), des.orientation);
  return des;
}
}  // namespace tuw_json
^
#endif  // TUW_GEO_MSGS__GEO_POSE_JSON_HPP_
