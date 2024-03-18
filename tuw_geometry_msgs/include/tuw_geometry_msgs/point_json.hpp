#ifndef TUW_JSON__POINT_JSON_HPP_
#define TUW_JSON__POINT_JSON_HPP_

#include <json/json.h>

#include <geometry_msgs/msg/point.hpp>

namespace tuw_json
{
inline Json::Value toJson(const geometry_msgs::msg::Point & src)
{
  Json::Value json;
  json["x"] = src.x;
  json["y"] = src.y;
  json["z"] = src.z;
  return json;
}

inline geometry_msgs::msg::Point & fromJson(
  const Json::Value & json, geometry_msgs::msg::Point & des)
{
  des.x = json.get("x", "").asDouble();
  des.y = json.get("y", "").asDouble();
  des.z = json.get("z", "").asDouble();
  return des;
}
}  // namespace tuw_json

#endif  // TUW_JSON__POINT_JSON_HPP_
