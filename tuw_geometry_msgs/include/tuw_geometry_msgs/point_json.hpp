#ifndef TUW_GEOMETRY_MSGS__POINT_JSON_HPP_
#define TUW_GEOMETRY_MSGS__POINT_JSON_HPP_

#include <geometry_msgs/msg/point.hpp>
#include <jsoncpp/json/json.h>

namespace tuw_geometry_msgs
{
inline Json::Value toJson(const geometry_msgs::msg::Point &src)
{
  Json::Value json;
  json["x"] = src.x;
  json["y"] = src.y;
  json["z"] = src.z;
  return json;
}

inline geometry_msgs::msg::Point &fromJson(const Json::Value & json, geometry_msgs::msg::Point & des)
{
  des.x = json.get("x", "").asDouble();
  des.y = json.get("y", "").asDouble();
  des.z = json.get("z", "").asDouble();
  return des;
}
}

#endif  // TUW_GEOMETRY_MSGS__JSON_HPP_