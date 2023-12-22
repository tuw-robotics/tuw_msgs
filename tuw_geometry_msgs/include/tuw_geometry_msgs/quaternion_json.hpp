#ifndef TUW_GEOMETRY_MSGS__QUATERNION_JSON_HPP_
#define TUW_GEOMETRY_MSGS__QUATERNION_JSON_HPP_

#include <geometry_msgs/msg/quaternion.hpp>
#include <jsoncpp/json/json.h>

namespace tuw_geometry_msgs
{
inline Json::Value toJson(const geometry_msgs::msg::Quaternion &src)
{
  Json::Value json;
  json["x"] = src.x;
  json["y"] = src.y;
  json["z"] = src.z;
  json["w"] = src.w;
  return json;
}

inline geometry_msgs::msg::Quaternion &fromJson(const Json::Value & json, geometry_msgs::msg::Quaternion & des)
{
  des.x = json.get("x", "").asDouble();
  des.y = json.get("y", "").asDouble();
  des.z = json.get("z", "").asDouble();
  des.w = json.get("w", "").asDouble();
  return des;
}
geometry_msgs::msg::Quaternion fromJsonQuaternion(const Json::Value & json)
{
  geometry_msgs::msg::Quaternion o;
  return fromJson(json, o);
}
}  // namespace tuw_geometry_msgs

#endif  // TUW_GEOMETRY_MSGS__QUATERNION_JSON_HPP_