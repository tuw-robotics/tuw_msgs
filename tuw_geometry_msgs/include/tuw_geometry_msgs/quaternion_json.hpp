#ifndef TUW_JSON__QUATERNION_JSON_HPP_
#define TUW_JSON__QUATERNION_JSON_HPP_

#include <geometry_msgs/msg/quaternion.hpp>
#include <jsoncpp/json/json.h>

namespace tuw_json
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
}
#endif  // TUW_JSON__QUATERNION_JSON_HPP_