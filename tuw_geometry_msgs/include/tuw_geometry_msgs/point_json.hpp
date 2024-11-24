#ifndef TUW_JSON__POINT_JSON_HPP_
#define TUW_JSON__POINT_JSON_HPP_

#include <json/json.h>
#include <tuw_geometry_msgs/point.hpp>

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
  if (json.isMember("z")) {
    des.z = json.get("z", "").asDouble();
  } else {
    des.z = 0.0;
  }
  return des;
}

inline Json::Value toJson(const std::vector<geometry_msgs::msg::Point> & src)
{
  Json::Value des;
  for (const auto & o : src) {
    des.append(tuw_json::toJson(o));
  }
  return des;
}

inline std::vector<geometry_msgs::msg::Point> & fromJson(
  const Json::Value & json, const std::string & key, std::vector<geometry_msgs::msg::Point> & des)
{
  if (json.isMember(key) && json[key].isArray()) {
    const Json::Value & jsonArray = json[key];
    for (auto & j : jsonArray) {
      geometry_msgs::msg::Point o;
      des.push_back(std::move(tuw_json::fromJson(j, o)));
    }
  }
  return des;
}
}  // namespace tuw_json

#endif  // TUW_JSON__POINT_JSON_HPP_
