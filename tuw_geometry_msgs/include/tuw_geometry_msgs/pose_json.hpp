#ifndef TUW_JSON__POSE_JSON_HPP_
#define TUW_JSON__POSE_JSON_HPP_

#include <geometry_msgs/msg/pose.hpp>
#include <tuw_geometry_msgs/point_json.hpp>
#include <tuw_geometry_msgs/quaternion_json.hpp>

namespace tuw_json
{
inline Json::Value toJson(const geometry_msgs::msg::Pose & src)
{
  Json::Value json;
  json["position"] = toJson(src.position);
  json["orientation"] = toJson(src.orientation);
  return json;
}

inline geometry_msgs::msg::Pose & fromJson(const Json::Value & json, geometry_msgs::msg::Pose & des)
{
  if (json.isMember("position")) {
    fromJson(json.get("position", ""), des.position);
  } else {
    des.position.x = 0.;
    des.position.y = 0.;
    des.position.z = 0.;
  }
  if (json.isMember("orientation")) {
    fromJson(json.get("orientation", ""), des.orientation);
  } else {
    des.orientation.x = 0.;
    des.orientation.y = 0.;
    des.orientation.z = 0.;
    des.orientation.w = 1.;
  }
  return des;
}

inline Json::Value toJson(const std::vector<geometry_msgs::msg::Pose> & src)
{
  Json::Value des;
  for (const auto & o : src) {
    des.append(tuw_json::toJson(o));
  }
  return des;
}

inline std::vector<geometry_msgs::msg::Pose> & fromJson(
  const Json::Value & json, const std::string & key, std::vector<geometry_msgs::msg::Pose> & des)
{
  if (json.isMember(key) && json[key].isArray()) {
    const Json::Value & jsonArray = json[key];
    for (auto & j : jsonArray) {
      geometry_msgs::msg::Pose o;
      des.push_back(std::move(tuw_json::fromJson(j, o)));
    }
  }
  return des;
}
}  // namespace tuw_json
#endif  // TUW_JSON__POSE_JSON_HPP_
