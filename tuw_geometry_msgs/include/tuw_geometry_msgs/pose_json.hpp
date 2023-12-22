#ifndef TUW_GEOMETRY_MSGS__POSE_JSON_HPP_
#define TUW_GEOMETRY_MSGS__POSE_JSON_HPP_

#include <tuw_geometry_msgs/point_json.hpp>
#include <tuw_geometry_msgs/quaternion_json.hpp>
#include <geometry_msgs/msg/pose.hpp>

namespace tuw_geometry_msgs
{
inline Json::Value toJson(const geometry_msgs::msg::Pose &src)
{
  Json::Value json;
  json["position"] = toJson(src.position);
  json["orientation"] = toJson(src.orientation);
  return json;
}

inline geometry_msgs::msg::Pose &fromJson(const Json::Value &json, geometry_msgs::msg::Pose &des)
{
  fromJson(json.get("position", ""), des.position);
  fromJson(json.get("orientation", ""), des.orientation); 
  return des;
}
}
#endif  // TUW_GEOMETRY_MSGS__POSE_JSON_HPP_