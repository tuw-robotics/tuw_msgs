#ifndef TUW_GRAPH_MSGS__NODE_JSON_HPP_
#define TUW_GRAPH_MSGS__NODE_JSON_HPP_

#include <tuw_geometry_msgs/pose_json.hpp>
#include <tuw_graph_msgs/msg/node.hpp>


namespace tuw_graph_msgs
{
inline Json::Value toJson(const tuw_graph_msgs::msg::Node &src)
{
  Json::Value json;
  json["id"] = src.id;
  json["valid"] = src.valid;
  json["pose"] = tuw_geometry_msgs::toJson(src.pose);
  return json;
}

inline tuw_graph_msgs::msg::Node &fromJson(const Json::Value & json, tuw_graph_msgs::msg::Node& des)
{
  des.id = json.get("id", "").asInt64();
  des.valid = json.get("valid", "").asBool();
  tuw_geometry_msgs::fromJson(json["pose"], des.pose);
  return des;
}
}
#endif  // TUW_GRAPH_MSGS__NODE_JSON_HPP_