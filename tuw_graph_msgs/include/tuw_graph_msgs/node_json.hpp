#ifndef TUW_JSON__NODE_JSON_HPP_
#define TUW_JSON__NODE_JSON_HPP_

#include <tuw_geometry_msgs/pose_json.hpp>
#include <tuw_graph_msgs/msg/node.hpp>


namespace tuw_json
{
inline Json::Value toJson(const tuw_graph_msgs::msg::Node &src)
{
  Json::Value json;
  json["id"] = src.id;
  json["valid"] = src.valid;
  json["pose"] = toJson(src.pose);
  return json;
}

inline tuw_graph_msgs::msg::Node &fromJson(const Json::Value & json, tuw_graph_msgs::msg::Node& des)
{
  des.id = json.get("id", "").asInt64();
  des.valid = json.get("valid", "").asBool();
  fromJson(json["pose"], des.pose);
  return des;
}
}
#endif  // TUW_JSON__NODE_JSON_HPP_