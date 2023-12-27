#ifndef TUW_JSON__NODE_JSON_HPP_
#define TUW_JSON__NODE_JSON_HPP_

#include <tuw_geometry_msgs/pose_json.hpp>
#include <tuw_graph_msgs/msg/node.hpp>

namespace tuw_json
{
inline Json::Value toJson(const tuw_graph_msgs::msg::Node & src)
{
  Json::Value json;
  json["id"] = src.id;
  json["valid"] = src.valid;
  json["pose"] = toJson(src.pose);
  Json::Value json_flags;
  for (const auto & f : src.flags) {
    json_flags.append(f);
  }
  json["flags"] = json_flags;
  return json;
}

inline tuw_graph_msgs::msg::Node & fromJson(
  const Json::Value & json, tuw_graph_msgs::msg::Node & des)
{
  des.id = json.get("id", "").asInt64();
  des.valid = json.get("valid", "").asBool();
  fromJson(json["pose"], des.pose);
  if (json.isMember("flags") && json["flags"].isArray()) {
    const Json::Value & jsonArray = json["flags"];
    // Iterate through the array
    for (auto & j : jsonArray) {
      des.flags.push_back(j.asInt());
    }
  }
  return des;
}
}  // namespace tuw_json
#endif  // TUW_JSON__NODE_JSON_HPP_
