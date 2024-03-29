#ifndef TUW_JSON__EDGE_JSON_HPP_
#define TUW_JSON__EDGE_JSON_HPP_

#include <tuw_graph_msgs/msg/edge.hpp>
#include <tuw_graph_msgs/node_json.hpp>

namespace tuw_json
{
inline Json::Value toJson(const tuw_graph_msgs::msg::Edge & src)
{
  Json::Value json;
  json["id"] = src.id;
  json["valid"] = src.valid;
  json["weight"] = src.weight;
  json["start"] = src.start;
  json["end"] = src.end;
  Json::Value json_flags;
  for (const auto & f : src.flags) {
    json_flags.append(f);
  }
  json["flags"] = json_flags;
  Json::Value json_poses;
  for (const auto & p : src.path) {
    json_poses.append(toJson(p));
  }
  json["path"] = json_poses;
  return json;
}

inline tuw_graph_msgs::msg::Edge & fromJson(
  const Json::Value & json, tuw_graph_msgs::msg::Edge & des)
{
  des.id = json.get("id", "-1").asInt64();
  des.valid = json.get("valid", "").asBool();
  des.weight = json.get("weight", "").asDouble();
  des.start = json.get("start", "-1").asInt64();
  des.end = json.get("end", "-1").asInt64();
  if (json.isMember("flags") && json["flags"].isArray()) {
    const Json::Value & jsonArray = json["flags"];
    // Iterate through the array
    for (auto & j : jsonArray) {
      des.flags.push_back(j.asUInt());
    }
  }
  if (json.isMember("path") && json["path"].isArray()) {
    const Json::Value & jsonArray = json["path"];
    // Iterate through the array
    for (auto & j : jsonArray) {
      geometry_msgs::msg::Pose p;
      des.path.push_back(std::move(tuw_json::fromJson(j, p)));
    }
  }
  return des;
}
}  // namespace tuw_json

#endif  // TUW_JSON__EDGE_JSON_HPP_
