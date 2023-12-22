#ifndef TUW_JSON__GRAPH_JSON_HPP_
#define TUW_JSON__GRAPH_JSON_HPP_

#include <fstream>
#include <iostream>
#include <stdexcept>
#include <tuw_graph_msgs/edge_json.hpp>
#include <tuw_graph_msgs/msg/graph.hpp>

namespace tuw_json
{
inline Json::Value toJson(const tuw_graph_msgs::msg::Graph & src)
{
  Json::Value json;
  json["frame_id"] = src.header.frame_id;
  json["origin"] = toJson(src.origin);
  Json::Value json_nodes;
  for (const auto & n : src.nodes) {
    json_nodes.append(toJson(n));
  }
  json["nodes"] = json_nodes;
  Json::Value json_edges;
  for (const auto & e : src.edges) {
    json_edges.append(toJson(e));
  }
  json["edges"] = json_edges;
  return json;
}

inline tuw_graph_msgs::msg::Graph & fromJson(
  const Json::Value & json, tuw_graph_msgs::msg::Graph & des)
{
  des.header.frame_id = json.get("frame_id", "-1").asCString();
  fromJson(json.get("origin", ""), des.origin);
  if (json.isMember("nodes") && json["nodes"].isArray()) {
    const Json::Value & jsonArray = json["nodes"];
    // Iterate through the array
    for (auto & j : jsonArray) {
      tuw_graph_msgs::msg::Node n;
      fromJson(j, n);
      des.nodes.push_back(std::move(n));
    }
  }
  if (json.isMember("edges") && json["edges"].isArray()) {
    const Json::Value & jsonArray = json["edges"];
    // Iterate through the array
    for (auto & j : jsonArray) {
      tuw_graph_msgs::msg::Edge e;
      fromJson(j, e);
      des.edges.push_back(std::move(e));
    }
  }
  return des;
}
}  // namespace tuw_json

#endif  // TUW_JSON__GRAPH_JSON_HPP_
