#ifndef TUW_GRAPH_MSGS__GRAPH_JSON_HPP_
#define TUW_GRAPH_MSGS__GRAPH_JSON_HPP_

#include <fstream>
#include <iostream>
#include <stdexcept>
#include <tuw_graph_msgs/edge_json.hpp>
#include <tuw_graph_msgs/msg/graph.hpp>


namespace tuw_graph_msgs
{
inline Json::Value toJson(const tuw_graph_msgs::msg::Graph &src)
{

  Json::Value json;
  json["frame_id"] = src.header.frame_id;
  json["origin"] = tuw_geometry_msgs::toJson(src.origin);
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

inline tuw_graph_msgs::msg::Graph &fromJson(const Json::Value & json, tuw_graph_msgs::msg::Graph& des)
{
  des.header.frame_id = json.get("frame_id", "-1").asCString();
  tuw_geometry_msgs::fromJson(json.get("origin", ""), des.origin);
  if (json.isMember("nodes") && json["nodes"].isArray()) {
    const Json::Value & jsonArray = json["nodes"];
    // Iterate through the array
    for (auto &j: jsonArray) {
      tuw_graph_msgs::msg::Node n;
      fromJson(j,n);
      des.nodes.push_back(std::move(n));
    }
  }
  if (json.isMember("edges") && json["edges"].isArray()) {
    const Json::Value & jsonArray = json["edges"];
    // Iterate through the array
    for (auto &j: jsonArray) {
      tuw_graph_msgs::msg::Edge e;
      fromJson(j,e);
      des.edges.push_back(std::move(e));
    }
  }
  return des;
}

inline void writeJson(const std::string &filename, const tuw_graph_msgs::msg::Graph &src)
{
  Json::Value json_data;
  json_data["graph"] = toJson(src);
  Json::StreamWriterBuilder writerBuilder;
  writerBuilder.settings_["indentation"] = " ";  // Disable indentation
  writerBuilder.settings_["sortKeys"] = false;

  std::string json_str = Json::writeString(writerBuilder, json_data);
  std::ofstream json_file(filename);
  if (json_file.is_open()) {
    json_file << json_str;
    json_file.close();
  } else {
    throw std::runtime_error("Failed to write json file " + filename);
  }
}

inline void readJson(const std::string &filename, tuw_graph_msgs::msg::Graph &des)
{
  std::ifstream file(filename);
  if (!file.is_open()) {
    throw std::runtime_error("Failed to open json file " + filename);
  }
  std::string jsonString((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
  file.close();

  // Parse the JSON string
  Json::Value root;
  Json::Reader reader;

  if (reader.parse(jsonString, root)) {
    if (root.isMember("graph")) {
      const Json::Value & json_graph = root["graph"];
      fromJson(json_graph, des);
    }
  } else {
    throw std::runtime_error("Failed to parse json file " + filename);
  }
}
}

#endif  // TUW_GRAPH_MSGS__GRAPH_JSON_HPP_