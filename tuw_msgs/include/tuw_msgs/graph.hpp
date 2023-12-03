#ifndef TUW_MSGS__GRAPH_HPP_
#define TUW_MSGS__GRAPH_HPP_

#include <tuw_graph_msgs/msg/graph.hpp>
#include <tuw_msgs/serialize.hpp>
#include <tuw_msgs/utils.hpp>

namespace tuw_msgs
{
  using Idx = int64_t;
  class Node : public tuw_graph_msgs::msg::Node
  {
  public:
    Node();
    Node(Idx id, const Pose &pose);
    void set(Idx id, const Pose &pose);
    Pose &get_pose();
    const Pose &get_pose() const;
    bool operator==(const Node& rhs) const;
  }; // class Node


  class Edge : public tuw_graph_msgs::msg::Edge
  {
  public:
    Edge();
    bool operator==(const Edge& rhs) const;
  }; // class Edge


  class Graph : public tuw_graph_msgs::msg::Graph
  {
  public:
    Graph();
    bool operator==(const Graph& rhs) const;
  }; // class Graph
  
    std::string &encode(tuw_graph_msgs::msg::Node &src, std::string &des);
    size_t decode(tuw_graph_msgs::msg::Node &des, std::string &line, size_t pos = 0);
    std::string &encode(tuw_graph_msgs::msg::Edge &src, std::string &des);
    size_t decode(tuw_graph_msgs::msg::Edge &des, std::string &line, size_t pos = 0);
};
#endif // TUW_MSGS__GRAPH_HPP_
