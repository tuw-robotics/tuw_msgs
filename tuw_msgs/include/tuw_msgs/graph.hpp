#ifndef TUW_MSGS__GRAPH_HPP_
#define TUW_MSGS__GRAPH_HPP_

#include <tuw_graph_msgs/msg/graph.hpp>
#include <tuw_msgs/utils.hpp>

namespace tuw_msgs
{
  class Graph : public tuw_graph_msgs::msg::Graph
  {
  public:
    Graph();
    bool operator==(const Graph& rhs) const;
  }; // class Graph
};
#endif // TUW_MSGS__GRAPH_HPP_
