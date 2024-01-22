#ifndef TUW_GRAPH_MSGS__EDGE_HPP_
#define TUW_GRAPH_MSGS__EDGE_HPP_

#include <tuw_graph_msgs/msg/edge.hpp>

namespace tuw_graph_msgs
{
struct Edge : public tuw_graph_msgs::msg::Edge
{
  Edge();
  Edge(int64_t id) {this->id = id;}
  Edge(int64_t id, bool valid, double weight, size_t idx_start, size_t idx_end)
  {
    this->id = id, this->valid = valid, this->weight = weight, this->start = idx_start,
    this->end = idx_end;
  }
};
}  // namespace tuw_graph_msgs
#endif  // TUW_GRAPH_MSGS__EDGE_HPP_
