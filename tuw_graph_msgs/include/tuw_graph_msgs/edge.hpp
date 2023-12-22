#ifndef TUW_GRAPH_MSGS__EDGE_HPP_
#define TUW_GRAPH_MSGS__EDGE_HPP_

#include <tuw_graph_msgs/msg/node.hpp>

namespace tuw_graph_msgs
{
struct Edge : public tuw_graph_msgs::msg::Edge
{
  Edge();
  Edge(int64_t id) {
    this->id = id;
  }
  Edge(int64_t id, bool valid, bool directed, double weight, size_t idx_start, size_t idx_end){
    this->id = id, this->valid = valid, this->directed = directed, this->weight = weight, this->nodes[0] = idx_start, this->nodes[1] = idx_end;
  }
};
}  // namespace tuw_graph_msgs
#endif  // TUW_GRAPH_MSGS__EDGE_HPP_
