#ifndef TUW_GRAPH_MSGS__NODE_HPP_
#define TUW_GRAPH_MSGS__NODE_HPP_

#include <tuw_graph_msgs/msg/node.hpp>
#include <tuw_msgs/pose.hpp>
#include <tuw_msgs/utils.hpp>

namespace tuw_graph_msgs
{
struct Node : public tuw_graph_msgs::msg::Node
{
  Node();
  Node(Idx id) {
    this->id = id;
  }
};
}  // namespace tuw_msgs
#endif  // TUW_GRAPH_MSGS__NODE_HPP_
