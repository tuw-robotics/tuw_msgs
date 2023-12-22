#ifndef TUW_GRAPH_MSGS__NODE_HPP_
#define TUW_GRAPH_MSGS__NODE_HPP_

#include <tuw_graph_msgs/msg/node.hpp>
#include <tuw_geometry_msgs/pose.hpp>

namespace tuw_graph_msgs
{
struct Node : public tuw_graph_msgs::msg::Node
{
  Node();
  Node(int64_t id) {
    this->id = id;
  }
  Node(int64_t id, const geometry_msgs::msg::Pose &pose) {
    this->id = id;
    this->pose = pose;
  }
};
}  // namespace tuw_msgs
#endif  // TUW_GRAPH_MSGS__NODE_HPP_
