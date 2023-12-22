#ifndef TUW_GRAPH_MSGS__GRAPH_HPP_
#define TUW_GRAPH_MSGS__GRAPH_HPP_

#include <tuw_graph_msgs/msg/graph.hpp>
#include <tuw_graph_msgs/edge.hpp>
#include <tuw_graph_msgs/node.hpp>
#include <tuw_geometry_msgs/pose.hpp>

namespace tuw_graph_msgs
{
struct Graph : public tuw_graph_msgs::msg::Graph
{
  Graph(){};
  Graph(const std::string & frame, const geometry_msgs::msg::Pose &origin = tuw_geometry_msgs::Pose()){
    this->header.frame_id = frame;
    this->origin = origin;
  }
};
}  // namespace tuw_graph_msgs
#endif  // TUW_GRAPH_MSGS__GRAPH_HPP_
