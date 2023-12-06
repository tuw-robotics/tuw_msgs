#ifndef TUW_MSGS__NODE_HPP_
#define TUW_MSGS__NODE_HPP_

#include <tuw_graph_msgs/msg/node.hpp>
#include <tuw_msgs/utils.hpp>
#include <tuw_msgs/pose.hpp>

namespace tuw_msgs
{
  struct Node : public tuw_graph_msgs::msg::Node
  {
    Node();
    Node(Idx id);
    Node(Idx id, const Pose& pose);
    Node(const std::string &str);

    Node &set(Idx id, const Pose &pose);
    Pose &get_pose()
    {
      return static_cast<Pose &>(this->pose);
    }
    const Pose &get_pose() const
    {
      return static_cast<const Pose &>(this->pose);
    }
    tuw_graph_msgs::msg::Node &msg()
    {
      return static_cast<tuw_graph_msgs::msg::Node &>(*this);
    }
    const tuw_graph_msgs::msg::Node &msg() const
    {
      return static_cast<const tuw_graph_msgs::msg::Node &>(*this);
    }
    bool operator==(const Node &rhs) const;

    double similar(const Node &rhs, double threshold_position = 0.0001, double threshold_orientation = 0.0001) const;

    std::string to_str(tuw_msgs::Format format = LOOSE) const;
    std::string &to_str(std::string &des, tuw_msgs::Format format = LOOSE, bool append = false) const;
    size_t from_str(const std::string &src);
  };
}
#endif // TUW_MSGS__NODE_HPP_
