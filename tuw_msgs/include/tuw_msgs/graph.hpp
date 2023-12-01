#ifndef TUW_MSGS__GRAPH_HPP_
#define TUW_MSGS__GRAPH_HPP_

#include <tuw_graph_msgs/msg/node.hpp>
#include <tuw_msgs/serialize.hpp>

namespace tuw_msgs
{
  class Node : public Serialize, public tuw_graph_msgs::msg::Node
  {
  public:
    Node();
    ~Node();

    std::string &encode(tuw_graph_msgs::msg::Node &src, std::string &des);
    size_t decode(tuw_graph_msgs::msg::Node &des, std::string &line, size_t pos = 0);

  }; // namespace tuw_msgs
}
#endif // TUW_MSGS__GRAPH_HPP_
