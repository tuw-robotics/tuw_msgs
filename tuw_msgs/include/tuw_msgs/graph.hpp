#ifndef TUW_MSGS__GRAPH_HPP_
#define TUW_MSGS__GRAPH_HPP_

#include <tuw_graph_msgs/msg/graph.hpp>
#include <tuw_msgs/edge.hpp>
#include <tuw_msgs/utils.hpp>

namespace tuw_msgs
{
class Graph : public tuw_graph_msgs::msg::Graph
{
public:
  Graph();
  Graph(const std::string & frame, const tuw_msgs::Pose & origin = tuw_msgs::Pose());
  bool operator==(const Graph & rhs) const;
  bool similar(
    const Graph & rhs, double epsilon_position = 0.0001, double epsilon_orientation = 0.0001) const;

  std::string & frame();
  const std::string & frame() const;
  Pose & get_origin();
  const Pose & get_origin() const;
  std::string to_str(tuw_msgs::Format format = LOOSE) const;
  std::string & to_str(
    std::string & des, tuw_msgs::Format format = LOOSE, bool append = false) const;
  size_t from_str(const std::string & src);
  template<class T>
  Graph & set_nodes(const T & src, bool append = false)
  {
    if (append == false) {this->nodes.clear();}
    for (auto & o : src) {
      this->nodes.push_back(o);
    }
    return *this;
  }
  template<class T>
  Graph & set_edges(const T & src, bool append = false)
  {
    if (append == false) {this->edges.clear();}
    for (auto & o : src) {
      this->edges.push_back(o);
    }
    return *this;
  }

  tuw_graph_msgs::msg::Graph & msg();
  const tuw_graph_msgs::msg::Graph & msg() const;
  void write(std::string filename, tuw_msgs::Format format = LOOSE) const;
  void read(std::string filename);
};      // class Graph
}       // namespace tuw_msgs
#endif  // TUW_MSGS__GRAPH_HPP_
