#ifndef TUW_MSGS__EDGE_HPP_
#define TUW_MSGS__EDGE_HPP_

#include <tuw_graph_msgs/msg/edge.hpp>
#include <tuw_msgs/node.hpp>

namespace tuw_msgs
{
struct Edge : public tuw_graph_msgs::msg::Edge
{
  Edge();
  Edge(Idx id);
  Edge(Idx id, bool valid, bool directed, double weight, size_t idx_start, size_t idx_end);
  template<class T>
  Edge(
    Idx id, bool valid, bool directed, double weight, size_t idx_start, size_t idx_end,
    const T & path, bool append = false)
  {
    this->set(id, valid, directed, weight, idx_start, idx_end);
    this->set_path(path, append);
  }
  Edge(Idx id, size_t idx_start, size_t idx_end, bool valid = true);
  Edge(const std::string & str);

  Edge & set(Idx id, bool valid, bool directed, double weight, size_t idx_start, size_t idx_end);
  Edge & set(Idx id, size_t idx_start, size_t idx_end, bool valid = true);

  template<class T>
  Edge & set_path(const T & src, bool append = false)
  {
    if (append == false) {this->path.clear();}
    for (auto & pose : src) {
      this->path.push_back(pose);
    }
    return *this;
  }

  tuw_graph_msgs::msg::Edge & msg();
  const tuw_graph_msgs::msg::Edge & msg() const;
  bool operator==(const Edge & rhs) const;

  double similar(
    const Edge & rhs, double epsilon_position = 0.0001, double epsilon_orientation = 0.0001) const;

  std::string to_str(tuw_msgs::Format format = LOOSE) const;
  std::string & to_str(
    std::string & des, tuw_msgs::Format format = LOOSE, bool append = false) const;
  size_t from_str(const std::string & src);

  Json::Value toJson() const;
  static Edge fromJson(const Json::Value& jsonValue);
  static Edge &fromJson(const Json::Value& json, Edge &des);
};
bool is_similar(
  const tuw_graph_msgs::msg::Edge & a, const tuw_graph_msgs::msg::Edge & b,
  double epsilon_position = 0.0001, double epsilon_orientation = 0.0001);
}  // namespace tuw_msgs
#endif  // TUW_MSGS__EDGE_HPP_
