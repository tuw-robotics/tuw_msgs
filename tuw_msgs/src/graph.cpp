#include <stdexcept>
#include <string>
#include <cstring>
#include <tuw_msgs/graph.hpp>

using namespace tuw_msgs;

Graph::Graph() {}
bool Graph::operator==(const Graph &rhs) const
{
  bool edges_eq = this->edges.size() == rhs.edges.size();
  bool nodes_eq = this->nodes.size() == rhs.nodes.size();
  return true;
}
