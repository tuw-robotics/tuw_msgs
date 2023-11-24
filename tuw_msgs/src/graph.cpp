#include <algorithm>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuw_msgs/graph.hpp>

using namespace tuw_msgs;

Graph::Graph()
: Serialize() {}

Graph::~Graph() {}

int Graph::read(std::string const & filename, tuw_graph_msgs::msg::Graph & graph)
{
  std::ifstream file(filename, std::ios_base::binary | std::ios_base::in);
  if (!file.is_open()) {throw std::runtime_error("Failed to open " + filename);}
  file.open(filename);
  std::string line;
  if (file.is_open()) {
    long line_number = 0;
    using std::operator""sv;
    constexpr auto delim{";"sv};
    while (file) {
      line_number++;
      std::getline(file, line);
      /// trim all spaces
      line.erase(
        std::remove_if(line.begin(), line.end(), [](unsigned char x) {return std::isspace(x);}),
        line.end());
      if (line.empty() || (line.rfind("#", 0) == 0)) {
        continue;
      } else if (check("frame:", line, true)) {
        if (decode_value(line, graph.header.frame_id) == DECODE_SUCCESSFUL) {
          continue;
        } else {
          return line_number;
        }
      } else if (check("node:", line, true)) {
        tuw_graph_msgs::msg::Node node;
        if (decode_node(line, node) == DECODE_SUCCESSFUL) {
          graph.nodes.push_back(std::move(node));
        } else {
          return line_number;
        }
      } else if (check("edge:", line, true)) {
        tuw_graph_msgs::msg::Edge edge;
        if (decode_edge(line, edge) == DECODE_SUCCESSFUL) {
          graph.edges.push_back(std::move(edge));
        } else {
          return line_number;
        }
      } else {
        return line_number;
      }
    }
    file.close();
  }
  return DECODE_SUCCESSFUL;
}

int Graph::decode_edge(const std::string & line, tuw_graph_msgs::msg::Edge & edge)
{
  static const int ID = 0;
  static const int VALID = 1;
  static const int DIRECTED = 2;
  static const int WEIGHT = 3;
  static const int NODES = 4;
  static const int PATH = 5;
  std::vector<std::string> entries;
  split(entries, line, delim_entries);
  try {
    edge.id = std::stoi(entries[ID]);
  } catch (const std::invalid_argument & ia) {
    return ID;
  }
  if (decode_value(entries[VALID], edge.valid) != DECODE_SUCCESSFUL) {return VALID;}
  if (decode_value(entries[DIRECTED], edge.directed) != DECODE_SUCCESSFUL) {return DIRECTED;}
  if (decode_value(entries[WEIGHT], edge.weight) != DECODE_SUCCESSFUL) {return WEIGHT;}
  if (decode_list(entries[NODES], edge.nodes) != DECODE_SUCCESSFUL) {return NODES;}
  if (decode_list(entries[PATH], edge.path) != DECODE_SUCCESSFUL) {return PATH;}

  return DECODE_SUCCESSFUL;
}

int Graph::decode_node(const std::string & line, tuw_graph_msgs::msg::Node & node)
{
  static const int ID = 0;
  static const int POSITION = 1;

  std::vector<std::string> entries;
  split(entries, line, delim_entries);

  try {
    node.id = std::stoi(entries[ID]);
  } catch (const std::invalid_argument & ia) {
    return ID;
  }
  if (decode_value(entries[POSITION], node.position) != DECODE_SUCCESSFUL) {
    return POSITION;
  }
  return DECODE_SUCCESSFUL;
}
