#include <cstring>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <tuw_msgs/graph.hpp>

using namespace tuw_msgs;

Graph::Graph() {}
Graph::Graph(const std::string & frame, const tuw_msgs::Pose & origin)
{
  this->header.frame_id = frame;
  this->origin = origin;
}
std::string & Graph::frame() {return this->header.frame_id;}
const std::string & Graph::frame() const {return this->header.frame_id;}

Pose & Graph::get_origin() {return static_cast<Pose &>(this->origin);}
const Pose & Graph::get_origin() const {return static_cast<const Pose &>(this->origin);}

bool Graph::operator==(const Graph & rhs) const
{
  if (this->header.frame_id != rhs.header.frame_id) {return false;}
  if (this->origin != rhs.origin) {return false;}
  if (this->nodes != rhs.nodes) {return false;}
  if (this->edges != rhs.edges) {return false;}
  return true;
}

bool Graph::similar(const Graph & rhs, double epsilon_position, double epsilon_orientation) const
{
  if (this->header.frame_id != rhs.header.frame_id) {return false;}
  if (!this->get_origin().similar(rhs.get_origin(), epsilon_position, epsilon_orientation)) {
    return false;
  }
  {
    auto it_a = this->nodes.begin();
    auto it_b = rhs.nodes.begin();
    if (this->nodes.size() != rhs.nodes.size()) {return false;}
    while (it_a != this->nodes.end()) {
      if (!static_cast<const Node &>(*it_a).similar(
          static_cast<const Node &>(*it_b), epsilon_position, epsilon_orientation))
      {
        return false;
      }
      it_a++;
      it_b++;
    }
  }

  {
    auto it_a = this->edges.begin();
    auto it_b = rhs.edges.begin();
    if (this->edges.size() != this->edges.size()) {return false;}
    while (it_a != this->edges.end()) {
      if (!static_cast<const Edge &>(*it_a).similar(
          static_cast<const Edge &>(*it_b), epsilon_position, epsilon_orientation))
      {
        return false;
      }
      it_a++;
      it_b++;
    }
  }
  return true;
}

std::string Graph::to_str(tuw_msgs::Format format) const
{
  std::string str;
  return this->to_str(str, format);
}
std::string & Graph::to_str(std::string & des, tuw_msgs::Format format, bool append) const
{
  if (!append) {des.clear();}
  (void)format;
  std::stringstream ss;
  {
    ss << "# graph" << std::endl;
    ss << "frame: " << (frame().empty() ? "map" : frame()) << std::endl;
    ss << "origin: " << get_origin().to_str(format) << std::endl;
    ss << std::endl;
    ss << "# nodes: id: position" << std::endl;
    for (const auto & node : this->nodes) {
      ss << "node: " << static_cast<const Node &>(node).to_str(format) << std::endl;
    }
    ss << std::endl;
    ss << "# edges: id: valid: directed: weight: nodes: path" << std::endl;
    for (const auto & edge : this->edges) {
      ss << "edge: " << static_cast<const Edge &>(edge).to_str(format) << std::endl;
    }
  }
  return des.append(ss.str());
}
size_t Graph::from_str(const std::string & src)
{
  std::istringstream in_stream(src);
  std::string line;
  long line_number = 0;
  while (std::getline(in_stream, line)) {
    line_number++;
    size_t pos = 0;
    if (line.empty() || (line.rfind("#", 0) == 0)) {
      continue;
    } else if ((pos = line.find("frame:")) != std::string::npos) {
      // frame
      pos = pos + strlen("frame:") + 1;
      this->header.frame_id = remove_spaces(line.substr(pos));
    } else if ((pos = line.find("origin:")) != std::string::npos) {
      // origin
      pos = pos + strlen("origin:") + 1;
      this->get_origin().from_str(line.substr(pos));
    } else if ((pos = line.find("node:")) != std::string::npos) {
      Node node;
      pos = pos + strlen("node:") + 1;
      node.from_str(line.substr(pos));
      this->nodes.push_back(std::move(node));
    } else if ((pos = line.find("edge:")) != std::string::npos) {
      Edge edge;
      pos = pos + strlen("edge:") + 1;
      edge.from_str(line.substr(pos));
      this->edges.push_back(std::move(edge));
    }
  }
  return line_number;
}

void Graph::read(std::string filename)
{
  std::ifstream file(filename, std::ios_base::binary | std::ios_base::in);
  if (!file.is_open()) {throw std::runtime_error("Failed to open " + filename);}
  using Iterator = std::istreambuf_iterator<char>;
  std::string text(Iterator{file}, Iterator{});
  if (!file) {throw std::runtime_error("Failed to read " + filename);}
  this->from_str(text);
  file.close();
}

void Graph::write(std::string filename, tuw_msgs::Format format) const
{
  std::ofstream file;
  file.open(filename, std::ios::binary);
  if (!file.is_open()) {throw std::runtime_error("Failed to open " + filename);}
  file << this->to_str(format);
  file.close();
}
tuw_graph_msgs::msg::Graph & Graph::msg()
{
  return static_cast<tuw_graph_msgs::msg::Graph &>(*this);
}
const tuw_graph_msgs::msg::Graph & Graph::msg() const
{
  return static_cast<const tuw_graph_msgs::msg::Graph &>(*this);
}
