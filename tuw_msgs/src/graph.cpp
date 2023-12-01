#include <stdexcept>
#include <string>
#include <cstring>
#include <tuw_msgs/graph.hpp>

using namespace tuw_msgs;

    Node::Node(){}
    Node::Node(Idx id, const Pose &pose){
      set(id, pose);
    }
    void Node::set(Idx id, const Pose &pose){
      this->id = id, this->pose = pose;
    }
    Pose &Node::get_pose(){
      return static_cast<Pose&>(this->pose);
    }
    const Pose &Node::get_pose() const{
      return static_cast<const Pose&>(this->pose);
    }
    bool Node::operator==(const Node& rhs) const { 
      return (id == rhs.id) && (get_pose() == rhs.get_pose()); 
    }
    bool Edge::operator==(const Edge& rhs) const { 
      bool header_eq = (id == rhs.id) && (valid == rhs.valid) && (directed == rhs.directed) && (weight == rhs.weight);
      bool nodes_eq = (nodes[0] == rhs.nodes[0]) && (nodes[1] == rhs.nodes[1]);
      bool path_eq = path.size() == rhs.path.size();
      for(size_t i = 0; path_eq & (i < path.size()); i++ ){
        const Pose &a = static_cast<const Pose &>(path[i]);
        const Pose &b = static_cast<const Pose &>(rhs.path[i]);
        if(a != b) {
          path_eq = false;
          break;
        }
      }
      return header_eq && nodes_eq && path_eq; 
    }

std::string &tuw_msgs::encode(tuw_graph_msgs::msg::Node &src, std::string &des)
{
  char txt[0xFF];
  sprintf(txt, "node: %4ld: ", src.id);
  des.append(txt);
  src.pose.position.x = 3;
  encode(src.pose, des);
  return des;
}

size_t tuw_msgs::decode(tuw_graph_msgs::msg::Node &des, std::string &line, size_t pos)
{
  static constexpr const char* key_word = "node:";
  pos = line.find(key_word, pos);
  if (pos == std::string::npos)
    throw std::runtime_error("Failed decode Node key word in line: " + line);
  pos += std::strlen(key_word);
  const char *str = line.c_str() + pos;
  int n = sscanf(str, "%4ld:%*s", &des.id);
  if (n != 1)
  {
    throw std::runtime_error("Failed decode Node id in line " + line);
  }
  pos = decode(des.pose, line, pos);
  return pos;
}


std::string &tuw_msgs::encode(tuw_graph_msgs::msg::Edge &src, std::string &des)
{
  char txt[0x1FF]; 
  sprintf(txt, "edge:  %4ld: %s: %s: %8.4lf: [%4ld, %4ld]: ", src.id, (src.valid ? "  valid" : "invalid"), (src.directed ? "  directed" : "undirected"), src.weight, src.nodes[0], src.nodes[1]);
  des.append(txt);
  for (auto it = src.path.begin(); it != src.path.end();)
  {
      des.append(it == src.path.begin() ? "[" : "; ");
      encode(*it, des);
      des.append(++it == src.path.end() ? "]" : "");
  }
  return des;
}

size_t tuw_msgs::decode(tuw_graph_msgs::msg::Edge &des, std::string &line, size_t pos)
{

  static constexpr const char* key_word = "edge:";
  line.erase(std::remove_if(line.begin(), line.end(), [](unsigned char x) { return std::isspace(x); }), line.end());
  pos = line.find(key_word, pos);
  if (pos == std::string::npos)
    throw std::runtime_error("Failed decode Node key word in line: " + line);
  pos += std::strlen(key_word);
  const char *str = line.c_str() + pos;
  char str_valid[12], str_directed[12];
  int n = sscanf(str, "%ld:%[a-z]:%[a-z]:%lf:[%ld, %ld]",
                  &des.id, str_valid, str_directed, &des.weight, &des.nodes[0], &des.nodes[1]);
  if (n != 6)
  {
    throw std::runtime_error("Failed decode Edge header in line " + line);
  }
  des.valid = (strcmp(str_valid, "valid") == 0);
  des.directed = (strcmp(str_directed, "directed") == 0);
  pos = line.find("]:");
  if (pos == std::string::npos)
    throw std::runtime_error("Failed decode Edge in line: " + line);
  pos += std::strlen("]:");
  pos = line.find("[", pos);
  if (pos == std::string::npos)
    throw std::runtime_error("Failed decode Edge in line: " + line);
  pos++;
  str = line.c_str() + pos;
  while(line.at(pos) == '['){
    tuw_msgs::Pose pose;
    str = line.c_str() + pos;
    pos = decode(pose, line, pos);
    str = line.c_str() + pos;
    if(line.at(pos) == ';'){
      pos++;
    }
    if(line.at(pos) == ']'){
      break;
    }
  }

  str = line.c_str() + pos;
  return pos;
}