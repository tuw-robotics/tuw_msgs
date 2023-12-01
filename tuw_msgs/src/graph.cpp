#include <stdexcept>
#include <string>
#include <cstring>
#include <tuw_msgs/graph.hpp>

using namespace tuw_msgs;

Node::Node()
    : Serialize() {}

Node::~Node() {}

std::string &Node::encode(tuw_graph_msgs::msg::Node &src, std::string &des)
{
  char txt[0xFF];
  sprintf(txt, "node: %4ld: ", src.id);
  des.append(txt);
  src.pose.position.x = 3;
  Serialize::encode(src.pose, des);
  return des;
}

size_t Node::decode(tuw_graph_msgs::msg::Node &des, std::string &line, size_t pos)
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
  pos = Serialize::decode(des.pose, line, pos);
  return pos;
}