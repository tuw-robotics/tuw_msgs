#include <tuw_msgs/node.hpp>

using namespace tuw_msgs;

Node::Node() {}

Node::Node(Idx id) {this->id = id;}

Node::Node(Idx id, const Pose & pose) {set(id, pose);}

Node::Node(const std::string & str) {this->from_str(str);}

Node & Node::set(Idx id, const Pose & pose)
{
  this->id = id;
  this->pose = pose;
  return *this;
}

bool Node::operator==(const Node & rhs) const
{
  return (this->id == rhs.id) && (get_pose() == rhs.get_pose());
}

bool Node::similar(const Node & rhs, double epsilon_position, double epsilon_orientation) const
{
  return is_similar(*this, rhs, epsilon_position, epsilon_orientation);
}
bool tuw_msgs::is_similar(
  const tuw_graph_msgs::msg::Node & a, const tuw_graph_msgs::msg::Node & b, double epsilon_position,
  double epsilon_orientation)
{
  return (a.id == b.id) && is_similar(a.pose, b.pose, epsilon_position, epsilon_orientation);
}
std::string Node::to_str(tuw_msgs::Format format) const
{
  std::string str;
  return this->to_str(str, format);
}
std::string & Node::to_str(std::string & des, tuw_msgs::Format format, bool append) const
{
  char txt[128];
  sprintf(txt, "%4ld: %s", id, this->get_pose().to_str(format).c_str());
  if (!append) {
    des.clear();
  }
  des.append(txt);
  return des;
}
size_t Node::from_str(const std::string & str)
{
  int n = sscanf(str.c_str(), "%4ld:%*s", &this->id);
  if (n != 1) {
    throw std::runtime_error("Failed decode Node: " + str);
  }
  size_t offset = str.rfind(":");
  if (offset == std::string::npos) {
    throw std::runtime_error("Failed decode node: " + str);
  }

  offset = this->get_pose().from_str(str.substr(offset)) + offset;
  return offset;
}

#include <jsoncpp/json/json.h>
Json::Value Node::toJson() const
{
  Json::Value json;
  json["id"] = this->id;
  json["valid"] = this->valid;
  json["pose"] = get_pose().toJson();
  return json;
}
Node & Node::fromJson(const Json::Value & json, Node & o)
{
  o.id = json.get("id", "-1").asInt64();
  o.valid = json.get("valid", "").asBool();
  Pose::fromJson(json.get("pose", ""), o.get_pose());
  return o;
}

Node Node::fromJson(const Json::Value & json)
{
  Node o;
  return fromJson(json, o);
}
