#include <stdio.h>
#include <string.h>

#include <tuw_msgs/edge.hpp>

using namespace tuw_msgs;

Edge::Edge() {}

Edge::Edge(Idx id) {this->id = id;}

Edge::Edge(const std::string & str) {this->from_str(str);}
Edge::Edge(Idx id, bool valid, bool directed, double weight, size_t idx_start, size_t idx_end)
{
  set(id, valid, directed, weight, idx_start, idx_end);
}

Edge::Edge(Idx id, size_t idx_start, size_t idx_end, bool valid)
{
  set(id, idx_start, idx_end, valid);
}

Edge & Edge::set(Idx id, bool valid, bool directed, double weight, size_t idx_start, size_t idx_end)
{
  this->id = id, this->valid = valid, this->directed = directed, this->weight = weight;
  this->nodes[0] = idx_start, this->nodes[1] = idx_end;
  return *this;
}
Edge & Edge::set(Idx id, size_t idx_start, size_t idx_end, bool valid)
{
  this->id = id, this->valid = valid, this->directed = true, this->weight = 0.;
  this->id = id, this->nodes[0] = idx_start, this->nodes[1] = idx_end;
  return *this;
}
tuw_graph_msgs::msg::Edge & Edge::msg() {return static_cast<tuw_graph_msgs::msg::Edge &>(*this);}
const tuw_graph_msgs::msg::Edge & Edge::msg() const
{
  return static_cast<const tuw_graph_msgs::msg::Edge &>(*this);
}

bool Edge::operator==(const Edge & rhs) const
{
  bool header_eq = (this->id == rhs.id) && (this->valid == rhs.valid) &&
    (this->directed == rhs.directed) && (this->weight == rhs.weight) &&
    (this->nodes[0] == rhs.nodes[0]) && (this->nodes[1] == rhs.nodes[1]) &&
    (this->path.size() == rhs.path.size());
  if(!header_eq) return false;
  auto it_path_a = this->path.begin();
  auto it_path_b = rhs.path.begin();
  for (; it_path_a != this->path.end(); it_path_a++, it_path_b++) {
    if (!is_equal(*it_path_a, *it_path_b)) {return false;}
  }
  auto it_flags_a = this->flags.begin();
  auto it_flags_b = rhs.flags.begin();
  for (; it_flags_a != this->flags.end(); it_flags_a++, it_flags_b++) {
    if (*it_flags_a != *it_flags_b) return false;
  }
  return true;
}

double Edge::similar(const Edge & rhs, double epsilon_position, double epsilon_orientation) const
{
  return is_similar(*this, rhs, epsilon_position, epsilon_orientation);
}

bool tuw_msgs::is_similar(
  const tuw_graph_msgs::msg::Edge & a, const tuw_graph_msgs::msg::Edge & b, double epsilon_position,
  double epsilon_orientation)
{
  bool header_eq = (a.id == b.id) && (a.valid == b.valid) && (a.directed == b.directed) &&
    (a.weight == b.weight) && (a.nodes[0] == b.nodes[0]) &&
    (a.nodes[1] == b.nodes[1]) && (a.path.size() == b.path.size());
  if(!header_eq) return false;
  auto it_path_a = a.path.begin();
  auto it_path_b = b.path.begin();
  for (; it_path_a != a.path.end(); it_path_a++, it_path_b++) {
    if (!is_similar(*it_path_a, *it_path_b, epsilon_position, epsilon_orientation)) {return false;}
  }
 
  auto it_flags_a = a.flags.begin();
  auto it_flags_b = b.flags.begin();
  for (; it_flags_a != a.flags.end(); it_flags_a++, it_flags_b++) {
    if (*it_flags_a != *it_flags_b) return false;
  }
  
  return true;
}

std::string Edge::to_str(tuw_msgs::Format format) const
{
  std::string str;
  return this->to_str(str, format);
}
std::string & Edge::to_str(std::string & des, tuw_msgs::Format format, bool append) const
{
  if (!append) {des.clear();}
  char txt[0xFF];
  sprintf(
    txt, "%4ld: %s: %s: %8.4lf: [%4ld, %4ld]: ", this->id, (this->valid ? "  valid" : "invalid"),
    (this->directed ? "  directed" : "undirected"), this->weight, this->nodes[0], this->nodes[1]);
  des.append(txt);

  des.append("[");
  for (auto it = this->path.begin(); it != this->path.end(); ) {
    des.append(static_cast<const Pose &>(*it).to_str(format));
    des.append(++it == this->path.end() ? "" : "; ");
  }
  des.append("]");
  return des;
}
size_t Edge::from_str(const std::string & str)
{
  char str_valid[0xF], str_directed[0xF];
  int n = sscanf(
    str.c_str(), "%ld:%16[^:]:%16[^:]:%lf:%*[^[][%ld,%ld]%*s", &this->id, str_valid, str_directed,
    &this->weight, &this->nodes[0], &this->nodes[1]);
  if (n != 6) {throw std::runtime_error("Failed decode edge header" + str);}

  this->valid = (std::string(str_valid).find("invalid") == std::string::npos);
  this->directed = (std::string(str_directed).find("undirected") == std::string::npos);

  size_t offset = str.rfind(":");
  if (offset == std::string::npos) {
    throw std::runtime_error("Failed decode Edge header ending: " + str);
  }
  offset++;
  offset = tuw_msgs::Pose::from_str(str.substr(offset), this->path) + offset;
  return offset;
}
#include <jsoncpp/json/json.h>
Json::Value Edge::toJson() const {
  Json::Value json;
  json["id"] = this->id;
  json["valid"] = this->valid;
  json["directed"] = this->directed;
  json["weight"] = this->weight;
  json["start"] = nodes[0];
  json["end"] = nodes[1];
  Json::Value json_flags;
  for (const auto& f : this->flags) {
      json_flags.append(f);
  }
  json["flags"] = json_flags;
  Json::Value json_poses;
  for (const auto& p : this->path) {
      json_poses.append(static_cast<const Pose&>(p).toJson());
  }
  json["path"] = json_poses;
  return json;
}

Edge &Edge::fromJson(const Json::Value& json, Edge &o){
  o.id = json.get("id", "-1").asInt64();
  o.valid = json.get("valid", "").asBool();
  o.directed = json.get("directed", "").asBool();
  o.weight = json.get("weight", "").asDouble();
  o.nodes[0] = json.get("start", "-1").asInt64();
  o.nodes[1] = json.get("end", "-1").asInt64();
  if(json.isMember("flags") && json["flags"].isArray()) {
    const Json::Value& jsonArray = json["flags"];
    // Iterate through the array
    for (size_t i = 0; i < jsonArray.size(); ++i) {
      o.flags.push_back(jsonArray[(int) i].asInt());
    }
  } 
  if(json.isMember("path") && json["path"].isArray()) {
    const Json::Value& jsonArray = json["path"];
    // Iterate through the array
    for (size_t i = 0; i < jsonArray.size(); ++i) {
      o.path.push_back(Pose::fromJson(jsonArray[(int)i]));
    }
  }
  return o;
}

Edge Edge::fromJson(const Json::Value& json){
  Edge o;
  return fromJson(json, o);
}