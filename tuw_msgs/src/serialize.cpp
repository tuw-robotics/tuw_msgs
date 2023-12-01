#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <tuw_msgs/serialize.hpp>
#include <tuw_msgs/utils.hpp>

using namespace tuw_msgs;

Serialize::Serialize() { this->delim(); }

void Serialize::delim(char delim_entries, char delim_rows, char delim_cols)
{
  this->delim_entries = delim_entries;
  this->delim_rows = delim_rows;
  this->delim_cols = delim_cols;
}

Serialize::~Serialize() {}

 std::string &Serialize::encode(geometry_msgs::msg::Point &src, std::string &des){
  char txt[0xFF];
  if(src.z == 0.){
    sprintf(txt, "[%f, %f]", src.x, src.y);
  } else {
    sprintf(txt, "[%f, %f, %f]", src.x, src.y, src.z);
  }
  des.append(txt);
  return des;
 }

 std::string &Serialize::encode(geometry_msgs::msg::Quaternion &src, std::string &des){
  char txt[0xFF];
  if((src.x == 0.) && (src.y == 0.) && (src.z == 0.) && (src.w == 1.)){
    sprintf(txt, "[]");
  } else {
    sprintf(txt, "[%f, %f, %f, %f]", src.x, src.y, src.z, src.w);
  }
  des.append(txt);
  return des;
 }

 std::string &Serialize::encode(geometry_msgs::msg::Pose &src, std::string &des){
  des.append("[");
  encode(src.position, des);
  des.append(", ");
  encode(src.orientation, des);
  des.append("]");
  return des;
 }

size_t Serialize::decode(geometry_msgs::msg::Point &des, std::string &line, size_t pos)
{
  pos = line.find("[", pos);
  if (pos == std::string::npos)
    throw std::runtime_error("Failed decode Point in line: " + line);
  const char *str = line.c_str() + pos;
  int n = sscanf(str, "[%lf,%lf,%lf]%*s", &des.x, &des.y, &des.z);
  if (n != 3)
  {
    int n = sscanf(str, "[%lf,%lf]%*s", &des.x, &des.y);
    if (n != 2)
    {
      throw std::runtime_error("Failed decode Point incorrect number of values: " + line);
    }
    des.z = 0;
  }
  pos = line.find("]", pos);
  if (pos == std::string::npos)
    throw std::runtime_error("Failed decode Point in line: " + line);
  return pos++;
}

size_t Serialize::decode(geometry_msgs::msg::Quaternion &des, std::string &line, size_t pos)
{
  pos = line.find("[", pos);
  if (pos == std::string::npos)
    throw std::runtime_error("Failed decode Quaternion in line: " + line);
  const char *str = line.c_str() + pos;
  int n = sscanf(str, "[%lf,%lf,%lf,%lf]%*s", &des.x, &des.y, &des.z, &des.w);
  if (n != 4)
  {
    double roll, pitch, yaw;
    int n = sscanf(str, "[%lf,%lf,%lf]%*s", &roll, &pitch, &yaw);
    if (n == 3)
    {
      tuw_msgs::to_msg(roll, pitch, yaw, des);
    } else {
      pos = line.find("[]", pos, 2);
      if (pos == std::string::npos)
        throw std::runtime_error("Failed decode Quaternion incorrect number of values: " + line);
      tuw_msgs::to_msg(0,0,0,1, des);
    }

  }
  pos = line.find("]", pos);
  if (pos == std::string::npos)
    throw std::runtime_error("Failed decode Quaternion in line: " + line);
  return pos++;
}

size_t Serialize::decode(geometry_msgs::msg::Pose &des, std::string &line, size_t pos)
{
  pos = line.find("[", pos);
  if (pos == std::string::npos)
    throw std::runtime_error("Failed decode Pose in line: " + line);
  pos++;
  pos = decode(des.position, line, pos);
  pos = line.find(",", pos);
  if (pos == std::string::npos)
    throw std::runtime_error("Failed decode Pose in line: " + line);
  pos++;
  pos = decode(des.orientation, line, pos);
  pos = line.find("]", pos);
  if (pos == std::string::npos)
    throw std::runtime_error("Failed decode Pose in line: " + line);
  return pos++;
}
