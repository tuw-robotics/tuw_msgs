#include <stdexcept>
#include <string>
#include <tuw_msgs/pose.hpp>

using namespace tuw_msgs;

 std::string &tuw_msgs::encode(geometry_msgs::msg::Pose &src, std::string &des){
  des.append("[");
  encode(src.position, des);
  des.append(", ");
  encode(src.orientation, des);
  des.append("]");
  return des;
 }

size_t tuw_msgs::decode(geometry_msgs::msg::Pose &des, std::string &line, size_t pos)
{
  pos = line.find("[", pos);
  const char *str = line.c_str() + pos; /// simplyfies debugging
  if (pos == std::string::npos)
    throw std::runtime_error("Failed decode Pose in line: " + line);
  pos++;
  pos = decode(des.position, line, pos);
  str = line.c_str() + pos;     /// simplyfies debugging
  pos = line.find(",", pos);
  if (pos == std::string::npos)
    throw std::runtime_error("Failed decode Pose in line: " + line);
  pos++;
  pos = decode(des.orientation, line, pos);
  str = line.c_str() + pos;     /// simplyfies debugging
  pos = line.find("]", pos);
  if (pos == std::string::npos)
    throw std::runtime_error("Failed decode Pose in line: " + line);
  pos++;
  str = line.c_str() + pos;     /// simplyfies debugging
  return pos;
}

