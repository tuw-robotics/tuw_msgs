#include <stdexcept>
#include <string>
#include <tuw_msgs/pose.hpp>
#include <cmath>

using namespace tuw_msgs;

bool Pose::is_zero() const
{
  return get_orientation().is_zero() && get_position().is_zero();
}

bool Pose::operator==(const tuw_msgs::Pose &rhs) const
{
  return (get_position() == rhs.get_position()) && (get_orientation() == rhs.get_orientation());
}

bool Pose::similar(const tuw_msgs::Pose &rhs, double epsilon_position, double epsilon_orientation) const
{
  return get_position().similar(rhs.get_position(), epsilon_position) && get_orientation().similar(rhs.get_orientation(), epsilon_orientation);
}
std::string Pose::to_str(tuw_msgs::Format format) const
{
  std::string str;
  return this->to_str(str, format);
}

std::string &Pose::to_str(std::string &des, tuw_msgs::Format format, bool append) const
{
  if (!append)
    des.clear();
  des.append("[" + get_position().to_str(format) + ", " + get_orientation().to_str(format) + "]");
  return des;
}

size_t Pose::from_str(const std::string &str)
{
  size_t offset = str.find("[");
  if (offset == std::string::npos)
    throw std::runtime_error("Failed decode Pose: " + str);
  offset++;
  offset = this->get_position().from_str(str.substr(offset)) + offset;
  offset = str.find(",", offset);
  if (offset == std::string::npos)
    throw std::runtime_error("Failed decode Pose: " + str);
  offset++;
  offset = this->get_orientation().from_str(str.substr(offset)) + offset;
  offset = str.find("]", offset);
  if (offset == std::string::npos)
    throw std::runtime_error("Failed decode Point: " + str);
  return offset;
};

