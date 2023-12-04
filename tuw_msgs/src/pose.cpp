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

double Pose::similar(const tuw_msgs::Pose &rhs, double threshold_position, double threshold_orientation) const
{
  return get_position().similar(rhs.get_position(), threshold_position) && get_orientation().similar(rhs.get_orientation(), threshold_orientation);
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

Pose &Pose::from_str(const std::string &str)
{
  size_t offset_position = str.find("[");
      if (offset_position == std::string::npos)
        throw std::runtime_error("Failed decode Pose: " + str);
  offset_position++;
  std::string str_position = str.substr(offset_position);
  this->get_position().from_str(str_position);
  size_t offset_position_end = str_position.find("]");
      if (offset_position_end == std::string::npos)
        throw std::runtime_error("Failed decode Pose: " + str);
  size_t offset_orientation = str_position.find(",", offset_position_end);
      if (offset_orientation == std::string::npos)
        throw std::runtime_error("Failed decode Pose: " + str);
  offset_orientation++;
  std::string str_orientation = str.substr(offset_orientation);
  this->get_orientation().from_str(str_orientation);
  return *this;
};
