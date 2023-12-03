#ifndef TUW_MSGS__QUATERNION_HPP_
#define TUW_MSGS__QUATERNION_HPP_

#include <geometry_msgs/msg/quaternion.hpp>

namespace tuw_msgs
{
  geometry_msgs::msg::Quaternion &to_msg(double x, double y, double z, double w, geometry_msgs::msg::Quaternion &des);
  geometry_msgs::msg::Quaternion &to_msg(double roll, double pitch, double yaw, geometry_msgs::msg::Quaternion &des);

  std::string &encode(geometry_msgs::msg::Quaternion &src, std::string &des);

  size_t decode(geometry_msgs::msg::Quaternion &des, std::string &line, size_t pos = 0);

  struct Quaternion : public geometry_msgs::msg::Quaternion
  {
    Quaternion(){};
    Quaternion(double x, double y, double z, double w)
    {
      this->set(x, y, z, w);
    };
    Quaternion(double roll, double pitch, double yaw)
    {
      this->rpy(roll, pitch, yaw);
    };
    Quaternion &set(double x, double y, double z, double w)
    {
      this->x = x, this->y = y, this->z = z, this->w = w;
      return *this;
    };
    Quaternion &rpy(double roll, double pitch, double yaw)
    {
      to_msg(roll, pitch, yaw, *this);
      return *this;
    };
    bool operator==(const Quaternion &rhs) const
    {
      return (x == rhs.x) && (y == rhs.y) && (z == rhs.z) && (w == rhs.w);
    }
  };
}
#endif // TUW_MSGS__QUATERNION_HPP_
