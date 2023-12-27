#ifndef TUW_GEOMETRY_MSGS__QUATERNION_HPP_
#define TUW_GEOMETRY_MSGS__QUATERNION_HPP_

#include <geometry_msgs/msg/quaternion.hpp>

namespace tuw_geometry_msgs
{
struct Quaternion : public geometry_msgs::msg::Quaternion
{
  Quaternion() {this->x = 0., this->y = 0., this->z = 0., this->w = 1.;}
  Quaternion(double x, double y, double z, double w)
  {
    this->x = x, this->y = y, this->z = z, this->w = w;
  }
};
}  // namespace tuw_geometry_msgs
#endif  // TUW_GEOMETRY_MSGS__QUATERNION_HPP_
