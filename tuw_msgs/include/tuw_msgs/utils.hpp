#ifndef TUW_MSGS__UTILS_HPP_
#define TUW_MSGS__UTILS_HPP_

#include <geometry_msgs/msg/quaternion.hpp>

namespace tuw_msgs
{
geometry_msgs::msg::Quaternion & to_msg(
  double roll, double pitch, double yaw, geometry_msgs::msg::Quaternion & des);
}
#endif  // TUW_MSGS__SERIALIZE_HPP_
