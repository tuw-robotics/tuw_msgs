#include <cmath>
#include <tuw_msgs/utils.hpp>

geometry_msgs::msg::Quaternion & tuw_msgs::to_msg(
  double roll, double pitch, double yaw, geometry_msgs::msg::Quaternion & des)
{
  double halfYaw = yaw * 0.5;
  double halfPitch = pitch * 0.5;
  double halfRoll = roll * 0.5;
  double cosYaw = cos(halfYaw);
  double sinYaw = sin(halfYaw);
  double cosPitch = cos(halfPitch);
  double sinPitch = sin(halfPitch);
  double cosRoll = cos(halfRoll);
  double sinRoll = sin(halfRoll);
  des.x = sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw;  //x
  des.y = cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw;  //y
  des.z = cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw;  //z
  des.w = cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw;  //formerly yzx
  return des;
}
