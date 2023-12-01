#include <cmath>
#include <tuw_msgs/utils.hpp>

using namespace tuw_msgs;

geometry_msgs::msg::Quaternion &tuw_msgs::to_msg(
    double roll, double pitch, double yaw, geometry_msgs::msg::Quaternion &des)
{
  double halfYaw = yaw * 0.5;
  double halfPitch = pitch * 0.5;
  double halfRoll = roll * 0.5;
  double cosYaw = std::cos(halfYaw);
  double sinYaw = std::sin(halfYaw);
  double cosPitch = std::cos(halfPitch);
  double sinPitch = std::sin(halfPitch);
  double cosRoll = std::cos(halfRoll);
  double sinRoll = std::sin(halfRoll);
  des.x = sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw; // x
  des.y = cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw; // y
  des.z = cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw; // z
  des.w = cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw; // formerly yzx
  return des;
}

geometry_msgs::msg::Point &tuw_msgs::to_msg(double x, double y, double z, geometry_msgs::msg::Point &des)
{
  des.x = x, des.y = y, des.z = z;
  return des;
}
geometry_msgs::msg::Quaternion &tuw_msgs::to_msg(double x, double y, double z, double w, geometry_msgs::msg::Quaternion &des)
{
  des.x = x, des.y = y, des.z = z, des.w = w;
  return des;
}
