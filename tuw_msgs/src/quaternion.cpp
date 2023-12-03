#include <stdexcept>
#include <string>
#include <cmath>
#include <tuw_msgs/quaternion.hpp>

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

geometry_msgs::msg::Quaternion &tuw_msgs::to_msg(double x, double y, double z, double w, geometry_msgs::msg::Quaternion &des)
{
  des.x = x, des.y = y, des.z = z, des.w = w;
  return des;
}

 std::string &tuw_msgs::encode(geometry_msgs::msg::Quaternion &src, std::string &des){
  char txt[0xFF];
  if((src.x == 0.) && (src.y == 0.) && (src.z == 0.) && (src.w == 1.)){
    sprintf(txt, "[]");
  } else {
    sprintf(txt, "[%f, %f, %f, %f]", src.x, src.y, src.z, src.w);
  }
  des.append(txt);
  return des;
 }

size_t tuw_msgs::decode(geometry_msgs::msg::Quaternion &des, std::string &line, size_t pos)
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
  pos++;
  return pos;
}