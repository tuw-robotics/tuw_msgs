#include <stdexcept>
#include <string>
#include <cmath>
#include <tuw_msgs/quaternion.hpp>

using namespace tuw_msgs;

tuw_msgs::Quaternion &Quaternion::from_rpy(double roll, double pitch, double yaw)
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
  this->x = sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw; // x
  this->y = cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw; // y
  this->z = cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw; // z
  this->w = cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw; // formerly yzx
  return *this;
}

bool Quaternion::is_zero() const
{
  return (this->x == 0.) && (this->y == 0.) && (this->z == 0.) && (this->w == 1.);
}

std::string Quaternion::to_str(tuw_msgs::Format format) const
{
  std::string str;
  return to_str(str, format);
}

std::string &Quaternion::to_str(std::string &des, tuw_msgs::Format format, bool append) const
{
  char txt[0x40];
  if ((format == COMPACT) && is_zero())
  {
    sprintf(txt, "[]");
  }
  else
  {
    sprintf(txt, "[%f, %f, %f, %f]", this->x, this->y, this->z, this->w);
  }
  des.append(txt);
  if(append) des.append(txt);
  else des.assign(txt);
  return des;
}


tuw_msgs::Quaternion &Quaternion::from_str(const std::string &src)
{
  size_t offset = src.find("[");
  if (offset == std::string::npos)
    throw std::runtime_error("Failed decode Quaternion: " + src);
  int n = sscanf(src.c_str()+offset, "[%lf,%lf,%lf,%lf]%*s", &this->x, &this->y, &this->z, &this->w);
  if (n != 4)
  {
    double roll, pitch, yaw;
    int n = sscanf(src.c_str()+offset, "[%lf,%lf,%lf]%*s", &roll, &pitch, &yaw);
    if (n != 3)
    {
      size_t pos = src.find("[]",offset);
      if (pos == std::string::npos)
        throw std::runtime_error("Failed decode Point incorrect number of values: " + src); 
      this->x = 0, this->y = 0, this->z = 0, this->w = 1;
    } else {
      from_rpy(roll, pitch, yaw);
    }
  }
  return *this;
}