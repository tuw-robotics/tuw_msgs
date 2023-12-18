#include <cmath>
#include <stdexcept>
#include <string>
#include <tuw_msgs/quaternion.hpp>

using namespace tuw_msgs;

tuw_msgs::Quaternion & Quaternion::from_rpy(double roll, double pitch, double yaw)
{
  rpy_to_quaternion(roll, pitch, yaw, this->x, this->y, this->z, this->w);
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

std::string & Quaternion::to_str(std::string & des, tuw_msgs::Format format, bool append) const
{
  char txt[0x40];
  if ((format == COMPACT) && is_zero()) {
    sprintf(txt, "[]");
  } else {
    sprintf(txt, "[%f, %f, %f, %f]", this->x, this->y, this->z, this->w);
  }
  des.append(txt);
  if (append) {
    des.append(txt);
  } else {
    des.assign(txt);
  }
  return des;
}

size_t Quaternion::from_str(const std::string & src)
{
  size_t offset = src.find("[");
  if (offset == std::string::npos) {throw std::runtime_error("Failed decode Quaternion: " + src);}
  int n =
    sscanf(src.c_str() + offset, "[%lf,%lf,%lf,%lf]%*s", &this->x, &this->y, &this->z, &this->w);
  if (n != 4) {
    double roll, pitch, yaw;
    int n = sscanf(src.c_str() + offset, "[%lf,%lf,%lf]%*s", &roll, &pitch, &yaw);
    if (n != 3) {
      size_t pos = src.find("[]", offset);
      if (pos == std::string::npos) {
        throw std::runtime_error("Failed decode Point incorrect number of values: " + src);
      }
      this->x = 0, this->y = 0, this->z = 0, this->w = 1;
    } else {
      from_rpy(roll, pitch, yaw);
    }
  }
  offset = src.find("]");
  if (offset == std::string::npos) {throw std::runtime_error("Failed decode Point: " + src);}
  return offset;
}

Quaternion & Quaternion::get_orientation() {return static_cast<Quaternion &>(*this);}
const Quaternion & Quaternion::get_orientation() const
{
  return static_cast<const Quaternion &>(*this);
}
bool Quaternion::operator==(const geometry_msgs::msg::Quaternion & rhs) const
{
  return (x == rhs.x) && (y == rhs.y) && (z == rhs.z) && (w == rhs.w);
}

bool Quaternion::similar(const Quaternion & rhs, double epsilon) const
{
  return is_similar(this->msg(), rhs.msg(), epsilon);
}
geometry_msgs::msg::Quaternion & Quaternion::msg()
{
  return static_cast<geometry_msgs::msg::Quaternion &>(*this);
}
const geometry_msgs::msg::Quaternion & Quaternion::msg() const
{
  return static_cast<const geometry_msgs::msg::Quaternion &>(*this);
}
