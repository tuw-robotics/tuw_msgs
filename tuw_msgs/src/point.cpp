#include <stdexcept>
#include <string>
#include <tuw_msgs/point.hpp>

using namespace tuw_msgs;

bool Point::operator==(const Point &rhs) const
{
  return (x == rhs.x) && (y == rhs.y) && (z == rhs.z);
}
bool Point::is_zero() const
{
  return (this->x == 0.) && (this->y == 0.) && (this->z == 0.);
}

std::string Point::to_str(tuw_msgs::Format format) const
{
  std::string str;
  return to_str(str, format);
}

std::string &Point::to_str(std::string &des, tuw_msgs::Format format, bool append) const
{
  char txt[0x40];
  if ((format == COMPACT) && this->is_zero())
  {
    sprintf(txt, "[]");
  }
  else if ((format == COMPACT) && this->z == 0.)
  {
    sprintf(txt, "[%f, %f]", this->x, this->y);
  }
  else
  {
    sprintf(txt, "[%f, %f, %f]", this->x, this->y, this->z);
  }
  if(append) des.append(txt);
  else des.assign(txt);
  return des;
}

tuw_msgs::Point &Point::from_str(const std::string &src)
{
  size_t offset = src.find("[");
  if (offset == std::string::npos)
    throw std::runtime_error("Failed decode Point: " + src);
  int n = sscanf(src.c_str() + offset, "[%lf,%lf,%lf]%*s", &this->x, &this->y, &this->z);
  if (n != 3)
  {
    int n = sscanf(src.c_str() + offset, "[%lf,%lf]%*s", &this->x, &this->y);
    if (n != 2)
    {
      size_t pos = src.find("[]", offset);
      if (pos == std::string::npos)
        throw std::runtime_error("Failed decode Point incorrect number of values: " + src);
      this->x = 0, this->y = 0, this->z = 0;
    }
    this->z = 0;
  }
  return *this;
}