#include <stdexcept>
#include <string>
#include <tuw_msgs/point.hpp>

using namespace tuw_msgs;


geometry_msgs::msg::Point &tuw_msgs::to_msg(double x, double y, double z, geometry_msgs::msg::Point &des)
{
  des.x = x, des.y = y, des.z = z;
  return des;
}

std::string &tuw_msgs::encode(geometry_msgs::msg::Point &src, std::string &des)
{
  char txt[0xFF];
  if (src.z == 0.)
  {
    sprintf(txt, "[%f, %f]", src.x, src.y);
  }
  else
  {
    sprintf(txt, "[%f, %f, %f]", src.x, src.y, src.z);
  }
  des.append(txt);
  return des;
}

size_t tuw_msgs::decode(geometry_msgs::msg::Point &des, std::string &line, size_t pos)
{
  pos = line.find("[", pos);
  if (pos == std::string::npos)
    throw std::runtime_error("Failed decode Point in line: " + line);
  const char *str = line.c_str() + pos;
  int n = sscanf(str, "[%lf,%lf,%lf]%*s", &des.x, &des.y, &des.z);
  if (n != 3)
  {
    int n = sscanf(str, "[%lf,%lf]%*s", &des.x, &des.y);
    if (n != 2)
    {
      throw std::runtime_error("Failed decode Point incorrect number of values: " + line);
    }
    des.z = 0;
  }
  pos = line.find("]", pos);
  if (pos == std::string::npos)
    throw std::runtime_error("Failed decode Point in line: " + line);
  pos++;
  return pos;
}

bool tuw_msgs::is_zero(const geometry_msgs::msg::Point &src)
{
  return (src.x == 0.) && (src.y == 0.) && (src.z == 0.);
}

std::string tuw_msgs::to_str(const geometry_msgs::msg::Point &src, tuw_msgs::Format format)
{
  std::string str;
  return to_str(src, str, format);
}

std::string &tuw_msgs::to_str(const geometry_msgs::msg::Point &src, std::string &des, tuw_msgs::Format format)
{
  char txt[0xFF];
  if ((format == COMPACT) && is_zero(src))
  {
    sprintf(txt, "[]");
  }
  else if ((format == COMPACT) && src.z == 0.)
  {
    sprintf(txt, "[%f, %f]", src.x, src.y);
  }
  else
  {
    sprintf(txt, "[%f, %f, %f]", src.x, src.y, src.z);
  }
  des.append(txt);
  return des;
}

geometry_msgs::msg::Point &tuw_msgs::from_str(const std::string &src, geometry_msgs::msg::Point &des)
{
  size_t offset = nr_of_leading_spaces(src);
  int n = sscanf(src.c_str()+offset, "[%lf,%lf,%lf]%*s", &des.x, &des.y, &des.z);
  if (n != 3)
  {
    int n = sscanf(src.c_str()+offset, "[%lf,%lf]%*s", &des.x, &des.y);
    if (n != 2)
    {
      size_t pos = src.find("[]",offset);
      if (pos == std::string::npos)
        throw std::runtime_error("Failed decode Point incorrect number of values: " + src); 
      des.x = 0, des.y = 0;
    }
    des.z = 0;
  }
  return des;
}