#ifndef TUW_MSGS__POINT_HPP_
#define TUW_MSGS__POINT_HPP_

#include <geometry_msgs/msg/point.hpp>
#include <tuw_msgs/utils.hpp>

namespace tuw_msgs
{
  struct Point : public geometry_msgs::msg::Point
  {
    Point(){};
    Point(double x, double y, double z)
    {
      this->set(x, y, z);
    };
    Point &set(double x, double y, double z)
    {
      this->x = x, this->y = y, this->z = z;
      return *this;
    };
    bool operator==(const Point &rhs) const
    {
      return (x == rhs.x) && (y == rhs.y) && (z == rhs.z);
    }
  };
  geometry_msgs::msg::Point &to_msg(double x, double y, double z, geometry_msgs::msg::Point &des);
  std::string to_str(const geometry_msgs::msg::Point &src);
  std::string to_str(const geometry_msgs::msg::Point &src, std::string &des);
  std::string &encode(geometry_msgs::msg::Point &src, std::string &des);
  size_t decode(geometry_msgs::msg::Point &des, std::string &line, size_t pos = 0);
}
#endif // TUW_MSGS__POINT_HPP_
