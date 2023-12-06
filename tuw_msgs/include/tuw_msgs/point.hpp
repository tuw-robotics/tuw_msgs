#ifndef TUW_MSGS__POINT_HPP_
#define TUW_MSGS__POINT_HPP_

#include <geometry_msgs/msg/point.hpp>
#include <tuw_msgs/utils.hpp>
#include <cmath>

namespace tuw_msgs
{
  struct Point : public geometry_msgs::msg::Point
  {
    Point(){};
    Point(double x, double y, double z);
    Point(const std::string &src);
    Point &set(double x, double y, double z);
    geometry_msgs::msg::Point &msg();
    const geometry_msgs::msg::Point &msg() const;
    double similar(const Point &rhs, double epsilon = 0.0001) const;
    bool operator==(const Point &rhs) const;
    bool is_zero() const;
    std::string to_str(tuw_msgs::Format format = LOOSE) const;
    std::string &to_str(std::string &des, tuw_msgs::Format format = LOOSE, bool append = false) const;
    size_t from_str(const std::string &src);
  };
}
#endif // TUW_MSGS__POINT_HPP_
