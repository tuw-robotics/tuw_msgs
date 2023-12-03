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
    Point(double x, double y, double z)
    {
      this->set(x, y, z);
    };
    Point(const std::string &src)
    {
      this->from_str(src);
    };
    Point &set(double x, double y, double z)
    {
      this->x = x, this->y = y, this->z = z;
      return *this;
    };
    geometry_msgs::msg::Point &msg(){
      return static_cast<geometry_msgs::msg::Point&>(*this);
    }
    const geometry_msgs::msg::Point &msg() const {
      return static_cast<const geometry_msgs::msg::Point&>(*this);
    }
    double similar(const Point &rhs, double threshold = 0.0001) const
    {
      double d = std::sqrt( std::pow(x - rhs.x, 2) + std::pow(y - rhs.y, 2) + std::pow(z - rhs.z, 2));
      return (fabs(d) < threshold);
    }
    bool operator==(const Point &rhs) const;
    bool is_zero() const;
    std::string to_str(tuw_msgs::Format format = LOOSE) const;
    std::string &to_str(std::string &des, tuw_msgs::Format format = LOOSE, bool append = false) const;
    Point &from_str(const std::string &src);
  };
}
#endif // TUW_MSGS__POINT_HPP_
