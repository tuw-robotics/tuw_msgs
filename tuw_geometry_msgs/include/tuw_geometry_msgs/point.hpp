#ifndef TUW_GEOMETRY_MSGS__POINT_HPP_
#define TUW_GEOMETRY_MSGS__POINT_HPP_

#include <cmath>
#include <geometry_msgs/msg/point.hpp>
namespace tuw_geometry_msgs
{
struct Point : public geometry_msgs::msg::Point
{
  Point() {
    this->x = 0., this->y = 0., this->z = 0.;
  }
  Point(double x, double y, double z){
    this->x = x, this->y = y, this->z = z; 
  }
};
}  // namespace tuw_geometry_msgs
#endif  // TUW_GEOMETRY_MSGS__POINT_HPP_
