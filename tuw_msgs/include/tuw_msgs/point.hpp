#ifndef TUW_MSGS__POINT_HPP_
#define TUW_MSGS__POINT_HPP_

#include <cmath>
#include <geometry_msgs/msg/point.hpp>
#include <tuw_msgs/utils.hpp>
namespace Json{
  class Value;
}
namespace tuw_msgs
{
struct Point : public geometry_msgs::msg::Point
{
  Point() {}
  Point(double x, double y, double z);
  Point(const std::string & src);
  Point & set(double x, double y, double z);
  geometry_msgs::msg::Point & msg();
  const geometry_msgs::msg::Point & msg() const;
  double similar(const Point & rhs, double epsilon = 0.0001) const;
  bool operator==(const Point & rhs) const;
  bool is_zero() const;
  std::string to_str(tuw_msgs::Format format = LOOSE) const;
  std::string & to_str(
    std::string & des, tuw_msgs::Format format = LOOSE, bool append = false) const;
  size_t from_str(const std::string & src);
  int json_get(Json::Value &value);
  int json_add(const char* key, Json::Value &value);
  Json::Value toJson() const;
   
};
}  // namespace tuw_msgs
#endif  // TUW_MSGS__POINT_HPP_
