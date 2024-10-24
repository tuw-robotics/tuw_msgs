#ifndef TUW_OBJECT_MAP_MSGS_MSGS__VALUE_HPP_
#define TUW_OBJECT_MAP_MSGS_MSGS__VALUE_HPP_

#include <string>
#include <tuw_object_map_msgs/msg/value.hpp>

namespace tuw_object_map_msgs
{
struct Value : public tuw_object_map_msgs::msg::Value
{
  Value(const std::string &name) 
  {
    this->name.data = name;
    this->data = std::numeric_limits<double>::quiet_NaN();
  }
  Value(const std::string &name, double data) 
  {
    this->name.data = name;
    this->data = data;
  }
};
}  // namespace tuw_object_map_msgs
#endif  // TUW_OBJECT_MAP_MSGS_MSGS__VALUE_HPP_
