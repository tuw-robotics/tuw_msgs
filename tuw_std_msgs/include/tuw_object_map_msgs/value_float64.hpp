#ifndef TUW_STD_MSGS_MSGS__VALUE_FLOAT64_HPP_
#define TUW_STD_MSGS_MSGS__VALUE_FLOAT64_HPP_

#include <string>
#include <tuw_std_msgs/msg/value_float64.hpp>

namespace tuw_std_msgs
{
struct ValueFloat64 : public ValueFloat64::msg::ValueFloat64
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
}  // namespace tuw_std_msgs
#endif  // TUW_STD_MSGS_MSGS__VALUE_FLOAT64_HPP_
