#ifndef TUW_STD_MSGS_MSGS__PARAMETER_FLOAT64_HPP_
#define TUW_STD_MSGS_MSGS__PARAMETER_FLOAT64_HPP_

#include <string>
#include <limits>
#include <tuw_std_msgs/msg/parameter_float64.hpp>

namespace tuw_std_msgs
{
struct ParameterFloat64 : public tuw_std_msgs::msg::ParameterFloat64
{
  ParameterFloat64() 
  {
    this->data = std::numeric_limits<double>::quiet_NaN();
  }
  ParameterFloat64(const std::string &name) 
  {
    this->name.data = name;
    this->data = std::numeric_limits<double>::quiet_NaN();
  }
  ParameterFloat64(const std::string &name, const double &data) 
  {
    this->name.data = name;
    this->data = data;
  }
  double &get(double &data) const{
    data = this->data;
    return data;
  }
  double get() const{
    return this->data;
  }
};
}  // namespace tuw_std_msgs
#endif  // TUW_STD_MSGS_MSGS__PARAMETER_FLOAT64_HPP_
