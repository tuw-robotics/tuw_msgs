#ifndef TUW_STD_MSGS_MSGS__PARAMETER_HPP_
#define TUW_STD_MSGS_MSGS__PARAMETER_HPP_

#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <tuw_std_msgs/msg/parameter.hpp>
#include <vector>

namespace tuw_std_msgs
{
struct Parameter : public tuw_std_msgs::msg::Parameter
{
  Parameter() {

  }
  Parameter(const std::string & name)
  {
    this->name = name;
    this->value = "";
  }
  Parameter(const std::string & name, const double & data)
  {
    this->name = name;
    this->value = std::to_string(data);
  }
  double & get(double & data)
  {
    data = std::stod(this->value);
    return data;
  }
  Parameter(const std::string & name, const int & data)
  {
    this->name = name;
    this->value = std::to_string(data);
  }
  int & get(int & data)
  {
    data = std::stoi(this->value);
    return data;
  }
  Parameter(const std::string & name, const std::string & data)
  {
    this->name = name;
    this->value = data;
  }
  std::string & get(std::string & data)
  {
    data = this->value;
    return data;
  }
  Parameter(const std::string & name, const std::vector<double> & data, int precision = 10)
  {
    this->name = name;

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(precision);
    for (size_t i = 0; i < data.size(); ++i) {
      oss << data[i];
      if (i != data.size() - 1) {
        oss << ", ";
      }
    }
    this->value = oss.str();
  }

  std::vector<double> & get(std::vector<double> & data)
  {
    data.clear();
    std::istringstream ss(this->value);
    std::string token;
    while (std::getline(ss, token, ',')) {
      data.push_back(std::stod(token));
    }
    return data;
  }

  Parameter(const std::string & name, const std::vector<int> & data)
  {
    this->name = name;

    std::ostringstream oss;
    for (size_t i = 0; i < data.size(); ++i) {
      oss << data[i];
      if (i != data.size() - 1) {
        oss << ", ";
      }
    }
    this->value = oss.str();
  }

  std::vector<int> & get(std::vector<int> & data)
  {
    data.clear();
    std::istringstream ss(this->value);
    std::string token;
    while (std::getline(ss, token, ',')) {
      data.push_back(std::stoi(token));
    }
    return data;
  }
};
}  // namespace tuw_std_msgs
#endif  // TUW_STD_MSGS_MSGS__PARAMETER_HPP_
