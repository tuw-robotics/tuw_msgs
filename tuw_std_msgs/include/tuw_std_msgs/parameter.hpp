#ifndef TUW_STD_MSGS_MSGS__PARAMETER_HPP_
#define TUW_STD_MSGS_MSGS__PARAMETER_HPP_

#include <string>
#include <limits>
#include <vector>
#include <sstream>
#include <iomanip>
#include <tuw_std_msgs/msg/parameter.hpp>

namespace tuw_std_msgs
{
struct Parameter : public tuw_std_msgs::msg::Parameter
{
  Parameter() 
  {
    this->type = tuw_std_msgs::msg::Parameter::TYPE_NA;
  }
  Parameter(const std::string &name) 
  {
    this->name.data = name;
    this->value.data = "";
  }
  Parameter(const std::string &name, const double &data) 
  {
    this->name.data = name;
    this->value.data = std::to_string(data);
    this->type = tuw_std_msgs::msg::Parameter::TYPE_FLOAT;
  }
  double& get(double &data){
    if(this->type != tuw_std_msgs::msg::Parameter::TYPE_FLOAT) 
        throw std::runtime_error("Parameter type missmatch"); 
    data = std::stod(this->value.data);
    return data;
  }
  Parameter(const std::string &name, const int &data) 
  {
    this->name.data = name;
    this->value.data = std::to_string(data);
    this->type = tuw_std_msgs::msg::Parameter::TYPE_INT;
  }
  int& get(int &data){
    if(this->type != tuw_std_msgs::msg::Parameter::TYPE_INT) 
        throw std::runtime_error("Parameter type missmatch"); 
    data = std::stoi(this->value.data);
    return data;
  }
  Parameter(const std::string &name, const std::string &data) 
  {
    this->name.data = name;
    this->value.data = data;
    this->type = tuw_std_msgs::msg::Parameter::TYPE_STRING;
  }
  std::string& get(std::string &data){
    if(this->type != tuw_std_msgs::msg::Parameter::TYPE_STRING) 
        throw std::runtime_error("Parameter type missmatch"); 
    data = this->value.data;
    return data;
  }
  Parameter(const std::string &name, const std::vector<double> &data, int precision = 10) 
  {
    this->name.data = name;

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(precision);
    for (size_t i = 0; i < data.size(); ++i) {
        oss << data[i];
        if (i != data.size() - 1) {
            oss << ", ";
        }
    }
    this->value.data = oss.str();
    this->type = tuw_std_msgs::msg::Parameter::TYPE_FLOAT_ARRAY;
  }

  std::vector<double>& get(std::vector<double> &data){
    if(this->type != tuw_std_msgs::msg::Parameter::TYPE_FLOAT_ARRAY) 
        throw std::runtime_error("Parameter type missmatch"); 
    data.clear();
    std::istringstream ss(this->value.data);
    std::string token;
    while (std::getline(ss, token, ',')) {
        data.push_back(std::stod(token));  
    }
    return data;
  }

  Parameter(const std::string &name, const std::vector<int> &data) 
  {
    this->name.data = name;

    std::ostringstream oss;
    for (size_t i = 0; i < data.size(); ++i) {
        oss << data[i];
        if (i != data.size() - 1) {
            oss << ", ";
        }
    }
    this->value.data = oss.str();
    this->type = tuw_std_msgs::msg::Parameter::TYPE_INT_ARRAY;
  }

  std::vector<int>& get(std::vector<int> &data){
    if(this->type != tuw_std_msgs::msg::Parameter::TYPE_INT_ARRAY) 
        throw std::runtime_error("Parameter type missmatch"); 
    data.clear();
    std::istringstream ss(this->value.data);
    std::string token;
    while (std::getline(ss, token, ',')) {
        data.push_back(std::stoi(token)); 
    }
    return data;
  }
};
}  // namespace tuw_std_msgs
#endif  // TUW_STD_MSGS_MSGS__PARAMETER_HPP_
