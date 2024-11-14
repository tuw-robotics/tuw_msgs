#ifndef TUW_STD_MSGS__PARAMETER_HPP_
#define TUW_STD_MSGS__PARAMETER_HPP_

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
  Parameter() {}
  Parameter(const std::string & name)
  {
    this->name = name;
    this->value = "";
  }
  Parameter(const std::string & name, const double & value) {this->set(name, value);}
  /**
   * set the name and the value entry and retuns the created string
   * @param name name to set
   * @param value value to set
   * @return string crated
   */
  const std::string & set(const std::string & name, const double & value)
  {
    this->name = name;
    return this->set(value);
  }
  /**
   * set a value entry and retuns the created string
   * @param values value to set
   * @return string crated
   */
  const std::string & set(const double & value)
  {
    this->value = std::to_string(value);
    return this->value;
  }

  template<typename T>
  T get() const
  {
    T result;
    this->get(result);
    return result;
  }

  double & get(double & data) const
  {
    data = std::stod(this->value);
    return data;
  }
  Parameter(const std::string & name, const int & value) {this->set(name, value);}
  /**
   * set the name and the value entry and retuns the created string
   * @param name name to set
   * @param value value to set
   * @return string crated
   */
  const std::string & set(const std::string & name, const int & value)
  {
    this->name = name;
    return this->set(value);
  }
  /**
   * set a value entry and retuns the created string
   * @param values value to set
   * @return string crated
   */
  const std::string & set(int & value)
  {
    this->value = std::to_string(value);
    return this->value;
  }
  int & get(int & data) const
  {
    data = std::stoi(this->value);
    return data;
  }
  Parameter(const std::string & name, const std::string & value) {this->set(name, value);}
  /**
   * set the name and the value entry and retuns the created string
   * @param name name to set
   * @param value value to set
   * @return string crated
   */
  const std::string & set(const std::string & name, const std::string & value)
  {
    this->name = name;
    return this->set(value);
  }
  /**
   * set a value entry and retuns the created string
   * @param values value to set
   * @return string crated
   */
  const std::string & set(const std::string & value)
  {
    this->value = value;
    return this->value;
  }
  std::string & get(std::string & data) const
  {
    data = this->value;
    return data;
  }
  Parameter(const std::string & name, const std::vector<double> & values, int precision = 10)
  {
    this->set(name, values, precision);
  }
  /**
   * set the name and the value entry and retuns the created string
   * @param name values to set
   * @param values values to set
   * @return string crated
   */
  const std::string & set(
    const std::string & name, const std::vector<double> & values, int precision = 10)
  {
    this->name = name;
    return this->set(values, precision);
  }
  /**
   * set a value entry and retuns the created string
   * @param values values to set
   * @return string crated
   */
  const std::string & set(const std::vector<double> & values, int precision = 10)
  {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(precision);
    for (size_t i = 0; i < values.size(); ++i) {
      oss << values[i];
      if (i != values.size() - 1) {
        oss << ", ";
      }
    }
    this->value = oss.str();
    return this->value;
  }

  std::vector<double> & get(std::vector<double> & data) const
  {
    data.clear();
    std::istringstream ss(this->value);
    std::string token;
    while (std::getline(ss, token, ',')) {
      data.push_back(std::stod(token));
    }
    return data;
  }

  Parameter(const std::string & name, const std::vector<int> & values) {this->set(name, values);}

  /**
   * set the name and the value entry and retuns the created string
   * @param name values to set
   * @param values values to set
   * @return string crated
   */
  const std::string & set(const std::string & name, const std::vector<int> & values)
  {
    this->name = name;
    return this->set(values);
  }
  /**
   * set a value entry and retuns the created string
   * @param values values to set
   * @return string crated
   */
  const std::string & set(const std::vector<int> & values)
  {
    std::ostringstream oss;
    for (size_t i = 0; i < values.size(); ++i) {
      oss << values[i];
      if (i != values.size() - 1) {
        oss << ", ";
      }
    }
    this->value = oss.str();
    return this->value;
  }

  std::vector<int> & get(std::vector<int> & data) const
  {
    data.clear();
    std::istringstream ss(this->value);
    std::string token;
    while (std::getline(ss, token, ',')) {
      data.push_back(std::stoi(token));
    }
    return data;
  }

  /**
   * retuns the values by index
   * @param index
   * @return values
   * @note it is slow
   */
  template<typename T>
  T at(size_t index)
  {
    std::vector<T> v;
    this->get(v);
    return v[index];
  }
};
}  // namespace tuw_std_msgs
#endif  // TUW_STD_MSGS__PARAMETER_HPP_
