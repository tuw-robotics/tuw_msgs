/*
Copyright (c) 2024, Markus Bader
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.
 * Neither the name of TU Wien nor the names of its contributors may be
   used to endorse or promote products derived from this software without
   specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef TUW_STD_MSGS__PARAMETER_HPP_
#define TUW_STD_MSGS__PARAMETER_HPP_

#include <tuw_std_msgs/msg/parameter.hpp>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace tuw_std_msgs
{
struct Parameter : public tuw_std_msgs::msg::Parameter
{
  Parameter() {}
  explicit Parameter(const std::string & name)
  {
    this->name = name;
    this->value = "";
  }
  Parameter(const std::string & name, const double & value, int precision = 6)
  {
    this->set(name, value, precision);
  }
  /**
   * set the name and the value entry and retuns the created string
   * @param name name to set
   * @param value value to set
   * @param precision precision used to convert doubles to strings
   * @return string crated
   */
  const std::string & set(const std::string & name, const double & value, int precision = 6)
  {
    this->name = name;
    return this->set(value, precision);
  }
  /**
   * set a value entry and retuns the created string
   * @param values value to set
   * @param precision precision used to convert doubles to strings
   * @return string crated
   */
  const std::string & set(const double & value, int precision = 6)
  {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(precision) << value;
    this->value = oss.str();
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
  Parameter(const std::string & name, const std::string & value)
  {
    this->set(name, value);
  }
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
  /**
   * set a value entry and retuns the created string
   * @param values value to set
   * @param precision precision used to convert doubles to strings
   * @return string crated
   */
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
  Parameter(const std::string & name, const std::vector<int> & values)
  {
    this->set(name, values);
  }
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
