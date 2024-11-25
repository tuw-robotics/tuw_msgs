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

#ifndef TUW_STD_MSGS__PARAMETER_ARRAY_HPP_
#define TUW_STD_MSGS__PARAMETER_ARRAY_HPP_

#include <tuw_std_msgs/msg/parameter_array.hpp>
#include <tuw_std_msgs/parameter.hpp>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <vector>
#include <utility>

namespace tuw_std_msgs
{
struct ParameterArray : public tuw_std_msgs::msg::ParameterArray
{
  ParameterArray() {}

  template<typename T>
  ParameterArray(const std::vector<std::string> & names, const std::vector<T> & values)
  {
    for (size_t i = 0; i < names.size(); i++) {
      Parameter p(names[i], values[i]);
      this->data.push_back(std::move(p));
    }
  }

  /**
   * searches in the array for a parameter name
   * @param name name of the parameter
   * @param data value to add
   * @return false if it exited and it was set, true if was newly added
   */
  template<typename T>
  bool add(const std::string & name, const T & data)
  {
    Parameter * p = get(name);
    if (p != NULL) {
      p->set(data);
      return false;
    } else {
      Parameter p(name, data);
      this->data.push_back(std::move(p));
      return true;
    }
  }

  /**
    * searches in the array for a parameter name
    * @param name name of the parameter
    * @param data value to add
    * @param precision precision used
    * @return false if it exited and it was set, true if was newly added
    */
  template<typename T>
  bool add(const std::string & name, const T & data, int precision)
  {
    Parameter * p = get(name);
    if (p != NULL) {
      p->set(data, precision);
      return false;
    } else {
      Parameter p(name, data, precision);
      this->data.push_back(std::move(p));
      return true;
    }
  }
  /**
   * searches in the array for a parameter name
   * @param name name of the parameter
   * @return pointer to the parameter or null if it does not exist
   * @see
   */
  const Parameter * get(const std::string & name) const
  {
    for (size_t i = 0; i < this->data.size(); i++) {
      const Parameter * p = static_cast<const Parameter *>(&this->data[i]);
      if (p->name == name) {
        return p;
      }
    }
    return NULL;
  }
  Parameter * get(const std::string & name)
  {
    return const_cast<Parameter *>(static_cast<const ParameterArray &>(*this).get(name));
  }
  const Parameter & operator[](const std::string & name) const
  {
    for (const auto & param : this->data) {
      if (param.name == name) {
        return static_cast<const Parameter &>(param);
      }
    }
    throw std::out_of_range("Parameter not found");
  }
  Parameter & operator[](const std::string & name)
  {
    return const_cast<Parameter &>(static_cast<const ParameterArray &>(*this)[name]);
  }
  template<typename T>
  T value(const std::string & name) const
  {
    for (const auto & p : this->data) {
      if (p.name == name) {
        return static_cast<const Parameter &>(p).get<T>();
      }
    }
    throw std::out_of_range("Parameter not found");
  }
  template<typename T>
  T value(const std::string & name)
  {
    return static_cast<const ParameterArray &>(*this).value<T>(name);
  }

  /**
   * used to read a parameter
   * @param name name of the parameter
   * @param des varaible to filled with the parameter if exists
   * @return true if the parameter exists
   */
  template<typename T>
  bool get(const std::string & name, T & des) const
  {
    const Parameter * p = get(name);
    if (p != NULL) {
      p->get(des);
      return true;
    }
    return false;
  }
};
}  // namespace tuw_std_msgs
#endif  // TUW_STD_MSGS__PARAMETER_ARRAY_HPP_
