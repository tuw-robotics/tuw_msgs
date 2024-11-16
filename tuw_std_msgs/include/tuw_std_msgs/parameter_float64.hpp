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

#ifndef TUW_STD_MSGS__PARAMETER_FLOAT64_HPP_
#define TUW_STD_MSGS__PARAMETER_FLOAT64_HPP_

#include <tuw_std_msgs/msg/parameter_float64.hpp>
#include <limits>
#include <string>

namespace tuw_std_msgs
{
struct ParameterFloat64 : public tuw_std_msgs::msg::ParameterFloat64
{
  ParameterFloat64() {this->data = std::numeric_limits<double>::quiet_NaN();}
  explicit ParameterFloat64(const std::string & name)
  {
    this->name.data = name;
    this->data = std::numeric_limits<double>::quiet_NaN();
  }
  ParameterFloat64(const std::string & name, const double & data)
  {
    this->name.data = name;
    this->data = data;
  }
  double & get(double & data) const
  {
    data = this->data;
    return data;
  }
  double get() const {return this->data;}
};
}  // namespace tuw_std_msgs
#endif  // TUW_STD_MSGS__PARAMETER_FLOAT64_HPP_
