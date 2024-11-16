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

#include "gtest/gtest.h"
#include "tuw_std_msgs/parameter_array.hpp"

TEST(json, ParameterArray_double)
{
  std::vector<std::string> names = {"length", "width"};
  std::vector<double> values = {22.9, 44.3};
  tuw_std_msgs::ParameterArray parameters(names, values);
  double des;
  parameters.get(names[0], des);
  ASSERT_EQ(values[0], des);
  ASSERT_EQ(parameters.get(names[0], des), true);
  ASSERT_EQ(parameters.get("border", des), false);
  const tuw_std_msgs::Parameter * p = parameters.get(names[1]);
  ASSERT_EQ(values[1], p->get<double>());
  double v1 = 120.;
  double border = 10.;
  ASSERT_EQ(parameters.add("border", v1), true);
  ASSERT_EQ(parameters.add("border", border), false);
  ASSERT_EQ(parameters.get("border", des), true);
  ASSERT_EQ(des, border);
  double l1 = 1.1;
  double l2 = 2.2;
  double tolerance = 0.001;
  std::vector<double> list_values = {l1, l2};
  ASSERT_EQ(parameters.add("list", list_values), true);
  ASSERT_NEAR(parameters.get("list")->get<std::vector<double>>()[0], l1, tolerance);
  ASSERT_NEAR(parameters.get("list")->get<std::vector<double>>()[1], l2, tolerance);
  ASSERT_NEAR(parameters.get("list")->at<double>(1), l2, tolerance);
  ASSERT_EQ(parameters.value<double>("border"), border);
}

TEST(json, ParameterArray_int)
{
  std::vector<std::string> names = {"length", "width"};
  std::vector<int> values = {22, 44};
  tuw_std_msgs::ParameterArray parameters(names, values);
  int des;
  parameters.get(names[0], des);
  ASSERT_EQ(values[0], des);
  ASSERT_EQ(parameters.get(names[0], des), true);
  ASSERT_EQ(parameters.get("border", des), false);
  const tuw_std_msgs::Parameter * p = parameters.get(names[1]);
  ASSERT_EQ(values[1], p->get<double>());
  int v1 = 120;
  int border = 10;
  ASSERT_EQ(parameters.add("border", v1), true);
  ASSERT_EQ(parameters.add("border", border), false);
  ASSERT_EQ(parameters.get("border", des), true);
  ASSERT_EQ(des, border);
  int l1 = 1;
  int l2 = 2;
  std::vector<int> list_values = {l1, l2};
  ASSERT_EQ(parameters.add("list", list_values), true);
  ASSERT_EQ(parameters.get("list")->get<std::vector<int>>()[0], l1);
  ASSERT_EQ(parameters.get("list")->get<std::vector<int>>()[1], l2);
  ASSERT_EQ(parameters.get("list")->at<int>(1), l2);
  ASSERT_EQ(parameters["list"].at<int>(1), l2);
  ASSERT_EQ(parameters.value<int>("border"), border);
}
