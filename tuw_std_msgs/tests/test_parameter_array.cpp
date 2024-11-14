#include "gtest/gtest.h"
#include "tuw_std_msgs/parameter_array.hpp"

TEST(json, ParameterArray_double)
{
  std::vector<std::string> names = {"length", "width"};
  std::vector<double> values = {22.9, 44.3};
  tuw_std_msgs::ParameterArray parameters(2, names, values);
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
  tuw_std_msgs::ParameterArray parameters(2, names, values);
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
