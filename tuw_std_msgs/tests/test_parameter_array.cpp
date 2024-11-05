#include "gtest/gtest.h"
#include "tuw_std_msgs/parameter_array.hpp"

TEST(json, ParameterArray)
{
  std::vector<std::string> names = {"length", "width"};
  std::vector<double> values = {22.9, 44.3};
  tuw_std_msgs::ParameterArray parameters(2, names, values);
  double des;
  parameters.get(names[0], des);
  ASSERT_EQ(names[0], des);

  const tuw_std_msgs::Parameter *p = parameters.get(names[1]);
  ASSERT_EQ(names[1], p->get<double>());
}