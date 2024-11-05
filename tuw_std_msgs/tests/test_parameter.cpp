#include "gtest/gtest.h"
#include "tuw_std_msgs/parameter.hpp"

TEST(json, Parameter)
{
  double length = 22.9;
  tuw_std_msgs::Parameter p("length", 22.9);
  double des;
  p.get(des);
  ASSERT_EQ(length, des);
  ASSERT_EQ(length, p.get<double>());
}