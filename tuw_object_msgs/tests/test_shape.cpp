#include "gtest/gtest.h"
#include "tuw_object_msgs/shape.hpp"

TEST(json, Parameter)
{
  int64_t id = 1;
  tuw_object_msgs::Shape shape(id, 22);
  ASSERT_EQ(shape.id, id);
}
