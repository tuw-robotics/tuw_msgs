#include <tuw_msgs/utils.hpp>

#include "gtest/gtest.h"

TEST(utlis, is_similar_string)
{
  ASSERT_TRUE(tuw_msgs::is_similar("hihi", "hihi"));
  ASSERT_TRUE(tuw_msgs::is_similar("hihi", "hi hi"));
  ASSERT_TRUE(tuw_msgs::is_similar("h ihi", "hi hi"));
  ASSERT_TRUE(tuw_msgs::is_similar(" h ihi", "hi hi"));
  ASSERT_TRUE(tuw_msgs::is_similar("hihi", "hihi "));
  ASSERT_TRUE(tuw_msgs::is_similar("hihi ", "hihi "));
  ASSERT_TRUE(tuw_msgs::is_similar("hihi ", "hihi"));
}
