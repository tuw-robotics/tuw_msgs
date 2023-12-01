#include <iostream>
#include <tuw_msgs/utils.hpp>
#include <tuw_msgs/serialize.hpp>

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;
  std::string str("[[3.3, 221.2, 1], [3.3, 221.2, 1, 0.2]]");

  geometry_msgs::msg::Pose p;
  size_t pos = 0;
  pos = tuw_msgs::decode(p, str, 0);

  std::cout << pos;
}
