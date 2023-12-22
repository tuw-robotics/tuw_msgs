#include <algorithm>
#include <cctype>
#include <cmath>
#include <iostream>
#include <string>
#include <tuw_msgs/utils.hpp>

size_t tuw_msgs::nr_of_leading_spaces(const std::string & inputString)
{
  size_t nr_of_spaces = 0;
  for (char ch : inputString) {
    if (ch == ' ') {
      nr_of_spaces++;
    } else {
      break;
    }
  }
  return nr_of_spaces;
}
std::string tuw_msgs::remove_spaces(std::string str)
{
  str.erase(
    remove_if(str.begin(), str.end(), [](unsigned char x) {return std::isspace(x);}), str.end());
  return str;
}

bool tuw_msgs::is_equal(const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b)
{
  return (a.x == b.x) && (a.y == b.y) && (a.z == b.z);
}
bool tuw_msgs::is_equal(
  const geometry_msgs::msg::Quaternion & a, const geometry_msgs::msg::Quaternion & b)
{
  return (a.x == b.x) && (a.y == b.y) && (a.z == b.z) && (a.w == b.w);
}
bool tuw_msgs::is_equal(const geometry_msgs::msg::Pose & a, const geometry_msgs::msg::Pose & b)
{
  return is_equal(a.position, b.position) && is_equal(a.orientation, b.orientation);
}

bool tuw_msgs::is_similar(
  const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b, double epsilon)
{
  return is_similar(a.x, b.x, epsilon) && is_similar(a.y, b.y, epsilon) &&
         is_similar(a.z, b.z, epsilon);
}

bool tuw_msgs::is_similar(
  const geometry_msgs::msg::Quaternion & a, const geometry_msgs::msg::Quaternion & b,
  double epsilon)
{
  return is_similar(a.x, b.x, epsilon) && is_similar(a.y, b.y, epsilon) &&
         is_similar(a.z, b.z, epsilon) && is_similar(a.w, b.w, epsilon);
}

bool tuw_msgs::is_similar(
  const geometry_msgs::msg::Pose & a, const geometry_msgs::msg::Pose & b, double epsilon_position,
  double epsilon_orientation)
{
  return is_similar(a.position, b.position, epsilon_position) &&
         is_similar(a.orientation, b.orientation, epsilon_orientation);
}

void tuw_msgs::rpy_to_quaternion(
  double roll, double pitch, double yaw, double & qx, double & qy, double & qz, double & qw)
{
  double halfYaw = yaw * 0.5;
  double halfPitch = pitch * 0.5;
  double halfRoll = roll * 0.5;
  double cosYaw = std::cos(halfYaw);
  double sinYaw = std::sin(halfYaw);
  double cosPitch = std::cos(halfPitch);
  double sinPitch = std::sin(halfPitch);
  double cosRoll = std::cos(halfRoll);
  double sinRoll = std::sin(halfRoll);
  qx = sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw;  // x
  qy = cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw;  // y
  qz = cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw;  // z
  qw = cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw;  // formerly yzx
}

bool tuw_msgs::is_similar_with_cout(const std::string & a, const std::string & b)
{
  bool is_same = is_similar(a, b);
  if (!is_same) {
    std::cout << "a: "
              << "\033[0;35m" << a << "\033[0m" << std::endl;
    std::cout << "b: "
              << "\033[0;35m" << b << "\033[0m" << std::endl;
  }
  return is_same;
}
bool tuw_msgs::is_similar(const std::string & a, const std::string & b)
{
  using namespace std;

  auto it_a = a.begin();
  auto it_b = b.begin();
  while (it_a != a.end() || it_b != b.end()) {
    while (isspace(static_cast<unsigned char>(*it_a))) {
      it_a++;
    }
    while (isspace(static_cast<unsigned char>(*it_b))) {
      it_b++;
    }
    if (*it_a != *it_b) {
      return false;
    }
    it_a++, it_b++;
    while (isspace(static_cast<unsigned char>(*it_a))) {
      it_a++;
    }
    while (isspace(static_cast<unsigned char>(*it_b))) {
      it_b++;
    }
  }
  return true;
}
