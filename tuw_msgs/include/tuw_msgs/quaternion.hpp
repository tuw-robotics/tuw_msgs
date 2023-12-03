#ifndef TUW_MSGS__QUATERNION_HPP_
#define TUW_MSGS__QUATERNION_HPP_

#include <geometry_msgs/msg/quaternion.hpp>
#include <cmath>
#include <tuw_msgs/utils.hpp>

namespace tuw_msgs
{
  geometry_msgs::msg::Quaternion &to_msg(double x, double y, double z, double w, geometry_msgs::msg::Quaternion &des);
  geometry_msgs::msg::Quaternion &to_msg(double roll, double pitch, double yaw, geometry_msgs::msg::Quaternion &des);

  std::string &encode(geometry_msgs::msg::Quaternion &src, std::string &des);
  size_t decode(geometry_msgs::msg::Quaternion &des, std::string &line, size_t pos = 0);

  bool is_zero(const geometry_msgs::msg::Quaternion &src);
  geometry_msgs::msg::Quaternion &to_msg(double x, double y, double z, geometry_msgs::msg::Quaternion &des);
  std::string to_str(const geometry_msgs::msg::Quaternion &src, tuw_msgs::Format format = LOOSE);
  std::string &to_str(const geometry_msgs::msg::Quaternion &src, std::string &des, tuw_msgs::Format format = LOOSE);
  geometry_msgs::msg::Quaternion &from_str(const std::string &src, geometry_msgs::msg::Quaternion &des);

  struct Quaternion : public geometry_msgs::msg::Quaternion
  {
    Quaternion(){};
    Quaternion(double x, double y, double z, double w)
    {
      this->set(x, y, z, w);
    };
    Quaternion(double roll, double pitch, double yaw)
    {
      this->rpy(roll, pitch, yaw);
    };
    Quaternion(const std::string &src)
    {
      tuw_msgs::from_str(src, *this);
    };
    Quaternion &set(double x, double y, double z, double w)
    {
      this->x = x, this->y = y, this->z = z, this->w = w;
      return *this;
    };
    Quaternion &rpy(double roll, double pitch, double yaw)
    {
      to_msg(roll, pitch, yaw, *this);
      return *this;
    };
    bool operator==(const geometry_msgs::msg::Quaternion &rhs) const
    {
      return (x == rhs.x) && (y == rhs.y) && (z == rhs.z) && (w == rhs.w);
    }
    double difference(const geometry_msgs::msg::Quaternion &rhs) const
    {
      
      return std::sqrt( std::pow(x - rhs.x, 2) + std::pow(y - rhs.y, 2) + std::pow(z - rhs.z, 2) + std::pow(w - rhs.w, 2));
    }
    double close_to(const geometry_msgs::msg::Quaternion &rhs, double threshold = 0.0001) const
    {
      return (fabs(difference(rhs)) < threshold);
    }
    static const geometry_msgs::msg::Quaternion from_str(const std::string &src)
    {
      geometry_msgs::msg::Quaternion des;
      return tuw_msgs::from_str(src, des);
    }
  };
}
#endif // TUW_MSGS__QUATERNION_HPP_
