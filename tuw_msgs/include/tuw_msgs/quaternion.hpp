#ifndef TUW_MSGS__QUATERNION_HPP_
#define TUW_MSGS__QUATERNION_HPP_

#include <cmath>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tuw_msgs/utils.hpp>
namespace Json{
  class Value;
}
namespace tuw_msgs
{
struct Quaternion : public geometry_msgs::msg::Quaternion
{
  Quaternion() {}
  Quaternion(double x, double y, double z, double w) {this->set(x, y, z, w);}
  Quaternion(double roll, double pitch, double yaw) {this->from_rpy(roll, pitch, yaw);}
  Quaternion(const std::string & src) {this->from_str(src);}
  Quaternion & set(double x, double y, double z, double w)
  {
    this->x = x, this->y = y, this->z = z, this->w = w;
    return *this;
  }

  Quaternion & from_rpy(double roll, double pitch, double yaw);

  Quaternion & get_orientation();
  const Quaternion & get_orientation() const;
  bool operator==(const geometry_msgs::msg::Quaternion & rhs) const;
  bool is_zero() const;
  bool similar(const Quaternion & rhs, double epsilon = 0.0001) const;
  geometry_msgs::msg::Quaternion & msg();
  const geometry_msgs::msg::Quaternion & msg() const;
  size_t from_str(const std::string & src);
  std::string to_str(tuw_msgs::Format format = LOOSE) const;
  std::string & to_str(
    std::string & des, tuw_msgs::Format format = LOOSE, bool append = false) const;
  Json::Value toJson() const;
  static Quaternion fromJson(const Json::Value& jsonValue);
  static Quaternion &fromJson(const Json::Value& json, Quaternion &des);
};
}  // namespace tuw_msgs
#endif  // TUW_MSGS__QUATERNION_HPP_
