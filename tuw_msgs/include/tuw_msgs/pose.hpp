#ifndef TUW_MSGS__POSE_HPP_
#define TUW_MSGS__POSE_HPP_

#include <geometry_msgs/msg/pose.hpp>
#include <tuw_msgs/point.hpp>
#include <tuw_msgs/quaternion.hpp>

namespace tuw_msgs
{

struct Pose : public geometry_msgs::msg::Pose
{
  Pose()
  {
    this->position.x = 0, this->position.y = 0, this->position.z = 0;
    this->orientation.x = 0, this->orientation.y = 0, this->orientation.z = 0,
    this->orientation.w = 1;
  }
  Pose(double px, double py, double pz)
  {
    this->position.x = px, this->position.y = py, this->position.z = pz;
    this->orientation.x = 0, this->orientation.y = 0, this->orientation.z = 0,
    this->orientation.w = 1;
  }
  Pose(const Point & p, const Quaternion & q)
  {
    this->position.x = p.x, this->position.y = p.y, this->position.z = p.z;
    this->orientation.x = q.x, this->orientation.y = q.y, this->orientation.z = q.z,
    this->orientation.w = q.w;
  }
  Pose(double px, double py, double pz, double qx, double qy, double qz, double qw)
  {
    this->position.x = px, this->position.y = py, this->position.z = pz;
    this->orientation.x = qx, this->orientation.y = qy, this->orientation.z = qz,
    this->orientation.w = qw;
  }
  Pose(const std::string & str) {this->from_str(str);}
  Pose & set(double px, double py, double pz, double qx, double qy, double qz, double qw)
  {
    get_position().set(px, py, pz);
    get_orientation().set(qx, qy, qz, qw);
    return *this;
  }
  Pose & set(const Point & p)
  {
    this->position = p;
    return *this;
  }
  Pose & set(const Quaternion & q)
  {
    this->orientation = q;
    return *this;
  }
  Pose & set(const Point & p, const Quaternion & q)
  {
    this->set(p), this->set(q);
    return *this;
  }
  Point & get_position() {return static_cast<Point &>(this->position);}
  const Point & get_position() const {return static_cast<const Point &>(this->position);}
  Quaternion & get_orientation() {return static_cast<Quaternion &>(this->orientation);}
  const Quaternion & get_orientation() const
  {
    return static_cast<const Quaternion &>(this->orientation);
  }
  geometry_msgs::msg::Pose & msg() {return static_cast<geometry_msgs::msg::Pose &>(*this);}
  const geometry_msgs::msg::Pose & msg() const
  {
    return static_cast<const geometry_msgs::msg::Pose &>(*this);
  }
  bool operator==(const Pose & rhs) const;

  bool similar(
    const Pose & rhs, double epsilon_position = 0.0001, double epsilon_orientation = 0.0001) const;

  bool is_zero() const;
  std::string to_str(tuw_msgs::Format format = LOOSE) const;
  std::string & to_str(
    std::string & des, tuw_msgs::Format format = LOOSE, bool append = false) const;
  size_t from_str(const std::string & src);

  template<typename T>
  static size_t from_str(const std::string & str, T & des)
  {
    size_t offset = str.find("[");
    if (offset == std::string::npos) {
      throw std::runtime_error("Failed decode pose vector: " + str);
    }
    offset++;
    while ((offset = str.find("[", offset)) != std::string::npos) {
      Pose pose;
      std::string sub_str = str.substr(offset);
      offset = pose.from_str(sub_str) + offset;
      des.push_back(std::move(pose));
      if (str.find(";", offset) == std::string::npos) {
        break;
      }
    }
    offset = str.find("]", offset);
    if (offset == std::string::npos) {throw std::runtime_error("Failed decode Point: " + str);}
    return offset;
  }
  int json_get(Json::Value &value);
  int json_add(const char* key, Json::Value &value);
};

}  // namespace tuw_msgs
#endif  // TUW_MSGS__POSE_HPP_
