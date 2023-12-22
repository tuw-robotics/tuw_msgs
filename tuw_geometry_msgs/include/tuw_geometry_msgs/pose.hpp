#ifndef TUW_GEOMETRY_MSGS__POSE_HPP_
#define TUW_GEOMETRY_MSGS__POSE_HPP_

#include <geometry_msgs/msg/pose.hpp>
#include <tuw_geometry_msgs/point.hpp>
#include <tuw_geometry_msgs/quaternion.hpp>

namespace tuw_geometry_msgs
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
  Pose(double px, double py, double pz, double qx, double qy, double qz, double qw)
  {
    this->position.x = px, this->position.y = py, this->position.z = pz;
    this->orientation.x = qx, this->orientation.y = qy, this->orientation.z = qz,
    this->orientation.w = qw;
  }
  Pose(const Point & p, const Quaternion & q)
  {
    this->position.x = p.x, this->position.y = p.y, this->position.z = p.z;
    this->orientation.x = q.x, this->orientation.y = q.y, this->orientation.z = q.z,
    this->orientation.w = q.w;
  }
};

}  // namespace tuw_geometry_msgs
#endif  // TUW_GEOMETRY_MSGS__POSE_HPP_
