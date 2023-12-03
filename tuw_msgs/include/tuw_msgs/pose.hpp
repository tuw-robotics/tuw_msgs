#ifndef TUW_MSGS__POSE_HPP_
#define TUW_MSGS__POSE_HPP_

#include <tuw_msgs/point.hpp>
#include <tuw_msgs/quaternion.hpp>
#include <geometry_msgs/msg/pose.hpp>

namespace tuw_msgs
{
    std::string &encode(geometry_msgs::msg::Pose &src, std::string &des);
    size_t decode(geometry_msgs::msg::Pose &des, std::string &line, size_t pos = 0);
  struct Pose : public geometry_msgs::msg::Pose
  {
    Pose() {};
    Pose(double px, double py, double pz)
    {
      this->set(px, py, pz, 0., 0., 0., 1.);
    };
    Pose(const Point &p, const Quaternion &q)
    {
      this->set(p, q);
    };
    Pose(double px, double py, double pz, double qx, double qy, double qz, double qw)
    {
      this->set(px, py, pz, qx, qy, qz, qw);
    };
    Pose &set(double px, double py, double pz, double qx, double qy, double qz, double qw)
    {
      get_position().set(px, py, pz);
      get_orientation().set(qx, qy, qz, qw);
      return *this;
    };
    Pose &set(const Point &p)
    {
      this->position = p; 
      return *this;
    };
    Pose &set(const Quaternion &q)
    {
      this->orientation = q; 
      return *this;
    };
    Pose &set(const Point &p, const Quaternion &q)
    {
      this->set(p), this->set(q); 
      return *this;
    };
    Point &get_position(){
      return static_cast<Point&>(this->position);
    }
    const Point &get_position() const {
      return static_cast<const Point&>(this->position);
    }
    Quaternion &get_orientation(){
      return static_cast<Quaternion&>(this->orientation);
    }
    const Quaternion &get_orientation() const {
      return static_cast<const Quaternion&>(this->orientation);
    }
    bool operator==(const Pose& rhs) const { 
      return (get_position() == rhs.get_position()) && (get_orientation() == rhs.get_orientation()); 
    }
  };

}
#endif // TUW_MSGS__POSE_HPP_
