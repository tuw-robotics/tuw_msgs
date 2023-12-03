#ifndef TUW_MSGS__POSE_HPP_
#define TUW_MSGS__POSE_HPP_

#include <tuw_msgs/point.hpp>
#include <tuw_msgs/quaternion.hpp>
#include <geometry_msgs/msg/pose.hpp>

namespace tuw_msgs
{

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
    bool operator==(const Pose& rhs) const ;

    double similar(const Pose &rhs, double threshold_position = 0.0001, double threshold_orientation = 0.0001) const;

    bool is_zero() const;
    std::string to_str(tuw_msgs::Format format = LOOSE) const;
    std::string &to_str(std::string &des, tuw_msgs::Format format = LOOSE, bool append = false) const;
    Pose &from_str(const std::string &src);
  };

}
#endif // TUW_MSGS__POSE_HPP_
