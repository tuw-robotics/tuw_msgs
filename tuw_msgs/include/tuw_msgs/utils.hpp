#ifndef TUW_MSGS__UTILS_HPP_
#define TUW_MSGS__UTILS_HPP_

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

namespace tuw_msgs
{
  geometry_msgs::msg::Quaternion &to_msg(double x, double y, double z, double w, geometry_msgs::msg::Quaternion &des);
  geometry_msgs::msg::Quaternion &to_msg(double roll, double pitch, double yaw, geometry_msgs::msg::Quaternion &des);
  geometry_msgs::msg::Point &to_msg(double x, double y, double z, geometry_msgs::msg::Point &des);

  struct Point : public geometry_msgs::msg::Point
  {
    Point() {};
    Point(double x, double y, double z)
    {
      this->set(x, y, z);
    };
    Point &set(double x, double y, double z)
    {
      this->x = x, this->y = y, this->z = z;
      return *this;
    };
    bool operator==(const Point& rhs) const { 
      return (x ==  rhs.x) && (y ==  rhs.y) && (z ==  rhs.z); 
    }
  };

  struct Quaternion : public geometry_msgs::msg::Quaternion
  {
    Quaternion() {};
    Quaternion(double x, double y, double z, double w)
    {
      this->set(x, y, z, w);
    };
    Quaternion(double roll, double pitch, double yaw)
    {
      this->rpy(roll, pitch, yaw);
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
    bool operator==(const Quaternion& rhs) const { 
      return (x ==  rhs.x) && (y ==  rhs.y) && (z ==  rhs.z) && (w ==  rhs.w); 
    }
  };

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
#endif // TUW_MSGS__SERIALIZE_HPP_
