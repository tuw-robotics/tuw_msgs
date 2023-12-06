#ifndef TUW_MSGS__POSE_HPP_
#define TUW_MSGS__POSE_HPP_

#include <tuw_msgs/point.hpp>
#include <tuw_msgs/quaternion.hpp>
#include <geometry_msgs/msg/pose.hpp>

namespace tuw_msgs
{

  struct Pose : public geometry_msgs::msg::Pose
  {
    Pose(){};
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
    Pose(const std::string &str)
    {
      this->from_str(str);
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
    Point &get_position()
    {
      return static_cast<Point &>(this->position);
    }
    const Point &get_position() const
    {
      return static_cast<const Point &>(this->position);
    }
    Quaternion &get_orientation()
    {
      return static_cast<Quaternion &>(this->orientation);
    }
    const Quaternion &get_orientation() const
    {
      return static_cast<const Quaternion &>(this->orientation);
    }
    geometry_msgs::msg::Pose &msg()
    {
      return static_cast<geometry_msgs::msg::Pose &>(*this);
    }
    const geometry_msgs::msg::Pose &msg() const
    {
      return static_cast<const geometry_msgs::msg::Pose &>(*this);
    }
    bool operator==(const Pose &rhs) const;

    bool similar(const Pose &rhs, double epsilon_position = 0.0001, double epsilon_orientation = 0.0001) const;

    bool is_zero() const;
    std::string to_str(tuw_msgs::Format format = LOOSE) const;
    std::string &to_str(std::string &des, tuw_msgs::Format format = LOOSE, bool append = false) const;
    size_t from_str(const std::string &src);

    template <typename T>
    static size_t from_str(const std::string &str, T &des)
    {
      size_t offset = str.find("[");
      if (offset == std::string::npos)
        throw std::runtime_error("Failed decode pose vector: " + str);
      offset++;
      while ((offset = str.find("[", offset)) != std::string::npos)
      {
        Pose pose;
        std::string sub_str = str.substr(offset);
        offset = pose.from_str(sub_str) + offset;
        des.push_back(std::move(pose));
        if (str.find(";", offset) == std::string::npos){
            break;
        }
      }
      offset = str.find("]", offset);
      if (offset == std::string::npos)
        throw std::runtime_error("Failed decode Point: " + str);
      return offset;
    }

  };
  

}
#endif // TUW_MSGS__POSE_HPP_
