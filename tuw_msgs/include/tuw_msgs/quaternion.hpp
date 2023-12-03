#ifndef TUW_MSGS__QUATERNION_HPP_
#define TUW_MSGS__QUATERNION_HPP_

#include <geometry_msgs/msg/quaternion.hpp>
#include <cmath>
#include <tuw_msgs/utils.hpp>

namespace tuw_msgs
{
  struct Quaternion : public geometry_msgs::msg::Quaternion
  {
    Quaternion(){};
    Quaternion(double x, double y, double z, double w)
    {
      this->set(x, y, z, w);
    };
    Quaternion(double roll, double pitch, double yaw)
    {
      this->from_rpy(roll, pitch, yaw);
    };
    Quaternion(const std::string &src)
    {
      this->from_str(src);
    };
    Quaternion &set(double x, double y, double z, double w)
    {
      this->x = x, this->y = y, this->z = z, this->w = w;
      return *this;
    };

    Quaternion &from_rpy(double roll, double pitch, double yaw);

    Quaternion &get_orientation(){
      return static_cast<Quaternion&>(*this);
    }
    const Quaternion &get_orientation() const {
      return static_cast<const Quaternion&>(*this);
    }
    bool operator==(const geometry_msgs::msg::Quaternion &rhs) const
    {
      return (x == rhs.x) && (y == rhs.y) && (z == rhs.z) && (w == rhs.w);
    }
    bool is_zero() const;
    
    double similar(const Quaternion &rhs, double threshold = 0.0001) const
    {
      double d = std::sqrt( std::pow(x - rhs.x, 2) + std::pow(y - rhs.y, 2) + std::pow(z - rhs.z, 2) + std::pow(w - rhs.w, 2));
      return (fabs(d) < threshold);
    }
    Quaternion &from_str(const std::string &src);
    std::string to_str(tuw_msgs::Format format = LOOSE) const;
    std::string &to_str(std::string &des, tuw_msgs::Format format = LOOSE, bool append = false) const;
  };
}
#endif // TUW_MSGS__QUATERNION_HPP_
