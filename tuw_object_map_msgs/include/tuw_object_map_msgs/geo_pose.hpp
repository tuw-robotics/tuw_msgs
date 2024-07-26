#ifndef TUW_OBJECT_MAP_MSGS_MSGS__GEO_POSE_HPP_
#define TUW_OBJECT_MAP_MSGS_MSGS__GEO_POSE_HPP_

#include <tuw_object_map_msgs/msg/geo_pose.hpp>

namespace tuw_object_map_msgs
{
struct GeoPose : public tuw_object_map_msgs::msg::GeoPose
{
  GeoPose()
  {
    this->position.x = 0, this->position.y = 0, this->position.z = 0;
    this->orientation.x = 0, this->orientation.y = 0, this->orientation.z = 0,
    this->orientation.w = 1;
  }
  GeoPose(double latitude, double longitude, double altitude)
  {
    this->position.latitude = latitude, this->position.longitude = longitude, this->position.altitude = altitude;
    this->orientation.x = 0, this->orientation.y = 0, this->orientation.z = 0,
    this->orientation.w = 1;
  }
  GeoPose(double latitude, double longitude, double altitude, double qx, double qy, double qz, double qw)
  {
    this->position.latitude = latitude, this->position.longitude = longitude, this->position.altitude = altitude;
    this->orientation.x = qx, this->orientation.y = qy, this->orientation.z = qz,
    this->orientation.w = qw;
  }
  GeoPose(const GeoPoint & p, const Quaternion & q)
  {
    this->position.latitude = p.latitude, this->position.longitude = p.longitude, this->position.altitude = p.altitude;
    this->orientation.x = q.x, this->orientation.y = q.y, this->orientation.z = q.z,
    this->orientation.w = q.w;
  }
};
}  // namespace tuw_object_map_msgs
#endif  // TUW_OBJECT_MAP_MSGS_MSGS__GEO_POSE_HPP_
