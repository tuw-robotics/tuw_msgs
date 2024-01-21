#ifndef TUW_OBJECT_MAP_MSGS_MSGS__GEO_POINT_HPP_
#define TUW_OBJECT_MAP_MSGS_MSGS__GEO_POINT_HPP_

#include <tuw_object_map_msgs/msg/geo_point.hpp>

namespace tuw_object_map_msgs
{
struct GeoPoint : public tuw_object_map_msgs::msg::GeoPoint
{
  GeoPoint() {this->latitude = 0., this->longitude = 0., this->altitude = 0.;}
  GeoPoint(double latitude, double longitude, double altitude) {this->latitude = latitude, this->longitude = longitude, this->altitude = altitude;}
};
}  // namespace tuw_object_map_msgs
#endif  // TUW_OBJECT_MAP_MSGS_MSGS__GEO_POINT_HPP_
