#ifndef TUW_MSGS_MSGS__GEO_POINT_HPP_
#define TUW_MSGS_MSGS__GEO_POINT_HPP_

#include <geographic_msgs/msg/geo_point.hpp>

namespace geographic_msgs
{
struct GeoPoint : public geographic_msgs::msg::GeoPoint
{
  GeoPoint() {this->latitude = 0., this->longitude = 0., this->altitude = 0.;}
  GeoPoint(double latitude, double longitude, double altitude)
  {
    this->latitude = latitude, this->longitude = longitude, this->altitude = altitude;
  }
};
}  // namespace geographic_msgs
#endif  // TUW_MSGS_MSGS__GEO_POINT_HPP_
