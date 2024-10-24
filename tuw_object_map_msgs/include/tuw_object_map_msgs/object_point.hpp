#ifndef TUW_OBJECT_MAP_MSGS_MSGS__OBJECT_POINT_HPP_
#define TUW_OBJECT_MAP_MSGS_MSGS__OBJECT_POINT_HPP_

#include <tuw_object_map_msgs/msg/object_point.hpp>
#include <tuw_object_map_msgs/geo_point.hpp>
#include <tuw_geometry_msgs/point.hpp>

namespace tuw_object_map_msgs
{
struct ObjectPoint : public tuw_object_map_msgs::msg::ObjectPoint
{
  ObjectPoint() 
  {
    this->id = tuw_object_map_msgs::msg::ObjectPoint::ID_NA;
    this->type = tuw_object_map_msgs::msg::ObjectPoint::TYPE_NA;
    this->wgs84 = tuw_object_map_msgs::GeoPoint();
    this->map = tuw_geometry_msgs::Point();
  }
  ObjectPoint(int64_t id) 
  {
    this->id = id;
    this->type = tuw_object_map_msgs::msg::ObjectPoint::TYPE_NA;
  }

  ObjectPoint(int64_t id, uint32_t type) 
  {
    this->id = id;
    this->type = type;
  }

  ObjectPoint(int64_t id, uint32_t type, const geometry_msgs::msg::Point &map) 
  {
    this->id = id;
    this->type = type;
    this->wgs84 = tuw_object_map_msgs::GeoPoint();
    this->map = map;
  }

  ObjectPoint(int64_t id, uint32_t type, const tuw_object_map_msgs::msg::GeoPoint &wgs84) 
  {
    this->id = id;
    this->type = type;
    this->wgs84 = wgs84;
    this->map = tuw_geometry_msgs::Point();
  }

  ObjectPoint(const tuw_object_map_msgs::msg::GeoPoint &p)
  {
    this->wgs84 = p;
  }
  
  ObjectPoint(const geometry_msgs::msg::Point &p)
  {
    this->map = p;
  }
};
}  // namespace tuw_object_map_msgs
#endif  // TUW_OBJECT_MAP_MSGS_MSGS__OBJECT_POINT_HPP_
