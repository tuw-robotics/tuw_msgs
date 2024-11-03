#ifndef TUW_MAP_MSGS_MSGS__OBJECT_POINT_HPP_
#define TUW_MAP_MSGS_MSGS__OBJECT_POINT_HPP_

#include <tuw_std_msgs/parameter.hpp>
#include <tuw_geometry_msgs/point.hpp>
#include <tuw_msgs/geo_point.hpp>
#include <tuw_map_msgs/msg/object_point.hpp>

namespace tuw_map_msgs
{
struct ObjectPoint : public tuw_map_msgs::msg::ObjectPoint
{
  ObjectPoint()
  {
    this->id = tuw_map_msgs::msg::ObjectPoint::ID_NA;
    this->type = tuw_map_msgs::msg::ObjectPoint::TYPE_NA;
    this->wgs84 = geographic_msgs::GeoPoint();
    this->map = tuw_geometry_msgs::Point();
  }
  ObjectPoint(int64_t id)
  {
    this->id = id;
    this->type = tuw_map_msgs::msg::ObjectPoint::TYPE_NA;
  }

  ObjectPoint(int64_t id, uint32_t type)
  {
    this->id = id;
    this->type = type;
  }

  ObjectPoint(int64_t id, uint32_t type, const geometry_msgs::msg::Point & map)
  {
    this->id = id;
    this->type = type;
    this->wgs84 = geographic_msgs::GeoPoint();
    this->map = map;
  }

  ObjectPoint(int64_t id, uint32_t type, const geographic_msgs::msg::GeoPoint & wgs84)
  {
    this->id = id;
    this->type = type;
    this->wgs84 = wgs84;
    this->map = tuw_geometry_msgs::Point();
  }

  ObjectPoint(const geographic_msgs::msg::GeoPoint & p) { this->wgs84 = p; }

  ObjectPoint(const geometry_msgs::msg::Point & p) { this->map = p; }

  /**
   * used to read a parameter
   * @param name name of the parameter
   * @param des varaible to filled with the parameter if exists
   * @return true if the parameter exists
   */
  template<typename T>
  bool getParameter(const std::string &name, T &des) const{
    for(size_t i = 0; i < this->parameters.size(); i++){
      tuw_std_msgs::Parameter *p = (tuw_std_msgs::Parameter*) &this->parameters[i] ;
      if(p->name == name){
        p->get(des);
        return true;
      }
    }
    return false;
  }
};
}  // namespace tuw_map_msgs
#endif  // TUW_MAP_MSGS_MSGS__OBJECT_POINT_HPP_
