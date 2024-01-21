#ifndef TUW_OBJECT_MAP_MSGS__GEO_POINT_JSON_HPP_
#define TUW_OBJECT_MAP_MSGS__GEO_POINT_JSON_HPP_

#include <tuw_object_map_msgs/geo_point.hpp>

namespace tuw_json
{
inline Json::Value toJson(const tuw_object_map_msgs::msg::GeoPoint & src)
{
  Json::Value json;
  json["latitude"] = src.latitude;
  json["longitude"] = src.longitude;
  json["altitude"] = src.altitude;
  return json;
}

inline tuw_object_map_msgs::msg::GeoPoint & fromJson(
  const Json::Value & json, tuw_object_map_msgs::msg::GeoPoint & des)
{
  des.latitude = json.get("latitude", "-1").asDouble();
  des.longitude = json.get("longitude", "").asDouble();
  des.altitude = json.get("altitude", "").asDouble();
  return des;
}
}  // namespace tuw_json

#endif  // TUW_OBJECT_MAP_MSGS__GEO_POINT_JSON_HPP_
