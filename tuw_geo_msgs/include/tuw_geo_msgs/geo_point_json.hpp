#ifndef TUW_GEO_MSGS__GEO_POINT_JSON_HPP_
#define TUW_GEO_MSGS__GEO_POINT_JSON_HPP_

#include <tuw_geo_msgs/geo_point.hpp>

namespace tuw_json
{
inline Json::Value toJson(const geographic_msgs::msg::GeoPoint & src)
{
  Json::Value json;
  json["latitude"] = src.latitude;
  json["longitude"] = src.longitude;
  json["altitude"] = src.altitude;
  return json;
}

inline geographic_msgs::msg::GeoPoint & fromJson(
  const Json::Value & json, geographic_msgs::msg::GeoPoint & des)
{
  des.latitude = json.get("latitude", "-1").asDouble();
  des.longitude = json.get("longitude", "").asDouble();
  des.altitude = json.get("altitude", "").asDouble();
  return des;
}
inline Json::Value toJson(const std::vector<geographic_msgs::msg::GeoPoint> & src)
{
  Json::Value des;
  for (const auto & o : src) {
    des.append(tuw_json::toJson(o));
  }
  return des;
}

inline std::vector<geographic_msgs::msg::GeoPoint> & fromJson(
  const Json::Value & json, const std::string & key,
  std::vector<geographic_msgs::msg::GeoPoint> & des)
{
  if (json.isMember(key) && json[key].isArray()) {
    const Json::Value & jsonArray = json[key];
    for (auto & j : jsonArray) {
      geographic_msgs::msg::GeoPoint o;
      des.push_back(std::move(tuw_json::fromJson(j, o)));
    }
  }
  return des;
}
}  // namespace tuw_json

#endif  // TUW_GEO_MSGS__GEO_POINT_JSON_HPP_
