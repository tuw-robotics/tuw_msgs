#ifndef TUW_GEO_MSGS_MSGS__GEO_JSON_GEOMETRY_JSON_HPP_
#define TUW_GEO_MSGS_MSGS__GEO_JSON_GEOMETRY_JSON_HPP_

#include <json/json.h>

#include "tuw_geo_msgs/msg/geo_json_geometry.hpp"

namespace tuw_json
{
inline tuw_geo_msgs::msg::GeoJSONGeometry & fromJson(
  const Json::Value & json, tuw_geo_msgs::msg::GeoJSONGeometry & des)
{
  des.type = json.get("type", "").asCString();

  if (json["coordinates"][0].isArray()) {
    const Json::Value & coordinatesArray = json["coordinates"];
    for (auto & coordinateJson : coordinatesArray) {
      tuw_geo_msgs::msg::GeoPoint point;

      point.longitude = coordinateJson[0].asDouble();
      point.latitude = coordinateJson[1].asDouble();
      // point.altitude = coordinateJson[2].asDouble(); TODO: Add altitude to GeoPoint

      des.coordinates.push_back(std::move(point));
    }
  } else {
    const Json::Value & coordinates = json["coordinates"];

    tuw_geo_msgs::msg::GeoPoint point;

    point.longitude = coordinates[0].asDouble();
    point.latitude = coordinates[1].asDouble();
    // point.altitude = coordinateJson[2].asDouble(); TODO: Add altitude to GeoPoint

    des.coordinates.push_back(std::move(point));
  }

  return des;
}
}  // namespace tuw_json
#endif  // TUW_GEO_MSGS_MSGS__GEO_JSON_GEOMETRY_JSON_HPP_
