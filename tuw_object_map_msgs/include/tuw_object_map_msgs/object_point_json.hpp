#ifndef TUW_OBJECT_MAP_MSGS__OBJECT_POINT_JSON_HPP_
#define TUW_OBJECT_MAP_MSGS__OBJECT_POINT_JSON_HPP_

#include <tuw_object_map_msgs/object_point.hpp>

#include <tuw_geometry_msgs/point_json.hpp>
#include <tuw_object_map_msgs/geo_point_json.hpp>
#include <tuw_object_map_msgs/value_json.hpp>

namespace tuw_json
{
inline Json::Value toJson(const tuw_object_map_msgs::msg::ObjectPoint & src)
{
  Json::Value json;
  json["id"] = src.id;
  json["type"] = src.type;

  Json::Value wgs84;
  json["wgs84"] = toJson(src.wgs84);

  Json::Value map;
  json["map"] = toJson(src.map);;

  Json::Value parameters;
  for (const auto & p : src.parameters) {
    parameters.append(toJson(p));
  }
  json["parameters"] = parameters;

  return json;
}

inline tuw_object_map_msgs::msg::ObjectPoint & fromJson(
  const Json::Value & json, tuw_object_map_msgs::msg::ObjectPoint & des)
{
  des.id = json.get("id", "-1").asInt64();
  des.type = json.get("type", "").asUInt();
  tuw_object_map_msgs::msg::GeoPoint g;
  des.wgs84 = std::move(tuw_json::fromJson(json["wgs84"],  g));
  geometry_msgs::msg::Point p;
  des.map = std::move(tuw_json::fromJson(json["map"], p));

  if (json.isMember("parameters") && json["parameters"].isArray()) {
    const Json::Value & jsonArray = json["parameters"];
    for (auto & j : jsonArray) {
      tuw_object_map_msgs::msg::Value o;
      des.parameters.push_back(std::move(tuw_json::fromJson(j, o)));
    }
  }
  return des;
}
}  // namespace tuw_json

#endif  // TUW_OBJECT_MAP_MSGS__OBJECT_POINT_JSON_HPP_
