#ifndef TUW_OBJECT_MAP_MSGS__OBJECT_JSON_HPP_
#define TUW_OBJECT_MAP_MSGS__OBJECT_JSON_HPP_

#include <tuw_geometry_msgs/point_json.hpp>
#include <tuw_object_map_msgs/object.hpp>
#include <tuw_object_map_msgs/geo_point_json.hpp>

namespace tuw_json
{
inline Json::Value toJson(const tuw_object_map_msgs::msg::Object & src)
{

  Json::Value json;
  json["id"] = src.id;
  json["type"] = src.type;

  Json::Value geo_points;
  for (const auto & p : src.geo_points)
    geo_points.append(toJson(p));
  json["geo_points"] = geo_points;

  Json::Value map_points;
  for (const auto & p : src.map_points)
    map_points.append(toJson(p));
  json["map_points"] = map_points;

  Json::Value enflation_radius;
  for (const auto & p : src.enflation_radius)
    enflation_radius.append(p);
  json["enflation_radius"] = enflation_radius;

  Json::Value bondary_radius;
  for (const auto & p : src.bondary_radius)
    bondary_radius.append(p);
  json["bondary_radius"] = bondary_radius;

  return json;
}

inline tuw_object_map_msgs::msg::Object & fromJson(
  const Json::Value & json, tuw_object_map_msgs::msg::Object & des)
{
  des.id = json.get("id", "-1").asInt64();
  des.type = json.get("type", "").asUInt();

  if (json.isMember("geo_points") && json["geo_points"].isArray()) {
    const Json::Value & jsonArray = json["geo_points"];
    for (auto & j : jsonArray) {
      tuw_object_map_msgs::msg::GeoPoint p;
      des.geo_points.push_back(std::move(tuw_json::fromJson(j, p)));
    }
  }
  if (json.isMember("map_points") && json["map_points"].isArray()) {
    const Json::Value & jsonArray = json["map_points"];
    for (auto & j : jsonArray) {
      geometry_msgs::msg::Point p;
      des.map_points.push_back(std::move(tuw_json::fromJson(j, p)));
    }
  }
  if (json.isMember("enflation_radius") && json["enflation_radius"].isArray()) {
    const Json::Value & jsonArray = json["enflation_radius"];
    for (auto & j : jsonArray) {
      des.enflation_radius.push_back(j.asDouble());
    }
  }
  if (json.isMember("bondary_radius") && json["bondary_radius"].isArray()) {
    const Json::Value & jsonArray = json["bondary_radius"];
    for (auto & j : jsonArray) {
      des.bondary_radius.push_back(j.asDouble());
    }
  }
  return des;
}
}  // namespace tuw_json

#endif  // TUW_OBJECT_MAP_MSGS__OBJECT_JSON_HPP_
