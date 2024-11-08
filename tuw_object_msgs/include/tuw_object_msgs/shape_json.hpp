#ifndef TUW_OBJECT_MSGS__SHAPE_JSON_HPP_
#define TUW_OBJECT_MSGS__SHAPE_JSON_HPP_

#include <tuw_object_msgs/shape.hpp>
#include <tuw_geometry_msgs/point_json.hpp>
#include <tuw_geo_msgs/geo_point_json.hpp>

namespace tuw_json
{
inline Json::Value toJson(const tuw_object_msgs::msg::Shape & src)
{
  Json::Value json;
  json["id"] = src.id;
  json["type"] = src.type;
  json["wgs84"] = toJson(src.wgs84);
  json["points"] = toJson(src.points);
  json["params_points"] = toJson(src.params_points);
  json["params"] = toJson(src.params);
  return json;
}

inline tuw_object_msgs::msg::Shape & fromJson(
  const Json::Value & json, tuw_object_msgs::msg::Shape & des)
{
  des.id = json.get("id", "-1").asInt64();
  des.type = json.get("type", "").asUInt();
  fromJson(json, "wgs84", des.wgs84);
  fromJson(json, "points", des.points);
  fromJson(json, "params_points", des.params_points);
  fromJson(json.get("params", ""), des.params);
  return des;
}
inline Json::Value toJson(const std::vector<tuw_object_msgs::msg::Shape> & src)
{
  Json::Value des;
  for (const auto & o : src) {
    des.append(tuw_json::toJson(o));
  }
  return des;
}

inline std::vector<tuw_object_msgs::msg::Shape> & fromJson(
  const Json::Value & json, const std::string & key,
  std::vector<tuw_object_msgs::msg::Shape> & des)
{
  if (json.isMember(key) && json[key].isArray()) {
    const Json::Value & jsonArray = json[key];
    for (auto & j : jsonArray) {
      tuw_object_msgs::msg::Shape o;
      des.push_back(std::move(tuw_json::fromJson(j, o)));
    }
  }
  return des;
}
}  // namespace tuw_json

#endif  // TUW_OBJECT_MSGS__SHAPE_JSON_HPP_
