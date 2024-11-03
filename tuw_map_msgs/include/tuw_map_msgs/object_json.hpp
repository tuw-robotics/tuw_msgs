#ifndef TUW_MAP_MSGS__OBJECT_JSON_HPP_
#define TUW_MAP_MSGS__OBJECT_JSON_HPP_

#include <tuw_map_msgs/object.hpp>
#include <tuw_map_msgs/object_point_json.hpp>

namespace tuw_json
{
inline Json::Value toJson(const tuw_map_msgs::msg::Object & src)
{
  Json::Value json;
  json["id"] = src.id;
  json["type"] = src.type;
  json["points"] = tuw_json::toJson(src.points);
  json["parameters"] = tuw_json::toJson(src.parameters);
  return json;
}

inline tuw_map_msgs::msg::Object & fromJson(
  const Json::Value & json, tuw_map_msgs::msg::Object & des)
{
  des.id = json.get("id", "-1").asInt64();
  des.type = json.get("type", "").asUInt();
  tuw_json::fromJson(json, "points", des.points);
  tuw_json::fromJson(json, "parameters", des.parameters);
  return des;
}
inline Json::Value toJson(const std::vector<tuw_map_msgs::msg::Object> & src)
{
  Json::Value des;
  for (const auto & o : src) {
    des.append(tuw_json::toJson(o));
  }
  return des;
}

inline std::vector<tuw_map_msgs::msg::Object> & fromJson(
  const Json::Value & json, const std::string & key,
  std::vector<tuw_map_msgs::msg::Object> & des)
{
  if (json.isMember(key) && json[key].isArray()) {
    const Json::Value & jsonArray = json[key];
    for (auto & j : jsonArray) {
      tuw_map_msgs::msg::Object o;
      des.push_back(std::move(tuw_json::fromJson(j, o)));
    }
  }
  return des;
}
}  // namespace tuw_json

#endif  // TUW_MAP_MSGS__OBJECT_JSON_HPP_
