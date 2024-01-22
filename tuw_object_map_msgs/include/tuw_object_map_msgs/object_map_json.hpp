#ifndef TUW_OBJECT_MAP_MSGS__OBJECT_MAP_JSON_HPP_
#define TUW_OBJECT_MAP_MSGS__OBJECT_MAP_JSON_HPP_

#include <tuw_object_map_msgs/object_json.hpp>
#include <tuw_object_map_msgs/object_map.hpp>

namespace tuw_json
{
inline Json::Value toJson(const tuw_object_map_msgs::msg::ObjectMap & src)
{
  Json::Value json;
  json["frame_id"] = src.header.frame_id;
  Json::Value objects;
  for (const auto & p : src.objects) {
    objects.append(toJson(p));
  }
  json["objects"] = objects;
  return json;
}

inline tuw_object_map_msgs::msg::ObjectMap & fromJson(
  const Json::Value & json, tuw_object_map_msgs::msg::ObjectMap & des)
{
  des.header.frame_id = json.get("frame_id", "-1").asCString();
  if (json.isMember("objects") && json["objects"].isArray()) {
    const Json::Value & jsonArray = json["objects"];
    for (auto & j : jsonArray) {
      tuw_object_map_msgs::msg::Object p;
      des.objects.push_back(std::move(tuw_json::fromJson(j, p)));
    }
  }
  return des;
}
}  // namespace tuw_json

#endif  // TUW_OBJECT_MAP_MSGS__OBJECT_MAP_JSON_HPP_
