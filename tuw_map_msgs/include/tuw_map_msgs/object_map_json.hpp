#ifndef TUW_MAP_MSGS__OBJECT_MAP_JSON_HPP_
#define TUW_MAP_MSGS__OBJECT_MAP_JSON_HPP_

#include <tuw_map_msgs/object_json.hpp>
#include <tuw_map_msgs/object_map.hpp>

namespace tuw_json
{
inline Json::Value toJson(const tuw_map_msgs::msg::ObjectMap & src)
{
  Json::Value json;
  json["frame_id"] = src.header.frame_id;
  json["objects"] = tuw_json::toJson(src.objects);
  return json;
}

inline tuw_map_msgs::msg::ObjectMap & fromJson(
  const Json::Value & json, tuw_map_msgs::msg::ObjectMap & des)
{
  des.header.frame_id = json.get("frame_id", "-1").asCString();
  tuw_json::fromJson(json, "objects", des.objects);
  return des;
}
}  // namespace tuw_json

#endif  // TUW_MAP_MSGS__OBJECT_MAP_JSON_HPP_
