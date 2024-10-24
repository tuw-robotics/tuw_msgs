#ifndef TUW_OBJECT_MAP_MSGS__VALUE_JSON_HPP_
#define TUW_OBJECT_MAP_MSGS__VALUE_JSON_HPP_

#include <json/json.h>
#include <tuw_object_map_msgs/value.hpp>

namespace tuw_json
{
inline Json::Value toJson(const tuw_object_map_msgs::msg::Value & src)
{
  Json::Value json;
  json["name"] = src.name.data.c_str();
  json["data"] = src.data;
  return json;
}

inline tuw_object_map_msgs::msg::Value & fromJson(
  const Json::Value & json, tuw_object_map_msgs::msg::Value & des)
{
  des.name.data = json.get("name", "").asCString();
  des.data = json.get("data", "").asFloat();
  return des;
}
}  // namespace tuw_json

#endif  // TUW_OBJECT_MAP_MSGS__VALUE_JSON_HPP_
