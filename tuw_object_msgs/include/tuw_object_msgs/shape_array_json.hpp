#ifndef TUW_OBJECT_MSGS__SHAPE_ARRAY_JSON_HPP_
#define TUW_OBJECT_MSGS__SHAPE_ARRAY_JSON_HPP_

#include <tuw_object_msgs/shape_json.hpp>
#include <tuw_object_msgs/shape_array.hpp>

namespace tuw_json
{
inline Json::Value toJson(const tuw_object_msgs::msg::ShapeArray & src)
{
  Json::Value json;
  json["frame_id"] = src.header.frame_id;
  json["shapes"] = tuw_json::toJson(src.shapes);
  return json;
}

inline tuw_object_msgs::msg::ShapeArray & fromJson(
  const Json::Value & json, tuw_object_msgs::msg::ShapeArray & des)
{
  des.header.frame_id = json.get("frame_id", "-1").asCString();
  tuw_json::fromJson(json, "shapes", des.shapes);
  return des;
}
}  // namespace tuw_json

#endif  // TUW_OBJECT_MSGS__SHAPE_ARRAY_JSON_HPP_
