#ifndef TUW_STD_MSGS__PARAMETER_JSON_HPP_
#define TUW_STD_MSGS__PARAMETER_JSON_HPP_

#include <json/json.h>
#include <tuw_std_msgs/parameter.hpp>

namespace tuw_json
{
inline Json::Value toJson(const tuw_std_msgs::msg::Parameter & src)
{
  Json::Value json;
  json["name"] = src.name.data.c_str();
  json["value"] = src.value.data.c_str();
  json["type"] = src.type;
  return json;
}

inline tuw_std_msgs::msg::Parameter & fromJson(
  const Json::Value & json, tuw_std_msgs::msg::Parameter & des)
{
  des.name.data = json.get("name", "").asCString();
  des.value.data = json.get("value", "").asCString();
  des.type = json.get("type", "").asInt();
  return des;
}
}  // namespace tuw_json

#endif  // TUW_STD_MSGS__PARAMETER_JSON_HPP_
