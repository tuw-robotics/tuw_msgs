#ifndef TUW_OBJECT_MAP_MSGS__PARAMETER_FLOAT64_JSON_HPP_
#define TUW_OBJECT_MAP_MSGS__PARAMETER_FLOAT64_JSON_HPP_

#include <json/json.h>
#include <tuw_std_msgs/parameter_float64.hpp>

namespace tuw_json
{
inline Json::Value toJson(const tuw_std_msgs::msg::ParameterFloat64 & src)
{
  Json::Value json;
  json["name"] = src.name.data.c_str();
  json["data"] = src.data;
  return json;
}

inline tuw_std_msgs::msg::ParameterFloat64 & fromJson(
  const Json::Value & json, tuw_std_msgs::msg::ParameterFloat64 & des)
{
  des.name.data = json.get("name", "").asCString();
  des.data = json.get("data", "").asFloat();
  return des;
}
}  // namespace tuw_json

#endif  // TUW_OBJECT_MAP_MSGS__PARAMETER_FLOAT64_JSON_HPP_
