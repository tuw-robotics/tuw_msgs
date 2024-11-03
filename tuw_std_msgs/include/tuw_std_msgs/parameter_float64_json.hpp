#ifndef TUW_OBJECT_MAP_MSGS__PARAMETER_FLOAT64_JSON_HPP_
#define TUW_OBJECT_MAP_MSGS__PARAMETER_FLOAT64_JSON_HPP_

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

inline Json::Value toJson(const std::vector<tuw_std_msgs::msg::ParameterFloat64> & src)
{
  Json::Value des;
  for (const auto & o : src) {
    des.append(tuw_json::toJson(o));
  }
  return des;
}

inline std::vector<tuw_std_msgs::msg::ParameterFloat64> & fromJson(
  const Json::Value & json, const std::string & key,
  std::vector<tuw_std_msgs::msg::ParameterFloat64> & des)
{
  if (json.isMember(key) && json[key].isArray()) {
    const Json::Value & jsonArray = json[key];
    for (auto & j : jsonArray) {
      tuw_std_msgs::msg::ParameterFloat64 o;
      des.push_back(std::move(tuw_json::fromJson(j, o)));
    }
  }
  return des;
}
}  // namespace tuw_json

#endif  // TUW_OBJECT_MAP_MSGS__PARAMETER_FLOAT64_JSON_HPP_
