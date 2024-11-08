#ifndef TUW_STD_MSGS__PARAMETER_JSON_HPP_
#define TUW_STD_MSGS__PARAMETER_JSON_HPP_

#include <vector>
#include <tuw_std_msgs/parameter.hpp>

namespace tuw_json
{
inline Json::Value toJson(const tuw_std_msgs::msg::Parameter & src)
{
  Json::Value json;
  json["name"] = src.name.c_str();
  json["value"] = src.value.c_str();
  return json;
}

inline tuw_std_msgs::msg::Parameter & fromJson(
  const Json::Value & json, tuw_std_msgs::msg::Parameter & des)
{
  des.name = json.get("name", "").asCString();
  des.value = json.get("value", "").asCString();
  return des;
}

inline Json::Value toJson(const std::vector<tuw_std_msgs::msg::Parameter> & src)
{
  Json::Value des;
  for (const auto & o : src) {
    des.append(tuw_json::toJson(o));
  }
  return des;
}

inline std::vector<tuw_std_msgs::msg::Parameter> & fromJson(
  const Json::Value & json, const std::string & key,
  std::vector<tuw_std_msgs::msg::Parameter> & des)
{
  if (json.isMember(key) && json[key].isArray()) {
    const Json::Value & jsonArray = json[key];
    for (auto & j : jsonArray) {
      tuw_std_msgs::msg::Parameter o;
      des.push_back(std::move(tuw_json::fromJson(j, o)));
    }
  }
  return des;
}

}  // namespace tuw_json

#endif  // TUW_STD_MSGS__PARAMETER_JSON_HPP_
