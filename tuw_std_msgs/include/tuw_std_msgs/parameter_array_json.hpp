#ifndef TUW_STD_MSGS__PARAMETER_ARRAY_JSON_HPP_
#define TUW_STD_MSGS__PARAMETER_ARRAY_JSON_HPP_

#include <tuw_std_msgs/parameter_array.hpp>
#include <tuw_std_msgs/parameter_json.hpp>
#include <vector>

namespace tuw_json
{
inline Json::Value toJson(const tuw_std_msgs::msg::ParameterArray & src)
{
  Json::Value json;
  Json::Value objects;
  for (const auto & o : src.data) {
    objects.append(toJson(o));
  }
  json["data"] = objects;
  return json;
}

inline tuw_std_msgs::msg::ParameterArray & fromJson(
  const Json::Value & json, tuw_std_msgs::msg::ParameterArray & des)
{
  if (json.isMember("data") && json["data"].isArray()) {
    const Json::Value & jsonArray = json["data"];
    for (auto & j : jsonArray) {
      tuw_std_msgs::msg::Parameter p;
      des.data.push_back(std::move(tuw_json::fromJson(j, p)));
    }
  }
  return des;
}

inline Json::Value toJson(const std::vector<tuw_std_msgs::msg::ParameterArray> & src)
{
  Json::Value des;
  for (const auto & o : src) {
    des.append(tuw_json::toJson(o));
  }
  return des;
}

inline std::vector<tuw_std_msgs::msg::ParameterArray> & fromJson(
  const Json::Value & json, const std::string & key,
  std::vector<tuw_std_msgs::msg::ParameterArray> & des)
{
  if (json.isMember(key) && json[key].isArray()) {
    const Json::Value & jsonArray = json[key];
    for (auto & j : jsonArray) {
      tuw_std_msgs::msg::ParameterArray o;
      des.push_back(std::move(tuw_json::fromJson(j, o)));
    }
  }
  return des;
}

}  // namespace tuw_json

#endif  // TUW_STD_MSGS__PARAMETER_ARRAY_JSON_HPP_
