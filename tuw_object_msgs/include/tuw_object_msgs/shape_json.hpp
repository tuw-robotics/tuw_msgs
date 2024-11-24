/*
Copyright (c) 2024, Markus Bader
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.
 * Neither the name of TU Wien nor the names of its contributors may be
   used to endorse or promote products derived from this software without
   specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef TUW_OBJECT_MSGS__SHAPE_JSON_HPP_
#define TUW_OBJECT_MSGS__SHAPE_JSON_HPP_

#include <string>
#include <vector>
#include <utility>
#include <tuw_geometry_msgs/pose_json.hpp>
#include <tuw_object_msgs/shape.hpp>
#include <tuw_std_msgs/parameter_array_json.hpp>

namespace tuw_json
{
inline Json::Value toJson(const tuw_object_msgs::msg::Shape & src)
{
  Json::Value json;
  json["id"] = src.id;
  json["shape"] = src.shape;
  json["type"] = src.type;
  json["poses"] = toJson(src.poses);
  json["params_poses"] = toJson(src.params_poses);
  json["params"] = toJson(src.params);
  return json;
}

inline tuw_object_msgs::msg::Shape & fromJson(
  const Json::Value & json, tuw_object_msgs::msg::Shape & des)
{
  des.id = json.get("id", "-1").asInt64();
  if (json.isMember("shape")) {
    des.shape = json.get("shape", "").asUInt();
  } else {
    des.shape = tuw_object_msgs::msg::Shape::SHAPE_NA;
  }
  if (json.isMember("type")) {
    des.type = json.get("type", "").asUInt();
  } else {
    des.type = tuw_object_msgs::msg::Shape::TYPE_NA;
  }
  fromJson(json, "poses", des.poses);
  fromJson(json, "params_poses", des.params_poses);
  if (json.isMember("params")) {
    fromJson(json.get("params", ""), des.params);
  }
  return des;
}
inline Json::Value toJson(const std::vector<tuw_object_msgs::msg::Shape> & src)
{
  Json::Value des;
  for (const auto & o : src) {
    des.append(tuw_json::toJson(o));
  }
  return des;
}

inline std::vector<tuw_object_msgs::msg::Shape> & fromJson(
  const Json::Value & json, const std::string & key, std::vector<tuw_object_msgs::msg::Shape> & des)
{
  if (json.isMember(key) && json[key].isArray()) {
    const Json::Value & jsonArray = json[key];
    for (auto & j : jsonArray) {
      tuw_object_msgs::msg::Shape o;
      des.push_back(std::move(tuw_json::fromJson(j, o)));
    }
  }
  return des;
}
}  // namespace tuw_json

#endif  // TUW_OBJECT_MSGS__SHAPE_JSON_HPP_
