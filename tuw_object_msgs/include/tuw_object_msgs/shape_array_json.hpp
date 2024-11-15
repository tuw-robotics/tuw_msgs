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

#ifndef TUW_OBJECT_MSGS__SHAPE_ARRAY_JSON_HPP_
#define TUW_OBJECT_MSGS__SHAPE_ARRAY_JSON_HPP_

#include <string>
#include <vector>
#include <tuw_object_msgs/shape_array.hpp>
#include <tuw_object_msgs/shape_json.hpp>

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
