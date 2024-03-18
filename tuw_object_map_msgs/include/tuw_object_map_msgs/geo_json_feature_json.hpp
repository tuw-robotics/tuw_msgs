#pragma once

#include <json/json.h>

#include "tuw_object_map_msgs/msg/geo_json_feature.hpp"
#include "tuw_object_map_msgs/geo_json_geometry_json.hpp"

namespace tuw_json
{
    inline tuw_object_map_msgs::msg::GeoJSONFeature &fromJson(
        const Json::Value &json, tuw_object_map_msgs::msg::GeoJSONFeature &des)
    {
        des.type = json.get("type", "").asCString();

        const Json::Value &geometryJson = json["geometry"];
        tuw_object_map_msgs::msg::GeoJSONGeometry geometry;
        des.geometry = std::move(tuw_json::fromJson(geometryJson, geometry));

        return des;
    }
} // namespace tuw_json
