#pragma once

#include <json/json.h>

#include "tuw_object_map_msgs/msg/geo_json_geometry.hpp"

namespace tuw_json
{
    inline tuw_object_map_msgs::msg::GeoJSONGeometry &fromJson(
        const Json::Value &json, tuw_object_map_msgs::msg::GeoJSONGeometry &des)
    {
        des.type = json.get("type", "").asCString();

        if (json["coordinates"][0].isArray())
        {
            const Json::Value &coordinatesArray = json["coordinates"];
            for (auto &coordinateJson : coordinatesArray)
            {
                tuw_object_map_msgs::msg::GeoPoint point;

                point.longitude = coordinateJson[0].asDouble();
                point.latitude = coordinateJson[1].asDouble();
                // point.altitude = coordinateJson[2].asDouble(); TODO: Add altitude to GeoPoint

                des.coordinates.push_back(std::move(point));
            }
        }
        else
        {
            const Json::Value &coordinates = json["coordinates"];

            tuw_object_map_msgs::msg::GeoPoint point;

            point.longitude = coordinates[0].asDouble();
            point.latitude = coordinates[1].asDouble();
            // point.altitude = coordinateJson[2].asDouble(); TODO: Add altitude to GeoPoint

            des.coordinates.push_back(std::move(point));
        }

        return des;
    }
} // namespace tuw_json
