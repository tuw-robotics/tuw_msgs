#ifndef TUW_MSGS__SERIALIZE_HPP_
#define TUW_MSGS__SERIALIZE_HPP_

#include <geometry_msgs/msg/pose.hpp>
#include <tuw_graph_msgs/msg/node.hpp>

namespace tuw_msgs
{
    std::string &encode(geometry_msgs::msg::Point &src, std::string &des);
    std::string &encode(geometry_msgs::msg::Quaternion &src, std::string &des);
    std::string &encode(geometry_msgs::msg::Pose &src, std::string &des);

    size_t decode(geometry_msgs::msg::Point &des, std::string &line, size_t pos = 0);
    size_t decode(geometry_msgs::msg::Quaternion &des, std::string &line, size_t pos = 0);
    size_t decode(geometry_msgs::msg::Pose &des, std::string &line, size_t pos = 0);

}; // namespace tuw_msgs

#endif // TUW_MSGS__SERIALIZE_HPP_
