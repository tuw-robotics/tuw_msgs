#ifndef TUW_MSGS__UTILS_HPP_
#define TUW_MSGS__UTILS_HPP_

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

namespace tuw_msgs
{

    using Idx = int64_t;

    enum Format
    {
        COMPACT,
        LOOSE
    };

    size_t nr_of_leading_spaces(const std::string &inputString);

    std::string remove_spaces(std::string str);

    void rpy_to_quaternion(double roll, double pitch, double yaw, double &qx, double &qy, double &qz, double &qw);

    template <typename T>
    bool is_similar(const T& a, const T& b, T epsilon){
        return std::abs(a - b) < epsilon;
    }
    bool is_equal(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b);
    bool is_equal(const geometry_msgs::msg::Quaternion &a, const geometry_msgs::msg::Quaternion &b);
    bool is_equal(const geometry_msgs::msg::Pose &a, const geometry_msgs::msg::Pose &b);

    bool is_similar(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b, double epsilon = 0.0001);
    bool is_similar(const geometry_msgs::msg::Quaternion &a, const geometry_msgs::msg::Quaternion &b, double epsilon = 0.0001);
    bool is_similar(const geometry_msgs::msg::Pose &a, const geometry_msgs::msg::Pose &b, double epsilon_position = 0.0001, double epsilon_orientation = 0.0001);

    bool is_similar(const std::string &a, const std::string &b);
    bool is_similar_with_cout(const std::string &a, const std::string &b);
};

#endif // TUW_MSGS__UTILS_HPP_
