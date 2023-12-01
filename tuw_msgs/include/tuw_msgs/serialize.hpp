#ifndef TUW_MSGS__SERIALIZE_HPP_
#define TUW_MSGS__SERIALIZE_HPP_

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>


namespace tuw_msgs
{
  /**
   * Class to read a costomized message format for selected tuw_msgs
   */
  class Serialize
  {
  protected:
    static const int DECODE_ERROR_INVALID = 1;
    static const int DECODE_ERROR_SYNTAX = 0;
    static const int DECODE_SUCCESSFUL = -1;
    static const int ENCODE_SUCCESSFUL = -2;
    static const int DECODE_EMPTY = -3;
    char delim_entries;
    char delim_rows;
    char delim_cols;

  public:
    Serialize();
    ~Serialize();

    void delim(char delim_entries = ':', char delim_rows = ';', char delim_cols = ',');

    std::string &encode(geometry_msgs::msg::Point &src, std::string &des);
    std::string &encode(geometry_msgs::msg::Quaternion &src, std::string &des);
    std::string &encode(geometry_msgs::msg::Pose &src, std::string &des);

    size_t decode(geometry_msgs::msg::Point &des, std::string &line, size_t pos = 0);
    size_t decode(geometry_msgs::msg::Quaternion &des, std::string &line, size_t pos = 0);
    size_t decode(geometry_msgs::msg::Pose &des, std::string &line, size_t pos = 0);
  };
} // namespace tuw_msgs

#endif // TUW_MSGS__SERIALIZE_HPP_
