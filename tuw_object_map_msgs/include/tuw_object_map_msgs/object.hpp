#ifndef TUW_OBJECT_MAP_MSGS_MSGS__OBJECT_HPP_
#define TUW_OBJECT_MAP_MSGS_MSGS__OBJECT_HPP_

#include <tuw_object_map_msgs/msg/object.hpp>

namespace tuw_object_map_msgs
{
struct Object : public tuw_object_map_msgs::msg::Object
{
  Object(int64_t id) {this->id = id;}
  Object(int64_t id, uint32_t type)
  {
    this->id = id, this->type = type;
  }
};
}  // namespace tuw_object_map_msgs
#endif  // TUW_OBJECT_MAP_MSGS_MSGS__OBJECT_HPP_
