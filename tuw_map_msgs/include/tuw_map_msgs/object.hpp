#ifndef TUW_MAP_MSGS_MSGS__OBJECT_HPP_
#define TUW_MAP_MSGS_MSGS__OBJECT_HPP_

#include <tuw_map_msgs/msg/object.hpp>

namespace tuw_map_msgs
{
struct Object : public tuw_map_msgs::msg::Object
{
  Object(int64_t id) { this->id = id; }
  Object(int64_t id, uint32_t type) { this->id = id, this->type = type; }
};
}  // namespace tuw_map_msgs
#endif  // TUW_MAP_MSGS_MSGS__OBJECT_HPP_
