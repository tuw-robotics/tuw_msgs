#ifndef TUW_OBJECT_MSGS__SHAPE_HPP_
#define TUW_OBJECT_MSGS__SHAPE_HPP_

#include <tuw_object_msgs/msg/shape.hpp>

namespace tuw_object_msgs
{
struct Shape : public tuw_object_msgs::msg::Shape
{
  Shape() { this->id = -1; }
  Shape(int64_t id) { this->id = id; }
  Shape(int64_t id, uint32_t type) { this->id = id, this->type = type; }
};
}  // namespace tuw_object_msgs
#endif  // TUW_OBJECT_MSGS__SHAPE_HPP_
