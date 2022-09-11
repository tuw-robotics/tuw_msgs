cmake_minimum_required(VERSION 3.5)

project(tuw_multi_robot_msgs)


# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(msg_files
  "msg/RobotGoals.msg"
  "msg/RobotGoalsArray.msg"
  "msg/RoutePrecondition.msg"
  "msg/RouteSegment.msg"
  "msg/Route.msg"
  "msg/RouterStatus.msg"
  "msg/Vertex.msg"
  "msg/Graph.msg"
  "msg/RobotInfo.msg"
  "msg/Order.msg"
  "msg/OrderArray.msg"
  "msg/Pickup.msg"
  "msg/OrderPosition.msg"
  "msg/Station.msg"
  "msg/StationArray.msg"
  "msg/RouteProgress.msg"
)

set(srv_files
  "srv/StationManagerControlProtocol.srv"
  "srv/StationManagerStationProtocol.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
  ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()