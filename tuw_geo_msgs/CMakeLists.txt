cmake_minimum_required(VERSION 3.5)

project(tuw_geo_msgs)

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
find_package(geographic_msgs REQUIRED)


set(msg_files
  "msg/GeoJSON.msg"
  "msg/GeoJSONFeature.msg"
  "msg/GeoJSONGeometry.msg"
  "msg/GeoJSONProperties.msg"
)

set(srv_files
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs geographic_msgs
  ADD_LINTER_TESTS
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
ament_export_dependencies(rosidl_default_runtime geometry_msgs std_msgs geographic_msgs)

ament_export_include_directories(
  include
)
ament_package()
