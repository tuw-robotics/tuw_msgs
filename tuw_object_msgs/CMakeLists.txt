cmake_minimum_required(VERSION 3.5)

project(tuw_object_msgs)


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
find_package(tuw_std_msgs REQUIRED)
find_package(tuw_geometry_msgs REQUIRED)
find_package(tuw_geo_msgs REQUIRED)

set(msg_files
  msg/Shape.msg
  msg/ShapeArray.msg
  msg/Object.msg
  msg/ObjectDetection.msg
  msg/ObjectStamped.msg
  msg/ObjectWithCovarianceArray.msg
  msg/ObjectWithCovarianceArrayArray.msg
  msg/ObjectWithCovariance.msg
  msg/ObjectWithCovarianceStamped.msg
)

set(srv_files
  srv/GetShapeArray.srv
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES builtin_interfaces tuw_std_msgs tuw_geometry_msgs tuw_geo_msgs
  ADD_LINTER_TESTS
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()


  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(unittest
    tests/test_shape.cpp
  )
  target_include_directories(unittest PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(unittest ${PROJECT_NAME}__rosidl_typesupport_cpp)
endif()

ament_export_dependencies(rosidl_default_runtime tuw_std_msgs tuw_geometry_msgs tuw_geo_msgs)

ament_export_include_directories(
  include
)

ament_package()