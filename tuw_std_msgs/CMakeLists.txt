cmake_minimum_required(VERSION 3.5)

project(tuw_std_msgs)

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


set(msg_files
  "msg/Parameter.msg"
  "msg/ParameterArray.msg"
  "msg/ParameterFloat64.msg"
)

set(srv_files
  "srv/GetParameterFloat64.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES builtin_interfaces std_msgs
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
    tests/test_parameter.cpp
    tests/test_parameter_array.cpp
  )
  target_include_directories(unittest PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(unittest ${PROJECT_NAME}__rosidl_typesupport_cpp)
endif()

ament_export_dependencies(rosidl_default_runtime std_msgs)

ament_export_include_directories(
  include
)

ament_package()