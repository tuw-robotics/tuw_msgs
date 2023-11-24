cmake_minimum_required(VERSION 3.8)
project(tuw_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tuw_graph_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)


add_library(tuw_msgs src/serialize.cpp src/graph.cpp)
target_compile_features(tuw_msgs PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(tuw_msgs PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  tuw_msgs
  "geometry_msgs"
  "tuw_graph_msgs"
)


add_executable(example_graph src/examples/main_graph.cpp)
target_include_directories(example_graph PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(example_graph tuw_msgs)
target_compile_features(example_graph PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  example_graph
  "tuw_graph_msgs"
  "rclcpp"
  "rclcpp_components"
)

install(TARGETS example_graph tuw_msgs
  DESTINATION lib/${PROJECT_NAME})

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
endif()

ament_package()
