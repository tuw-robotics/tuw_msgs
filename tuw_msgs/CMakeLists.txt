cmake_minimum_required(VERSION 3.8)
project(tuw_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(tuw_graph_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(jsoncpp REQUIRED)

set(TUW_MSGS_CPP
  src/utils.cpp
  src/point.cpp
  src/quaternion.cpp 
  src/pose.cpp 
  src/node.cpp
  src/edge.cpp
  src/graph.cpp
)

add_library(tuw_msgs ${TUW_MSGS_CPP} )
target_compile_features(tuw_msgs PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(tuw_msgs PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  tuw_msgs
  "tuw_graph_msgs"
  "geometry_msgs"
)
target_link_libraries(tuw_msgs jsoncpp_lib)

install(TARGETS ${PROJECT_NAME} 
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


install(DIRECTORY include/
  DESTINATION include
)


add_executable(example_json src/examples/main_json.cpp)
target_include_directories(example_json PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(example_json PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  example_json
  "tuw_graph_msgs"
  "rclcpp"
  "rclcpp_components"
)

target_link_libraries(example_json jsoncpp_lib tuw_msgs)

install(TARGETS example_json
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

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_utils test/test_utils.cpp )  # Need to link .cpp file under test
  target_include_directories(test_utils PUBLIC src/)  # Need to specify directories in order to #include from them in the test
  target_link_libraries(test_utils ${PROJECT_NAME})

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_std_msgs test/test_std_msgs.cpp)  # Need to link .cpp file under test
  target_include_directories(test_std_msgs PUBLIC src/)  # Need to specify directories in order to #include from them in the test
  target_link_libraries(test_std_msgs ${PROJECT_NAME})


  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_graph_msgs test/test_graph_msgs.cpp)  # Need to link .cpp file under test
  target_include_directories(test_graph_msgs PUBLIC src/)  # Need to specify directories in order to #include from them in the test
  target_link_libraries(test_graph_msgs ${PROJECT_NAME})

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_graph_json test/test_graph_json.cpp)  # Need to link .cpp file under test
  target_include_directories(test_graph_json PUBLIC src/)  # Need to specify directories in order to #include from them in the test
  target_link_libraries(test_graph_json ${PROJECT_NAME})

endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
