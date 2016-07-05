cmake_minimum_required(VERSION 2.8.3)
project(tuw_spline_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)

add_message_files(
  DIRECTORY msg
  FILES
  float64Array.msg
  Spline.msg
)

generate_messages(DEPENDENCIES std_msgs )

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  std_msgs
  message_generation
)
