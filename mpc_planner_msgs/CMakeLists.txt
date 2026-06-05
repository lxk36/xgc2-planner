cmake_minimum_required(VERSION 2.8.3)
project(mpc_planner_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  nav_msgs
  std_msgs
  geometry_msgs
)

include_directories(include)

add_message_files(
  DIRECTORY msg
  FILES Gaussian.msg ObstacleGMM.msg ObstacleArray.msg
)

generate_messages(DEPENDENCIES nav_msgs std_msgs geometry_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime nav_msgs std_msgs geometry_msgs
)
