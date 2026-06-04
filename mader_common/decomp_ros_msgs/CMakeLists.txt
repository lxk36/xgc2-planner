cmake_minimum_required(VERSION 2.8.3)
project(decomp_ros_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
)

add_message_files(
  FILES
  Ellipsoid.msg
  EllipsoidArray.msg
  Polyhedron.msg
  PolyhedronArray.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs message_runtime
)
