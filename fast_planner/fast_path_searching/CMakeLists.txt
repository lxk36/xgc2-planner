cmake_minimum_required(VERSION 2.8.3)
project(fast_path_searching)

set(CMAKE_CXX_STANDARD 14)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  fast_plan_env
  cv_bridge
)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES fast_path_searching_lib
 CATKIN_DEPENDS fast_plan_env
#  DEPENDS system_lib
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -O3 -Wall")

add_library( fast_path_searching_lib 
    src/kinodynamic_astar.cpp 
    src/astar.cpp
    src/topo_prm.cpp
    )
target_link_libraries( fast_path_searching_lib
    ${catkin_LIBRARIES} 
    )  

install(TARGETS fast_path_searching_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
