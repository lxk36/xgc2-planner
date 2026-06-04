cmake_minimum_required(VERSION 2.8.3)
project(fast_bspline_opt)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_path(NLOPT_INCLUDE_DIR nlopt.hpp)
find_library(NLOPT_LIBRARY nlopt)
if(NOT NLOPT_INCLUDE_DIR OR NOT NLOPT_LIBRARY)
  message(FATAL_ERROR "NLopt not found. Install libnlopt-dev.")
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  cv_bridge
  fast_plan_env
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES fast_bspline_opt_lib
 CATKIN_DEPENDS fast_plan_env
#  DEPENDS system_lib
)

include_directories(
    SYSTEM
    include
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${NLOPT_INCLUDE_DIR}
)

add_library( fast_bspline_opt_lib 
    src/bspline_optimizer.cpp 
    )
target_link_libraries( fast_bspline_opt_lib
    ${catkin_LIBRARIES}
    ${NLOPT_LIBRARY}
    )

install(TARGETS fast_bspline_opt_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
