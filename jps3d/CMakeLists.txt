cmake_minimum_required(VERSION 2.8.3)
project(jps3d)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wno-deprecated-declarations")

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE RelWithDebInfo)
ENDIF()

set(BUILD_SHARED_LIBS ON)

## Find catkin macros and libraries
find_package(catkin REQUIRED)

## System dependencies
find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
PKG_CHECK_MODULES(YAMLCPP REQUIRED yaml-cpp)

IF(YAMLCPP_FOUND)
  MESSAGE(STATUS "Found yaml-cpp.")
ELSE()
  MESSAGE(FATAL_ERROR "Could not find yaml-cpp.")
ENDIF()

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES jps_lib dmp_lib
  DEPENDS EIGEN3 YAMLCPP
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${YAMLCPP_INCLUDE_DIRS}
)

## Libraries
add_library(jps_lib
  src/jps_planner/graph_search.cpp
  src/jps_planner/jps_planner.cpp
)

add_library(dmp_lib
  src/distance_map_planner/graph_search.cpp
  src/distance_map_planner/distance_map_planner.cpp
)

## Executables
add_executable(create_map test/create_map.cpp)
target_link_libraries(create_map ${YAMLCPP_LIBRARIES})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS jps_lib dmp_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/jps_basis include/jps_planner include/jps_collision
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)

## Mark other files for installation (e.g. launch and config files)
install(FILES
  "${PROJECT_NAME}Config.cmake"
  "${PROJECT_NAME}ConfigVersion.cmake"
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_planner_2d test/test_planner_2d.cpp)
  if(TARGET test_planner_2d)
    target_link_libraries(test_planner_2d jps_lib ${YAMLCPP_LIBRARIES})
  endif()

  catkin_add_gtest(test_planner_3d test/test_planner_3d.cpp)
  if(TARGET test_planner_3d)
    target_link_libraries(test_planner_3d jps_lib ${YAMLCPP_LIBRARIES})
  endif()

  catkin_add_gtest(test_distance_map_planner_2d test/test_distance_map_planner_2d.cpp)
  if(TARGET test_distance_map_planner_2d)
    target_link_libraries(test_distance_map_planner_2d jps_lib dmp_lib ${YAMLCPP_LIBRARIES})
  endif()
endif()
