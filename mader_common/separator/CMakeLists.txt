cmake_minimum_required(VERSION 2.8.3)
project(separator)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

option(USE_GLPK "Use GLPK as the solver" ON) #if OFF, then it will use Gurobi as the solver


find_package(catkin REQUIRED)
find_package( Eigen3 REQUIRED )
include_directories(${EIGEN3_INCLUDE_DIR})

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

set(CMAKE_BUILD_TYPE Release) #From terminal: catkin config -DCMAKE_BUILD_TYPE=Release (or RelWithDebInfo)

if(USE_GLPK)

		# Use system-installed GLPK
		find_path(GLPK_INCLUDE_DIRS glpk.h
			PATHS /usr/include /usr/local/include
		)

		find_library(GLPK_LIBRARIES
			NAMES glpk
			PATHS /usr/lib /usr/local/lib /usr/lib/x86_64-linux-gnu
		)

		if(NOT GLPK_INCLUDE_DIRS OR NOT GLPK_LIBRARIES)
			message(FATAL_ERROR "GLPK not found. Please install libglpk-dev")
		endif()

		catkin_package(
		    INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
		    LIBRARIES separator)

		include_directories( include ${catkin_INCLUDE_DIRS} ${GLPK_INCLUDE_DIRS})

		add_library(separator SHARED src/separator_glpk.cpp)
		target_link_libraries(separator ${catkin_LIBRARIES} ${GLPK_LIBRARIES})
		add_dependencies(separator ${separator_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
		

else() #If you don't wanna use GLPK

		list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}")
		find_package(GUROBI REQUIRED)

		if(GUROBI_FOUND)
		  message(STATUS "GUROBI FOUND")
		else(GUROBI_FOUND)
		  message(FATAL_ERROR "GUROBI NOT FOUND!")
		endif(GUROBI_FOUND)

		FILE(GLOB GurobiSOFiles $ENV{GUROBI_HOME}/lib/libgurobi*[0-9].so) #files that are start with libgurobi and end with number.so
		set(GUROBI_LIBRARIES "$ENV{GUROBI_HOME}/lib/libgurobi_c++.a;${GurobiSOFiles};$ENV{GUROBI_HOME}/lib/" )

		catkin_package(
		    INCLUDE_DIRS include ${catkin_INCLUDE_DIRS} ${GUROBI_INCLUDE_DIRS} 
		    LIBRARIES separator)

		include_directories( include ${catkin_INCLUDE_DIRS} ${GUROBI_INCLUDE_DIRS})

		add_library(separator SHARED src/separator_gurobi.cpp)
		target_link_libraries(separator ${catkin_LIBRARIES} ${GUROBI_LIBRARIES})
		add_dependencies(separator ${separator_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

endif()


add_executable(test_separator src/test_separator.cpp)
target_link_libraries(test_separator PUBLIC separator)
add_dependencies(test_separator separator)

unset(USE_GLPK CACHE) # To avoid https://stackoverflow.com/questions/22481647/cmake-if-else-with-option