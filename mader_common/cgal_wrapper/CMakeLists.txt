cmake_minimum_required(VERSION 2.8.3)
project(cgal_wrapper)

find_package(catkin REQUIRED)

# Set CGAL source directory
set(CGAL_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../CGAL-4.14.2)

# Check if CGAL source exists
if(NOT EXISTS ${CGAL_SOURCE_DIR})
  message(FATAL_ERROR "CGAL source not found at ${CGAL_SOURCE_DIR}")
endif()

# Add CGAL modules path
list(APPEND CMAKE_MODULE_PATH ${CGAL_SOURCE_DIR}/cmake/modules)

# Find required dependencies
find_package(GMP REQUIRED)
find_package(MPFR REQUIRED)
find_package(Boost REQUIRED)

# Set CGAL configuration
set(CGAL_HEADER_ONLY TRUE)
set(CGAL_INCLUDE_DIRS
  ${CGAL_SOURCE_DIR}/include
  ${GMP_INCLUDE_DIR}
  ${MPFR_INCLUDE_DIR}
)

# Configure CGAL
set(CGAL_LIBRARIES ${GMP_LIBRARIES} ${MPFR_LIBRARIES})

catkin_package(
  INCLUDE_DIRS ${CGAL_INCLUDE_DIRS}
  LIBRARIES ${CGAL_LIBRARIES}
  CFG_EXTRAS cgal-config.cmake
)

# Install the cmake extras file
install(FILES cmake/cgal-config.cmake
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake
)