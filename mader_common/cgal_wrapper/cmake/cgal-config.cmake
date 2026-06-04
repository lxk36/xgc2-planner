# CGAL configuration for downstream packages

set(CGAL_FOUND TRUE)
set(CGAL_VERSION "4.14.2")

# Set CGAL paths
set(CGAL_SOURCE_DIR ${cgal_wrapper_DIR}/../../../CGAL-4.14.2)
set(CGAL_INCLUDE_DIRS ${CGAL_SOURCE_DIR}/include)

# Add CGAL modules path
list(APPEND CMAKE_MODULE_PATH ${CGAL_SOURCE_DIR}/cmake/modules)

# Find dependencies directly without find_package
find_path(GMP_INCLUDE_DIR gmp.h
  PATHS /usr/include /usr/local/include
)
find_library(GMP_LIBRARIES
  NAMES gmp
  PATHS /usr/lib /usr/local/lib /usr/lib/x86_64-linux-gnu
)
find_path(MPFR_INCLUDE_DIR mpfr.h
  PATHS /usr/include /usr/local/include
)
find_library(MPFR_LIBRARIES
  NAMES mpfr
  PATHS /usr/lib /usr/local/lib /usr/lib/x86_64-linux-gnu
)
find_package(Boost QUIET)

# Combine all include directories
list(APPEND CGAL_INCLUDE_DIRS
  ${GMP_INCLUDE_DIR}
  ${MPFR_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
)

# Set libraries
set(CGAL_LIBRARIES ${GMP_LIBRARIES} ${MPFR_LIBRARIES})

# CGAL definitions
add_definitions(-DCGAL_HEADER_ONLY)

# Provide CGAL_USE_FILE for compatibility
set(CGAL_USE_FILE ${CMAKE_CURRENT_LIST_FILE})