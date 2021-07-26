cmake_minimum_required(VERSION 3.0.2)
project(lidar_calibration_lib)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  hector_calibration_msgs
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  tf_conversions
)

add_compile_options(-std=c++14)

#SET(CMAKE_BUILD_TYPE RelWithDebInfo)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES lidar_calibration_lib
  CATKIN_DEPENDS 
    hector_calibration_msgs 
    pcl_conversions 
    pcl_ros 
    roscpp 
    sensor_msgs 
    tf_conversions
#   DEPENDS 
)

###########
## Build ##
###########

set(HEADERS
  include/${PROJECT_NAME}/lidar_calibration_common.h
)

set(SOURCES
  src/lidar_calibration_common.cpp
)

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  ${HEADERS}
  ${SOURCES}
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_lidar_calibration_lib.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
