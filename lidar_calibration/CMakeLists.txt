cmake_minimum_required(VERSION 3.0.2)
project(lidar_calibration)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  lidar_calibration_lib
  ceres_catkin
)

add_compile_options(-std=c++14)

#SET(CMAKE_BUILD_TYPE RelWithDebInfo)

## System dependencies are found with CMake's conventions
#find_package(Ceres REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES lidar_calibration
  CATKIN_DEPENDS
    roscpp 
    lidar_calibration_lib
    ceres_catkin
#   DEPENDS 
)

###########
## Build ##
###########

set(HEADERS
  include/${PROJECT_NAME}/lidar_calibration.h
  include/${PROJECT_NAME}/point_plane_error.h
  include/${PROJECT_NAME}/calibration.h
  include/${PROJECT_NAME}/cloud_aggregator.h
)

set(SOURCES
  src/lidar_calibration.cpp
  src/cloud_aggregator.cpp
)

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(lidar_calibration
  ${HEADERS} ${SOURCES}
)

## Add cmake target dependencies of the library
add_dependencies(lidar_calibration
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

## Declare a C++ executable
add_executable(lidar_calibration_node src/lidar_calibration_node.cpp)
add_dependencies(lidar_calibration_node ${catkin_EXPORTED_TARGETS})
add_executable(cloud_aggregator_node src/cloud_aggregator_node.cpp)
add_dependencies(cloud_aggregator_node ${catkin_EXPORTED_TARGETS})

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(lidar_calibration_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
)
target_link_libraries(lidar_calibration_node
  ${PROJECT_NAME}
)
target_link_libraries(cloud_aggregator_node
  ${PROJECT_NAME}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_lidar_calibration.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
