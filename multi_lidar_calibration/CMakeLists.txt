cmake_minimum_required(VERSION 2.8.3)
project(multi_lidar_calibration)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  lidar_calibration_lib
)

add_compile_options(-std=c++11)

find_package(Ceres REQUIRED)

#SET(CMAKE_BUILD_TYPE RelWithDebInfo)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    lidar_calibration_lib
#   DEPENDS 
)

###########
## Build ##
###########

set(HEADERS
  include/${PROJECT_NAME}/multi_lidar_calibration.h
  include/${PROJECT_NAME}/lidar_pose_error.h
)

set(SOURCES
  src/multi_lidar_calibration.cpp
)

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  ${HEADERS}
  ${SOURCES}
)

## Add cmake target dependencies of the library
add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

## Declare a C++ executable
add_executable(${PROJECT_NAME}_node src/multi_lidar_calibration_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(calibration_test_node src/calibration_test_node.cpp)
add_dependencies(calibration_test_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
)
target_link_libraries(multi_lidar_calibration_node
  ${PROJECT_NAME}
)

target_link_libraries(calibration_test_node
  ${PROJECT_NAME}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_multi_lidar_calibration.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
