cmake_minimum_required(VERSION 3.5)
project(lidar_calibration_lib)

## Find ament macros and libraries
## if COMPONENTS list like find_package(ament_cmake REQUIRED COMPONENTS xyz)
## is used, also find other ament packages
find_package(ament_cmake REQUIRED)
find_package(hector_calibration_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)

add_compile_options(-std=c++17)

## System dependencies are found with CMake's conventions
find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

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
  ${hector_calibration_msgs_INCLUDE_DIRS}
  ${pcl_conversions_INCLUDE_DIRS}
  ${pcl_ros_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${visualization_msgs_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_eigen_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  ${HEADERS}
  ${SOURCES}
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${hector_calibration_msgs_LIBRARIES}
  ${pcl_conversions_LIBRARIES}
  ${pcl_ros_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${visualization_msgs_LIBRARIES}
  ${tf2_LIBRARIES}
  ${tf2_eigen_LIBRARIES}
  ${tf2_ros_LIBRARIES}
)

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

#############
## ament specific configuration ##
#############

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(hector_calibration_msgs pcl_conversions pcl_ros rclcpp sensor_msgs visualization_msgs tf2 tf2_eigen tf2_ros)
ament_package()
