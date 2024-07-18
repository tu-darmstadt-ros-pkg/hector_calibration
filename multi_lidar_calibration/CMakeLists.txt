cmake_minimum_required(VERSION 3.5)
project(multi_lidar_calibration)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(lidar_calibration_lib REQUIRED)
find_package(Ceres REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

add_compile_options(-std=c++17)

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
  ${rclcpp_INCLUDE_DIRS}
  ${lidar_calibration_lib_INCLUDE_DIRS}
  ${Ceres_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  ${HEADERS}
  ${SOURCES}
)

## Declare a C++ executable
add_executable(${PROJECT_NAME}_node src/multi_lidar_calibration_node.cpp)

# add_executable(calibration_test_node src/calibration_test_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${rclcpp_LIBRARIES}
  ${lidar_calibration_lib_LIBRARIES}
  ${CERES_LIBRARIES}
  ${tf2_LIBRARIES}
  ${tf2_ros_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(
  TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
