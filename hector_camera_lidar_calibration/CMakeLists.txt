cmake_minimum_required(VERSION 2.8.3)
project(hector_camera_lidar_calibration)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  camera_model_loader
  hector_calibration_msgs
  roscpp
  rosbag
  pcl_ros
  tf2_ros
  tf2_eigen
  eigen_conversions
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Ceres REQUIRED)


################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES hector_camera_lidar_calibration
#  CATKIN_DEPENDS camera_model_loader hector_calibration_msgs roscpp rosbag
#  DEPENDS system_lib
)

###########
## Build ##
###########

set(HEADERS
  include/${PROJECT_NAME}/data_collector.h
  include/${PROJECT_NAME}/optimizer.h
  include/${PROJECT_NAME}/cost_function/mutual_information_cost.h
  include/${PROJECT_NAME}/cost_function/first_order_mi_cost.h
  include/${PROJECT_NAME}/cost_function/numeric_diff_mi_cost.h
  include/${PROJECT_NAME}/helper.h
)

set(SOURCES
  src/data_collector.cpp
  src/optimizer.cpp
  src/cost_function/mutual_information_cost.cpp
  src/cost_function/first_order_mi_cost.cpp
  src/cost_function/numeric_diff_mi_cost.cpp
  src/helper.cpp
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
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
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(data_collector_node src/data_collector_node.cpp)
add_executable(camera_lidar_calibration_node src/camera_lidar_calibration_node.cpp)


## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(data_collector_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(camera_lidar_calibration_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${CERES_LIBRARIES})
target_link_libraries(data_collector_node ${PROJECT_NAME})
target_link_libraries(camera_lidar_calibration_node ${PROJECT_NAME})
