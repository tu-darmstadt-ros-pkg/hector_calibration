cmake_minimum_required(VERSION 3.5)
project(hector_calibration_msgs)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

## Generate messages in the 'msg' folder
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CameraLidarCalibrationData.msg"
  "msg/CameraObservation.msg"
  "srv/RequestScans.srv"
  DEPENDENCIES std_msgs sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
)

ament_package()
