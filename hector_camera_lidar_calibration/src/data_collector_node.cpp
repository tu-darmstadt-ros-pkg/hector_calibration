#include <ros/ros.h>

#include <hector_camera_lidar_calibration/data_collector.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "data_collector_node");

  hector_calibration::camera_lidar_calibration::DataCollector collector;
  collector.captureData();
 
  return 0;
}
