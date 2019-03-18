#include <ros/ros.h>

#include <hector_camera_lidar_calibration/data_collector.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "data_collector_node");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  hector_calibration::hector_camera_lidar_calibration::DataCollector collector(nh, pnh);
  collector.captureData();
 
  return 0;
}
