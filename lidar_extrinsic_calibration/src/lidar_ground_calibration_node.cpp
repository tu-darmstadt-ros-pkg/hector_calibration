#include <ros/ros.h>
#include <lidar_extrinsic_calibration/lidar_extrinsic_calibration.h>

#include <stdio.h>

int main(int argc, char** argv) {
  printf("test");
  ros::init(argc, argv, "lidar_ground_calibration_node");

  ROS_INFO_STREAM("Starting ground calibration");

  ros::NodeHandle nh;
  hector_calibration::LidarExtrinsicCalibration calibration(nh);
  calibration.calibrateGround();
  return 0;
}
