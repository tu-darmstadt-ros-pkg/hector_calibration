#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "camera_lidar_calibration_node");
 
  ros::spin();
  return 0;
}
