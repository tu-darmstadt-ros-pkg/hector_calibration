#include <lidar_calibration/lidar_calibration.h>

int main(int argc, char** argv) {
  std::cout << "Node started." << std::endl;
  ros::init(argc, argv, "lidar_calibration_node");
  ROS_INFO_STREAM("Lidar calibration node created");

  ros::NodeHandle nh("lidar_calibration");

  google::InitGoogleLogging(argv[0]);
  hector_calibration::LidarCalibration calibration(nh);
  ROS_INFO_STREAM("Starting calibration");
  calibration.start_calibration("/head_lidar/half_scans");

  ros::spin();

  return 0;
}
