#include <lidar_calibration/lidar_calibration.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "lidar_calibration_node");
  ROS_INFO_STREAM("Lidar calibration node created");

  ros::NodeHandle nh;

  google::InitGoogleLogging(argv[0]);
  hector_calibration::LidarCalibration calibration(nh);
  calibration.setManualMode(true);
  calibration.calibrate();

  ros::spin();

  return 0;
}
