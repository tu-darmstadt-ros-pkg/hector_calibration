#include <multi_lidar_calibration/multi_lidar_calibration.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "lidar_calibration_node");

  ros::NodeHandle nh;
  // google::InitGoogleLogging(argv[0]);

  hector_calibration::lidar_calibration::MultiLidarCalibration mlc;


  return 0;
}
