#include <ros/ros.h>

#include <hector_camera_lidar_calibration/data_collector.h>
#include <hector_camera_lidar_calibration/optimizer.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "camera_lidar_calibration_node");

  hector_calibration::camera_lidar_calibration::DataCollector collector;
  hector_calibration_msgs::CameraLidarCalibrationData data = collector.captureData();
  std::vector<hector_calibration_msgs::CameraLidarCalibrationData> data_vec;
  data_vec.push_back(data);

  hector_calibration::camera_lidar_calibration::Optimizer opt;
  opt.run(data_vec);
 
  return 0;
}
