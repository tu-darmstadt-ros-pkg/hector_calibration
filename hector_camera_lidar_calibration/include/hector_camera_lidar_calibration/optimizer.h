#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include <ros/ros.h>
#include <rosbag/view.h>

#include <hector_camera_lidar_calibration/data_collector.h>
#include <hector_camera_lidar_calibration/cost_function/first_order_mi_cost.h>

#include <ceres/ceres.h>

namespace hector_calibration {
namespace hector_camera_lidar_calibration {

class Optimizer {
public:
  Optimizer();
  void loadFromBag(std::string file_path);
  void loadData(const std::vector<hector_calibration_msgs::CameraLidarCalibrationData>& data);
  void optimize();
  void manualCalibration();
private:
  bool initCalibration();
  void printResult();

  camera_model::CameraModelLoader camera_model_loader_;

  double parameters_[6];
  double initial_parameters_[6];
  std::vector<double> initial_offset_;
  FirstOrderMICost* mi_cost_;

  // Parameters
  int bin_fraction_;
  int scan_sample_size_;
  std::vector<hector_calibration_msgs::CameraLidarCalibrationData> data_;
};

}
}

#endif
