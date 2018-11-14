#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include <ros/ros.h>
#include <rosbag/view.h>

#include <hector_camera_lidar_calibration/data_collector.h>
#include <hector_camera_lidar_calibration/cost_function/first_order_mi_cost.h>

#include <ceres/ceres.h>
#include <ceres_nlopt_wrapper/ceres_nlopt_wrapper.h>

namespace hector_calibration {
namespace hector_camera_lidar_calibration {

class Optimizer {
public:
  Optimizer(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
  void loadFromBag(std::string file_path);
  void loadData(const std::vector<hector_calibration_msgs::CameraLidarCalibrationData>& data);
  void optimize();
  void manualCalibration();
private:
  bool initCalibration();
  void printResult();

  void optimizeCeres();
  void optimizeNLOpt();

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  camera_model::CameraModelLoader camera_model_loader_;

  std::vector<double> initial_parameters_;
  std::vector<double> parameters_;

  double initial_cost_;
  double final_cost_;

  FirstOrderMICost* mi_cost_;

  // Parameters
  std::string optimizer_;
  int bin_fraction_;
  int scan_sample_size_;
  std::vector<hector_calibration_msgs::CameraLidarCalibrationData> data_;
};

}
}

#endif
