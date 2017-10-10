#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include <ros/ros.h>
#include <rosbag/view.h>

#include <hector_camera_lidar_calibration/data_collector.h>
#include <hector_camera_lidar_calibration/mutual_information_cost.h>

#include <ceres/ceres.h>

namespace hector_calibration {
namespace camera_lidar_calibration {

class Optimizer {
public:
  Optimizer();
  void loadFromBag(std::string file_path);
  void loadData(const std::vector<hector_calibration_msgs::CameraLidarCalibrationData>& data);
  void run();
  void visualizeCost();
private:
  camera_model::CameraModelLoader camera_model_loader_;

  // Parameters
  int bin_fraction_;
  std::vector<hector_calibration_msgs::CameraLidarCalibrationData> data_;
};

}
}

#endif
