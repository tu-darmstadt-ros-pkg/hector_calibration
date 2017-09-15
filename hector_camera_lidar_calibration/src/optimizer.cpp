#include <hector_camera_lidar_calibration/optimizer.h>

namespace hector_calibration {
namespace camera_lidar_calibration {

Optimizer::Optimizer() {
  ros::NodeHandle cam_nh("~");
  camera_model_loader_.loadCamerasFromNamespace(cam_nh);
}

void Optimizer::runOptimization(const hector_calibration_msgs::CameraLidarCalibrationData& data) {

}

}
}
