#include <hector_camera_lidar_calibration/optimizer.h>

namespace hector_calibration {
namespace camera_lidar_calibration {

Optimizer::Optimizer() {
  ros::NodeHandle cam_nh("~");
  camera_model_loader_.loadCamerasFromNamespace(cam_nh);

  ros::NodeHandle pnh("~");
  pnh.param("bin_fraction", bin_fraction_, 1);
}

void Optimizer::runOptimization(const std::vector<hector_calibration_msgs::CameraLidarCalibrationData>& data) {
  double parameters[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // read parameters from URDF

  ceres::GradientProblem problem(new MutualInformationError(data, camera_model_loader_, bin_fraction_));

  ceres::GradientProblemSolver::Options options;
  options.minimizer_progress_to_stdout = true;
  ceres::GradientProblemSolver::Summary summary;
  ceres::Solve(options, problem, parameters, &summary);

  std::cout << summary.FullReport() << std::endl;
}

}
}
