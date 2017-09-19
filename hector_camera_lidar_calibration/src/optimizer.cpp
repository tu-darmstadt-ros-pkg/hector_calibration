#include <hector_camera_lidar_calibration/optimizer.h>

namespace hector_calibration {
namespace camera_lidar_calibration {

Optimizer::Optimizer() {
  ros::NodeHandle cam_nh("~");
  camera_model_loader_.loadCamerasFromNamespace(cam_nh);

  ros::NodeHandle pnh("~");
  pnh.param("bin_fraction", bin_fraction_, 1);
}

void Optimizer::runFromBag(std::string file_path) {

}

void Optimizer::run(const std::vector<hector_calibration_msgs::CameraLidarCalibrationData>& data) {
  ROS_INFO_STREAM("Starting optimization");
  if (data.empty()) {
    ROS_ERROR_STREAM("Calibration data vector is empty.");
    return;
  }

  // Load initial calibration
  Eigen::Affine3d init_transform;
  tf::transformMsgToEigen(data[0].cam_transform.transform, init_transform);

  Eigen::Vector3d ypr = init_transform.linear().eulerAngles(2, 1, 0);
  Eigen::Vector3d xyz = init_transform.translation();

  double parameters[6];
  for (unsigned int i = 0; i < 3; i++) {
    parameters[i] = xyz(i);
    parameters[i+3] = ypr(2-i);
  }

  ROS_INFO_STREAM("Initial calibration:");
  printParameters(parameters);

  // Solve problem with ceres
  ceres::GradientProblem problem(new MutualInformationCost(data, camera_model_loader_, bin_fraction_));

  ceres::GradientProblemSolver::Options options;
  options.minimizer_progress_to_stdout = true;
  ceres::GradientProblemSolver::Summary summary;
  ceres::Solve(options, problem, parameters, &summary);

  std::cout << summary.FullReport() << std::endl;

  ROS_INFO_STREAM("Optimization result:");
  printParameters(parameters);

}

void Optimizer::printParameters(double * parameters) {
  std::stringstream ss;
  ss << "[x=" << parameters[0] << ", y=" << parameters[1] << ", z=" << parameters[2]
     << "; roll=" << parameters[3] << ", pitch=" << parameters[4] << ", yaw=" << parameters[5] << "]";
  ROS_INFO_STREAM(ss.str());
}

}
}
