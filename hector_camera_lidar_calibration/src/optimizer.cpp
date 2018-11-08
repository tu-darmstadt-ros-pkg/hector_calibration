#include <hector_camera_lidar_calibration/optimizer.h>

namespace hector_calibration {
namespace hector_camera_lidar_calibration {

Optimizer::Optimizer() {
  ros::NodeHandle cam_nh("~");
  camera_model_loader_.loadCamerasFromNamespace(cam_nh);

  ros::NodeHandle pnh("~");
  pnh.param("bin_fraction", bin_fraction_, 1);
  pnh.param("scan_sample_size", scan_sample_size_, 300000);
  pnh.param("initial_offset", initial_offset_, std::vector<double>(6, 0.0));
  if (initial_offset_.size() != 6) {
    ROS_ERROR_STREAM("Initial offset has an invalid size of " << initial_offset_.size() << ". Expected 6.");
    initial_offset_.resize(6, 0.0);
  }
}

void Optimizer::loadFromBag(std::string file_path) {
  rosbag::Bag bag;
  try {
    bag.open(file_path, rosbag::bagmode::Read);
  } catch (rosbag::BagException& e) {
    ROS_ERROR_STREAM("Cannot open " << file_path << ". " << e.what());
    return;
  }
  rosbag::View data_view(bag, rosbag::TopicQuery("calibration_data"));
  std::vector<hector_calibration_msgs::CameraLidarCalibrationData> calibration_data;
  for (const rosbag::MessageInstance& m: data_view) {
    hector_calibration_msgs::CameraLidarCalibrationData::ConstPtr msg = m.instantiate<hector_calibration_msgs::CameraLidarCalibrationData>();
    data_.push_back(*msg);
  }
}

void Optimizer::loadData(const std::vector<hector_calibration_msgs::CameraLidarCalibrationData>& data) {
  data_ = data;
}


void Optimizer::optimize() {
  if (!initCalibration()) {
    return;
  }

  // Solve problem with ceres
  ceres::GradientProblem problem(mi_cost_);

  ceres::GradientProblemSolver::Options options;
  options.minimizer_progress_to_stdout = true;
  options.line_search_direction_type = ceres::BFGS; // only 6 parameters, we don't need the approximated variant
//  options.line_search_direction_type = ceres::STEEPEST_DESCENT; // only 6 parameters, we don't need the approximated variant
//  options.line_search_type = ceres::ARMIJO;
  ceres::GradientProblemSolver::Summary summary;
  ceres::Solve(options, problem, parameters_, &summary);

  std::cout << summary.FullReport() << std::endl;

  ROS_INFO_STREAM("Optimization result: " << parametersToString(parameters_));
  printResult();
}

void Optimizer::manualCalibration() {
  if (!initCalibration()) {
    return;
  }

  double previous_cost = 0;
  while (ros::ok()) {
    std::cout << "******************************" << std::endl;
    int param_num;
    std::cout << "Choose parameter number [0-5], enter -1 to exit: ";
    std::cin >> param_num;
    if (param_num == -1) {
      break;
    }
    if (param_num < 0 || param_num > 5) {
      std::cout << "Out of limits" << std::endl;
      continue;
    }
    double offset;
    std::cout << "Enter offset: ";
    std::cin >> offset;

    double cost;
    parameters_[param_num] += offset;
    mi_cost_->Evaluate(parameters_, &cost, NULL);
    ros::spinOnce();
    std::cout << "Cost difference: " << cost - previous_cost << std::endl;
    previous_cost = cost;
    std::cout << std::endl;
    std::cout << std::endl;
  }

  printResult();

  delete mi_cost_;
}

bool Optimizer::initCalibration()
{
  ROS_INFO_STREAM("Starting optimization");
  if (data_.empty()) {
    ROS_ERROR_STREAM("Calibration data vector is empty.");
    return false;
  }

  // Load initial calibration
  Eigen::Affine3d init_transform;
  tf::transformMsgToEigen(data_[0].cam_transform.transform, init_transform);

  Eigen::Vector3d rpy = rotToRpy(init_transform.linear());
//  Eigen::Vector3d ypr = init_transform.linear().eulerAngles(2, 1, 0);
  Eigen::Vector3d xyz = init_transform.translation();

  for (unsigned int i = 0; i < 3; i++) {
    parameters_[i] = xyz(i) + initial_offset_[i];
    parameters_[i+3] = rpy(i) + initial_offset_[i+3];
  }
  ROS_INFO_STREAM("=== Initial calibration ===");
  ROS_INFO_STREAM("Transform from " << data_[0].cam_transform.header.frame_id  << " to " << data_[0].cam_transform.child_frame_id << ":");
  ROS_INFO_STREAM(parametersToString(parameters_));

  mi_cost_ = new FirstOrderMICost(data_, camera_model_loader_, bin_fraction_, scan_sample_size_);
  return true;
}

void Optimizer::printResult()
{
  // Convert to transformation matrix
  Eigen::Vector3d rpy(parameters_[3], parameters_[4], parameters_[5]);
  Eigen::Affine3d transform(rpyToRot(rpy));
  transform.translation() = Eigen::Vector3d(parameters_[0], parameters_[1], parameters_[2]);

  // Invert transform
  Eigen::Affine3d transform_inv = transform.inverse();

  // Convert back to rpy
  Eigen::Vector3d xyz_inv = transform_inv.translation();
  Eigen::Vector3d rpy_inv = rotToRpy(transform_inv.linear());
  double result[6];
  const double f = 10000.0;
  for (unsigned int i = 0; i < 3; i++) {
    result[i] = std::round(xyz_inv(i) * f) / f;
    result[i+3] = std::round(rpy_inv(i) * f) / f;
  }

  ROS_INFO_STREAM("==== Result ====");
  ROS_INFO_STREAM("Transform from " << data_[0].cam_transform.child_frame_id  << " to " << data_[0].cam_transform.header.frame_id << ":");
  ROS_INFO_STREAM(parametersToString(result));
}

}
}
