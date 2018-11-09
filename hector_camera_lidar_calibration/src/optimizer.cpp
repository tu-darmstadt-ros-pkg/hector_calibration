#include <hector_camera_lidar_calibration/optimizer.h>

namespace hector_calibration {
namespace hector_camera_lidar_calibration {

Optimizer::Optimizer(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
  : nh_(nh), pnh_(pnh){
  ros::NodeHandle cam_nh(pnh);
  camera_model_loader_.loadCamerasFromNamespace(cam_nh);

  pnh.param("optimizer", optimizer_, std::string("ceres"));
  pnh.param("bin_fraction", bin_fraction_, 1);
  pnh.param("scan_sample_size", scan_sample_size_, 300000);
  pnh.param("initial_offset", initial_offset_, std::vector<double>(6, 0.0));
  if (initial_offset_.size() != 6) {
    ROS_ERROR_STREAM("Initial offset has an invalid size of " << initial_offset_.size() << ". Expected 6.");
    initial_offset_.resize(6, 0.0);
  }

  parameters_.resize(6, 0);
  initial_parameters_.resize(6, 0);
}

void Optimizer::loadFromBag(std::string file_path) {
  ROS_INFO_STREAM("Loading calibration bag from '" << file_path << "'.");
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

  if (optimizer_ == "ceres") {
    optimizeCeres();
  } else {
    optimizeNLOpt();
  }
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
    parameters_[static_cast<size_t>(param_num)] += offset;
    mi_cost_->Evaluate(&parameters_[0], &cost, NULL);
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
    initial_parameters_[i] = parameters_[i] = xyz(i);
    parameters_[i] += initial_offset_[i];
    initial_parameters_[i+3] = parameters_[i+3] = rpy(i);
    parameters_[i+3] += initial_offset_[i+3];
  }
  ROS_INFO_STREAM("=== Initial calibration ===");
  ROS_INFO_STREAM("Transform from " << data_[0].cam_transform.header.frame_id  << " to " << data_[0].cam_transform.child_frame_id << ":");
  ROS_INFO_STREAM(ceres_nlopt_wrapper::vecToString(parameters_));

  mi_cost_ = new FirstOrderMICost(data_, camera_model_loader_, bin_fraction_, scan_sample_size_);
  mi_cost_->Evaluate(&parameters_[0], &initial_cost_, NULL);
  return true;
}

void Optimizer::printResult()
{
  std::stringstream result_ss;
  result_ss << std::endl;
  result_ss << "==== Result ====" << std::endl;
  result_ss << "Initial:\t" << ceres_nlopt_wrapper::vecToString(initial_parameters_) << std::endl;
  result_ss << "Result:\t\t" << ceres_nlopt_wrapper::vecToString(parameters_) << std::endl;
  std::vector<double> difference(6);
  for (unsigned int i = 0; i < 6; i++) {
    difference[i] = parameters_[i] - initial_parameters_[i];
  }
  result_ss << "Difference:\t" << ceres_nlopt_wrapper::vecToString((difference)) << std::endl;
  result_ss << std::endl;
  result_ss << "Initial cost: " << initial_cost_ << std::endl;
  mi_cost_->Evaluate(&parameters_[0], &final_cost_, NULL);
  result_ss << "Final cost: " << final_cost_ << std::endl;
  result_ss << "Difference: " << initial_cost_ - final_cost_ << std::endl;
  result_ss << "-----------------------" << std::endl;
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
  result_ss << "Transform from " << data_[0].cam_transform.child_frame_id  << " to " << data_[0].cam_transform.header.frame_id << ":" << std::endl;
  result_ss << parametersToString(result) << std::endl;
  ROS_INFO_STREAM(result_ss.str());
}

void Optimizer::optimizeCeres()
{
  // Solve problem with ceres
  ceres::GradientProblem problem(mi_cost_);

  ceres::GradientProblemSolver::Options options;
  options.minimizer_progress_to_stdout = true;
  options.line_search_direction_type = ceres::BFGS; // only 6 parameters, we don't need the approximated variant
//  options.line_search_direction_type = ceres::STEEPEST_DESCENT; // only 6 parameters, we don't need the approximated variant
//  options.line_search_type = ceres::ARMIJO;
  ceres::GradientProblemSolver::Summary summary;
  ceres::Solve(options, problem, &parameters_[0], &summary);

  std::cout << summary.FullReport() << std::endl;
}

void Optimizer::optimizeNLOpt()
{
  nlopt::opt opt(ceres_nlopt_wrapper::stringToAlgorithm(optimizer_), 6);

  ros::NodeHandle stopping_criteria_nh(pnh_, "stopping_criteria");
  ceres_nlopt_wrapper::setParametersFromServer(stopping_criteria_nh, opt);

  // Define parameter bounds
  if (pnh_.param("add_bounds", false)) {
    std::vector<double> lower_bounds(6);
    std::vector<double> upper_bounds(6);
    const double bound = 0.1;
    for (unsigned int i = 0; i < lower_bounds.size(); i++) {
      lower_bounds[i] = initial_parameters_[i] - bound;
      upper_bounds[i] = initial_parameters_[i] + bound;
    }
    opt.set_lower_bounds(lower_bounds);
    opt.set_upper_bounds(upper_bounds);
  }

  // Set convergence criterion
//  opt.set_xtol_rel(1e-4);
//  opt.set_ftol_rel(1e-5);
//  opt.set_xtol_abs(0.0001);
//  opt.set_maxtime(0.1);
//  opt.set_ftol_abs(0.00001);

  // Add objective function

  ceres_nlopt_wrapper::CeresCostFunctionWrapper mi_cost_wrapper(mi_cost_->getCostFunction(), 0, false, ceres::DO_NOT_TAKE_OWNERSHIP);
  mi_cost_wrapper.setName("mi_cost");
  mi_cost_wrapper.enableInfCheck(true);
  mi_cost_wrapper.enableNanCheck(true);
  opt.set_min_objective(ceres_nlopt_wrapper::CeresCostFunctionWrapper::wrap, &mi_cost_wrapper);

  double minf;
  auto start = std::chrono::high_resolution_clock::now();
  nlopt::result nlopt_result = nlopt::result::FAILURE;
  try {
    nlopt_result = opt.optimize(parameters_, minf);
  }
  catch (std::invalid_argument e) {
    ROS_ERROR_STREAM("Optimization failed with invalid argument error: " << e.what());
  }
  catch (nlopt::roundoff_limited e) {
    ROS_ERROR_STREAM("Optimization failed: " << e.what());
  }
  catch (std::runtime_error e) {
    ROS_ERROR_STREAM("Optimization failed: " << e.what());
  }
  std::chrono::duration<double> elapsed = std::chrono::high_resolution_clock::now() - start;
  ROS_INFO_STREAM("Result: " << ceres_nlopt_wrapper::resultToString(nlopt_result));
  ROS_INFO_STREAM("Combined optimization took " << mi_cost_wrapper.getEvaluations() << " evaluations in " << elapsed.count()*1000.0 << " ms.");
  ROS_INFO_STREAM("Time per evaluation " << elapsed.count()*1000.0 / mi_cost_wrapper.getEvaluations() << " ms.");
}

}
}
