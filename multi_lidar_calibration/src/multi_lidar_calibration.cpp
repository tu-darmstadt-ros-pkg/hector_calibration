#include <multi_lidar_calibration/multi_lidar_calibration.h>

using namespace std::chrono_literals;

namespace hector_calibration {
namespace lidar_calibration {

MultiLidarCalibration::MultiLidarCalibration()
  : Node("multi_lidar_calibration"),
    scan1_counter_(0),
    scan2_counter_(0),
    imu_counter_(0),
    tf_published_(false),
    cloud1_msg_(std::make_shared<sensor_msgs::msg::PointCloud2>()),
    cloud2_msg_(std::make_shared<sensor_msgs::msg::PointCloud2>()),
    cloud1_prepr_msg_(std::make_shared<sensor_msgs::msg::PointCloud2>()),
    cloud2_prepr_msg_(std::make_shared<sensor_msgs::msg::PointCloud2>()),
    cloud2_init_guess_msg_(std::make_shared<sensor_msgs::msg::PointCloud2>()),
    cloud2_result_msg_(std::make_shared<sensor_msgs::msg::PointCloud2>())  
{  
  // Declare and load parameters
  this->declare_parameter("min_scans", 1);
  this->declare_parameter("init_guess_rpy", std::vector<double>({0, 0, 0}));
  this->declare_parameter("init_guess_xyz", std::vector<double>({0, 0, 0}));
  this->declare_parameter("max_sqr_dist", 0.0025);
  this->declare_parameter("neighbor_mapping_vis_count", 100);
  this->declare_parameter("normals_radius", 0.07);
  this->declare_parameter("crop_dist", 1.0);
  this->declare_parameter("voxel_leaf_size", 0.01);
  this->declare_parameter("max_iterations", 20);
  this->declare_parameter("parameter_diff_thres", 1e-3);
  this->declare_parameter("save_path", "");
  this->declare_parameter("use_imu", false);
  std::vector<double> init_guess_rpy;
  std::vector<double> init_guess_xyz;
  this->get_parameter("init_guess_rpy", init_guess_rpy);
  this->get_parameter("init_guess_xyz", init_guess_xyz);
  double roll = init_guess_rpy[0] * M_PI / 180.0;
  double pitch = init_guess_rpy[1] * M_PI / 180.0;
  double yaw = init_guess_rpy[2] * M_PI / 180.0;
  Eigen::Matrix3d rotation;
  rotation = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
            * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
            * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
  Eigen::Vector3d translation(init_guess_xyz[0], init_guess_xyz[1], init_guess_xyz[2]);
  translation *= 1e-3; // convert to meters
  init_guess_ = Eigen::Affine3d::Identity();
  init_guess_.linear() = rotation;
  init_guess_.translation() = translation;  
  this->get_parameter("min_scans", min_scans_);
  this->get_parameter("max_sqr_dist", max_sqr_dist_);
  this->get_parameter("neighbor_mapping_vis_count", neighbor_mapping_vis_count_);
  this->get_parameter("normals_radius", normals_radius_);
  this->get_parameter("crop_dist", crop_dist_);
  this->get_parameter("voxel_leaf_size", voxel_leaf_size_);
  this->get_parameter("max_iterations", max_iterations_);
  this->get_parameter("parameter_diff_thres", parameter_diff_thres_);
  this->get_parameter("save_path", save_path_);
  this->get_parameter("use_imu", use_imu_);
  imu_ok_ = !use_imu_;
  min_imu_msgs_ = 100;

  // Init subscribers
  cloud1_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "cloud1", 1, std::bind(&MultiLidarCalibration::cloudCb1, this, std::placeholders::_1));
  cloud2_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "cloud2", 1, std::bind(&MultiLidarCalibration::cloudCb2, this, std::placeholders::_1));
  if (use_imu_) {
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu1", 1, std::bind(&MultiLidarCalibration::imuCb, this, std::placeholders::_1));
  }

  // Init publishers
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  raw_pub_[0] = this->create_publisher<sensor_msgs::msg::PointCloud2>("raw_cloud_1", qos);
  raw_pub_[1] = this->create_publisher<sensor_msgs::msg::PointCloud2>("raw_cloud_2", qos);
  preprocessed_pub_[0] = this->create_publisher<sensor_msgs::msg::PointCloud2>("preprocessed_cloud_1", 1);
  preprocessed_pub_[1] = this->create_publisher<sensor_msgs::msg::PointCloud2>("preprocessed_cloud_2", 1);
  init_guess_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("init_guess_cloud", 1);
  result_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("result_cloud", 1);
  tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

  world_frame_ = "world";
  lidar_frame1_ = "livox1";
  lidar_frame2_ = "livox2";
  publish_timer_ = this->create_wall_timer(1000ms, std::bind(&MultiLidarCalibration::publishClouds, this));
}

void MultiLidarCalibration::cloudCb1(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (scan1_counter_ < min_scans_) {                
    pcl::PointCloud<pcl::PointXYZ> tmp_cloud;
    pcl::fromROSMsg(*msg, tmp_cloud);
    cloud1_ += tmp_cloud;
    scan1_counter_++;
    std::cout << "topic " << cloud1_sub_->get_topic_name() << ": received cloud " << scan1_counter_ << "/" << min_scans_ << " | total points: " << cloud1_.size() << std::endl;
  }
}

void MultiLidarCalibration::cloudCb2(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (scan1_counter_ >= min_scans_ && scan2_counter_ >= min_scans_ && imu_ok_) {
    cloud1_sub_.reset();
    cloud2_sub_.reset();
    std::cout << "Received enough scans. Starting calibration." << std::endl;
    this->calibrate();
  }
  if (scan2_counter_ < min_scans_) {                
    pcl::PointCloud<pcl::PointXYZ> tmp_cloud;
    pcl::fromROSMsg(*msg, tmp_cloud);
    cloud2_ += tmp_cloud;
    scan2_counter_++;
    std::cout << "topic " << cloud1_sub_->get_topic_name() << ": received cloud " << scan2_counter_ << "/" << min_scans_ << " | total points: " << cloud2_.size() << std::endl;
  }
}

void MultiLidarCalibration::imuCb(const sensor_msgs::msg::Imu::SharedPtr msg)
{  
  if (imu_counter_ < min_imu_msgs_) {
    sum_imu_acc_[0] += msg->linear_acceleration.x;
    sum_imu_acc_[1] += msg->linear_acceleration.y;
    sum_imu_acc_[2] += msg->linear_acceleration.z;
    imu_counter_++;
  }
  else {
    imu_ok_ = true;
    std::cout << "Received enough imu messages." << std::endl;
    imu_sub_.reset();    
    }
}

void MultiLidarCalibration::calibrate()
{
  this->cloudToMsg(cloud1_, cloud1_msg_);
  this->cloudToMsg(cloud2_, cloud2_msg_);
  std::cout << "========================================" << std::endl;
  std::cout << "Cloud 1 size: " << cloud1_.size() << std::endl;
  std::cout << "Cloud 2 size: " << cloud2_.size() << std::endl;

  this->preprocessClouds();
  this->cloudToMsg(cloud1_, cloud1_prepr_msg_);
  this->cloudToMsg(cloud2_, cloud2_prepr_msg_);
  std::cout << "Cloud 1 preprocessed size: " << cloud1_.size() << std::endl;
  std::cout << "Cloud 2 preprocessed size: " << cloud2_.size() << std::endl;

  std::vector<WeightedNormal> normals = computeNormals(cloud1_, normals_radius_);
  std::cout << "Computed " << normals.size() << " normals." << std::endl;  

  Eigen::Affine3d calibration = init_guess_;
  Eigen::Affine3d prev_calibration = init_guess_;
  
  // Apply initial guess for visualization
  std::cout << "\n\nInitial Transformation Matrix:\n" << init_guess_.matrix() << std::endl;
  pcl::PointCloud<pcl::PointXYZ> cloud2_t;
  pcl::transformPointCloud(cloud2_, cloud2_t, init_guess_);
  this->cloudToMsg(cloud2_t, cloud2_init_guess_msg_);

  unsigned int iteration_counter = 0;
  double max_distance = max_sqr_dist_;
  while (rclcpp::ok()) 
  {
    std::cout << "\n-------------- Starting iteration " << (iteration_counter+1) << "--------------" << std::endl;
    std::cout << "Searching neighbors with max dist of " << std::sqrt(max_distance) << std::endl;
    pcl::transformPointCloud(cloud2_, cloud2_t, calibration);
    std::map<unsigned int, unsigned int> neighbor_mapping = findNeighbors(cloud1_, cloud2_t, max_distance);
    std::cout << "Found " << neighbor_mapping.size() << " neighbors." << std::endl;
    std::cout << "Avg residual: " << this->computeAvgResidual(cloud1_, cloud2_t, neighbor_mapping) << std::endl;
    max_distance *= 0.5;

    prev_calibration = calibration;
    calibration = this->optimize(cloud1_, cloud2_, normals, neighbor_mapping, calibration);
    pcl::transformPointCloud(cloud2_, cloud2_t, calibration);
    this->cloudToMsg(cloud2_t, cloud2_result_msg_);

    iteration_counter++;

    if (maxIterationsReached(iteration_counter)) {
      std::cout << "-------- MAX ITERATIONS REACHED ---------" << std::endl;
      break;
    }
    else if (checkConvergence(prev_calibration, calibration)) {
      std::cout << "-------------- CONVERGENCE --------------" << std::endl;
      break;
    }
  }

  calibration_ = calibration;

  std::cout << std::endl << "INITIAL GUESS" << std::endl;
  std::cout << printCalibration(init_guess_);

  std::cout << std::endl << "RESULT" << std::endl;
  std::cout << printCalibration(calibration);

  if (save_path_ != "") {
    this->saveToDisk(save_path_, calibration);
  }
  else {
    RCLCPP_INFO(this->get_logger(), "No save path specified. Not saving calibration.");
  }
}

bool MultiLidarCalibration::maxIterationsReached(unsigned int current_iterations) const {
  return current_iterations >= max_iterations_;
}

bool MultiLidarCalibration::checkConvergence(const Eigen::Affine3d& prev_calibration,
                                             const Eigen::Affine3d& current_calibration) const
{
  double cum_sqrt_diff = this->getCumSqrtDiff(prev_calibration, current_calibration);
  return cum_sqrt_diff < parameter_diff_thres_;
}

double MultiLidarCalibration::getCumSqrtDiff(const Eigen::Affine3d& prev_calibration,
                                             const Eigen::Affine3d& current_calibration) const
{
  Eigen::Vector3d prev_ypr = prev_calibration.linear().eulerAngles(2, 1, 0);
  Eigen::Vector3d prev_xyz = prev_calibration.translation();

  Eigen::Vector3d current_ypr = current_calibration.linear().eulerAngles(2, 1, 0);
  Eigen::Vector3d current_xyz = current_calibration.translation();

  double cum_sqrt_diff = 0;
  for (unsigned int i = 0; i < 3; i++) {
    cum_sqrt_diff += std::pow(prev_ypr(i) - current_ypr(i), 2);
    cum_sqrt_diff += std::pow(prev_xyz(i) - current_xyz(i), 2);
  }
  return cum_sqrt_diff;
  }

double MultiLidarCalibration::computeAvgResidual(const pcl::PointCloud<pcl::PointXYZ>& cloud1,
                          const pcl::PointCloud<pcl::PointXYZ>& cloud2,
                          const std::map<unsigned int, unsigned int>& mapping) const
{
  if (mapping.empty()) return 0;
  double cum_residual = 0;
  for (std::map<unsigned int, unsigned int>::const_iterator it = mapping.begin(); it != mapping.end(); it++) {
    unsigned int x1_index = it->first;
    unsigned int x2_index = it->second;
    Eigen::Vector3d x1(cloud1[x1_index].x, cloud1[x1_index].y, cloud1[x1_index].z);
    Eigen::Vector3d x2(cloud2[x2_index].x, cloud2[x2_index].y, cloud2[x2_index].z);
    cum_residual += (x1 - x2).norm();
  }
  return cum_residual / mapping.size();
}

void MultiLidarCalibration::preprocessClouds()
{
  this->cropCloud(cloud1_, crop_dist_);
  this->cropCloud(cloud2_, crop_dist_);
  if (voxel_leaf_size_ > 0) {
    this->downsampleCloud(cloud1_, voxel_leaf_size_);
    this->downsampleCloud(cloud2_, voxel_leaf_size_);
  }  
}

void MultiLidarCalibration::cropCloud(pcl::PointCloud<pcl::PointXYZ>& cloud, double distance)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());
  *cloud_ptr = cloud;

  pcl::CropBox<pcl::PointXYZ> crop_box_filter;
  crop_box_filter.setInputCloud(cloud_ptr);
  crop_box_filter.setNegative(true);
  Eigen::Vector4f box_v(1, 1, 1, 1);
  Eigen::Vector4f min_v = -distance * box_v;
  min_v[3] = 1;
  Eigen::Vector4f max_v = distance * box_v;
  max_v[3] = 1;

  crop_box_filter.setMin(min_v);
  crop_box_filter.setMax(max_v);
  crop_box_filter.filter(cloud);
}

void MultiLidarCalibration::downsampleCloud(pcl::PointCloud<pcl::PointXYZ>& cloud, float leaf_size)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  *cloud_ptr = cloud;

  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud(cloud_ptr);
  vg.setLeafSize(leaf_size, leaf_size, leaf_size);

  vg.filter(cloud);
}

Eigen::Affine3d
MultiLidarCalibration::optimize(const pcl::PointCloud<pcl::PointXYZ> &cloud1,
              const pcl::PointCloud<pcl::PointXYZ> &cloud2,
              const std::vector<WeightedNormal> &normals,
              const std::map<unsigned int, unsigned int> &mapping,
              const Eigen::Affine3d& initial_calibration)
{
  if (cloud1.size() != normals.size()) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Size of cloud1 (" << cloud1.size() << ") doesn't match size of normals (" << normals.size() << ").");
    return Eigen::Affine3d::Identity();
  }

  ceres::Problem problem;

  Eigen::Vector3d ypr = initial_calibration.linear().eulerAngles(2, 1, 0);
  Eigen::Vector3d xyz = initial_calibration.translation();

  double translation[3];
  double rotation[3];
  for (unsigned int i = 0; i < 3; i++) {
    translation[i] = xyz(i);
    rotation[i] = ypr(2-i);
  }

  unsigned int residual_count = 0;
  for(std::map<unsigned int, unsigned int>::const_iterator it = mapping.begin(); it != mapping.end(); it++) {
    unsigned int x1_index = it->first;
    unsigned int x2_index = it->second;
    Eigen::Vector3d x1(cloud1[x1_index].x, cloud1[x1_index].y, cloud1[x1_index].z);
    Eigen::Vector3d x2(cloud2[x2_index].x, cloud2[x2_index].y, cloud2[x2_index].z);

    ceres::CostFunction* cost_function = LidarPoseError::Create(x1, x2, normals[x1_index]);

    problem.AddResidualBlock(cost_function, NULL, rotation, translation);
    residual_count++;
  }
  std::cout << "Number of residuals: " << residual_count << std::endl;

  ceres::Solver::Options options;
  //options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  //std::cout << summary.BriefReport() << "\n";

  Eigen::Affine3d calibration(
        Eigen::AngleAxisd(rotation[2], Eigen::Vector3d::UnitZ())
      * Eigen::AngleAxisd(rotation[1], Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(rotation[0], Eigen::Vector3d::UnitX())
  );
  calibration.translation() = Eigen::Vector3d(translation[0], translation[1], translation[2]);

  return calibration;
}

std::string MultiLidarCalibration::printCalibration(const Eigen::Affine3d& calibration) const {
  std::stringstream ss;
  Eigen::Vector3d ypr = calibration.linear().eulerAngles(2, 1, 0);
  Eigen::Vector3d xyz = calibration.translation();
  ss << "x: " << xyz(0) << ", y: " << xyz(1) <<  ", z: " << xyz(2)  << std::endl;
  ss << "rpy [rad] | roll: " << normalizeAngle(ypr(2)) << ", pitch: " << normalizeAngle(ypr(1)) << ", yaw: " << normalizeAngle(ypr(0)) << std::endl;
  ss << "rpy [deg] | roll: " << normalizeAngle(ypr(2)) * 180.0 / M_PI << ", pitch: " << normalizeAngle(ypr(1)) * 180.0 / M_PI << ", yaw: " << normalizeAngle(ypr(0)) * 180.0 / M_PI << std::endl;
  ss << "Transformation matrix:\n" << calibration.matrix() << std::endl;
  return ss.str();           
}

std::string MultiLidarCalibration::printCalibration(double x, double y, double z, double roll, double pitch, double yaw) const {
  std::stringstream ss;
  ss << "x: " << x << ", y: " << y << ", z: " << z << std::endl;
  ss << "rpy [rad] | roll: " << normalizeAngle(roll) << ", pitch: " << normalizeAngle(pitch) << ", yaw: " << normalizeAngle(yaw) << std::endl;
  ss << "rpy [deg] | roll: " << normalizeAngle(roll) * 180.0 / M_PI << ", pitch: " << normalizeAngle(pitch) * 180.0 / M_PI << ", yaw: " << normalizeAngle(yaw) * 180.0 / M_PI << std::endl;
  ss << "Transformation matrix:\n" << Eigen::Affine3d(Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
                                                            * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
                                                            * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())).matrix() << std::endl;
  return ss.str();
}

void MultiLidarCalibration::cloudToMsg(const pcl::PointCloud<pcl::PointXYZ>& cloud, sensor_msgs::msg::PointCloud2::SharedPtr& msg)
{
  pcl::toROSMsg(cloud, *msg);
  msg->header.frame_id = lidar_frame1_;
}

void MultiLidarCalibration::publishClouds()
{
  if (!cloud1_msg_->data.empty()) {
    raw_pub_[0]->publish(*cloud1_msg_);    
  }
  if (!cloud2_msg_->data.empty()) {
    raw_pub_[1]->publish(*cloud2_msg_);
  } 
  if (!cloud1_prepr_msg_->data.empty()) {
    preprocessed_pub_[0]->publish(*cloud1_prepr_msg_);
  }
  if (!cloud2_prepr_msg_->data.empty()) {
    preprocessed_pub_[1]->publish(*cloud2_prepr_msg_);
  } 
  if (!cloud2_init_guess_msg_->data.empty()) {
    init_guess_pub_->publish(*cloud2_init_guess_msg_);
  } 
  if (!cloud2_result_msg_->data.empty()) {
    result_pub_->publish(*cloud2_result_msg_);
  }
  if (!tf_published_ && imu_ok_) {
    this->publishTf();
    tf_published_ = true;
  }
}

void MultiLidarCalibration::publishTf() {

  // world -> lidar1
  tf2::Quaternion orientation;

  if (use_imu_ && imu_ok_) {
    double accel_x = sum_imu_acc_[0] / imu_counter_;
    double accel_y = sum_imu_acc_[1] / imu_counter_;
    double accel_z = sum_imu_acc_[2] / imu_counter_;

    double roll = std::atan2(accel_y, accel_z);
    double pitch = std::atan2(-accel_x, std::sqrt(accel_y*accel_y + accel_z*accel_z));
    double yaw = 0;

    orientation.setRPY(roll, pitch, yaw);
  }
  else {
    orientation.setRPY(0, 0, 0);
  }

  geometry_msgs::msg::TransformStamped tf_world_l1;

  tf_world_l1.header.stamp = this->get_clock()->now();
  tf_world_l1.header.frame_id = world_frame_;
  tf_world_l1.child_frame_id = lidar_frame1_;
  tf_world_l1.transform.translation.x = 0.0;
  tf_world_l1.transform.translation.y = 0.0;
  tf_world_l1.transform.translation.z = 0.0;
  tf_world_l1.transform.rotation.x = orientation.x();
  tf_world_l1.transform.rotation.y = orientation.y();
  tf_world_l1.transform.rotation.z = orientation.z();
  tf_world_l1.transform.rotation.w = orientation.w();

  tf_static_broadcaster_->sendTransform(tf_world_l1);

  // lidar1 -> lidar2
  //Eigen::Affine3d calibration_lidar1_to_lidar2 = calibration_.inverse();
  
  geometry_msgs::msg::TransformStamped tf_l1_l2;
  tf_l1_l2.header.stamp = this->get_clock()->now();
  tf_l1_l2.header.frame_id = lidar_frame1_;
  tf_l1_l2.child_frame_id = lidar_frame2_;
  tf_l1_l2.transform.translation.x = calibration_.translation().x();
  tf_l1_l2.transform.translation.y = calibration_.translation().y();
  tf_l1_l2.transform.translation.z = calibration_.translation().z();
  Eigen::Quaterniond q(calibration_.linear());
  tf_l1_l2.transform.rotation.x = q.x();
  tf_l1_l2.transform.rotation.y = q.y();
  tf_l1_l2.transform.rotation.z = q.z();
  tf_l1_l2.transform.rotation.w = q.w();

  tf_static_broadcaster_->sendTransform(tf_l1_l2);


}

bool MultiLidarCalibration::saveToDisk(std::string path, const Eigen::Affine3d& calibration) const {

  std::cout << "Saving calibration to " << path << std::endl;

  std::ofstream outfile (path);
  outfile << printCalibration(calibration);
  outfile.close();
  return true;
}

}
}

