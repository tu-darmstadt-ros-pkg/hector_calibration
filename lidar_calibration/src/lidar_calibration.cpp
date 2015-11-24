#include <lidar_calibration/lidar_calibration.h>

namespace hector_calibration {

bool is_valid_point(const pcl::PointXYZ& point) {
  for (unsigned int i = 0; i < 3; i++) {
    if (std::isnan(point.data[i]) || std::isinf(point.data[i])) {
      return false;
    }
  }
  return true;
}

bool is_valid_cloud(const pcl::PointCloud<pcl::PointXYZ>& cloud) {
  for (unsigned int i = 0; i < cloud.size(); i++) {
    if (!is_valid_point(cloud[i])) {
      return false;
    }
  }
  return true;
}

LidarCalibration::LidarCalibration(const ros::NodeHandle& nh) {
  nh_ = nh;
  calibration_running_ = false;
  received_half_scans_ = 0;
}

void LidarCalibration::set_options(CalibrationOptions options) {
  options_ = options;
}

bool LidarCalibration::start_calibration(std::string cloud_topic) {
  received_half_scans_ = 0;
  cloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(cloud_topic, 1000, &LidarCalibration::cloud_cb, this);
}

void LidarCalibration::cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_ptr) {
  received_half_scans_++;
  ROS_INFO_STREAM("Received scan " << received_half_scans_ << "/3.");
  if (calibration_running_) {
    return;
  }
  // ditch first half-scan since it could be incomplete
  if (received_half_scans_ == 2) { // get second half-scan
    pcl::fromROSMsg(*cloud_ptr, cloud1_);
    ROS_INFO_STREAM("Copied first cloud. Size:" << cloud1_.size());
    return;
  } else if (received_half_scans_ == 3) { // get third half-scan
    calibration_running_ = true;
    pcl::fromROSMsg(*cloud_ptr, cloud2_);
    ROS_INFO_STREAM("Copied second cloud. Size:" << cloud2_.size());
    if (cloud1_.size() <= cloud2_.size()) {
      calibrate(cloud1_, cloud2_, options_.init_calibration);
    } else {
      calibrate(cloud2_, cloud1_, options_.init_calibration);
    }

    calibration_running_ = false;
    return;
  }
  ROS_INFO_STREAM("Skipping cloud");
}


void LidarCalibration::calibrate(pcl::PointCloud<pcl::PointXYZ>& cloud1,
                                 pcl::PointCloud<pcl::PointXYZ>& cloud2,
                                 const Calibration& init_calibration) {
  ROS_INFO_STREAM("Starting calibration.  Sizes: " << cloud1.size() << ", " << cloud2.size());

  if(!is_valid_cloud(cloud1)) {
    ROS_WARN("Cloud 1 has invalid points");
  }
  if(!is_valid_cloud(cloud2)) {
    ROS_WARN("Cloud 2 has invalid points");
  }

  std::vector<Calibration> calibrations;
  calibrations.push_back(init_calibration);

  unsigned int iteration_counter = 0;
  do {
    ROS_INFO_STREAM("[LidarCalibration] Starting iteration " << (iteration_counter+1));
    applyCalibration(cloud1, cloud2, calibrations[calibrations.size()-1]);
    if(!is_valid_cloud(cloud1)) {
      ROS_WARN("Cloud 1 has invalid points");
    }
    if(!is_valid_cloud(cloud2)) {
      ROS_WARN("Cloud 2 has invalid points");
    }
    pcl::PointCloud<pcl::PointNormal> normals = computeNormals(cloud1);
    pcl::PointCloud<pcl::PointXYZ> cloud1_reduced;
    pcl::copyPointCloud(normals, cloud1_reduced);
    if(!is_valid_cloud(cloud1_reduced)) {
      ROS_WARN("cloud1_reduced has invalid points");
    }
    std::vector<int> neighbor_mapping = findNeighbors(cloud1_reduced, cloud2);

    calibrations.push_back(optimize_calibration(cloud1_reduced, cloud2, normals, neighbor_mapping));
    iteration_counter++;
  } while(iteration_counter < options_.max_iterations
          && (iteration_counter < 2 || !check_convergence(calibrations[calibrations.size()-2], calibrations[calibrations.size()-1])));
}

void LidarCalibration::applyCalibration(pcl::PointCloud<pcl::PointXYZ>& cloud1,
                                        pcl::PointCloud<pcl::PointXYZ>& cloud2,
                                        const Calibration& calibration) const {
  ROS_INFO_STREAM("Applying new calibration. Sizes: " << cloud1.size() << ", " << cloud2.size());
  Eigen::Affine3d transform(Eigen::AngleAxisd(calibration.yaw, Eigen::Vector3d::UnitZ())
      * Eigen::AngleAxisd(calibration.pitch, Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(calibration.roll, Eigen::Vector3d::UnitX()));
  transform.translation() = Eigen::Vector3d(calibration.x, calibration.y, 0);
  pcl::transformPointCloud(cloud1, cloud1, transform); //Note: Can be used with cloud_in equal to cloud_out
  pcl::transformPointCloud(cloud2, cloud2, transform);
}

// TODO Voxelgrid
pcl::PointCloud<pcl::PointNormal>
LidarCalibration::computeNormals(const pcl::PointCloud<pcl::PointXYZ>& cloud) const{
  ROS_INFO_STREAM("Compute normals. Point cloud size: " << cloud.size());
//  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
  mls.setComputeNormals(true);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::copyPointCloud(cloud, *cloud_ptr);
  mls.setInputCloud(cloud_ptr);
  mls.setPolynomialOrder(1);
//  mls.setPolynomialFit(true);
//  mls.setSearchMethod(tree);
  mls.setSearchRadius(0.05);

  pcl::PointCloud<pcl::PointNormal> normals;
  mls.process(normals);
  ROS_INFO_STREAM("Reduced point cloud size: " << normals.size());
  return normals;
}

std::vector<int>
LidarCalibration::findNeighbors(const pcl::PointCloud<pcl::PointXYZ>& cloud1,
                                const pcl::PointCloud<pcl::PointXYZ>& cloud2) const {
  ROS_INFO_STREAM("Compute neighbor mapping. Sizes: " << cloud1.size() << ", " << cloud2.size());
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;  // Maybe optimize by using same tree as in normal estimation?
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2_ptr(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::copyPointCloud(cloud2, *cloud2_ptr);
  kdtree.setInputCloud(cloud2_ptr); // Search in second cloud to retrieve mapping from cloud1 -> cloud2

  std::vector<int> index(1);
  std::vector<float> sqrt_dist(1);

  std::vector<int> mapping(cloud1.size(), -1);
  for (unsigned int i = 0; i < cloud1.size(); i++) {
    if (kdtree.nearestKSearch(cloud1[i], 1, index, sqrt_dist) > 0) {
      mapping[i] = index[0];
    } else {
      mapping[i] = -1; // No neighbor found
    }
  }

  return mapping;
}

LidarCalibration::Calibration
LidarCalibration::optimize_calibration(const pcl::PointCloud<pcl::PointXYZ> &cloud1,
                                       const pcl::PointCloud<pcl::PointXYZ> &cloud2,
                                       const pcl::PointCloud<pcl::PointNormal>& normals,
                                       const std::vector<int> neighbor_mapping) const {
  ROS_INFO_STREAM("Solving Non-Linear Least Squares.");
  ROS_INFO_STREAM("Sizes:" << cloud1.size() << ", " << cloud2.size() << ", " << normals.size());

  if (cloud1.size() != normals.size()) {
    ROS_ERROR_STREAM("Size of cloud1 (" << cloud1.size() << ") doesn't match size of normals (" << normals.size() << ").");
    return Calibration();
  }

  ceres::Problem problem;

  double rotation[3] = {0, 0, 0};
  double translation[2] = {0, 0};

  unsigned int residual_count = 0;
  for(unsigned int i = 0; i < cloud1.size(); i++) {
    unsigned int x2_index = neighbor_mapping[i];
    if (x2_index < 0 || x2_index >= cloud2.size()) {
      continue; // Invalid index
    }
    pcl::PointXYZ pcl_x1 = cloud1[i];
    pcl::PointXYZ pcl_x2 = cloud2[x2_index];
    pcl::PointNormal pcl_normal = normals[i];
    ceres::CostFunction* cost_function = PointPlaneError::Create(pcl_x1, pcl_x2, pcl_normal);
    problem.AddResidualBlock(cost_function, NULL, rotation, translation);
    residual_count++;
  }
  ROS_INFO_STREAM("Number of residuals: " << residual_count);

  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.BriefReport() << "\n";

  Calibration calibration;
  calibration.x = translation[0];
  calibration.y = translation[1];
  calibration.roll = rotation[0];
  calibration.pitch = rotation[1];
  calibration.yaw = rotation[2];


  ROS_INFO_STREAM("[LidarCalibration] Optimization result: " << calibration.to_string());
  return calibration;
}


bool LidarCalibration::check_convergence(const LidarCalibration::Calibration& prev_calibration,
                                         const LidarCalibration::Calibration& current_calibration) const {
  ROS_INFO_STREAM("Checking convergence.");
  return false;
}

}
