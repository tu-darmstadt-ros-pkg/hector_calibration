#include <lidar_calibration/lidar_calibration.h>

namespace hector_calibration {

LidarCalibration::LidarCalibration(const ros::NodeHandle& nh) {
  nh_ = nh;
  calibration_running_ = false;
  received_half_scans_ = 0;
}

void LidarCalibration::set_options(CalibrationOptions options) {
  //options_ = options;
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
    calibrate(cloud1_, cloud2_, options_.init_calibration);
    calibration_running_ = false;
    return;
  }
  ROS_INFO_STREAM("Skipping cloud");
}


void LidarCalibration::calibrate(pcl::PointCloud<pcl::PointXYZ>& cloud1,
                                 pcl::PointCloud<pcl::PointXYZ>& cloud2,
                                 const Calibration& init_calibration) {
  ROS_INFO_STREAM("Starting calibration.  Sizes: " << cloud1.size() << ", " << cloud2.size());

  std::vector<Calibration> calibrations;
  calibrations.push_back(init_calibration);

  unsigned int iteration_counter = 0;
  do {
    ROS_INFO_STREAM("[LidarCalibration] Starting iteration " << (iteration_counter+1));
    applyCalibration(cloud1, cloud2, calibrations[calibrations.size()-1]);
    pcl::PointCloud<pcl::PointNormal> normals = computeNormals(cloud1);
    pcl::PointCloud<pcl::PointXYZ> cloud1_reduced;
    pcl::copyPointCloud(normals, cloud1_reduced);
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

struct PointPlaneResidual {
    PointPlaneResidual(double x1[3], double x2[3], double normal[3]) {
    for (unsigned int i = 0; i < 3; i++) {
      x1_[i] = x1[i];
      x2_[i] = x2[i];
      normal_[i] = normal[i];
    }
  }

  template<typename T>
  bool operator()(const T* const rpy_rotation, const T* const translation, T* residuals) const{
    // residual = n' * (x1 - H* x2)

    // Translate x2
    T x2_t[3];
    for (unsigned int i = 0; i < 2; i++) {
      x2_t[i] = T(x2_[i]) + translation[i];
    }
    x2_t[2] = T(x2_[2]);

    // Rotate x2
    T rotation_matrix[9];
    ceres::EulerAnglesToRotationMatrix(rpy_rotation, 3, rotation_matrix);
    T quaternion[4];
    ceres::RotationMatrixToQuaternion(rotation_matrix, quaternion);
    T x2_r[3];
    ceres::QuaternionRotatePoint(quaternion, x2_t, x2_r);

    residuals[0] = T(0);
    for (unsigned int i = 0; i < 3; i++) {
      residuals[0] += T(normal_[i]) * (T(x1_[i]) - x2_r[i]);
    }
  }

  double x1_[3];
  double x2_[3];
  double normal_[3];
};

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
    double x1[3];
    double x2[3];
    double normal[3];
    for (unsigned int i = 0; i < 3; i++) {
      x1[i] = (double) pcl_x1.data[i];
      x2[i] = (double) pcl_x2.data[i];
      normal[i] = (double) pcl_normal.normal[i];
    }
    ceres::CostFunction* cost_function =
        new ceres::AutoDiffCostFunction<PointPlaneResidual, 1, 1, 1>(
          new PointPlaneResidual(x1, x2, normal));
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
