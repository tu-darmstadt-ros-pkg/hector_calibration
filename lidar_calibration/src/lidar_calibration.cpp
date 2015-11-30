#include <lidar_calibration/lidar_calibration.h>

namespace hector_calibration {

template<typename T>
bool is_valid_point(const T& point) {
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
template<typename T>
pcl::PointCloud<T> remove_invalid_points(pcl::PointCloud<T>& cloud) {
  pcl::PointCloud<T> cleaned_cloud;
  unsigned int invalid_counter = 0;
  for (unsigned int i = 0; i < cloud.size(); i++) {
    if (is_valid_point<T>(cloud[i])) {
      cleaned_cloud.push_back(cloud[i]);
      invalid_counter++;
    }
  }
  ROS_INFO_STREAM("Removed " << invalid_counter << " invalid points");
  return cleaned_cloud;
}

void publish_cloud(const pcl::PointCloud<pcl::PointXYZ>& cloud, const ros::Publisher& pub) {
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  cloud_msg.header.frame_id = "head_lidar_actuator_frame";
  pub.publish(cloud_msg);
}

void LidarCalibration::publish_neighbors(const pcl::PointCloud<pcl::PointXYZ>& cloud1,
                       const pcl::PointCloud<pcl::PointXYZ>& cloud2,
                       const std::vector<int>& mapping) const
{
  visualization_msgs::MarkerArray marker_array;
  unsigned int number_of_markers = 100;
  unsigned int step = floor(mapping.size() / number_of_markers);
  unsigned int id_cnt = 0;
  for (unsigned int i = 0; i < mapping.size(); i+=step) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "head_lidar_actuator_frame";
    marker.header.stamp = ros::Time();
    marker.ns = "neighbor_mapping";
    marker.id = id_cnt++;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;

    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    geometry_msgs::Point point1;
    geometry_msgs::Point point2;

    point1.x = (double) cloud1[i].x;
    point1.y = (double) cloud1[i].y;
    point1.z = (double) cloud1[i].z;

    point2.x = (double) cloud2[mapping[i]].x;
    point2.y = (double) cloud2[mapping[i]].y;
    point2.z = (double) cloud2[mapping[i]].z;
    marker.points.push_back(point1);
    marker.points.push_back(point2);
    marker_array.markers.push_back(marker);
  }
  neighbor_pub_.publish(marker_array);
}

template<typename PointT>
pcl::PointCloud<PointT> crop_cloud(const pcl::PointCloud<PointT>& cloud, double range) {
  typename pcl::PointCloud<PointT>::Ptr cloud_ptr(new pcl::PointCloud<PointT>);
  pcl::copyPointCloud(cloud, *cloud_ptr);

  pcl::CropBox<PointT> crop_box_filter;
  crop_box_filter.setInputCloud(cloud_ptr);
  crop_box_filter.setNegative(true);
  Eigen::Vector4f box_v(1, 1, 1, 1);

  Eigen::Vector4f min_v = -range*box_v;
  min_v[3] = 1;
  Eigen::Vector4f max_v = range*box_v;
  min_v[3] = 1;

  crop_box_filter.setMin(min_v);
  crop_box_filter.setMax(max_v);

  pcl::PointCloud<PointT> cloud_cropped;
  crop_box_filter.filter(cloud_cropped);
  return cloud_cropped;
}


LidarCalibration::LidarCalibration(const ros::NodeHandle& nh) {
  nh_ = nh;

  mls_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("mls_cloud", 1000);
  results_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("results_cloud", 1000);
  neighbor_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("neighbor_mapping", 1000);
}

void LidarCalibration::set_options(CalibrationOptions options) {
  options_ = options;
}

//void LidarCalibration::cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_ptr) {
//    cloud1_ = crop_cloud<pcl::PointXYZ>(cloud1_, 1);
//    cloud2_ = crop_cloud<pcl::PointXYZ>(cloud2_, 1);
//    if (cloud1_.size() <= cloud2_.size()) {
//      calibrate(cloud1_, cloud2_, options_.init_calibration);
//    } else {
//      calibrate(cloud2_, cloud1_, options_.init_calibration);
//    }    transformCloud(cloud_agg1_, half_scan1_);
//  ROS_INFO_STREAM("Skipping cloud");
//}

void LidarCalibration::calibrate() {
  ROS_INFO_STREAM("Starting calibration.");
  // TODO send reset to agg -- maybe as service to wait for completion?

  pcl::PointCloud<pcl::PointXYZ> cloud1;
  pcl::PointCloud<pcl::PointXYZ> cloud2;

  std::vector<Calibration> calibrations;
  calibrations.push_back(options_.init_calibration);

  unsigned int iteration_counter = 0;
  do {
    ROS_INFO_STREAM("[LidarCalibration] Starting iteration " << (iteration_counter+1));
    applyCalibration(cloud1, cloud2, calibrations[calibrations.size()-1]);
    publish_cloud(cloud1, results_pub_);
    publish_cloud(cloud2, results_pub_);
    if(!is_valid_cloud(cloud1)) {
      ROS_WARN("Cloud 1 has invalid points");
    }
    if(!is_valid_cloud(cloud2)) {
      ROS_WARN("Cloud 2 has invalid points");
    }
    pcl::PointCloud<pcl::PointNormal> normals = computeNormals(cloud1);
    pcl::PointCloud<pcl::PointXYZ> cloud1_reduced;
    pcl::copyPointCloud(normals, cloud1_reduced);
    publish_cloud(cloud1_reduced, mls_cloud_pub_);
    if(!is_valid_cloud(cloud1_reduced)) {
      ROS_WARN("cloud1_reduced has invalid points");
    }
    std::vector<int> neighbor_mapping = findNeighbors(cloud1_reduced, cloud2);
    publish_neighbors(cloud1_reduced, cloud2, neighbor_mapping);

    calibrations.push_back(optimize_calibration(cloud1_reduced, cloud2, normals, neighbor_mapping));
    iteration_counter++;
  } while(ros::ok() && iteration_counter < options_.max_iterations
          && (iteration_counter < 2 || !check_convergence(calibrations[calibrations.size()-2], calibrations[calibrations.size()-1])));
}

void LidarCalibration::applyCalibration(pcl::PointCloud<pcl::PointXYZ>& cloud1,
                                        pcl::PointCloud<pcl::PointXYZ>& cloud2,
                                        const Calibration& calibration) const {
  ROS_INFO_STREAM("Applying new calibration. Sizes: " << cloud1.size() << ", " << cloud2.size());
  // TODO request new clouds via service with new calibration data
}

// TODO Voxelgrid
pcl::PointCloud<pcl::PointNormal>
LidarCalibration::computeNormals(const pcl::PointCloud<pcl::PointXYZ>& cloud) const{
  ROS_INFO_STREAM("Compute normals. Point cloud size: " << cloud.size());
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
  mls.setComputeNormals(true);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::copyPointCloud(cloud, *cloud_ptr);
  mls.setInputCloud(cloud_ptr);
 // mls.setPolynomialOrder(1);
  mls.setPolynomialFit(true);
  mls.setSearchMethod(tree);
  mls.setSearchRadius(0.05);

  pcl::PointCloud<pcl::PointNormal> normals;
  mls.process(normals);
  ROS_INFO_STREAM("Reduced point cloud size: " << normals.size());
  pcl::PointCloud<pcl::PointNormal> cloud_cleaned = remove_invalid_points<pcl::PointNormal>(normals);
  return cloud_cleaned;
}

std::vector<int>
LidarCalibration::findNeighbors(const pcl::PointCloud<pcl::PointXYZ>& cloud1,
                                const pcl::PointCloud<pcl::PointXYZ>& cloud2) const {
  ROS_INFO_STREAM("Computing neighbor mapping. Sizes: " << cloud1.size() << ", " << cloud2.size());
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
  calibration.y = translation[0];
  calibration.z = translation[1];
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
