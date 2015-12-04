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
    } else {
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

template <class Iter, class Incr>
  void safe_advance(Iter& curr, const Iter& end, Incr n)
  {
    size_t remaining(std::distance(curr, end));
    if (remaining < n)
    {
      n = remaining;
    }
    std::advance(curr, n);
  }

void LidarCalibration::publish_neighbors(const pcl::PointCloud<pcl::PointXYZ>& cloud1,
                       const pcl::PointCloud<pcl::PointXYZ>& cloud2,
                       const std::map<unsigned int, unsigned int> &mapping) const
{
  visualization_msgs::MarkerArray marker_array;
  unsigned int number_of_markers = 100;
  unsigned int step = floor(mapping.size() / number_of_markers);
  unsigned int id_cnt = 0;
  for (std::map<unsigned int, unsigned int>::const_iterator it = mapping.begin();
       it != mapping.end();
       safe_advance<std::map<unsigned int, unsigned int>::const_iterator, unsigned int>(it, mapping.end(), step)) {
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

    point1.x = (double) cloud1[it->first].x;
    point1.y = (double) cloud1[it->first].y;
    point1.z = (double) cloud1[it->first].z;

    point2.x = (double) cloud2[it->second].x;
    point2.y = (double) cloud2[it->second].y;
    point2.z = (double) cloud2[it->second].z;
    marker.points.push_back(point1);
    marker.points.push_back(point2);
    marker_array.markers.push_back(marker);
  }
  neighbor_pub_.publish(marker_array);
}

std::vector<LaserPoint<double> >
LidarCalibration::crop_cloud(const std::vector<LaserPoint<double> > &scan, double range) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  *cloud_ptr = laserToActuatorCloud(scan, options_.init_calibration);

  pcl::CropBox<pcl::PointXYZ> crop_box_filter;
  crop_box_filter.setInputCloud(cloud_ptr);
  crop_box_filter.setNegative(true);
  Eigen::Vector4f box_v(1, 1, 1, 1);

  Eigen::Vector4f min_v = -range*box_v;
  min_v[3] = 1;
  Eigen::Vector4f max_v = range*box_v;
  min_v[3] = 1;

  crop_box_filter.setMin(min_v);
  crop_box_filter.setMax(max_v);

  pcl::PointCloud<pcl::PointXYZ> cloud_cropped;
  crop_box_filter.filter(cloud_cropped);

  pcl::PointIndices pcl_indices;
  crop_box_filter.getRemovedIndices(pcl_indices);
  std::vector<int> indices = pcl_indices.indices;
  std::sort(indices.begin(), indices.end());

  std::vector<int>::iterator to_delete_it = indices.begin();
  std::vector<LaserPoint<double> > scan_cropped;
  for (unsigned int i = 0; i < scan.size(); i++) {
    if (to_delete_it == indices.end()) { // no more indices
      scan_cropped.push_back(scan[i]);
    } else {
      if (*to_delete_it != i) { // current index not in to_delete
        scan_cropped.push_back(scan[i]);
      } else { // current index in to_delete
        to_delete_it++;
      }
    }
  }

  return scan_cropped;
}


LidarCalibration::LidarCalibration(const ros::NodeHandle& nh) :
  manual_mode_(false)
{
  nh_ = nh;

  mls_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("mls_cloud", 1000);
  results_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("results_cloud", 1000);
  neighbor_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("neighbor_mapping", 1000);

  apply_calibration_client_ = nh_.serviceClient<lidar_calibration::ApplyCalibration>("head_lidar/apply_calibration");
  reset_clouds_client_ = nh_.serviceClient<std_srvs::Empty>("head_lidar/reset_clouds");
}

void LidarCalibration::setOptions(CalibrationOptions options) {
  options_ = options;
}

void LidarCalibration::setManualMode(bool manual) {
  manual_mode_ = manual;
}

void LidarCalibration::calibrate() {
  ROS_INFO_STREAM("Starting calibration.");
  ROS_INFO_STREAM("Waiting for reset service...");
  reset_clouds_client_.waitForExistence();
  std_srvs::Empty empty_srv;
  reset_clouds_client_.call(empty_srv);

  // TODO request laser points with angles from cloud agg
  std::vector<LaserPoint<double> > scan1;
  std::vector<LaserPoint<double> > scan2;

  scan1 = crop_cloud(scan1, 1);
  scan2 = crop_cloud(scan2, 1);

  pcl::PointCloud<pcl::PointXYZ> cloud1;
  pcl::PointCloud<pcl::PointXYZ> cloud2;

  Calibration previous_calibration = options_.init_calibration;
  Calibration current_calibration = options_.init_calibration;

  unsigned int iteration_counter = 0;
  do {
    ROS_INFO_STREAM("[LidarCalibration] Starting iteration " << (iteration_counter+1));
    // Transform laser points to actuator frame using current calibration
    applyCalibration(scan1, scan2, cloud1, cloud2, current_calibration);
    publish_cloud(cloud1, results_pub_);
    publish_cloud(cloud2, results_pub_);

    // Compute normals (TODO with weight)
    std::vector<WeightedNormal> normals = computeNormals(cloud1);

    // Find neighbors
    std::map<unsigned int, unsigned int> neighbor_mapping = findNeighbors(cloud1, cloud2);
    publish_neighbors(cloud1, cloud2, neighbor_mapping);

    previous_calibration = current_calibration;
    current_calibration = optimize_calibration(scan1, scan2, current_calibration, normals, neighbor_mapping);
    iteration_counter++;
    if (manual_mode_ && ros::ok()) {
      ROS_INFO_STREAM("Press [ENTER] to proceed with next iteration.");
      std::cin.get();
    }
  } while(ros::ok() && iteration_counter < options_.max_iterations
          && (iteration_counter < 2 || !check_convergence(previous_calibration, current_calibration)));
}

pcl::PointCloud<pcl::PointXYZ>
LidarCalibration::laserToActuatorCloud(const std::vector<LaserPoint<double> >& laserpoints, const Calibration& calibration) const {
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  Eigen::Affine3d calibration_transform = calibration.getTransform();
  for (unsigned int i = 0; i < laserpoints.size(); i++) {
    Eigen::Vector3d x = laserpoints[i].getInActuatorFrame(calibration_transform);
    pcl::PointXYZ pcl_x;
    pcl_x.x = x.x();
    pcl_x.y = x.y();
    pcl_x.z = x.z();
    pcl_cloud.push_back(pcl_x);
  }
  return pcl_cloud;
}

void LidarCalibration::applyCalibration(const std::vector<LaserPoint<double> > &scan1,
                                        const std::vector<LaserPoint<double> > &scan2,
                                        pcl::PointCloud<pcl::PointXYZ>& cloud1,
                                        pcl::PointCloud<pcl::PointXYZ>& cloud2,
                                        const Calibration& calibration) {
  ROS_INFO_STREAM("Applying new calibration.");
  // TODO iterate over scans and transform them to actuator frame using current calibration
  cloud1 = laserToActuatorCloud(scan1, calibration);
  cloud2 = laserToActuatorCloud(scan2, calibration);
}

std::vector<WeightedNormal>
LidarCalibration::computeNormals(const pcl::PointCloud<pcl::PointXYZ>& cloud) const{
  pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> ne;

  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::copyPointCloud(cloud, *cloud_ptr);
  kdtree.setInputCloud(cloud_ptr);

  std::vector<WeightedNormal> normals;
  for (unsigned int i = 0; i < cloud.size(); i++) {
    pcl::PointXYZ p = cloud[i];

    // Compute neighbors
    std::vector<int> indices;
    std::vector<float> sqrt_dist;
    kdtree.radiusSearch(p, 0.03, indices, sqrt_dist);


    float nx, ny, nz, curvature;
    ne.computePointNormal(cloud, indices, nx, ny, nz, curvature);
    WeightedNormal normal;
    normal.normal = Eigen::Vector3d(nx, ny, nz);
    // TODO normal weight
    normals.push_back(normal);
  }

  return normals;

//  ROS_INFO_STREAM("Compute normals. Point cloud size: " << cloud.size());
//  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
//  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
//  mls.setComputeNormals(true);

//  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());
//  pcl::copyPointCloud(cloud, *cloud_ptr);
//  mls.setInputCloud(cloud_ptr);
// // mls.setPolynomialOrder(1);
//  mls.setPolynomialFit(true);
//  mls.setSearchMethod(tree);
//  mls.setSearchRadius(0.05);

//  pcl::PointCloud<pcl::PointNormal> normals;
//  mls.process(normals);
//  ROS_INFO_STREAM("Reduced point cloud size: " << normals.size());
//  pcl::PointCloud<pcl::PointNormal> cloud_cleaned = remove_invalid_points<pcl::PointNormal>(normals);
//  return cloud_cleaned;
}

std::map<unsigned int, unsigned int>
LidarCalibration::findNeighbors(const pcl::PointCloud<pcl::PointXYZ>& cloud1,
                                const pcl::PointCloud<pcl::PointXYZ>& cloud2) const {
  ROS_INFO_STREAM("Computing neighbor mapping. Sizes: " << cloud1.size() << ", " << cloud2.size());
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2_ptr(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::copyPointCloud(cloud2, *cloud2_ptr);
  kdtree.setInputCloud(cloud2_ptr); // Search in second cloud to retrieve mapping from cloud1 -> cloud2

  std::vector<int> index(1);
  std::vector<float> sqrt_dist(1);

  std::map<unsigned int, unsigned int> mapping;
  for (unsigned int i = 0; i < cloud1.size(); i++) {
    if (kdtree.nearestKSearch(cloud1[i], 1, index, sqrt_dist) > 0) { // Check if number of found neighbours > 0
      if (sqrt_dist[0] <= options_.max_sqrt_neighbor_dist) { // Only insert if smaller than max distance
        std::pair<unsigned int, unsigned int> pair(i, index[0]);
        mapping.insert(pair); // Mapping from cloud1 index to cloud2 index
      }
    }
  }

  return mapping;
}

LidarCalibration::Calibration
LidarCalibration::optimize_calibration(const std::vector<LaserPoint<double> >& scan1,
                                       const std::vector<LaserPoint<double> >& scan2,
                                       const Calibration& current_calibration,
                                       const std::vector<WeightedNormal> &normals,
                                       const std::map<unsigned int, unsigned int> neighbor_mapping) const {
  ROS_INFO_STREAM("Solving Non-Linear Least Squares.");

  if (scan1.size() != normals.size()) {
    ROS_ERROR_STREAM("Size of scan1 (" << scan1.size() << ") doesn't match size of normals (" << normals.size() << ").");
    return Calibration();
  }

  ceres::Problem problem;

  double rotation[3] = {current_calibration.roll, current_calibration.pitch, current_calibration.yaw};
  double translation[2] = {current_calibration.y, current_calibration.z};

  unsigned int residual_count = 0;
  for(std::map<unsigned int, unsigned int>::const_iterator it = neighbor_mapping.begin(); it != neighbor_mapping.end(); it++) {
    unsigned int s1_index = it->first;
    unsigned int s2_index = it->second;

    ceres::CostFunction* cost_function = PointPlaneError::Create(scan1[s1_index], scan2[s2_index], normals[s1_index]);
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
