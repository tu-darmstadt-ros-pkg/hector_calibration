#include <lidar_calibration/lidar_calibration.h>

namespace hector_calibration {

template<typename T>
bool isValidPoint(const T& point) {
  for (unsigned int i = 0; i < 3; i++) {
    if (std::isnan(point.data[i]) || std::isinf(point.data[i])) {
      return false;
    }
  }
  return true;
}

bool isValidCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud) {
  for (unsigned int i = 0; i < cloud.size(); i++) {
    if (!isValidPoint(cloud[i])) {
      return false;
    }
  }
  return true;
}
template<typename T>
pcl::PointCloud<T> removeInvalidPoints(pcl::PointCloud<T>& cloud) {
  pcl::PointCloud<T> cleaned_cloud;
  unsigned int invalid_counter = 0;
  for (unsigned int i = 0; i < cloud.size(); i++) {
    if (isValidPoint<T>(cloud[i])) {
      cleaned_cloud.push_back(cloud[i]);
    } else {
      invalid_counter++;
    }
  }
  ROS_INFO_STREAM("Removed " << invalid_counter << " invalid points");
  return cleaned_cloud;
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

void LidarCalibration::publishCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud, const ros::Publisher& pub) {
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  cloud_msg.header.frame_id = actuator_frame_;
  cloud_msg.header.stamp = ros::Time::now();
  pub.publish(cloud_msg);
}

void LidarCalibration::publishCloud(sensor_msgs::PointCloud2& cloud, const ros::Publisher& pub) {
  cloud.header.frame_id = actuator_frame_;
  cloud.header.stamp = ros::Time::now();
  pub.publish(cloud);
}

void LidarCalibration::publishNeighbors(const pcl::PointCloud<pcl::PointXYZ>& cloud1,
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
    marker.header.frame_id = actuator_frame_;
    marker.header.stamp = ros::Time();
    marker.ns = "neighbor_mapping";
    marker.id = id_cnt++;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;

    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    marker.color.a = 1.0;
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

void LidarCalibration::enableNormalVisualization(bool normals) {
  vis_normals_ = normals;
}

std::vector<LaserPoint<double> >
LidarCalibration::cropCloud(const std::vector<LaserPoint<double> > &scan, double range) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  *cloud_ptr = laserToActuatorCloud(scan, options_.init_calibration);

  pcl::CropBox<pcl::PointXYZ> crop_box_filter(true);
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

//  pcl::PointIndices pcl_indices = crop_box_filter.getRemovedIndices();
  std::vector<int> indices = *crop_box_filter.getRemovedIndices();
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
  ROS_INFO_STREAM("Cropped " << indices.size() << " points from scan");

  return scan_cropped;
}


LidarCalibration::LidarCalibration(const ros::NodeHandle& nh) :
  manual_mode_(false),
  vis_normals_(false),
  nh_(nh)
{
  cloud1_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("result_cloud1", 1000);
  cloud2_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("result_cloud2", 1000);
  neighbor_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("neighbor_mapping", 1000);

  request_scans_client_ = nh_.serviceClient<lidar_calibration::RequestScans>("request_scans");
  reset_clouds_client_ = nh_.serviceClient<std_srvs::Empty>("reset_clouds");

  ros::NodeHandle pnh("~");
  pnh.param<std::string>("actuator_frame", actuator_frame_, "lidar_actuator_frame");
}

bool LidarCalibration::loadOptionsFromParamServer() {
  int max_iterations;
  ros::NodeHandle pnh("~");
  pnh.param<int>("max_iterations", max_iterations, 20);
  max_iterations = options_.max_iterations;
  pnh.param<double>("max_sqrt_neighbor_dist", options_.max_sqrt_neighbor_dist, 0.1);
  pnh.param<double>("sqrt_convergence_diff_thres", options_.sqrt_convergence_diff_thres, 1e-6);
}

void LidarCalibration::setOptions(CalibrationOptions options) {
  options_ = options;
}

void LidarCalibration::setManualMode(bool manual) {
  manual_mode_ = manual;
}


void LidarCalibration::setPeriodicPublishing(bool status, double period) {
  if (status) {
//    ROS_INFO_STREAM("[LidarCalibration] Enabled periodic cloud publishing.");
    timer_ = nh_.createTimer(ros::Duration(period), &LidarCalibration::timerCallback, this, false);
  } else {
//    ROS_INFO_STREAM("[LidarCalibration] Disabled periodic cloud publishing.");
    timer_.stop();
  }

}


void LidarCalibration::timerCallback(const ros::TimerEvent&) {
  publishResults();
}

void LidarCalibration::publishResults() {
  publishCloud(cloud1_msg_, cloud1_pub_);
  publishCloud(cloud2_msg_, cloud2_pub_);
}

std::vector<LaserPoint<double> >
LidarCalibration::msgToLaserPoints(const sensor_msgs::PointCloud2& scan,
                                   const std_msgs::Float64MultiArray& angles)
{
  pcl::PointCloud<pcl::PointXYZ> pcl_scan;
  pcl::fromROSMsg(scan, pcl_scan);
  std::vector<LaserPoint<double> > laser_points;
  for (unsigned int i = 0; i < pcl_scan.size(); i++) {
    LaserPoint<double> lp;
    lp.point = Eigen::Vector3d(pcl_scan[i].x, pcl_scan[i].y, pcl_scan[i].z);
    lp.angle = angles.data[i];
    laser_points.push_back(lp);
  }
  return laser_points;
}

void LidarCalibration::requestScans(std::vector<LaserPoint<double> >& scan1,
                  std::vector<LaserPoint<double> >& scan2) {
  lidar_calibration::RequestScansRequest request;
  lidar_calibration::RequestScansResponse response;
  request_scans_client_.waitForExistence();
  request_scans_client_.call(request, response);
  scan1 = msgToLaserPoints(response.scan_1, response.angles1);
  scan2 = msgToLaserPoints(response.scan_2, response.angles2);
  if (scan2.size() < scan1.size()) { // Switch scan1 and scan2
    std::vector<LaserPoint<double> >tmp = scan1;
    scan1 = scan2;
    scan2 = tmp;
  }
}

void LidarCalibration::calibrate() {
  reset_clouds_client_.waitForExistence();
  std_srvs::Empty empty_srv;
//  reset_clouds_client_.call(empty_srv);

  std::vector<LaserPoint<double> > scan1;
  std::vector<LaserPoint<double> > scan2;
  requestScans(scan1, scan2);
  ROS_INFO_STREAM("Received point clouds of sizes " << scan1.size() << " and " << scan2.size() << ".");

  scan1 = cropCloud(scan1, 1);
  scan2 = cropCloud(scan2, 1);

  pcl::PointCloud<pcl::PointXYZ> cloud1;
  pcl::PointCloud<pcl::PointXYZ> cloud2;

  Calibration previous_calibration = options_.init_calibration;
  Calibration current_calibration = options_.init_calibration;

  unsigned int iteration_counter = 0;
  do {
    ROS_INFO_STREAM("-------------- Starting iteration " << (iteration_counter+1) << "--------------");
    // Transform laser points to actuator frame using current calibration
    applyCalibration(scan1, scan2, cloud1, cloud2, current_calibration);
    pcl::toROSMsg(cloud1, cloud1_msg_);
    pcl::toROSMsg(cloud2, cloud2_msg_);
    publishCloud(cloud1_msg_, cloud1_pub_);
    publishCloud(cloud2_msg_, cloud2_pub_);

    // Compute normals (TODO with weight)
    std::vector<WeightedNormal> normals = computeNormals(cloud1);

    // Find neighbors
    std::map<unsigned int, unsigned int> neighbor_mapping = findNeighbors(cloud1, cloud2);
    publishNeighbors(cloud1, cloud2, neighbor_mapping);

    previous_calibration = current_calibration;
    current_calibration = optimizeCalibration(scan1, scan2, current_calibration, normals, neighbor_mapping);
    iteration_counter++;
    if (manual_mode_ && ros::ok()) {
      ROS_INFO_STREAM("Press [ENTER] to proceed with next iteration.");
      std::cin.get();
    }
  } while(ros::ok() && !maxIterationsReached(iteration_counter)
          && (iteration_counter < 2 || !checkConvergence(previous_calibration, current_calibration)));

  ROS_INFO_STREAM("Result: " << current_calibration.toString());
}

pcl::PointCloud<pcl::PointXYZ>
LidarCalibration::laserToActuatorCloud(const std::vector<LaserPoint<double> >& laserpoints, const Calibration& calibration) const {
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  Eigen::Affine3d calibration_transform = calibration.getTransform();
  for (unsigned int i = 0; i < laserpoints.size(); i++) {
    Eigen::Vector3d actuator_point = laserpoints[i].getInActuatorFrame(calibration_transform);
    pcl::PointXYZ pcl_x;
    pcl_x.x = actuator_point.x();
    pcl_x.y = actuator_point.y();
    pcl_x.z = actuator_point.z();
    pcl_cloud.push_back(pcl_x);
  }
  return pcl_cloud;
}

void LidarCalibration::applyCalibration(const std::vector<LaserPoint<double> > &scan1,
                                        const std::vector<LaserPoint<double> > &scan2,
                                        pcl::PointCloud<pcl::PointXYZ>& cloud1,
                                        pcl::PointCloud<pcl::PointXYZ>& cloud2,
                                        const Calibration& calibration)
{
//  ROS_INFO_STREAM("Applying new calibration.");
  // TODO iterate over scans and transform them to actuator frame using current calibration
  cloud1 = laserToActuatorCloud(scan1, calibration);
  cloud2 = laserToActuatorCloud(scan2, calibration);
}

void fixNanInf(WeightedNormal& normal) {
  if (std::isnan(normal.normal.x()) || std::isnan(normal.normal.y()) || std::isnan(normal.normal.z())
      || std::isinf(normal.normal.x()) || std::isinf(normal.normal.y()) || std::isinf(normal.normal.z())) {
    normal.normal = Eigen::Vector3d::Zero();
    normal.weight = 0;
  }
}

std::vector<WeightedNormal>
LidarCalibration::computeNormals(const pcl::PointCloud<pcl::PointXYZ>& cloud) const{
//  ROS_INFO_STREAM("Computing normals.");
  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::PointNormal> ne;

  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::copyPointCloud(cloud, *cloud_ptr);
  kdtree.setInputCloud(cloud_ptr);

  std::vector<WeightedNormal> normals;
  pcl::PointCloud<pcl::Normal> pcl_normals;
  for (unsigned int i = 0; i < cloud.size(); i++) {
    pcl::PointXYZ p = cloud[i];

    // Compute neighbors
    std::vector<int> indices;
    std::vector<float> sqrt_dist;
    kdtree.radiusSearch(p, 0.07, indices, sqrt_dist);

    float nx, ny, nz, curvature;
    ne.computePointNormal(cloud, indices, nx, ny, nz, curvature);
    WeightedNormal normal;
    normal.normal = Eigen::Vector3d(nx, ny, nz);
    if (vis_normals_) {
      pcl::Normal pcl_normal(nx, ny, nz);
      pcl_normals.push_back(pcl_normal);
    }
    // TODO normal weight
    fixNanInf(normal);
    normals.push_back(normal);
  }

  if (vis_normals_) {
    visualizeNormals(cloud, pcl_normals);
  }

  return normals;
}

void LidarCalibration::visualizeNormals(const pcl::PointCloud<pcl::PointXYZ>& cloud,
                                        const pcl::PointCloud<pcl::Normal>& normals) const
{
  pcl::visualization::PCLVisualizer viewer;
  viewer.setBackgroundColor(0,0,0);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::copyPointCloud(cloud, *cloud_ptr);
  viewer.addPointCloud<pcl::PointXYZ>(cloud_ptr, "Cloud");

  pcl::PointCloud<pcl::Normal>::Ptr normals_ptr(new pcl::PointCloud<pcl::Normal>());
  pcl::copyPointCloud(normals, *normals_ptr);
  viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud_ptr, normals_ptr, 10, 0.05, "Normals");
  viewer.addCoordinateSystem(1.0);
  viewer.initCameraParameters();
  ros::Rate rate(10);
  while (!viewer.wasStopped())
   {
     viewer.spinOnce (100);
     ros::spinOnce();
     rate.sleep();
   }
}

std::map<unsigned int, unsigned int>
LidarCalibration::findNeighbors(const pcl::PointCloud<pcl::PointXYZ>& cloud1,
                                const pcl::PointCloud<pcl::PointXYZ>& cloud2) const {
//  ROS_INFO_STREAM("Computing neighbor mapping.");
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
  ROS_INFO_STREAM("Found " << mapping.size() << " neighbor matches.");
  return mapping;
}

LidarCalibration::Calibration
LidarCalibration::optimizeCalibration(const std::vector<LaserPoint<double> >& scan1,
                                      const std::vector<LaserPoint<double> >& scan2,
                                      const Calibration& current_calibration,
                                      const std::vector<WeightedNormal> &normals,
                                      const std::map<unsigned int, unsigned int> neighbor_mapping) const
{
//  ROS_INFO_STREAM("Solving Non-Linear Least Squares.");
  if (scan1.size() != normals.size()) {
    ROS_ERROR_STREAM("Size of scan1 (" << scan1.size() << ") doesn't match size of normals (" << normals.size() << ").");
    return Calibration();
  }

  ceres::Problem problem;

  double rotation[2] = {current_calibration.pitch, current_calibration.yaw};
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
  //options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.BriefReport() << "\n";

  Calibration calibration;
  calibration.y = translation[0];
  calibration.z = translation[1];
  calibration.pitch = rotation[0];
  calibration.yaw = rotation[1];


  ROS_INFO_STREAM("Optimization result: " << calibration.toString());
  return calibration;
}


bool LidarCalibration::checkConvergence(const LidarCalibration::Calibration& prev_calibration,
                                         const LidarCalibration::Calibration& current_calibration) const {
//  ROS_INFO_STREAM("Checking convergence.");
  double cum_sqrt_diff = 0;
  for (unsigned int i = 0; i < Calibration::NUM_FREE_PARAMS; i++) {
    cum_sqrt_diff += std::pow(current_calibration(i) - prev_calibration(i), 2);
  }
  ROS_INFO_STREAM("Squared change in parameters: " << cum_sqrt_diff);
  if (cum_sqrt_diff < options_.sqrt_convergence_diff_thres) {
    ROS_INFO_STREAM("-------------- CONVERGENCE --------------");
    return true;
  } else {
    return false;
  }
}

bool LidarCalibration::maxIterationsReached(unsigned int current_iterations) const {
  if (current_iterations < options_.max_iterations) {
    return false;
  }  else {
    ROS_INFO_STREAM("-------- MAX ITERATIONS REACHED ---------");
    return true;
  }
}

}
