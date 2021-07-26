#include <lidar_calibration/lidar_calibration.h>

namespace hector_calibration {

namespace lidar_calibration {

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
  nh_(nh),
  save_calibration_(false),
  save_path_(""),
  rotation_offset_(Eigen::Affine3d::Identity())
{
  cloud1_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("result_cloud1", 10, true);
  cloud2_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("result_cloud2", 10, true);
  ground_plane_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("ground_plane", 10, true);
  neighbor_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("neighbor_mapping", 10, true);
  planarity_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("planarity", 10, true);

  request_scans_client_ = nh_.serviceClient<hector_calibration_msgs::RequestScans>("request_scans");
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
  pnh.param<double>("normals_radius", options_.normals_radius, 0.07);
  pnh.param<bool>("detect_ground_plane", options_.detect_ground_plane, false);
  pnh.param<bool>("detect_ceiling", options_.detect_ceiling, false);
  pnh.param<std::string>("ground_frame", ground_frame_, "");

  pnh.param<bool>("save_calibration", save_calibration_, false);
  pnh.param<std::string>("save_path", save_path_, "");

  double roll, pitch, yaw;
  pnh.param<double>("rotation_offset_roll", roll, 0);
  pnh.param<double>("rotation_offset_pitch", pitch, 0);
  pnh.param<double>("rotation_offset_yaw", yaw, 0);
  rotation_offset_ = Eigen::Affine3d(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
        * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));

  pnh.param<std::string>("o_spin_frame", o_spin_frame_, "");
  pnh.param<std::string>("o_laser_frame", o_laser_frame_, "");
  double tf_wait_duration;
  pnh.param("tf_wait_duration", tf_wait_duration, 5.0);
  tf_wait_duration_ = ros::Duration(tf_wait_duration);
  return true;
}

void LidarCalibration::setOptions(CalibrationOptions options) {
  options_ = options;
}

void LidarCalibration::setManualMode(bool manual) {
  manual_mode_ = manual;
}

void LidarCalibration::setPeriodicPublishing(bool status, double period) {
  if (status) {
    timer_ = nh_.createTimer(ros::Duration(period), &LidarCalibration::timerCallback, this, false);
  } else {
    timer_.stop();
  }
}


void LidarCalibration::timerCallback(const ros::TimerEvent&) {
  publishResults();
}

void LidarCalibration::publishResults() {
  publishCloud(cloud1_msg_, cloud1_pub_, actuator_frame_);
  publishCloud(cloud2_msg_, cloud2_pub_, actuator_frame_);
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
  hector_calibration_msgs::RequestScansRequest request;
  hector_calibration_msgs::RequestScansResponse response;
  request_scans_client_.waitForExistence();
  request_scans_client_.call(request, response);
  scan1 = msgToLaserPoints(response.scan_1, response.angles1);
  scan2 = msgToLaserPoints(response.scan_2, response.angles2);
  laser_frame_ = response.scan_1.header.frame_id;
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

  // get transforms
  if (o_laser_frame_ != "" && o_spin_frame_ != "") {
    laser_transform_ = getTransform(o_spin_frame_, o_laser_frame_);
  }
  if (ground_frame_ != "") {
    plane_transform_ = getTransform(ground_frame_, actuator_frame_);
  }

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

    // Publish current results
    pcl::toROSMsg(cloud1, cloud1_msg_);
    pcl::toROSMsg(cloud2, cloud2_msg_);
    publishResults();

    // Compute normals with weight
    std::vector<WeightedNormal> normals = computeNormals(cloud1, options_.normals_radius);
//    if (vis_normals_) {
//      visualizeNormals(cloud1, normals);
//    }

    // Find neighbors
    std::map<unsigned int, unsigned int> neighbor_mapping = findNeighbors(cloud1, cloud2, options_.max_sqrt_neighbor_dist);
    publishNeighbors(cloud1, cloud2, neighbor_mapping, neighbor_pub_, actuator_frame_);

    previous_calibration = current_calibration;
    current_calibration = optimizeCalibration(scan1, scan2, current_calibration, normals, neighbor_mapping);
    iteration_counter++;
    if (manual_mode_ && ros::ok()) {
      ROS_INFO_STREAM("Press [ENTER] to proceed with next iteration.");
      std::cin.get();
    }
  } while(ros::ok() && !maxIterationsReached(iteration_counter)
          &&  !checkConvergence(previous_calibration, current_calibration));

  if (options_.detect_ground_plane || options_.detect_ceiling) {

    double ground_roll;
    double ground_pitch;

    detectGroundPlane(cloud1, cloud2, ground_roll, ground_pitch);

    Eigen::Affine3d ground_roll_transform(Eigen::AngleAxisd(ground_roll, Eigen::Vector3d::UnitX()));
    current_calibration = current_calibration.applyTransform(ground_roll_transform);

    applyCalibration(scan1, scan2, cloud1, cloud2, current_calibration);
    pcl::toROSMsg(cloud1, cloud1_msg_);
    pcl::toROSMsg(cloud2, cloud2_msg_);
    publishResults();
  }

  ROS_INFO_STREAM("Result: " << current_calibration.applyRotationOffset(rotation_offset_).toString());
  if (save_calibration_ && save_path_ != "") {
    ROS_INFO_STREAM("Saving calibration to: " << save_path_);
    saveToDisk(save_path_, current_calibration.applyRotationOffset(rotation_offset_));
  }
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
  cloud1 = laserToActuatorCloud(scan1, calibration);
  cloud2 = laserToActuatorCloud(scan2, calibration);
}

Calibration
LidarCalibration::optimizeCalibration(const std::vector<LaserPoint<double> >& scan1,
                                      const std::vector<LaserPoint<double> >& scan2,
                                      const Calibration& current_calibration,
                                      const std::vector<WeightedNormal> &normals,
                                      const std::map<unsigned int, unsigned int> neighbor_mapping) const
{
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

  Calibration rotated_calibration = calibration.applyRotationOffset(rotation_offset_);
  rotated_calibration.threshold(1e-3);
  ROS_INFO_STREAM("Optimization original: " << calibration.toString());
  ROS_INFO_STREAM("Optimization   result: " << rotated_calibration.toString());
  return calibration;
}

bool LidarCalibration::detectGroundPlane(const pcl::PointCloud<pcl::PointXYZ> &cloud1,
                                         const pcl::PointCloud<pcl::PointXYZ> &cloud2,
                                         double& roll,
                                         double& pitch) const
{
  // merge point clouds
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::copyPointCloud(cloud1, *cloud_ptr);
  *cloud_ptr += cloud2;

  // Transform to ground frame
  if (ground_frame_ != "") {
    pcl::transformPointCloud(*cloud_ptr, *cloud_ptr, plane_transform_);
  }

  // Cut off top or bottom part
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud_ptr);
  pass.setFilterFieldName ("z");
  if (options_.detect_ground_plane) {
    pass.setFilterLimits(-std::numeric_limits<float>::max() , 0);
  } else if (options_.detect_ceiling) {
    pass.setFilterLimits(0.0, std::numeric_limits<float>::max());
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_part(new pcl::PointCloud<pcl::PointXYZ>());
  pass.filter(*cloud_part);

  // Transform back
  if (ground_frame_ != "") {
    pcl::transformPointCloud(*cloud_part, *cloud_part, plane_transform_.inverse());
  }

  // init segmentation
  pcl::ModelCoefficients coefficients;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // segmentation parameters
  seg.setOptimizeCoefficients (true);

  seg.setAxis((plane_transform_.inverse().rotation() * Eigen::Vector3d(0,0,1)).cast<float>()); // ground is along z-axis of actuator frame (assumption)
  seg.setEpsAngle(M_PI/4); // 45° offset from model
  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.05);
  seg.setMaxIterations(1000);

  seg.setInputCloud(cloud_part);
  seg.segment(*inliers, coefficients);

  // Extract plane inliers
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(cloud_part);
  extract.setIndices(inliers);

  pcl::PointCloud<pcl::PointXYZ>::Ptr ground_plane(new pcl::PointCloud<pcl::PointXYZ>());
  extract.filter(*ground_plane);

  // Publish plane
  sensor_msgs::PointCloud2 ground_plane_msg;
  pcl::toROSMsg(*ground_plane, ground_plane_msg);
  ground_plane_msg.header.frame_id = actuator_frame_;
  ground_plane_msg.header.stamp = ros::Time::now();

  ground_plane_pub_.publish(ground_plane_msg);

  // Calculate angle from ground plane to actuator frame around x-axis
  double nx = coefficients.values[0]; double ny = coefficients.values[1]; double nz = coefficients.values[2];

  roll = M_PI/2 - std::acos(ny/std::sqrt(std::pow(ny, 2) + std::pow(nz, 2)));
  pitch = M_PI/2 - std::acos(nx/std::sqrt(std::pow(nx, 2) + std::pow(nz, 2))); // not needed

  if (options_.detect_ground_plane) {
    ROS_INFO_STREAM("Detecting ground plane. Roll Angle: " << roll << " Pitch Angle: " << pitch);
  } else if (options_.detect_ceiling) {
    ROS_INFO_STREAM("Detecting ceiling. Roll Angle: " << roll << " Pitch Angle: " << pitch);
  }

  return true;
}

bool LidarCalibration::checkConvergence(const Calibration& prev_calibration,
                                         const Calibration& current_calibration) const
{
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

bool LidarCalibration::saveToDisk(std::string path, const Calibration& calibration) const {
  Calibration C = calibration;

  if (o_laser_frame_ != "" && o_spin_frame_ != "") {
    ROS_INFO_STREAM("Transforming calibration to frame " << o_spin_frame_);
    Eigen::Affine3d HV = calibration.getTransform()*laser_transform_;
    C = Calibration(HV);
  }
  C.threshold(1e-3);
  boost::posix_time::ptime now = boost::posix_time::second_clock::local_time();

  std::ofstream outfile (path);
  outfile << "<?xml version=\"1.0\"?>" << std::endl;
  outfile <<
      "<!-- =================================================================================== -->" << std::endl <<
      "<!-- |    This document was autogenerated by lidar_calibration on " <<  now.date().day() << "." << std::setw(2) << std::setfill('0') <<
      now.date().month().as_number() << "." << now.date().year() << ", " << now.time_of_day() <<
      ".| -->" << std::endl <<
      "<!-- |    Insert this transformation between frames " << o_spin_frame_ << " and " << o_laser_frame_ <<  ". | -->" << std::endl <<
      "<!-- |    EDITING THIS FILE BY HAND IS NOT RECOMMENDED                                 | -->" << std::endl <<
      "<!-- =================================================================================== -->" << std::endl;
  outfile << "<robot xmlns:xacro=\"http://www.ros.org/wiki/xacro\" name=\"calibration\">" << std::endl;
  outfile << "  <origin xyz=\"" << C.x << " " << C.y << " " << C.z << "\" ";
  outfile << "rpy=\"" << C.roll << " " << C.pitch << " " << C.yaw << "\"/>" << std::endl;
  outfile << "</robot>";
  outfile.close();
  return true;
}

Eigen::Affine3d LidarCalibration::getTransform(std::string frame_base, std::string frame_target) const {
  ros::Time now = ros::Time::now();
  if (tfl_.waitForTransform(frame_base, frame_target, now, tf_wait_duration_)) {
    tf::StampedTransform transform;
    tfl_.lookupTransform(frame_base, frame_target, now, transform);

    Eigen::Affine3d transform_eigen;
    tf::transformTFToEigen(transform, transform_eigen);
    return transform_eigen;
  } else {
    ROS_WARN_STREAM("Could not find transform from " << frame_base << " to " << frame_target << ". Using identity.");
    return Eigen::Affine3d::Identity();
  }
}

}
}
