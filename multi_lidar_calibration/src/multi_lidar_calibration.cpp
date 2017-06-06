#include <multi_lidar_calibration/multi_lidar_calibration.h>

namespace hector_calibration {
namespace lidar_calibration {

MultiLidarCalibration::MultiLidarCalibration(ros::NodeHandle nh) :
  nh_(nh)
{
  // Init publishers
  for (unsigned int i = 0; i < 2; i++) {
    raw_pub_[i] = nh_.advertise<sensor_msgs::PointCloud2>("raw_cloud" + std::to_string(i), 1000);
    preprocessed_pub_[i] = nh_.advertise<sensor_msgs::PointCloud2>("preprocessed_cloud" + std::to_string(i), 1000);
    result_pub_[i] = nh_.advertise<sensor_msgs::PointCloud2>("result_cloud" + std::to_string(i), 1000);
  }
  mapping_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("neighbor_mapping", 1000);
  // Load parameters
  ros::NodeHandle pnh("~");
  pnh.param<std::string>("base_frame", base_frame_, "base_link");
  pnh.param<double>("max_sqr_dist", max_sqr_dist_, 0.0025);
  pnh.param<int>("neighbor_mapping_vis_count", neighbor_mapping_vis_count_, 100);
  pnh.param<double>("normals_radius", normals_radius_, 0.07);
  pnh.param<double>("crop_dist", crop_dist_, 1.0);
  pnh.param<double>("voxel_leaf_size", voxel_leaf_size_, 0.01);
  pnh.param<int>("max_iterations", max_iterations_, 20);
  pnh.param<double>("parameter_diff_thres", parameter_diff_thres_, 1e-3);

  pnh.param<std::string>("target_frame", target_frame_, "");
  double wait_duration;
  pnh.param<double>("tf_wait_duration", wait_duration, 1.0);
  tf_wait_duration_ = ros::Duration(wait_duration);
  pnh.param<std::string>("save_path", save_path_, "");
}

Eigen::Affine3d
MultiLidarCalibration::calibrate(const sensor_msgs::PointCloud2& cloud1_msg,
                                 const sensor_msgs::PointCloud2& cloud2_msg)
{
  if (cloud1_msg.header.frame_id != cloud2_msg.header.frame_id) {
    ROS_ERROR_STREAM("Frame of cloud 1 (" << cloud1_msg.header.frame_id <<
                     ") doesn't match frame of cloud 2 (" << cloud2_msg.header.frame_id << "). Aborting.");
    return Eigen::Affine3d::Identity();
  }
  if (base_frame_ != cloud1_msg.header.frame_id) {
    ROS_WARN_STREAM("Base frame (" << base_frame_ << ") doesn't match cloud frame id (" << cloud1_msg.header.frame_id << "). \n" <<
                    "Changing base frame to frame_id.");
    base_frame_ = cloud1_msg.header.frame_id;
  }

  pcl::PointCloud<pcl::PointXYZ> cloud1;
  pcl::PointCloud<pcl::PointXYZ> cloud2;
  pcl::fromROSMsg(cloud1_msg, cloud1);
  pcl::fromROSMsg(cloud2_msg, cloud2);
  return calibrate(cloud1, cloud2);
}

Eigen::Affine3d
MultiLidarCalibration::calibrate(pcl::PointCloud<pcl::PointXYZ> cloud1,
                                 pcl::PointCloud<pcl::PointXYZ> cloud2)
{
  if (target_frame_ != "")
    old_transform_ = getTransform(base_frame_, target_frame_);

  ROS_INFO_STREAM("Starting calibration");
  publishCloud(cloud1, raw_pub_[0], base_frame_);
  publishCloud(cloud2, raw_pub_[1], base_frame_);

  ROS_INFO_STREAM("Cloud 1 raw size: " << cloud1.size());
  ROS_INFO_STREAM("Cloud 2 raw size: " << cloud2.size());

  ROS_INFO_STREAM("Preprocessing clouds");
  preprocessClouds(cloud1, cloud2);
  ROS_INFO_STREAM("Cloud 1 preprocessed size: " << cloud1.size());
  ROS_INFO_STREAM("Cloud 2 preprocessed size: " << cloud2.size());

  ROS_INFO_STREAM("Computing Normals");
  std::vector<WeightedNormal> normals = computeNormals(cloud1, normals_radius_);

  Eigen::Affine3d calibration = Eigen::Affine3d::Identity();
  Eigen::Affine3d prev_calibration = Eigen::Affine3d::Identity();
  pcl::PointCloud<pcl::PointXYZ> cloud2_transformed = cloud2;

  // publish initial clouds
  publishCloud(cloud1, result_pub_[0], base_frame_);
  publishCloud(cloud2_transformed, result_pub_[1], base_frame_);

  unsigned int iteration_counter = 0;
  double max_distance = max_sqr_dist_;
  do {
    ROS_INFO_STREAM("-------------- Starting iteration " << (iteration_counter+1) << "--------------");
    ROS_INFO_STREAM("Searching neighbors with max dist of " << std::sqrt(max_distance));
    std::map<unsigned int, unsigned int> neighbor_mapping = findNeighbors(cloud1, cloud2_transformed, max_distance);
    publishNeighbors(cloud1, cloud2_transformed, neighbor_mapping, mapping_pub_, base_frame_, neighbor_mapping_vis_count_);
    max_distance *= 0.5;

    ROS_INFO_STREAM("Starting calibration");
    prev_calibration = calibration;
    calibration = optimize(cloud1, cloud2, normals, neighbor_mapping, calibration);
    pcl::transformPointCloud(cloud2, cloud2_transformed, calibration);
    publishCloud(cloud1, result_pub_[0], base_frame_);
    publishCloud(cloud2_transformed, result_pub_[1], base_frame_);

    iteration_counter++;
  } while (ros::ok() && !maxIterationsReached(iteration_counter) && !checkConvergence(prev_calibration, calibration));

  if (target_frame_ != "" && save_path_ != "") {
    saveToDisk(save_path_, calibration);
  }

  return calibration;
}

bool MultiLidarCalibration::maxIterationsReached(unsigned int current_iterations) const {
  if (current_iterations < max_iterations_) {
    return false;
  }  else {
    ROS_INFO_STREAM("-------- MAX ITERATIONS REACHED ---------");
    return true;
  }
}

bool MultiLidarCalibration::checkConvergence(const Eigen::Affine3d& prev_calibration,
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
  ROS_INFO_STREAM("Squared change in parameters: " << cum_sqrt_diff);
  if (cum_sqrt_diff < parameter_diff_thres_) {
    ROS_INFO_STREAM("-------------- CONVERGENCE --------------");
    return true;
  } else {
    return false;
  }
}

void MultiLidarCalibration::preprocessClouds(pcl::PointCloud<pcl::PointXYZ>& cloud1,
                                             pcl::PointCloud<pcl::PointXYZ>& cloud2)

{
  cropCloud(cloud1, crop_dist_);
  cropCloud(cloud2, crop_dist_);
  // downsampleCloud(cloud1, (float) voxel_leaf_size_);
  // downsampleCloud(cloud2, (float) voxel_leaf_size_);

  publishCloud(cloud1, preprocessed_pub_[0], base_frame_);
  publishCloud(cloud2, preprocessed_pub_[1], base_frame_);
}

void MultiLidarCalibration::cropCloud(pcl::PointCloud<pcl::PointXYZ>& cloud, double distance) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  *cloud_ptr = cloud;

  pcl::CropBox<pcl::PointXYZ> crop_box_filter;
  crop_box_filter.setInputCloud(cloud_ptr);
  crop_box_filter.setNegative(true);
  Eigen::Vector4f box_v(1, 1, 1, 1);

  Eigen::Vector4f min_v = -distance*box_v;
  min_v[3] = 1;
  Eigen::Vector4f max_v = distance*box_v;
  min_v[3] = 1;

  crop_box_filter.setMin(min_v);
  crop_box_filter.setMax(max_v);

  crop_box_filter.filter(cloud);
}

void MultiLidarCalibration::downsampleCloud(pcl::PointCloud<pcl::PointXYZ>& cloud, float leaf_size) {
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
    ROS_ERROR_STREAM("Size of cloud1 (" << cloud1.size() << ") doesn't match size of normals (" << normals.size() << ").");
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
  ROS_INFO_STREAM("Number of residuals: " << residual_count);

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

  printCalibration(translation[0], translation[1], translation[2], rotation[0], rotation[1], rotation[2]);
  return calibration;
}

Eigen::Affine3d MultiLidarCalibration::getTransform(std::string frame_base, std::string frame_target) const {
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

void MultiLidarCalibration::printCalibration(const Eigen::Affine3d& calibration) const {
  Eigen::Vector3d ypr = calibration.linear().eulerAngles(2, 1, 0);
  Eigen::Vector3d xyz = calibration.translation();
  printCalibration(xyz(0), xyz(1), xyz(2), ypr(2), ypr(1), ypr(0));
}

void MultiLidarCalibration::printCalibration(double x, double y, double z, double roll, double pitch, double yaw) const {
  ROS_INFO_STREAM("x: " << x << ", y: " << y <<  ", z: " << z  << "\n" <<
                  "roll: " << normalizeAngle(roll) << ", pitch: " << normalizeAngle(pitch) << ", yaw: " << normalizeAngle(yaw));
}

bool MultiLidarCalibration::saveToDisk(std::string path, const Eigen::Affine3d& calibration) const {
  Eigen::Affine3d new_transform = calibration * old_transform_; // apply calibration
  Eigen::Vector3d ypr = new_transform.linear().eulerAngles(2, 1, 0);
  Eigen::Vector3d xyz = new_transform.translation();

  ROS_INFO_STREAM("Old calibration");
  printCalibration(old_transform_);

  ROS_INFO_STREAM("New Calibration:");
  printCalibration(new_transform);

  // diff
  Eigen::Vector3d old_ypr = old_transform_.linear().eulerAngles(2, 1, 0);
  Eigen::Vector3d old_xyz = old_transform_.translation();

  Eigen::Vector3d new_ypr = new_transform.linear().eulerAngles(2, 1, 0);
  Eigen::Vector3d new_xyz = new_transform.translation();
  Eigen::Vector3d diff_xyz = new_xyz - old_xyz;
  Eigen::Vector3d diff_ypr = new_ypr - old_ypr;
  ROS_INFO_STREAM("Difference:");
  printCalibration(diff_xyz(0), diff_xyz(1), diff_xyz(2), diff_ypr(2), diff_ypr(1), diff_ypr(0));

  boost::posix_time::ptime now = boost::posix_time::second_clock::local_time();

  ROS_INFO_STREAM("Saving calibration file to: " << path);

  std::ofstream outfile (path);
  outfile << "<?xml version=\"1.0\"?>" << std::endl;
  outfile <<
      "<!-- =================================================================================== -->" << std::endl <<
      "<!-- |    This document was autogenerated by multi_lidar_calibration on " <<  now.date().day() << "." << std::setw(2) << std::setfill('0') <<
      now.date().month().as_number() << "." << now.date().year() << ", " << now.time_of_day() << ".| -->" << std::endl <<
      "<!-- |    Insert this transformation between frames " << base_frame_ << " and " << target_frame_ <<  ". | -->" << std::endl <<
      "<!-- |    EDITING THIS FILE BY HAND IS NOT RECOMMENDED                                 | -->" << std::endl <<
      "<!-- =================================================================================== -->" << std::endl;
  outfile << "<robot xmlns:xacro=\"http://www.ros.org/wiki/xacro\" name=\"calibration\">" << std::endl;
  outfile << "  <origin rpy=\"" << ypr[2] << " " << ypr[1] << " " << ypr[0] << "\" ";
  outfile << "xyz=\"" << xyz[0] << " " << xyz[1] << " " << xyz[2] << "\"/>" << std::endl;
  outfile << "</robot>";
  outfile.close();
  return true;
}

}
}
