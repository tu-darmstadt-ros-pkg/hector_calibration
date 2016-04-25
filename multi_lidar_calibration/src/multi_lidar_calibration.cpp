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
  ROS_INFO_STREAM("Starting calibration");
  publishCloud(cloud1, raw_pub_[0], base_frame_);
  publishCloud(cloud2, raw_pub_[1], base_frame_);

  ROS_INFO_STREAM("Cloud 1 raw size: " << cloud1.size());
  ROS_INFO_STREAM("Cloud 2 raw size: " << cloud2.size());

  ROS_INFO_STREAM("Preprocessing clouds");
  preprocessClouds(cloud1, cloud2);
  ROS_INFO_STREAM("Cloud 1 preprocessed size: " << cloud1.size());
  ROS_INFO_STREAM("Cloud 2 preprocessed size: " << cloud2.size());

  ROS_INFO_STREAM("Searching neighbors");
  std::map<unsigned int, unsigned int> neighbor_mapping = findNeighbors(cloud1, cloud2, max_sqr_dist_);
  publishNeighbors(cloud1, cloud2, neighbor_mapping, mapping_pub_, base_frame_, neighbor_mapping_vis_count_);
  ROS_INFO_STREAM("Computing Normals");
  std::vector<WeightedNormal> normals = computeNormals(cloud1, normals_radius_);
  ROS_INFO_STREAM("Starting calibration");
  Eigen::Affine3d calibration = optimize(cloud1, cloud2, normals, neighbor_mapping);
  publishOptimizationResult(cloud1, cloud2, calibration);
  return calibration;
}

void MultiLidarCalibration::preprocessClouds(pcl::PointCloud<pcl::PointXYZ>& cloud1,
                                             pcl::PointCloud<pcl::PointXYZ>& cloud2)

{
  cropCloud(cloud1, crop_dist_);
  cropCloud(cloud2, crop_dist_);
  downsampleCloud(cloud1, (float) voxel_leaf_size_);
  downsampleCloud(cloud2, (float) voxel_leaf_size_);

  publishCloud(cloud1, preprocessed_pub_[0], base_frame_);
  publishCloud(cloud2, preprocessed_pub_[1], base_frame_);
}

void MultiLidarCalibration::cropCloud(pcl::PointCloud<pcl::PointXYZ>& cloud, double distance) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  *cloud_ptr = cloud;
  // pcl::copyPointCloud(cloud, *cloud_ptr);
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

  //pcl::PointCloud<pcl::PointXYZ> cloud_cropped;
  crop_box_filter.filter(cloud);

  //cloud = cloud_cropped;
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
MultiLidarCalibration::optimize(pcl::PointCloud<pcl::PointXYZ>& cloud1,
              pcl::PointCloud<pcl::PointXYZ>& cloud2,
              std::vector<WeightedNormal> &normals,
              std::map<unsigned int, unsigned int> &mapping)
{
  if (cloud1.size() != normals.size()) {
    ROS_ERROR_STREAM("Size of cloud1 (" << cloud1.size() << ") doesn't match size of normals (" << normals.size() << ").");
    return Eigen::Affine3d::Identity();
  }

  ceres::Problem problem;

  double rotation[3];
  double translation[3];
  for (unsigned int i = 0; i < 3; i++) {
    rotation[i] = 0;
    translation[i] = 0;
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
  std::cout << summary.BriefReport() << "\n";

  Eigen::Affine3d calibration(
        Eigen::AngleAxisd(rotation[2], Eigen::Vector3d::UnitZ())
      * Eigen::AngleAxisd(rotation[1], Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(rotation[0], Eigen::Vector3d::UnitX())
  );
  calibration.translation() = Eigen::Vector3d(translation[0], translation[1], translation[2]);


  ROS_INFO_STREAM("Optimization result:\n" <<
                  "roll: " << rotation[0] << ", pitch: " << rotation[1] << ", yaw: " << rotation[2] << "\n" <<
                  "x: " << translation[0] << ", y: " << translation[1] <<  ", z: " << translation[2]);

  return calibration;
}


void MultiLidarCalibration::publishOptimizationResult(pcl::PointCloud<pcl::PointXYZ>& cloud1,
                               pcl::PointCloud<pcl::PointXYZ>& cloud2,
                               Eigen::Affine3d& calibration)
{
  pcl::PointCloud<pcl::PointXYZ> cloud2_calibrated;
  pcl::transformPointCloud(cloud2, cloud2_calibrated, calibration);

  publishCloud(cloud1, result_pub_[0], base_frame_);
  publishCloud(cloud2_calibrated, result_pub_[1], base_frame_);
  ROS_INFO_STREAM("Calibrated size: " << cloud2_calibrated.size());
}



}
}
