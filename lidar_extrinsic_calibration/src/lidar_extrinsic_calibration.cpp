#include <lidar_extrinsic_calibration/lidar_extrinsic_calibration.h>

namespace hector_calibration {

LidarExtrinsicCalibration::LidarExtrinsicCalibration(ros::NodeHandle &nh) :
nh_(nh),
first_cloud_(true) {
  cloud_sub_ = nh_.subscribe("cloud", 1000, &LidarExtrinsicCalibration::pointCloudCb, this);
  //result_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("result", 1000);
  ground_plane_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("ground_plane", 1000, true);

  ros::NodeHandle pnh("~");
  pnh.param<std::string>("ground_frame", ground_frame_, "base_link");
  double duration;
  pnh.param<double>("tf_wait_duration", duration, 10.0);
  tf_wait_duration_ = ros::Duration(duration);
}

void LidarExtrinsicCalibration::calibrateGround() {
  // Wait for cloud
  ROS_INFO_STREAM("Waiting for point cloud..");
  while (!last_cloud_ptr_ && ros::ok()) {
    ros::spinOnce();
    ros::Duration(1).sleep();
  }

  // convert msg to pointcloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*last_cloud_ptr_, *pcl_cloud_ptr);

  ROS_INFO_STREAM("Point cloud size: " << pcl_cloud_ptr->size());

  // Transform to ground frame
  Eigen::Affine3d plane_transform = getTransform(ground_frame_, last_cloud_ptr_->header.frame_id, last_cloud_ptr_->header.stamp);
  pcl::transformPointCloud(*pcl_cloud_ptr, *pcl_cloud_ptr, plane_transform);

  // Cut off top part
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(pcl_cloud_ptr);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(-std::numeric_limits<float>::max() , 0);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_part(new pcl::PointCloud<pcl::PointXYZ>());
  pass.filter(*cloud_part);

  // Transform back
  //pcl::transformPointCloud(*cloud_part, *cloud_part, plane_transform.inverse());

  // init segmentation
  pcl::ModelCoefficients coefficients;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // segmentation parameters
  seg.setOptimizeCoefficients (true);

  seg.setAxis(Eigen::Vector3f(0,0,1)); // ground is perpendicular to z axis (assumption)
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
  ground_plane_msg.header.frame_id = ground_frame_;
  ground_plane_msg.header.stamp = ros::Time::now();

  ground_plane_pub_.publish(ground_plane_msg);

  // Calculate angle from ground plane to ground frame around x-axis
  float nx = coefficients.values[0]; float ny = coefficients.values[1]; float nz = coefficients.values[2];

  double roll = M_PI/2 - std::acos(ny/std::sqrt(std::pow(ny, 2) + std::pow(nz, 2)));
  double pitch = M_PI/2 - std::acos(nx/std::sqrt(std::pow(nx, 2) + std::pow(nz, 2)));

  Eigen::Vector3d offset(roll, -pitch, 0);
  Eigen::Vector3d rotated_offset = plane_transform.rotation().inverse() * offset;

  ROS_INFO_STREAM("Detected ground plane: " << offset);
  ROS_INFO_STREAM("Rotated: " << rotated_offset);
  ROS_INFO_STREAM("Add these values to your mount frame: " << last_cloud_ptr_->header.frame_id);
}

void LidarExtrinsicCalibration::pointCloudCb(const sensor_msgs::PointCloud2ConstPtr& cloud_ptr) {
  if (first_cloud_) {
    first_cloud_ = false;
    return;
  }
  last_cloud_ptr_ = cloud_ptr;
}

Eigen::Affine3d LidarExtrinsicCalibration::getTransform(std::string frame_base, std::string frame_target, const ros::Time& time) const
{
  if (tfl_.waitForTransform(frame_base, frame_target, time, tf_wait_duration_)) {
    tf::StampedTransform transform;
    tfl_.lookupTransform(frame_base, frame_target, time, transform);

    Eigen::Affine3d transform_eigen;
    tf::transformTFToEigen(transform, transform_eigen);
    return transform_eigen;
  } else {
    ROS_WARN_STREAM("Could not find transform from " << frame_base << " to " << frame_target << ". Using identity.");
    return Eigen::Affine3d::Identity();
  }
}

Eigen::Affine3d LidarExtrinsicCalibration::getTransform(std::string frame_base, std::string frame_target) const {
  return getTransform(frame_base, frame_target, ros::Time::now());
}

}
