#ifndef MULTI_LIDAR_CALIBRATION_H
#define MULTI_LIDAR_CALIBRATION_H

#include <lidar_calibration_lib/lidar_calibration_common.h>

// pcl
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>

// ros
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// tf
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

// ceres solver
#include <ceres/ceres.h>
#include <multi_lidar_calibration/lidar_pose_error.h>

namespace hector_calibration {
namespace lidar_calibration {

class MultiLidarCalibration {
public:
  MultiLidarCalibration(ros::NodeHandle nh);
  Eigen::Affine3d calibrate(pcl::PointCloud<pcl::PointXYZ> cloud1, pcl::PointCloud<pcl::PointXYZ> cloud2);
  Eigen::Affine3d calibrate(const sensor_msgs::PointCloud2& cloud1_msg, const sensor_msgs::PointCloud2& cloud2_msg);
private:
  void preprocessClouds(pcl::PointCloud<pcl::PointXYZ>& cloud1, pcl::PointCloud<pcl::PointXYZ>& cloud2);
  void cropCloud(pcl::PointCloud<pcl::PointXYZ>& cloud, double distance);
  void downsampleCloud(pcl::PointCloud<pcl::PointXYZ>& cloud, float leaf_size);

  Eigen::Affine3d optimize(const pcl::PointCloud<pcl::PointXYZ>& cloud1,
                const pcl::PointCloud<pcl::PointXYZ>& cloud2,
                const std::vector<WeightedNormal>& normals,
                const std::map<unsigned int, unsigned int>& mapping,
                const Eigen::Affine3d initial_calibration);
  bool maxIterationsReached(unsigned int current_iterations) const;
  bool checkConvergence(const Eigen::Affine3d& calibration) const;
  bool saveToDisk(std::string path, const Eigen::Affine3d& calibration) const;

  Eigen::Affine3d getTransform(std::string frame_base, std::string frame_target) const;

  void printCalibration(const Eigen::Affine3d& calibration) const;
  void printCalibration(double x, double y, double z, double roll, double pitch, double yaw) const;

  ros::Publisher raw_pub_[2];
  ros::Publisher preprocessed_pub_[2];
  ros::Publisher mapping_pub_;
  ros::Publisher result_pub_[2];

  tf::TransformListener tfl_;
  ros::Duration tf_wait_duration_;

  std::string save_path_;

  ros::NodeHandle nh_;
  std::string base_frame_;
  std::string target_frame_;
  Eigen::Affine3d old_transform_;

  double max_sqr_dist_;
  int neighbor_mapping_vis_count_;
  double normals_radius_;
  double crop_dist_;
  double voxel_leaf_size_;
  int max_iterations_;
  double parameter_diff_thres_;

};

}
}

#endif
