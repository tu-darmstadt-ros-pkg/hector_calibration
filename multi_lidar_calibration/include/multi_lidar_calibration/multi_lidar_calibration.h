#pragma once

// #include <lidar_calibration_lib/lidar_calibration_common.h>

// pcl
// #include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>

// ros
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>

// tf
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
// #include <tf_conversions/tf_eigen.h>

// ceres solver
#include <ceres/ceres.h>
#include <multi_lidar_calibration/lidar_pose_error.h>

namespace hector_calibration {
namespace lidar_calibration {

class MultiLidarCalibration : public rclcpp::Node {
public:
  MultiLidarCalibration();
  ~MultiLidarCalibration() = default;

private:
  void calibrate();
  void preprocessClouds();
  void cropCloud(pcl::PointCloud<pcl::PointXYZ>& cloud, double distance);
  void downsampleCloud(pcl::PointCloud<pcl::PointXYZ>& cloud, float leaf_size);

  Eigen::Affine3d optimize(const pcl::PointCloud<pcl::PointXYZ>& cloud1,
                const pcl::PointCloud<pcl::PointXYZ>& cloud2,
                const std::vector<WeightedNormal>& normals,
                const std::map<unsigned int, unsigned int>& mapping,
                const Eigen::Affine3d &initial_calibration);
  bool maxIterationsReached(unsigned int current_iterations) const;
  double getCumSqrtDiff(const Eigen::Affine3d& prev_calibration, const Eigen::Affine3d& current_calibration) const;
  bool checkConvergence(const Eigen::Affine3d& prev_calibration, const Eigen::Affine3d& current_calibration) const;
  double computeAvgResidual(const pcl::PointCloud<pcl::PointXYZ>& cloud1,
                            const pcl::PointCloud<pcl::PointXYZ>& cloud2,
                            const std::map<unsigned int, unsigned int>& mapping) const;
  bool saveToDisk(std::string path, const Eigen::Affine3d& calibration) const;

  std::string printCalibration(const Eigen::Affine3d& calibration) const;
  std::string printCalibration(double x, double y, double z, double roll, double pitch, double yaw) const;

  void cloudToMsg(const pcl::PointCloud<pcl::PointXYZ>& cloud, sensor_msgs::msg::PointCloud2::SharedPtr& msg);
  void publishClouds();
  void publishTf();

  void cloudCb1(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void cloudCb2(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void imuCb(const sensor_msgs::msg::Imu::SharedPtr msg);


  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud1_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud2_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr raw_pub_[2];
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr preprocessed_pub_[2];
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr init_guess_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr result_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;  
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;

  pcl::PointCloud<pcl::PointXYZ> cloud1_;
  pcl::PointCloud<pcl::PointXYZ> cloud2_;
  std::array<double, 3> sum_imu_acc_;
  unsigned int scan1_counter_;
  unsigned int scan2_counter_;
  unsigned int imu_counter_;
  unsigned int min_scans_;
  unsigned int min_imu_msgs_;

  sensor_msgs::msg::PointCloud2::SharedPtr cloud1_msg_;
  sensor_msgs::msg::PointCloud2::SharedPtr cloud2_msg_;
  sensor_msgs::msg::PointCloud2::SharedPtr cloud1_prepr_msg_;
  sensor_msgs::msg::PointCloud2::SharedPtr cloud2_prepr_msg_;
  sensor_msgs::msg::PointCloud2::SharedPtr cloud2_init_guess_msg_;
  sensor_msgs::msg::PointCloud2::SharedPtr cloud2_result_msg_;

  std::string save_path_;
  bool use_imu_;
  bool imu_ok_;
  bool tf_published_;

  std::string world_frame_;
  std::string lidar_frame1_;
  std::string lidar_frame2_;
  Eigen::Affine3d init_guess_;
  Eigen::Affine3d old_transform_;
  Eigen::Affine3d calibration_;

  double max_sqr_dist_;
  unsigned int neighbor_mapping_vis_count_;
  double normals_radius_;
  double crop_dist_;
  double voxel_leaf_size_;
  unsigned int max_iterations_;
  double parameter_diff_thres_;
};

}
}
