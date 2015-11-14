#ifndef POINT_CLOUD_AGGREGATOR_H
#define POINT_CLOUD_AGGREGATOR_H

#include <functional>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

//PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

namespace hector_calibration {

class PointCloudAggregator {
public:
  bool load_settings(const ros::NodeHandle& nh);
  void get_scan(double start_angle, double goal_angle, std::function<void(sensor_msgs::PointCloud2)> cb_fun);
private:
  void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
  bool capture_running_;
  pcl::PointCloud<pcl::PointXYZI> cloud_agg_;

  bool settings_loaded_;
  std::string lidar_cmd_topic_;
  std::string lidar_interface_; /// joint interface of the lidar (position, velocity, torque)
  std::string point_cloud_topic_; /// topic name where the point cloud (sensor_msgs::PointCoud2) is published

};

}


#endif
