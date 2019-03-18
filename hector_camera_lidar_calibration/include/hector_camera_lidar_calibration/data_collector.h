#ifndef DATA_COLLECTOR_H
#define DATA_COLLECTOR_H

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <opencv2/highgui/highgui.hpp>
#include <boost/filesystem.hpp>

#include <kalibr_camera_loader/camera_loader.h>
#include <sensor_msgs/PointCloud2.h>
#include <hector_calibration_msgs/CameraLidarCalibrationData.h>

#include <tf2_ros/transform_listener.h>

namespace hector_calibration {
namespace hector_camera_lidar_calibration {

class DataCollector {
public:
  DataCollector(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
  hector_calibration_msgs::CameraLidarCalibrationData captureData();
private:
  void cloudCb(const sensor_msgs::PointCloud2ConstPtr& cloud_ptr);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  int captured_clouds;
  ros::Subscriber cloud_sub_;
  sensor_msgs::PointCloud2ConstPtr last_cloud_ptr_;
  kalibr_image_geometry::CameraLoader camera_loader_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Parameters
  std::string base_frame_;
  std::string cam_head_frame_;
  std::string mask_path_;

};

}
}

#endif
