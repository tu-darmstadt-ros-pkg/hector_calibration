#ifndef DATA_COLLECTOR_H
#define DATA_COLLECTOR_H

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <opencv2/highgui/highgui.hpp>
#include <boost/filesystem.hpp>

#include <camera_model_loader/camera_model_loader.h>
#include <sensor_msgs/PointCloud2.h>
#include <hector_calibration_msgs/CameraLidarCalibrationData.h>

#include <tf2_ros/transform_listener.h>

namespace hector_calibration {
namespace camera_lidar_calibration {

class DataCollector {
public:
  DataCollector();
  hector_calibration_msgs::CameraLidarCalibrationData captureData();
private:
  void cloudCb(const sensor_msgs::PointCloud2ConstPtr& cloud_ptr);
  bool receivedImages();

  int captured_clouds;
  ros::Subscriber cloud_sub_;
  sensor_msgs::PointCloud2ConstPtr last_cloud_ptr_;
  camera_model::CameraModelLoader camera_model_loader_;

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
