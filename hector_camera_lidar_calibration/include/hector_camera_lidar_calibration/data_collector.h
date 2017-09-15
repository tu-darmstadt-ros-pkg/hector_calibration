#ifndef DATA_COLLECTOR_H
#define DATA_COLLECTOR_H

#include <ros/ros.h>
#include <rosbag/bag.h>

#include <camera_model_loader/camera_model_loader.h>
#include <sensor_msgs/PointCloud2.h>
#include <hector_calibration_msgs/CameraLidarCalibrationData.h>

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

};

}
}

#endif
