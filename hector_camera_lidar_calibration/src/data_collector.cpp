#include <hector_camera_lidar_calibration/data_collector.h>

namespace hector_calibration {
namespace camera_lidar_calibration {

DataCollector::DataCollector()
  : captured_clouds(0) {
  ros::NodeHandle cam_nh("~");
  camera_model_loader_.loadCamerasFromNamespace(cam_nh);
  camera_model_loader_.startSubscribers();

  ros::NodeHandle nh;
  cloud_sub_ = nh.subscribe("cloud", 1, &DataCollector::cloudCb, this);
}

hector_calibration_msgs::CameraLidarCalibrationData DataCollector::captureData() {
  ROS_INFO_STREAM("Waiting for data..");
  ros::Rate rate(1);
  while (captured_clouds < 2 || !receivedImages()) {
    rate.sleep();
    ros::spinOnce();
  }
  ROS_INFO_STREAM("Received all data");

  hector_calibration_msgs::CameraLidarCalibrationData data;
  // Add cloud
  data.scan = *last_cloud_ptr_;
  // add images
  for (std::map<std::string, camera_model::Camera>::iterator c = camera_model_loader_.getCameraMap().begin(); c != camera_model_loader_.getCameraMap().end(); ++c) {
    camera_model::Camera& cam = c->second;
    data.images.push_back(*cam.getLastImage());
  }
  // Save to bag
  rosbag::Bag bag;
  std::string filename = "/tmp/lidar_calibration_data.bag";
  bag.open(filename, rosbag::bagmode::Write);
  ros::Time time(ros::Time::now());
  bag.write("calibration_data", time, data);

  ROS_INFO_STREAM("Saved data to " << filename);

  return data;
}

bool DataCollector::receivedImages() {
  for (std::map<std::string, camera_model::Camera>::iterator c = camera_model_loader_.getCameraMap().begin(); c != camera_model_loader_.getCameraMap().end(); ++c) {
    camera_model::Camera& cam = c->second;
    if(!cam.getLastImage()) {
      return false;
    }
  }
  return true;
}

void DataCollector::cloudCb(const sensor_msgs::PointCloud2ConstPtr& cloud_ptr) {
  captured_clouds++;
  if (captured_clouds > 1) {
    last_cloud_ptr_ = cloud_ptr;
  }
}

}
}
