#include <hector_camera_lidar_calibration/data_collector.h>

namespace hector_calibration {
namespace hector_camera_lidar_calibration {

DataCollector::DataCollector(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
  : nh_(nh), pnh_(pnh), captured_clouds(0), tf_listener_(tf_buffer_), camera_loader_(nh, pnh) {

  cloud_sub_ = nh_.subscribe("cloud", 1, &DataCollector::cloudCb, this);

  pnh.param<std::string>("base_frame", base_frame_, "base_link");
  pnh.param<std::string>("cam_head_frame", cam_head_frame_, "cam_head");
  pnh.param<std::string>("mask_path", mask_path_, "");
}

hector_calibration_msgs::CameraLidarCalibrationData DataCollector::captureData() {
  ROS_INFO_STREAM("Waiting for data..");
  ros::Rate rate(1);
  while (ros::ok() && (captured_clouds < 2 || camera_loader_.imagesReceived())) {
    rate.sleep();
    ros::spinOnce();
  }
  hector_calibration_msgs::CameraLidarCalibrationData data;
  if (!ros::ok()) {
    return data;
  }
  ROS_INFO_STREAM("Received all data");

  // Find transformation to transform data from base_frame to cam head
  try {
    data.cam_transform = tf_buffer_.lookupTransform(cam_head_frame_, base_frame_, ros::Time(0));
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
  }

  // Add cloud
  data.scan = *last_cloud_ptr_;
  if (data.scan.header.frame_id != base_frame_) {
    ROS_WARN_STREAM("Frame id of scan (" << data.scan.header.frame_id << ") doesn't match the base frame (" << base_frame_ << ").");
  }

  // Add images
  for (const kalibr_image_geometry::CameraPtr& cam: camera_loader_.cameras()) {
    hector_calibration_msgs::CameraObservation cam_obs;
    cam_obs.name.data = cam->getName();
    cam_obs.image = *cam->getLastImage();
    std::string frame_id;
    if (cam->model().cameraInfo().frame_id != "") {
      frame_id = cam->model().cameraInfo().frame_id;
    } else {
      frame_id = cam_obs.image.header.frame_id;
    }
    try {
      cam_obs.transform = tf_buffer_.lookupTransform(frame_id, cam_head_frame_, ros::Time(0));
    } catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
    }
    data.camera_observations.push_back(cam_obs);
  }

  // Save to bag
  rosbag::Bag bag;
  std::string filename = "/tmp/camera_lidar_calibration_data.bag";
  bag.open(filename, rosbag::bagmode::Write);
  ros::Time time(ros::Time::now());
  bag.write("calibration_data", time, data);

  ROS_INFO_STREAM("Saved data to " << filename);

  return data;
}

void DataCollector::cloudCb(const sensor_msgs::PointCloud2ConstPtr& cloud_ptr) {
  captured_clouds++;
  ROS_INFO_STREAM("Received cloud " << captured_clouds);
  if (captured_clouds > 1) {
    last_cloud_ptr_ = cloud_ptr;
  }
}

}
}
