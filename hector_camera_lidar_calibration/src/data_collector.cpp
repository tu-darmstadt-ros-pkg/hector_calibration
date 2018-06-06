#include <hector_camera_lidar_calibration/data_collector.h>

namespace hector_calibration {
namespace camera_lidar_calibration {

DataCollector::DataCollector()
  : captured_clouds(0), tf_listener_(tf_buffer_) {
  ros::NodeHandle cam_nh("~");
  camera_model_loader_.loadCamerasFromNamespace(cam_nh);
  camera_model_loader_.startSubscribers();

  ros::NodeHandle nh;
  cloud_sub_ = nh.subscribe("cloud", 1, &DataCollector::cloudCb, this);

  ros::NodeHandle pnh("~");
  pnh.param<std::string>("base_frame", base_frame_, "base_link");
  pnh.param<std::string>("cam_head_frame", cam_head_frame_, "cam_head");
  pnh.param<std::string>("mask_path", mask_path_, "");
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


  // add images
  for (std::map<std::string, camera_model::Camera>::iterator c = camera_model_loader_.getCameraMap().begin(); c != camera_model_loader_.getCameraMap().end(); ++c) {
    camera_model::Camera& cam = c->second;
    hector_calibration_msgs::CameraObservation cam_obs;
    cam_obs.name.data = cam.getName();
    cam_obs.image = *cam.getLastImage();
    std::string frame_id;
    if (cam.getFrameId() != "") {
      frame_id = cam.getFrameId();
    } else {
      frame_id = cam_obs.image.header.frame_id;
    }
    try {
      cam_obs.transform = tf_buffer_.lookupTransform(frame_id, cam_head_frame_, ros::Time(0));
    } catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
    }
    // add mask
    if (mask_path_ != "") {
      boost::filesystem::path path = mask_path_ / boost::filesystem::path(cam.getName() + "_mask.png");
      cv::Mat mask = cv::imread(path.string(), CV_LOAD_IMAGE_GRAYSCALE);
      cv_bridge::CvImage cv_image;
      cv_image.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
      cv_image.image = mask;
      cam_obs.mask = *  cv_image.toImageMsg();
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
  ROS_INFO_STREAM("Received cloud " << captured_clouds);
  if (captured_clouds > 1) {
    last_cloud_ptr_ = cloud_ptr;
  }
}

}
}
