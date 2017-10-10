#include <ros/ros.h>

#include <hector_camera_lidar_calibration/data_collector.h>
#include <hector_camera_lidar_calibration/optimizer.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "camera_lidar_calibration_vis_node");

  ros::NodeHandle pnh("~");

  bool capture_data;
  pnh.param("capture_data", capture_data, true);

  hector_calibration::camera_lidar_calibration::Optimizer opt;

  if (capture_data) {
    hector_calibration::camera_lidar_calibration::DataCollector collector;
    hector_calibration_msgs::CameraLidarCalibrationData data = collector.captureData();
    std::vector<hector_calibration_msgs::CameraLidarCalibrationData> data_vec;
    data_vec.push_back(data);
    opt.loadData(data_vec);
  } else {
    std::string bag_path;
    if (!pnh.getParam("bag_path", bag_path)) {
      ROS_ERROR_STREAM("Mandatory parameter 'bag_path', missing in nh '" << pnh.getNamespace() << "'!");
      return 0;
    }
    ROS_INFO_STREAM("Starting calibration from bag: " << bag_path);
    opt.loadFromBag(bag_path);
  }

  opt.visualizeCost();

  return 0;
}
