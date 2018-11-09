#include <ros/ros.h>

#include <hector_camera_lidar_calibration/data_collector.h>
#include <hector_camera_lidar_calibration/optimizer.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "camera_lidar_calibration_node");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  bool capture_data = pnh.param("capture_data", true);

  hector_calibration::hector_camera_lidar_calibration::Optimizer opt(nh, pnh);

  if (capture_data) {
    hector_calibration::hector_camera_lidar_calibration::DataCollector collector;
    hector_calibration_msgs::CameraLidarCalibrationData data = collector.captureData();
    std::vector<hector_calibration_msgs::CameraLidarCalibrationData> data_vec;
    data_vec.push_back(data);
    opt.loadData(data_vec);
  } else {
    std::vector<std::string> bag_paths;
    if (pnh.getParam("bag_paths", bag_paths) && !bag_paths.empty()) {
      for (const std::string& path: bag_paths) {
        opt.loadFromBag(path);
      }
    } else {
      std::string bag_path;
      if (!pnh.getParam("bag_path", bag_path)) {
        ROS_ERROR_STREAM("Mandatory parameter 'bag_path', missing in nh '" << pnh.getNamespace() << "'!");
        return 0;
      }
      opt.loadFromBag(bag_path);
    }
  }

  bool manual_calibration = pnh.param("manual_calibration", false);
  if (manual_calibration) {
    opt.manualCalibration();
  } else {
    opt.optimize();
  }

  return 0;
}
