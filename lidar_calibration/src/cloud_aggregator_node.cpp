#include <lidar_calibration/cloud_aggregator.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "cloud_aggregator_node");
  ros::NodeHandle nh;
  hector_calibration::lidar_calibration::CalibrationCloudAggregator agg;
  agg.setPeriodicPublishing(true, 10.0);
  ros::spin();

  return 0;
}
