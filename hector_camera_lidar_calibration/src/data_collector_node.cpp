#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "data_collector_node");
 
  ros::spin();
  return 0;
}
