#include <ros/ros.h>
#include <boost/date_time.hpp>

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_node");

  boost::posix_time::ptime now = boost::posix_time::second_clock::local_time();

  std::stringstream ss;
  ss << now.date().day() << "." << std::setw(2) << std::setfill('0') << now.date().month().as_number() << "." << now.date().year() << ", " << now.time_of_day();
  ROS_INFO_STREAM("Stamp: " << ss.str());

  ros::NodeHandle nh;

  return 0;
}
