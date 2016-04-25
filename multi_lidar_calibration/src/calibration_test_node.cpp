#include <multi_lidar_calibration/multi_lidar_calibration.h>

#include <std_msgs/Float64MultiArray.h>

sensor_msgs::PointCloud2 cloud1;
unsigned int cloud1_counter = 0;
sensor_msgs::PointCloud2 cloud2;
unsigned int cloud2_counter = 0;

pcl::PointCloud<pcl::PointXYZ> cloud2_calibrated;
sensor_msgs::PointCloud2 cloud2_calibrated_msg;

ros::Publisher cloud1_pub;
ros::Publisher cloud2_pub;

void cloud1_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg_ptr) {
  if (cloud1_counter == 0) {
    cloud1_counter++;
    ROS_INFO_STREAM("Received first cloud1. Throwing away..");
  } else {
    if (cloud1_counter == 1) {
      cloud1 = *cloud_msg_ptr;
      cloud1_counter++;
      ROS_INFO_STREAM("Received second cloud1.");
    }
  }
}

void cloud2_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg_ptr) {
  if (cloud2_counter == 0) {
    cloud2_counter++;
    ROS_INFO_STREAM("Received first cloud2. Throwing away..");
  } else {
    if (cloud2_counter == 1) {
      cloud2 = *cloud_msg_ptr;
      cloud2_counter++;
      ROS_INFO_STREAM("Received second cloud2.");
      cloud2_calibrated_msg = cloud2;
    }
  }
}

void applyCalibration(const std_msgs::Float64MultiArrayConstPtr& array_ptr) {
  std::vector<double> calibration_vec = array_ptr->data;
  if (calibration_vec.size() == 6) {
    Eigen::Affine3d calibration(
          Eigen::AngleAxisd(calibration_vec[5], Eigen::Vector3d::UnitZ())
        * Eigen::AngleAxisd(calibration_vec[4], Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(calibration_vec[3], Eigen::Vector3d::UnitX())
    );
    calibration.translation() = Eigen::Vector3d(calibration_vec[0], calibration_vec[1], calibration_vec[2]);
    pcl::fromROSMsg(cloud2, cloud2_calibrated);
    pcl::transformPointCloud(cloud2_calibrated, cloud2_calibrated, calibration);
    pcl::toROSMsg(cloud2_calibrated, cloud2_calibrated_msg);
    cloud2_calibrated_msg.header.frame_id = cloud2.header.frame_id;
    ROS_INFO_STREAM("Received new calibration data:\n" <<
                    "roll: " << calibration_vec[3] << ", pitch: " << calibration_vec[4] << ", yaw: " << calibration_vec[5] << "\n" <<
                    "x: " << calibration_vec[0] << ", y: " << calibration_vec[1] <<  ", z: " << calibration_vec[2]);

  } else {
    ROS_WARN_STREAM("Received calibration data with wrong size (" << calibration_vec.size() << "instead of 6).");
  }
}

void publishClouds() {
  cloud1.header.stamp = ros::Time::now();
  cloud2_calibrated_msg.header.stamp = ros::Time::now();
  cloud1_pub.publish(cloud1);
  cloud2_pub.publish(cloud2_calibrated_msg);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "calibration_test_node");
  ROS_INFO_STREAM("Calibration test node started. Waiting for point clouds.");

  ros::NodeHandle nh;

  ros::Subscriber cloud1_sub = nh.subscribe("cloud1", 10, &cloud1_cb);
  ros::Subscriber cloud2_sub = nh.subscribe("cloud2", 10, &cloud2_cb);

  ros::Subscriber apply_calibration_sub_ = nh.subscribe("apply_calibration", 10, &applyCalibration);
  
  cloud1_pub = nh.advertise<sensor_msgs::PointCloud2>("calibrated_cloud1", 1000);
  cloud2_pub = nh.advertise<sensor_msgs::PointCloud2>("calibrated_cloud2", 1000);

  ros::Rate rate(10);
  while (ros::ok() && (cloud1_counter < 2 || cloud2_counter < 2)) {
    ros::spinOnce();
    rate.sleep();
  }
  ROS_INFO_STREAM("Received both point clouds");

  while(ros::ok()) {
    ros::spinOnce();
    publishClouds();
    rate.sleep();
  }

  return 0;
}
