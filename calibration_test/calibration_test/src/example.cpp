#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

ros::Publisher pub;
pcl::PointCloud<pcl::PointXYZI> sum_cloud;
unsigned int counter = 0;
bool init = false;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  pcl::PointCloud<pcl::PointXYZI> cloud;

  if (init) {
    pcl::fromROSMsg(*cloud_msg, sum_cloud);
  } else {
    // Convert to PCL data type
    pcl::fromROSMsg(*cloud_msg, cloud);
    sum_cloud.header.frame_id = cloud.header.frame_id;
    sum_cloud += cloud;
  }


  if (counter == 200) {
    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(sum_cloud, output);

    // Publish the data
    pub.publish (output);

    counter = 0;
  } else {
    counter++;
  }
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe("/scan_cloud_filtered", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2>("scan_cloud_accumulated", 1);

  // Spin
  ros::spin ();
}
