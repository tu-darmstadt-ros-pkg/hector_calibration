#ifndef LIDAR_EXTRINSIC_CALIBRATION_H
#define LIDAR_EXTRINSIC_CALIBRATION_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>

#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

namespace hector_calibration {
  class LidarExtrinsicCalibration {
  public:
    LidarExtrinsicCalibration(ros::NodeHandle& nh);

    void calibrateGround();
    void publishLastResult();
  private:
    void pointCloudCb(const sensor_msgs::PointCloud2ConstPtr& cloud_ptr);
    Eigen::Affine3d getTransform(std::string frame_base, std::string frame_target) const;

    tf::TransformListener tfl_;

    ros::NodeHandle nh_;
    ros::Subscriber cloud_sub_;
    ros::Publisher result_pub_;
    ros::Publisher ground_plane_pub_;

    sensor_msgs::PointCloud2ConstPtr last_cloud_ptr_;
    bool first_cloud_;

    std::string ground_frame_;
    ros::Duration tf_wait_duration_;
  };
}
#endif
