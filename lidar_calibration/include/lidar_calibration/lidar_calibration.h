#ifndef LIDAR_CALIBRATION_H
#define LIDAR_CALIBRATION_H

// pcl
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>

#include <sensor_msgs/PointCloud2.h>

namespace hector_calibration {

class LidarCalibration {
public:
  struct Calibration {
    Calibration() {
      x = 0;
      y = 0;
      roll = 0;
      pitch = 0;
      yaw = 0;
    }

    double x;
    double y;

    double roll;
    double pitch;
    double yaw;
  };

  struct CalibrationOptions {
    CalibrationOptions() {
      max_iterations = 15;
      for (unsigned int i = 0; i < 6; i++) {
        change_threshold[i] = 0;
      }
    }

    unsigned int max_iterations;
    double change_threshold[5];
  };

  bool load_options_from_param_server(const ros::NodeHandle& nh);
  bool start_calibration();

private:
  void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_ptr);

  void calibrate(pcl::PointCloud<pcl::PointXYZ>& cloud1, pcl::PointCloud<pcl::PointXYZ>& cloud2, const Calibration& init_calibration);
  void applyCalibration(pcl::PointCloud<pcl::PointXYZ>& cloud1, pcl::PointCloud<pcl::PointXYZ>& cloud2, const Calibration& calibration) const;
  pcl::PointCloud<pcl::PointNormal> computeNormals(const pcl::PointCloud<pcl::PointXYZ>& cloud) const;
  std::vector<int> findNeighbors(const pcl::PointCloud<pcl::PointXYZ> &cloud1, const pcl::PointCloud<pcl::PointXYZ> &cloud2) const;
  Calibration optimize_calibration(const pcl::PointCloud<pcl::PointNormal>& normals, const std::vector<int> neighbor_mapping) const;

  bool check_convergence(const Calibration& prev_calibration, const Calibration& current_calibration) const;

  unsigned int max_iterations_;
};

}

#endif
