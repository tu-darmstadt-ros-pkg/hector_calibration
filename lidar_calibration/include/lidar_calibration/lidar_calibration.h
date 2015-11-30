#ifndef LIDAR_CALIBRATION_H
#define LIDAR_CALIBRATION_H

// standard

// pcl
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/crop_box.h>

// ros
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>

// ceres solver
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <lidar_calibration/point_plane_error.h>

namespace hector_calibration {

class LidarCalibration {
public:
  struct Calibration {
    Calibration() {
      y = 0;
      z = 0;
      roll = 0;
      pitch = 0;
      yaw = 0;
    }

    std::string to_string() {
      std::stringstream ss;
      ss << "[" << y << ", " << z << "; " << roll << ", " << pitch << ", " << yaw << "]";
      return ss.str();
    }

    double y;
    double z;

    double roll;
    double pitch;
    double yaw;
  };

  struct CalibrationOptions {
    CalibrationOptions() {
      max_iterations = 15;
      for (unsigned int i = 0; i < 5; i++) {
        change_threshold[i] = 0;
      }
    }

    unsigned int max_iterations;
    double change_threshold[5];
    Calibration init_calibration;
  };

  LidarCalibration(const ros::NodeHandle& nh);

  void set_options(CalibrationOptions options);
  bool load_options_from_param_server(const ros::NodeHandle& nh);
  void calibrate();

private:
  void publish_neighbors(const pcl::PointCloud<pcl::PointXYZ>& cloud1, const pcl::PointCloud<pcl::PointXYZ>& cloud2, const std::vector<int>& mapping) const;

  void applyCalibration(pcl::PointCloud<pcl::PointXYZ>& cloud1, pcl::PointCloud<pcl::PointXYZ>& cloud2, const Calibration& calibration) const;
  pcl::PointCloud<pcl::PointNormal> computeNormals(const pcl::PointCloud<pcl::PointXYZ>& cloud) const;
  std::vector<int> findNeighbors(const pcl::PointCloud<pcl::PointXYZ> &cloud1, const pcl::PointCloud<pcl::PointXYZ> &cloud2) const;
  Calibration optimize_calibration(const pcl::PointCloud<pcl::PointXYZ> &cloud1, const pcl::PointCloud<pcl::PointXYZ> &cloud2, const pcl::PointCloud<pcl::PointNormal>& normals, const std::vector<int> neighbor_mapping) const;

  bool check_convergence(const Calibration& prev_calibration, const Calibration& current_calibration) const;

  CalibrationOptions options_;

  ros::NodeHandle nh_;
  ros::Publisher mls_cloud_pub_;
  ros::Publisher results_pub_;
  ros::Publisher neighbor_pub_;

  pcl::PointCloud<pcl::PointXYZ> cloud1_;
  pcl::PointCloud<pcl::PointXYZ> cloud2_;
};

}

#endif
