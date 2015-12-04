#ifndef LIDAR_CALIBRATION_H
#define LIDAR_CALIBRATION_H

#include <lidar_calibration/ApplyCalibration.h>

// standard

// pcl
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/crop_box.h>
#include <pcl/features/normal_3d.h>

// ros
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>
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

    double operator()(int n) const {
      if (n == 0) return y;
      if (n == 1) return z;
      if (n == 2) return roll;
      if (n == 3) return pitch;
      if (n == 4) return yaw;
      return 0.0;
    }

    Eigen::Affine3d getTransform() const {
      Eigen::Affine3d transform(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
          * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
          * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
      transform.translation() = Eigen::Vector3d(0, y, z);
      return transform;
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
      max_sqrt_neighbor_dist = 0.1;
      for (unsigned int i = 0; i < 5; i++) {
        change_threshold[i] = 0;
      }
    }

    unsigned int max_iterations;
    double max_sqrt_neighbor_dist;
    double change_threshold[5];
    Calibration init_calibration;
  };

  LidarCalibration(const ros::NodeHandle& nh);

  void setOptions(CalibrationOptions options);
  bool loadOptionsFromParamServer(const ros::NodeHandle& nh);
  void calibrate();
  void setManualMode(bool manual);

private:
  void publish_neighbors(const pcl::PointCloud<pcl::PointXYZ>& cloud1,
                         const pcl::PointCloud<pcl::PointXYZ>& cloud2,
                         const std::map<unsigned int, unsigned int>& mapping) const;

  std::vector<LaserPoint<double> > crop_cloud(const std::vector<LaserPoint<double> >& scan, double range);

  void applyCalibration(const std::vector<LaserPoint<double> >& scan1,
                        const std::vector<LaserPoint<double> >& scan2,
                        pcl::PointCloud<pcl::PointXYZ>& cloud1,
                        pcl::PointCloud<pcl::PointXYZ>& cloud2,
                        const Calibration& calibration);

  pcl::PointCloud<pcl::PointXYZ> laserToActuatorCloud(const std::vector<LaserPoint<double> >& laserpoints, const Calibration& calibration) const;
  std::vector<WeightedNormal> computeNormals(const pcl::PointCloud<pcl::PointXYZ>& cloud) const;
  std::map<unsigned int, unsigned int> findNeighbors(const pcl::PointCloud<pcl::PointXYZ> &cloud1,
                                                     const pcl::PointCloud<pcl::PointXYZ> &cloud2) const;
  Calibration optimize_calibration(const std::vector<LaserPoint<double> >& scan1,
                                   const std::vector<LaserPoint<double> >& scan2,
                                   const Calibration& current_calibration,
                                   const std::vector<WeightedNormal> & normals,
                                   const std::map<unsigned int, unsigned int> neighbor_mapping) const;

  bool check_convergence(const Calibration& prev_calibration, const Calibration& current_calibration) const;

  CalibrationOptions options_;

  ros::NodeHandle nh_;
  ros::Publisher mls_cloud_pub_;
  ros::Publisher results_pub_;
  ros::Publisher neighbor_pub_;

  ros::ServiceClient apply_calibration_client_;
  ros::ServiceClient reset_clouds_client_;

  bool manual_mode_;
};

}

#endif
