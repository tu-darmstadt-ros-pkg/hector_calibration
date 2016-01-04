#ifndef LIDAR_CALIBRATION_H
#define LIDAR_CALIBRATION_H

#include <lidar_calibration/RequestScans.h>

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
#include <pcl/features/normal_3d_omp.h>

// pcl vis
#include <pcl/visualization/pcl_visualizer.h>

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

    static constexpr double NUM_FREE_PARAMS = 5;

    Calibration(double _y, double _z, double _roll, double _pitch, double _yaw) {
      y = _y;
      z = _z;
      roll = _roll;
      pitch = _pitch;
      yaw = _yaw;
    }

    std::string toString() {
      std::stringstream ss;
      ss << "[y=" << y << ", z=" << z << "; roll=" << roll << ", pitch=" << pitch << ", yaw=" << yaw << "]";
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

    double computeError(const Calibration& rhs) {

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
      max_iterations = 20;
      max_sqrt_neighbor_dist = 0.1;
      sqrt_convergence_diff_thres = 1e-6;
    }

    unsigned int max_iterations;
    double max_sqrt_neighbor_dist;
    double sqrt_convergence_diff_thres;
    Calibration init_calibration;
  };

  LidarCalibration(const ros::NodeHandle& nh);

  void setOptions(CalibrationOptions options);
  bool loadOptionsFromParamServer(const ros::NodeHandle& nh);
  void calibrate();
  void setManualMode(bool manual);
  void setPeriodicPublishing(bool status, double period);
  void enableNormalVisualization(bool normals);

private:
  void publishResults();
  void timerCallback(const ros::TimerEvent&);
  void publishNeighbors(const pcl::PointCloud<pcl::PointXYZ>& cloud1,
                         const pcl::PointCloud<pcl::PointXYZ>& cloud2,
                         const std::map<unsigned int, unsigned int>& mapping) const;

  void requestScans(std::vector<LaserPoint<double> >& scan1,
                    std::vector<LaserPoint<double> >& scan2);
  std::vector<LaserPoint<double> > msgToLaserPoints(const sensor_msgs::PointCloud2& scan, const std_msgs::Float64MultiArray& angles);

  std::vector<LaserPoint<double> > cropCloud(const std::vector<LaserPoint<double> >& scan, double range);

  void applyCalibration(const std::vector<LaserPoint<double> >& scan1,
                        const std::vector<LaserPoint<double> >& scan2,
                        pcl::PointCloud<pcl::PointXYZ>& cloud1,
                        pcl::PointCloud<pcl::PointXYZ>& cloud2,
                        const Calibration& calibration);

  pcl::PointCloud<pcl::PointXYZ> laserToActuatorCloud(const std::vector<LaserPoint<double> >& laserpoints, const Calibration& calibration) const;
  std::vector<WeightedNormal> computeNormals(const pcl::PointCloud<pcl::PointXYZ>& cloud) const;
  void visualizeNormals(const pcl::PointCloud<pcl::PointXYZ>& cloud, const pcl::PointCloud<pcl::Normal>& normals) const;
  std::map<unsigned int, unsigned int> findNeighbors(const pcl::PointCloud<pcl::PointXYZ> &cloud1,
                                                     const pcl::PointCloud<pcl::PointXYZ> &cloud2) const;
  Calibration optimizeCalibration(const std::vector<LaserPoint<double> >& scan1,
                                   const std::vector<LaserPoint<double> >& scan2,
                                   const Calibration& current_calibration,
                                   const std::vector<WeightedNormal> & normals,
                                   const std::map<unsigned int, unsigned int> neighbor_mapping) const;

  bool checkConvergence(const Calibration& prev_calibration, const Calibration& current_calibration) const;
  bool maxIterationsReached(unsigned int current_iterations) const;

  CalibrationOptions options_;

  ros::NodeHandle nh_;
  ros::Publisher cloud1_pub_;
  ros::Publisher cloud2_pub_;
  ros::Publisher neighbor_pub_;

  sensor_msgs::PointCloud2 cloud1_msg_;
  sensor_msgs::PointCloud2 cloud2_msg_;

  std::string actuator_frame_;

  ros::ServiceClient request_scans_client_;
  ros::ServiceClient reset_clouds_client_;

  bool manual_mode_;
  bool vis_normals_;
  ros::Timer timer_;
};

}

#endif

/*** TODO ***/
/*
 * 2. Visualize normals
 * 3. Adaptive k for normals
*/
