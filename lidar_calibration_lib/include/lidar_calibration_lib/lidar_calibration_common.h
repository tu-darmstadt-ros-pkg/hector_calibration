#ifndef LIDAR_CALIBRATION_COMMON_H
#define LIDAR_CALIBRATION_COMMON_H

// pcl
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>

// pcl vis
//#include <pcl/visualization/pcl_visualizer.h>

// ros
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>

namespace hector_calibration {

namespace lidar_calibration {

  struct WeightedNormal {
    WeightedNormal() {
      weight = 1;
    }

    WeightedNormal(Eigen::Vector3d _normal, double _weight) {
      normal = _normal;
      weight = _weight;
    }

    Eigen::Vector3d normal;
    double weight;
  };

  double normalizeAngle(double angle);
  template<typename T> bool isValidPoint(const T& point);
  bool isValidCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud);
  template<typename T> pcl::PointCloud<T> removeInvalidPoints(pcl::PointCloud<T>& cloud);
  template <class Iter, class Incr> void safe_advance(Iter& curr, const Iter& end, Incr n);
  void nanInfToZero(WeightedNormal& normal);

  void publishCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud, const ros::Publisher& pub, std::string frame);
  void publishCloud(sensor_msgs::PointCloud2& cloud, const ros::Publisher& pub, std::string frame);


  std::map<unsigned int, unsigned int> findNeighbors(const pcl::PointCloud<pcl::PointXYZ> &cloud1,
                                                     const pcl::PointCloud<pcl::PointXYZ> &cloud2, double max_sqr_dist = 0.1);
  void publishNeighbors(const pcl::PointCloud<pcl::PointXYZ>& cloud1,
                         const pcl::PointCloud<pcl::PointXYZ>& cloud2,
                         const std::map<unsigned int, unsigned int>& mapping, ros::Publisher &pub, std::string frame, unsigned int number_of_markers = 100);

  std::vector<WeightedNormal> computeNormals(const pcl::PointCloud<pcl::PointXYZ>& cloud, double radius = 0.07);
//  void visualizeNormals(const pcl::PointCloud<pcl::PointXYZ>& cloud, std::vector<WeightedNormal> &normals);
  void visualizePlanarity(const pcl::PointCloud<pcl::PointXYZ> &cloud, const std::vector<WeightedNormal> &normals, ros::Publisher &pub, std::string frame);

}
}

#endif
