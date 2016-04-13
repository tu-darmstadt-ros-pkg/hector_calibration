#include <lidar_calibration_lib/lidar_calibration_common.h>

namespace hector_calibration {
namespace lidar_calibration {

template<typename T>
bool isValidPoint(const T& point) {
  for (unsigned int i = 0; i < 3; i++) {
    if (std::isnan(point.data[i]) || std::isinf(point.data[i])) {
      return false;
    }
  }
  return true;
}

bool isValidCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud) {
  for (unsigned int i = 0; i < cloud.size(); i++) {
    if (!isValidPoint(cloud[i])) {
      return false;
    }
  }
  return true;
}

template<typename T>
pcl::PointCloud<T> removeInvalidPoints(pcl::PointCloud<T>& cloud) {
  pcl::PointCloud<T> cleaned_cloud;
  unsigned int invalid_counter = 0;
  for (unsigned int i = 0; i < cloud.size(); i++) {
    if (isValidPoint<T>(cloud[i])) {
      cleaned_cloud.push_back(cloud[i]);
    } else {
      invalid_counter++;
    }
  }
  ROS_INFO_STREAM("Removed " << invalid_counter << " invalid points");
  return cleaned_cloud;
}

template <class Iter, class Incr>
void safe_advance(Iter& curr, const Iter& end, Incr n)
{
  size_t remaining(std::distance(curr, end));
  if (remaining < n)
  {
    n = remaining;
  }
  std::advance(curr, n);
}

void fixNanInf(WeightedNormal& normal) {
  if (std::isnan(normal.normal.x()) || std::isnan(normal.normal.y()) || std::isnan(normal.normal.z())
      || std::isinf(normal.normal.x()) || std::isinf(normal.normal.y()) || std::isinf(normal.normal.z())) {
    normal.normal = Eigen::Vector3d::Zero();
    normal.weight = 0;
  }
}


void publishCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud, const ros::Publisher& pub, std::string frame) {
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  publishCloud(cloud_msg, pub, frame);
}

void publishCloud(sensor_msgs::PointCloud2& cloud, const ros::Publisher& pub, std::string frame) {
  cloud.header.frame_id = frame;
  cloud.header.stamp = ros::Time::now();
  pub.publish(cloud);
}

std::map<unsigned int, unsigned int>
findNeighbors(const pcl::PointCloud<pcl::PointXYZ>& cloud1,
              const pcl::PointCloud<pcl::PointXYZ>& cloud2,
              double max_sqr_dist)
{
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2_ptr(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::copyPointCloud(cloud2, *cloud2_ptr);
  kdtree.setInputCloud(cloud2_ptr); // Search in second cloud to retrieve mapping from cloud1 -> cloud2

  std::vector<int> index(1);
  std::vector<float> sqrt_dist(1);

  std::map<unsigned int, unsigned int> mapping;
  for (unsigned int i = 0; i < cloud1.size(); i++) {
    if (kdtree.nearestKSearch(cloud1[i], 1, index, sqrt_dist) > 0) { // Check if number of found neighbours > 0
      if (sqrt_dist[0] <= max_sqr_dist) { // Only insert if smaller than max distance
        std::pair<unsigned int, unsigned int> pair(i, index[0]);
        mapping.insert(pair); // Mapping from cloud1 index to cloud2 index
      }
    }
  }
  ROS_INFO_STREAM("Found " << mapping.size() << " neighbor matches.");
  return mapping;
}

void publishNeighbors(const pcl::PointCloud<pcl::PointXYZ>& cloud1,
                      const pcl::PointCloud<pcl::PointXYZ>& cloud2,
                      const std::map<unsigned int, unsigned int> &mapping,
                      ros::Publisher& pub,
                      std::string frame,
                      unsigned int number_of_markers)
{
  visualization_msgs::MarkerArray marker_array;
  unsigned int step = floor(mapping.size() / number_of_markers);
  unsigned int id_cnt = 0;
  for (std::map<unsigned int, unsigned int>::const_iterator it = mapping.begin();
       it != mapping.end();
       safe_advance<std::map<unsigned int, unsigned int>::const_iterator, unsigned int>(it, mapping.end(), step))
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame;
    marker.header.stamp = ros::Time::now();
    marker.ns = "neighbor_mapping";
    marker.id = id_cnt++;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;

    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    geometry_msgs::Point point1;
    geometry_msgs::Point point2;

    point1.x = (double) cloud1[it->first].x;
    point1.y = (double) cloud1[it->first].y;
    point1.z = (double) cloud1[it->first].z;

    point2.x = (double) cloud2[it->second].x;
    point2.y = (double) cloud2[it->second].y;
    point2.z = (double) cloud2[it->second].z;
    marker.points.push_back(point1);
    marker.points.push_back(point2);
    marker_array.markers.push_back(marker);
  }
  pub.publish(marker_array);
}

std::vector<WeightedNormal> computeNormals(const pcl::PointCloud<pcl::PointXYZ>& cloud, double radius)
{
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::copyPointCloud(cloud, *cloud_ptr);
  kdtree.setInputCloud(cloud_ptr);

  std::vector<WeightedNormal> normals(cloud.size());
#ifdef _OPENMP
#pragma omp parallel for shared (normals, kdtree, cloud)
#endif
  for (unsigned int i = 0; i < cloud.size(); i++) {
    pcl::PointXYZ p = cloud[i];

    // Compute neighbors
    std::vector<int> indices;
    std::vector<float> sqrt_dist;
    kdtree.radiusSearch(p, radius, indices, sqrt_dist);

    double weight;
    Eigen::Vector4f plane_parameters;
    Eigen::Matrix3f covariance_matrix;
    Eigen::Vector4f xyz_centroid;
    if (indices.size () < 3 || pcl::computeMeanAndCovarianceMatrix (cloud, indices, covariance_matrix, xyz_centroid) == 0){
      plane_parameters.setConstant(0);
      weight = 0;
    } else {
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eig(covariance_matrix);
      const Eigen::Vector3f& eigen_values(eig.eigenvalues());

      weight = 2* (eigen_values(1) - eigen_values(0)) / eigen_values.sum();
      plane_parameters.block<3,1>(0, 0) = eig.eigenvectors().col(0);
      plane_parameters[3] = 1;
      pcl::flipNormalTowardsViewpoint(cloud[i], 0.0, 0.0, 0.0, plane_parameters);
    }

    WeightedNormal normal;
    normal.normal = Eigen::Vector3d(plane_parameters[0], plane_parameters[1], plane_parameters[2]);
    normal.weight = weight;
    fixNanInf(normal);
    normals[i] = normal;
  }

  return normals;
}

void visualizeNormals(const pcl::PointCloud<pcl::PointXYZ>& cloud,
                     std::vector<WeightedNormal>& normals)
{
  pcl::PointCloud<pcl::Normal> pcl_normals;
  pcl_normals.resize(cloud.size());
  for (unsigned int i = 0; i < normals.size(); i++) {
    pcl::Normal pcl_normal(normals[i].normal(0), normals[i].normal(1), normals[i].normal(2));
    pcl_normals[i] = pcl_normal;
  }
  pcl::visualization::PCLVisualizer viewer;
  viewer.setBackgroundColor(0,0,0);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::copyPointCloud(cloud, *cloud_ptr);
  viewer.addPointCloud<pcl::PointXYZ>(cloud_ptr, "Cloud");

  pcl::PointCloud<pcl::Normal>::Ptr normals_ptr(new pcl::PointCloud<pcl::Normal>());
  pcl::copyPointCloud(pcl_normals, *normals_ptr);
  viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud_ptr, normals_ptr, 10, 0.05, "Normals");
  viewer.addCoordinateSystem(1.0);
  viewer.initCameraParameters();
  ros::Rate rate(10);
  while (!viewer.wasStopped())
   {
     viewer.spinOnce (100);
     ros::spinOnce();
     rate.sleep();
   }
}

void visualizePlanarity(const pcl::PointCloud<pcl::PointXYZ> &cloud,
                        const std::vector<WeightedNormal> &normals,
                        ros::Publisher& pub,
                        std::string frame)
{
  if (normals.size() != cloud.size()) {
    ROS_ERROR_STREAM("Size of cloud (" << cloud.size() << ") doesn't match size of normals (" << normals.size() << ").");
    return;
  }
  double thres = 0.1;
  int id_cnt = 0;

  int step = 100;
  visualization_msgs::MarkerArray marker_array;
  for (unsigned int i = 0; i < normals.size(); i++) {
    if (normals[i].weight <= thres && normals[i].weight != 0) {
      if (step != 0) {
        step--;
        continue;
      }
      step = 100;
      visualization_msgs::Marker marker;
      marker.header.frame_id = frame;
      marker.header.stamp = ros::Time::now();
      marker.ns = "planarity";
      marker.id = id_cnt++;
      marker.type = visualization_msgs::Marker::ARROW;
      marker.action = visualization_msgs::Marker::ADD;

      marker.scale.x = 0.01;
      marker.scale.y = 0.01;
      marker.scale.z = 0.01;
      marker.color.a = 1.0;
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 1.0;

      geometry_msgs::Point point1;
      geometry_msgs::Point point2;

      point1.x = (double) cloud[i].x;
      point1.y = (double) cloud[i].y;
      point1.z = (double) cloud[i].z;

      point2.x = (double) cloud[i].x + 0.2* normals[i].normal.x();
      point2.y = (double) cloud[i].y + 0.2* normals[i].normal.y();
      point2.z = (double) cloud[i].z + 0.2* normals[i].normal.z();
      marker.points.push_back(point1);
      marker.points.push_back(point2);
      marker_array.markers.push_back(marker);
    }

  }
  //ROS_INFO_STREAM("Drawing " << id_cnt << " normals.");
  pub.publish(marker_array);
}

}
}
