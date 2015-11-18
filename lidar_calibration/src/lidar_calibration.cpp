#include <lidar_calibration/lidar_calibration.h>

namespace hector_calibration {


void LidarCalibration::calibrate(pcl::PointCloud<pcl::PointXYZ>& cloud1,
                                 pcl::PointCloud<pcl::PointXYZ>& cloud2,
                                 const Calibration& init_calibration) {

  std::vector<Calibration> calibrations;
  calibrations.push_back(init_calibration);

  unsigned int iteration_counter = 0;
  do {
    applyCalibration(cloud1, cloud2, calibrations[calibrations.size()-1]);
    pcl::PointCloud<pcl::PointNormal> normals = computeNormals(cloud1);
    std::vector<int> neighbor_mapping = findNeighbors(cloud1, cloud2);

    calibrations.push_back(optimize_calibration(normals, neighbor_mapping));
    iteration_counter++;
  } while(iteration_counter < max_iterations_
          && (iteration_counter < 2 || !check_convergence(calibrations[calibrations.size()-2], calibrations[calibrations.size()-1])));
}

void LidarCalibration::applyCalibration(pcl::PointCloud<pcl::PointXYZ>& cloud1,
                                        pcl::PointCloud<pcl::PointXYZ>& cloud2,
                                        const Calibration& calibration) const {
  Eigen::Affine3d transform(Eigen::AngleAxisd(calibration.yaw, Eigen::Vector3d::UnitZ())
      * Eigen::AngleAxisd(calibration.pitch, Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(calibration.roll, Eigen::Vector3d::UnitX()));
  transform.translation() = Eigen::Vector3d(calibration.x, calibration.y, 0);
  pcl::transformPointCloud(cloud1, cloud1, transform); //Note: Can be used with cloud_in equal to cloud_out
  pcl::transformPointCloud(cloud2, cloud2, transform);
}

pcl::PointCloud<pcl::PointNormal>
LidarCalibration::computeNormals(const pcl::PointCloud<pcl::PointXYZ>& cloud) const{
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
  mls.setComputeNormals(true);

  pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_ptr(&cloud);
  mls.setInputCloud(cloud_ptr);
  mls.setPolynomialFit(true);
  mls.setSearchMethod(tree);
  mls.setSearchRadius(0.03);

  pcl::PointCloud<pcl::PointNormal> normals;
  mls.process(normals);

  return normals;
}

std::vector<int>
LidarCalibration::findNeighbors(const pcl::PointCloud<pcl::PointXYZ>& cloud1,
                                const pcl::PointCloud<pcl::PointXYZ>& cloud2) const {
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;  // Maybe optimize by using same tree as in normal estimation?
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud2_ptr(&cloud2);
  kdtree.setInputCloud(cloud2_ptr); // Search in second cloud to retrieve mapping from cloud1 -> cloud2

  std::vector<int> index(1);
  std::vector<float> sqrt_dist(1);

  std::vector<int> mapping(cloud1.size(), -1);
  for (unsigned int i = 0; i < cloud1.size(); i++) {
    if (kdtree.nearestKSearch(cloud1[i], 1, index, sqrt_dist) > 0) {
      mapping[i] = index[0];
    } else {
      mapping[i] = -1; // No neighbor found
    }
  }

  return mapping;
}

LidarCalibration::Calibration
LidarCalibration::optimize_calibration(const pcl::PointCloud<pcl::PointNormal>& normals,
                                       const std::vector<int> neighbor_mapping) const {
  Calibration calibration;

  // Use fancy google solver to optimize calibration
  return calibration;
}


bool LidarCalibration::check_convergence(const LidarCalibration::Calibration& prev_calibration,
                                         const LidarCalibration::Calibration& current_calibration) const {
  return false;
}

}
