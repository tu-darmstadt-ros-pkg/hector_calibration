#include <multi_lidar_calibration/multi_lidar_calibration.h>

namespace hector_calibration {
namespace lidar_calibration {
Eigen::Affine3d
MultiLidarCalibration::calibrate(pcl::PointCloud<pcl::PointXYZ> cloud1,
                                 pcl::PointCloud<pcl::PointXYZ> cloud2)
{
  preprocessClouds(cloud1, cloud2);
  std::map<unsigned int, unsigned int> neighbor_mapping = findNeighbors(cloud1, cloud2, 0.1);
  std::vector<WeightedNormal> normals = computeNormals(cloud1, 0.07);
  return optimize(cloud1, cloud2, normals, neighbor_mapping);
}

Eigen::Affine3d
MultiLidarCalibration::calibrate(const sensor_msgs::PointCloud2& cloud1_msg,
                                 const sensor_msgs::PointCloud2& cloud2_msg)
{
  if (cloud1_msg.header.frame_id != cloud2_msg.header.frame_id) {
    ROS_ERROR_STREAM("Frame of cloud 1 (" << cloud1_msg.header.frame_id <<
                     ") doesn't match frame of cloud 2 (" << cloud2_msg.header.frame_id << "). Aborting.");
    return Eigen::Affine3d::Identity();
  }

  pcl::PointCloud<pcl::PointXYZ> cloud1;
  pcl::PointCloud<pcl::PointXYZ> cloud2;
  pcl::fromROSMsg(cloud1_msg, cloud1);
  pcl::fromROSMsg(cloud2_msg, cloud2);
  return calibrate(cloud1, cloud2);
}

void MultiLidarCalibration::preprocessClouds(pcl::PointCloud<pcl::PointXYZ>& cloud1,
                                             pcl::PointCloud<pcl::PointXYZ>& cloud2)

{
  cropCloud(cloud1, 1);
  cropCloud(cloud2, 1);
  downsampleCloud(cloud1, 0.01f);
  downsampleCloud(cloud2, 0.01f);
}

void MultiLidarCalibration::cropCloud(pcl::PointCloud<pcl::PointXYZ>& cloud, double distance) {

}

void MultiLidarCalibration::downsampleCloud(pcl::PointCloud<pcl::PointXYZ>& cloud, float leaf_size) {

}

Eigen::Affine3d
MultiLidarCalibration::optimize(pcl::PointCloud<pcl::PointXYZ>& cloud1,
              pcl::PointCloud<pcl::PointXYZ>& cloud2,
              std::vector<WeightedNormal> &normals,
              std::map<unsigned int, unsigned int> &mapping)
{
  if (cloud1.size() != normals.size()) {
    ROS_ERROR_STREAM("Size of cloud1 (" << cloud1.size() << ") doesn't match size of normals (" << normals.size() << ").");
    return Eigen::Affine3d::Identity();
  }

  ceres::Problem problem;

  double rotation[3];
  double translation[3];
  for (unsigned int i = 0; i < 3; i++) {
    rotation[i] = 0;
    translation[i] = 0;
  }

  unsigned int residual_count = 0;
  for(std::map<unsigned int, unsigned int>::const_iterator it = mapping.begin(); it != mapping.end(); it++) {
    unsigned int x1_index = it->first;
    unsigned int x2_index = it->second;
    Eigen::Vector3d x1(cloud1[x1_index].x, cloud1[x1_index].y, cloud1[x1_index].z);
    Eigen::Vector3d x2(cloud2[x2_index].x, cloud2[x2_index].y, cloud2[x2_index].z);

    ceres::CostFunction* cost_function = LidarPoseError::Create(x1, x2, normals[x1_index]);

    problem.AddResidualBlock(cost_function, NULL, rotation, translation);
    residual_count++;
  }
  ROS_INFO_STREAM("Number of residuals: " << residual_count);

  ceres::Solver::Options options;
  //options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.BriefReport() << "\n";

  Eigen::Affine3d calibration(
        Eigen::AngleAxisd(rotation[2], Eigen::Vector3d::UnitZ())
      * Eigen::AngleAxisd(rotation[1], Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(rotation[0], Eigen::Vector3d::UnitX())
  );
  calibration.translation() = Eigen::Vector3d(translation[0], translation[1], translation[2]);


  ROS_INFO_STREAM("Optimization result:\n" <<
                  "roll: " << rotation[0] << ", pitch: " << rotation[1] << ", yaw: " << rotation[2] << "\n" <<
                  "x: " << translation[0] << ", y: " << translation[1] <<  ", z: " << translation[2]);
  return calibration;
}



}
}
