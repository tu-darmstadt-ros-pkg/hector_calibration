#include <hector_camera_lidar_calibration/cost_function/numeric_diff_mi_cost.h>

namespace hector_calibration {
namespace hector_camera_lidar_calibration {

NumericDiffMICost::NumericDiffMICost(const std::vector<hector_calibration_msgs::CameraLidarCalibrationData>& calibration_data,
                                                                   const camera_model::CameraModelLoader& camera_model, int bin_fraction, int scan_sample_size)
  : mi_cost_(calibration_data, camera_model, bin_fraction, scan_sample_size)
{

}

bool NumericDiffMICost::operator()(const double* const parameters, double* cost) const {
  ROS_INFO_STREAM("Evaluation with parameters: " << parametersToString(parameters));
  Eigen::Affine3d calibration(Eigen::AngleAxisd(parameters[5], Eigen::Vector3d::UnitZ())
      * Eigen::AngleAxisd(parameters[4], Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(parameters[3], Eigen::Vector3d::UnitX()));
  calibration.translation() = Eigen::Vector3d(parameters[0], parameters[1], parameters[2]);

  cost[0] = mi_cost_.computeMutualInformationCost(calibration);
  return true;
}

}
}
