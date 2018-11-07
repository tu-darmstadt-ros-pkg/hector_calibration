#ifndef HECTOR_CAMERA_LIDAR_CALIBRATION_NUMERIC_DIFF_MI_COST
#define HECTOR_CAMERA_LIDAR_CALIBRATION_NUMERIC_DIFF_MI_COST

#include <ceres/ceres.h>
#include <hector_camera_lidar_calibration/cost_function/mutual_information_cost.h>

namespace hector_calibration {
namespace hector_camera_lidar_calibration {

class NumericDiffMICost {
public:
  NumericDiffMICost(const std::vector<hector_calibration_msgs::CameraLidarCalibrationData>& calibration_data,
                                   const camera_model::CameraModelLoader& camera_model, int bin_fraction, int scan_sample_size);

  bool operator()(const double* const parameters, double* cost) const;

  static ceres::CostFunction* Create(const std::vector<hector_calibration_msgs::CameraLidarCalibrationData>& calibration_data,
                                     const camera_model::CameraModelLoader& camera_model, int bin_fraction, int scan_sample_size) {
    ceres::CostFunction* cost_function =
        new ceres::NumericDiffCostFunction<NumericDiffMICost, ceres::FORWARD, 1, 6>(
          new NumericDiffMICost(calibration_data, camera_model, bin_fraction, scan_sample_size));

    return cost_function;
  }

private:
  MutualInformationCost mi_cost_;

};

}
}

#endif
