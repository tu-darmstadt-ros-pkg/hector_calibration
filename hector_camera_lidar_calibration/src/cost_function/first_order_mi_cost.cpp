#include <hector_camera_lidar_calibration/cost_function/first_order_mi_cost.h>

namespace hector_calibration {
namespace hector_camera_lidar_calibration {

FirstOrderMICost::FirstOrderMICost(const std::vector<hector_calibration_msgs::CameraLidarCalibrationData>& calibration_data,
                       const camera_model::CameraModelLoader& camera_model, int bin_fraction, int scan_sample_size) {
  cost_function_ = NumericDiffMICost::Create(calibration_data, camera_model, bin_fraction, scan_sample_size);
}

FirstOrderMICost::~FirstOrderMICost() {
  delete cost_function_;
}

bool FirstOrderMICost::Evaluate(const double* parameters, double* cost, double* gradient) const {
  double const *const *parameters_ptr = &parameters;
  if (gradient != NULL) {
    double **jacobian_ptr = &gradient;
    if (!cost_function_->Evaluate(parameters_ptr, cost, jacobian_ptr)) {
      return false;
    }
    std::cout << "Current cost: " << cost[0] << std::endl;
    std::cout << " --- gradient: ";
    for (int i = 0; i < NumParameters(); i++) {
      std::cout << gradient[i] << ", ";
    }
    std::cout << std::endl;
  } else {
    if (!cost_function_->Evaluate(parameters_ptr, cost, NULL)) {
      return false;
    }
    std::cout << "Current cost: " << cost[0] << std::endl;
  }

  return true;
}

ceres::CostFunction*FirstOrderMICost::getCostFunction()
{
  return cost_function_;
}

}
}
