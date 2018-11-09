#ifndef HECTOR_CAMERA_LIDAR_CALIBRATION_FIRST_ORDER_MI_COST
#define HECTOR_CAMERA_LIDAR_CALIBRATION_FIRST_ORDER_MI_COST

#include <ceres/ceres.h>
#include <hector_camera_lidar_calibration/cost_function/numeric_diff_mi_cost.h>


namespace hector_calibration {
namespace hector_camera_lidar_calibration {

class FirstOrderMICost : public ceres::FirstOrderFunction {
public:
  FirstOrderMICost(const std::vector<hector_calibration_msgs::CameraLidarCalibrationData>& calibration_data,
                   const camera_model::CameraModelLoader& camera_model, int bin_fraction, int scan_sample_size);
  ~FirstOrderMICost();

  virtual bool Evaluate(const double* parameters, double* cost, double* gradient) const;
  ceres::CostFunction* getCostFunction();
private:
  virtual int NumParameters() const { return 6; }

  ceres::CostFunction* cost_function_;
};

}
}

#endif
