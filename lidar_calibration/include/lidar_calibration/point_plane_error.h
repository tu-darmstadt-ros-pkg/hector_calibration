#ifndef POINT_PLANE_ERROR_H
#define POINT_PLANE_ERROR_H

#include <ceres/ceres.h>
#include <pcl/point_types.h>

namespace hector_calibration {

struct PointPlaneError {
    PointPlaneError(const pcl::PointXYZ& x1, const pcl::PointXYZ& x2, const pcl::PointNormal& normal) {
      for (unsigned int i = 0; i < 3; i++) {
        x1_[i] = (double) x1.data[i];
        x2_[i] = (double) x2.data[i];
        normal_[i] = (double) normal.normal[i];
      }
  }

  template<typename T>
  bool operator()(const T* const rpy_rotation, const T* const translation, T* residuals) const{
    // residual = n' * (x1 - H* x2)

    // Translate x2
    T x2_t[3];
    for (unsigned int i = 0; i < 2; i++) {
      x2_t[i] = T(x2_[i]) + translation[i];
    }
    x2_t[2] = T(x2_[2]);

    // Rotate x2
    T rotation_matrix[9];
    ceres::EulerAnglesToRotationMatrix(rpy_rotation, 3, rotation_matrix);
    T quaternion[4];
    ceres::RotationMatrixToQuaternion(rotation_matrix, quaternion);
    T x2_r[3];
    ceres::QuaternionRotatePoint(quaternion, x2_t, x2_r);

    residuals[0] = T(0);
    for (unsigned int i = 0; i < 3; i++) {
      residuals[0] += T(normal_[i]) * (T(x1_[i]) - x2_r[i]);
    }

    return true;
  }

  static ceres::CostFunction* Create(const pcl::PointXYZ& x1, const pcl::PointXYZ& x2, const pcl::PointNormal& normal)
  {
    ceres::CostFunction* cost_function =
        new ceres::AutoDiffCostFunction<PointPlaneError, 1, 3, 2>(
          new PointPlaneError(x1, x2, normal));

    return cost_function;
  }

  double x1_[3];
  double x2_[3];
  double normal_[3];
};

}

#endif
