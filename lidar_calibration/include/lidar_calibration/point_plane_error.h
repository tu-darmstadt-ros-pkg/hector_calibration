#ifndef POINT_PLANE_ERROR_H
#define POINT_PLANE_ERROR_H

#include <ceres/ceres.h>
#include <pcl/point_types.h>

namespace hector_calibration {

template<typename T>
using Vector3T = Eigen::Matrix<T, 3, 1>;

template<typename T>
using Affine3T = Eigen::Transform<T, 3, Eigen::Affine>;

template<typename Scalar>
struct LaserPoint {
  LaserPoint() {}

  LaserPoint(const LaserPoint<double>& lpd) {
    point = Vector3T<Scalar>(Scalar(lpd.point(0)), Scalar(lpd.point(1)), Scalar(lpd.point(2)));
    angle = Scalar(lpd.angle);
  }

  LaserPoint(Vector3T<Scalar> _point, Scalar _angle) {
    point = _point;
    angle = _angle;
  }

  Vector3T<Scalar> getInActuatorFrame(Affine3T<Scalar> calibration) const {
    Vector3T<Scalar> actuator_frame = Eigen::AngleAxis<Scalar>(angle, Vector3T<Scalar>::UnitX()) * calibration * point;
    return actuator_frame;
  }

  Vector3T<Scalar> point;
  Scalar angle;
};

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

struct PointPlaneError {
  PointPlaneError(const LaserPoint<double>& s1, const LaserPoint<double>& s2, const WeightedNormal& normal) {
    s1_ = s1;
    s2_ = s2;
    normal_ = normal;
  }

  template<typename T>
  bool operator()(const T* const rpy_rotation, const T* const translation, T* residuals) const{
    // residual = n' * (Rz*H*s1 - Rz*H*s2)
    Affine3T<T> calibration(Eigen::AngleAxis<T>(T(rpy_rotation[1]), Vector3T<T>::UnitZ())
        * Eigen::AngleAxis<T>(T(rpy_rotation[0]), Vector3T<T>::UnitY()));
    calibration.translation() = Vector3T<T>(T(0.0), T(translation[0]), T(translation[1]));

    LaserPoint<T> s1t(s1_);
    LaserPoint<T> s2t(s2_);

    Vector3T<T> nt(T(normal_.normal(0)), T(normal_.normal(1)), T(normal_.normal(2)));

    Vector3T<T> x1 = s1t.getInActuatorFrame(calibration);
    Vector3T<T> x2 = s2t.getInActuatorFrame(calibration);

    residuals[0] = T(normal_.weight) * nt.transpose() * (x1 - x2);

    return true;
  }

  static ceres::CostFunction* Create(const LaserPoint<double>& s1, const LaserPoint<double>& s2, const WeightedNormal& normal)
  {
    ceres::CostFunction* cost_function =
        new ceres::AutoDiffCostFunction<PointPlaneError, 1, 2, 2>(
          new PointPlaneError(s1, s2, normal));

    return cost_function;
  }

  LaserPoint<double> s1_;
  LaserPoint<double> s2_;
  WeightedNormal normal_;
};

}

#endif
