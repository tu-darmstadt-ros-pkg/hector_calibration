//=================================================================================================
// Copyright (c) 2012, Martin Oehler, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#ifndef POINT_PLANE_ERROR_H
#define POINT_PLANE_ERROR_H

#include <ceres/ceres.h>
#include <pcl/point_types.h>

namespace hector_calibration {

namespace lidar_calibration {

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

template<typename T>
void printVector(const Vector3T<T>& vec, std::string prefix) {
  if (ceres::IsNaN(vec(0)) || ceres::IsNaN(vec(1)) || ceres::IsNaN(vec(2)))
    std::cout << prefix << ": [" << vec(0) << ", " << vec(1) << ", " << vec(2) << "]" << std::endl;
}

struct PointPlaneError {
  PointPlaneError(const LaserPoint<double>& s1, const LaserPoint<double>& s2, const WeightedNormal& normal) {
    s1_ = s1;
    s2_ = s2;
    normal_ = normal;
  }

  template<typename T>
  bool operator()(const T* const rpy_rotation, const T* const translation, T* residuals) const{
    // residual = n' * (Rx*s1 - Rx*H*s2)
    Affine3T<T> calibration(Eigen::AngleAxis<T>(rpy_rotation[2], Vector3T<T>::UnitZ())
        * Eigen::AngleAxis<T>(rpy_rotation[1], Vector3T<T>::UnitY())
        * Eigen::AngleAxis<T>(rpy_rotation[0], Vector3T<T>::UnitX()));
    calibration.translation() = Vector3T<T>(translation[0], translation[1], translation[2]);

    LaserPoint<T> s1t(s1_);
    LaserPoint<T> s2t(s2_);

    Vector3T<T> nt(T(normal_.normal(0)), T(normal_.normal(1)), T(normal_.normal(2)));

    Vector3T<T> x1 = s1t.getInActuatorFrame(Affine3T<T>::Identity());
    Vector3T<T> x2 = s2t.getInActuatorFrame(calibration);

    printVector<T>(nt, "nt");
    printVector<T>(x1, "x1");
    printVector<T>(x2, "x2");
    //std::cout << "residual: " << residuals[0] << std::endl;
//    std::cout << std::endl;

    residuals[0] = T(normal_.weight) * nt.transpose() * (x1 - x2);

    return true;
  }

  static ceres::CostFunction* Create(const LaserPoint<double>& s1, const LaserPoint<double>& s2, const WeightedNormal& normal)
  {
    ceres::CostFunction* cost_function =
        new ceres::AutoDiffCostFunction<PointPlaneError, 1, 3, 3>(
          new PointPlaneError(s1, s2, normal));

    return cost_function;
  }

  LaserPoint<double> s1_;
  LaserPoint<double> s2_;
  WeightedNormal normal_;
};

}
}

#endif
