//=================================================================================================
// Copyright (c) 2016, Martin Oehler, TU Darmstadt
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

#ifndef LIDAR_POSE_ERROR_H
#define LIDAR_POSE_ERROR_H

#include <ceres/ceres.h>
#include <pcl/point_types.h>

namespace hector_calibration {

namespace lidar_calibration {

template<typename T>
using Vector3T = Eigen::Matrix<T, 3, 1>;

template<typename T>
using Affine3T = Eigen::Transform<T, 3, Eigen::Affine>;

struct LidarPoseError {
  LidarPoseError(const Eigen::Vector3d x1, const Eigen::Vector3d& x2, const WeightedNormal& normal) {
    x1_ = x1;
    x2_ = x2;
    normal_ = normal;
  }

  template<typename T>
  bool operator()(const T* const rpy_rotation, const T* const translation, T* residuals) const{
    // residual = n' * (x1 - H*x2)
    Affine3T<T> calibration(
          Eigen::AngleAxis<T>(T(rpy_rotation[2]), Vector3T<T>::UnitZ())
        * Eigen::AngleAxis<T>(T(rpy_rotation[1]), Vector3T<T>::UnitY())
        * Eigen::AngleAxis<T>(T(rpy_rotation[0]), Vector3T<T>::UnitX())
    );
    calibration.translation() = Vector3T<T>(T(translation[0]), T(translation[1]), T(translation[2]));

    Vector3T<T> nt(T(normal_.normal(0)), T(normal_.normal(1)), T(normal_.normal(2)));

    Vector3T<T> x1(T(x1_(0)), T(x1_(1)), T(x1_(2)));
    Vector3T<T> x2(T(x2_(0)), T(x2_(1)), T(x2_(2)));

    residuals[0] = T(normal_.weight) * nt.transpose() * (x1 - calibration * x2);

    return true;
  }

  static ceres::CostFunction* Create(const Eigen::Vector3d x1, const Eigen::Vector3d& x2, const WeightedNormal& normal)
  {
    ceres::CostFunction* cost_function =
        new ceres::AutoDiffCostFunction<LidarPoseError, 1, 3, 3>(
          new LidarPoseError(x1, x2, normal));

    return cost_function;
  }

  Eigen::Vector3d x1_;
  Eigen::Vector3d x2_;
  WeightedNormal normal_;
};

}
}

#endif
