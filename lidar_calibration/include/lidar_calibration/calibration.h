//=================================================================================================
// Copyright (c) 2016 Martin Oehler, TU Darmstadt
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

#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <Eigen/Eigen>

namespace hector_calibration {

namespace lidar_calibration {

struct Calibration {
  Calibration() {
    x = 0;
    y = 0;
    z = 0;
    roll = 0;
    pitch = 0;
    yaw = 0;
  }

  static constexpr double NUM_FREE_PARAMS = 6;

  Calibration(double _x, double _y, double _z, double _roll, double _pitch, double _yaw) {
    x = _x;
    y = _y;
    z = _z;
    roll = _roll;
    pitch = _pitch;
    yaw = _yaw;
  }

  Calibration(const Eigen::Affine3d& transformation) {
    Eigen::Vector3d rpy = transformation.linear().eulerAngles(2, 1, 0);
    Eigen::Vector3d xyz = transformation.translation();

    x = xyz(0);
    y = xyz(1);
    z = xyz(2);

    roll = rpy(2);
    pitch = rpy(1);
    yaw = rpy(0);
  }

  std::string toString() {
    std::stringstream ss;
    ss << "[x=" << x << ", y=" << y << ", z=" << z << "; roll=" << roll << ", pitch=" << pitch << ", yaw=" << yaw << "]";
    return ss.str();
  }

  double operator()(int n) const {
    if (n == 0) return x;
    if (n == 1) return y;
    if (n == 2) return z;
    if (n == 3) return roll;
    if (n == 4) return pitch;
    if (n == 5) return yaw;
    return 0.0;
  }

  Eigen::Affine3d getTransform() const {
    Eigen::Affine3d transform(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
        * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
    transform.translation() = Eigen::Vector3d(x, y, z);
    return transform;
  }

  Calibration applyTransform(const Eigen::Affine3d& transform) {
    Eigen::Affine3d rotated = transform * getTransform();
    return Calibration(rotated);
  }

  Calibration applyRotationOffset(const Eigen::Affine3d offset) {
    Eigen::Affine3d rotated = offset * getTransform() * offset.inverse();
    return Calibration(rotated);
  }

  void threshold(double thres) {
    if (std::abs(x) < thres) {
      x = 0;
    }
    if (std::abs(y) < thres) {
      y = 0;
    }
    if (std::abs(z) < thres) {
      z = 0;
    }
    if (std::abs(roll) < thres) {
      roll = 0;
    }
    if (std::abs(pitch) < thres) {
      pitch = 0;
    }
    if (std::abs(yaw) < thres) {
      yaw = 0;
    }
  }

  double x;
  double y;
  double z;

  double roll;
  double pitch;
  double yaw;
};

}

}

#endif
