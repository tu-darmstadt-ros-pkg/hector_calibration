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
    ss << "[x =" << x << ", y=" << y << ", z=" << z << "; roll=" << roll << ", pitch=" << pitch << ", yaw=" << yaw << "]";
    return ss.str();
  }

  double operator()(int n) const {
    if (n == 0) return y;
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

  Calibration applyRotationOffset(const Eigen::Affine3d offset) {
    Eigen::Affine3d calibration = getTransform();
    Eigen::Affine3d rotated = offset * calibration * offset.inverse();
    return Calibration(rotated);
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
