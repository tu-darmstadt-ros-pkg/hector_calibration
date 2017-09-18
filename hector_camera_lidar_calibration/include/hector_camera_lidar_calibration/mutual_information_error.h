#ifndef MUTUAL_INFORMATION_ERROR
#define MUTUAL_INFORMATION_ERROR

#include <ceres/ceres.h>
#include <hector_calibration_msgs/CameraLidarCalibrationData.h>
#include <camera_model_loader/camera_model_loader.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>

#include <cv_bridge/cv_bridge.h>

#include <geometry_msgs/TransformStamped.h>
#include <Eigen/Eigen>
#include <eigen_conversions/eigen_msg.h>

#include <tf/tf.h>

namespace hector_calibration {
namespace camera_lidar_calibration {

struct CameraObservation {
  std::string name; // cam name
  cv_bridge::CvImagePtr image;
  Eigen::Affine3d transform; // transform from camera head to this camera
};

struct Observation {
  pcl::PointCloud<pcl::PointXYZI> scan;
  std::vector<CameraObservation> cam_observations;
};

struct Histogram {
  Histogram(int size) {
      joint_hist = cv::Mat::zeros(size, size, CV_32FC1);
      reflectance_hist = cv::Mat::zeros(1, size, CV_32FC1);
      intensity_hist = cv::Mat::zeros(1, size, CV_32FC1);
      count = 0;
      intensity_sum = 0;
      reflectance_sum = 0;
  }

  cv::Mat joint_hist;
  cv::Mat reflectance_hist;
  cv::Mat intensity_hist;
  int count;
  int intensity_sum;
  int reflectance_sum;
};

struct Probability {
  Probability(int size) {
      joint_prob = cv::Mat::zeros(size, size, CV_32FC1);
      reflectance_prob = cv::Mat::zeros(1, size, CV_32FC1);
      intensity_prob = cv::Mat::zeros(1, size, CV_32FC1);
      count = 0;
  }

  //joint Probability
  cv::Mat joint_prob;
  //marginal probability reflectivity
  cv::Mat reflectance_prob;
  //marginal probability grayscale
  cv::Mat intensity_prob;
  int count;
};

class MutualInformationError : public ceres::FirstOrderFunction {
public:
  MutualInformationError(const std::vector<hector_calibration_msgs::CameraLidarCalibrationData>& calibration_data,
                         const camera_model::CameraModelLoader& camera_model, int bin_fraction);

  virtual bool Evaluate(const double* parameters, double* cost, double* gradient) const;
private:
  virtual int NumParameters() const { return 6; }

  void readData(const std::vector<hector_calibration_msgs::CameraLidarCalibrationData> &calibration_data);
  Histogram computeHistogram(const Eigen::Affine3d& cam_transform);


  std::vector<Observation> observations_;
  const camera_model::CameraModelLoader& camera_model_;
  int bin_size_;
  int bin_fraction_;

};


}
}

#endif
