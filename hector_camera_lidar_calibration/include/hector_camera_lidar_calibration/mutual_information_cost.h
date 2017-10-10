#ifndef MUTUAL_INFORMATION_COST
#define MUTUAL_INFORMATION_COST

#include <iostream>
#include <iomanip>
#include <ctime>
#include <sstream>

#include <ceres/ceres.h>
#include <hector_calibration_msgs/CameraLidarCalibrationData.h>
#include <camera_model_loader/camera_model_loader.h>
#include <hector_camera_lidar_calibration/helper.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <geometry_msgs/TransformStamped.h>
#include <Eigen/Eigen>
#include <eigen_conversions/eigen_msg.h>

#include <tf/tf.h>

namespace hector_calibration {
namespace camera_lidar_calibration {

struct CameraObservation {
  std::string name; // cam name
  cv_bridge::CvImagePtr cv_image_ptr;
  cv_bridge::CvImagePtr cv_image_color_ptr;
  cv_bridge::CvImagePtr cv_mask_ptr;
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

class MutualInformationCost : public ceres::FirstOrderFunction {
public:
  MutualInformationCost(const std::vector<hector_calibration_msgs::CameraLidarCalibrationData>& calibration_data,
                         const camera_model::CameraModelLoader& camera_model, int bin_fraction);
  ~MutualInformationCost();

  virtual bool Evaluate(const double* parameters, double* cost, double* gradient) const;
private:
  virtual int NumParameters() const { return 6; }

  ceres::CostFunction* cost_function_;
};

class NumericDiffMutualInformationCost {
public:
  NumericDiffMutualInformationCost(const std::vector<hector_calibration_msgs::CameraLidarCalibrationData>& calibration_data,
                                   const camera_model::CameraModelLoader& camera_model, int bin_fraction);

  bool operator()(const double* const parameters, double* cost) const;

  static ceres::CostFunction* Create(const std::vector<hector_calibration_msgs::CameraLidarCalibrationData>& calibration_data,
                                     const camera_model::CameraModelLoader& camera_model, int bin_fraction) {
    ceres::CostFunction* cost_function =
        new ceres::NumericDiffCostFunction<NumericDiffMutualInformationCost, ceres::FORWARD, 1, 6>(
          new NumericDiffMutualInformationCost(calibration_data, camera_model, bin_fraction));

    return cost_function;
  }
  float computeMutualInformationCost(const Eigen::Affine3d& cam_transform) const;

private:
  void readData(const std::vector<hector_calibration_msgs::CameraLidarCalibrationData> &calibration_data);
  Histogram computeHistogram(const Eigen::Affine3d& cam_transform) const;
  Probability computeProbability(const Histogram& histogram) const;



  std::vector<Observation> observations_;
  const camera_model::CameraModelLoader& camera_model_;
  int bin_count_;
  int bin_fraction_;

  // Debug publishers
  std::vector<std::vector<ros::Publisher>> result_image_pubs_;
  ros::Publisher processed_cloud_pub_;
};

}
}

#endif
