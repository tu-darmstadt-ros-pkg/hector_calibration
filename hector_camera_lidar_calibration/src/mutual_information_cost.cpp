#include <hector_camera_lidar_calibration/mutual_information_cost.h>

namespace hector_calibration {
namespace camera_lidar_calibration {

MutualInformationCost::MutualInformationCost(const std::vector<hector_calibration_msgs::CameraLidarCalibrationData>& calibration_data,
                       const camera_model::CameraModelLoader& camera_model, int bin_fraction) {
  cost_function_ = NumericDiffMutualInformationCost::Create(calibration_data, camera_model, bin_fraction);
}

MutualInformationCost::~MutualInformationCost() {
  delete cost_function_;
}

bool MutualInformationCost::Evaluate(const double* parameters, double* cost, double* gradient) const {
  double const *const *parameters_ptr = &parameters;
  double *gradient_tmp = new double[NumParameters()];
  double **jacobian_ptr = &gradient_tmp;
  if (!cost_function_->Evaluate(parameters_ptr, cost, jacobian_ptr)) {
    return false;
  }
  if (gradient != NULL) {
    memcpy(gradient_tmp, gradient, NumParameters() * sizeof(double));
  }

  return true;
}

NumericDiffMutualInformationCost::NumericDiffMutualInformationCost(const std::vector<hector_calibration_msgs::CameraLidarCalibrationData> &calibration_data,
                                               const camera_model::CameraModelLoader& camera_model, int bin_fraction)
  : camera_model_(camera_model), bin_fraction_(bin_fraction) {
  readData(calibration_data);
  bin_count_ = 256 / bin_fraction;
}

bool NumericDiffMutualInformationCost::operator()(const double* const parameters, double* cost) const {
  Eigen::Affine3d calibration(Eigen::AngleAxisd(parameters[5], Eigen::Vector3d::UnitZ())
      * Eigen::AngleAxisd(parameters[4], Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(parameters[3], Eigen::Vector3d::UnitX()));
  calibration.translation() = Eigen::Vector3d(parameters[0], parameters[1], parameters[2]);
  Histogram hist = computeHistogram(calibration);
  Probability prob = computeProbability(hist);

  cost[0] = computeMutualInformationCost(prob);
  return true;
}

void NumericDiffMutualInformationCost::readData(const std::vector<hector_calibration_msgs::CameraLidarCalibrationData> &calibration_data) {
  for (std::vector<hector_calibration_msgs::CameraLidarCalibrationData>::const_iterator data_it = calibration_data.begin(); data_it != calibration_data.end(); ++data_it) {
    const hector_calibration_msgs::CameraLidarCalibrationData& data = *data_it;

    Observation observation;
    // Read scan
    pcl::fromROSMsg(data.scan, observation.scan);
    std::vector<int> mapping;
    pcl::removeNaNFromPointCloud(observation.scan, observation.scan, mapping);
    normalizeIntensity(observation.scan);

    // Read images
    for (std::vector<hector_calibration_msgs::CameraObservation>::const_iterator cam_obs_it = data.camera_observations.begin();
         cam_obs_it != data.camera_observations.end(); ++cam_obs_it) {
      const hector_calibration_msgs::CameraObservation& cam_obs_msg = *cam_obs_it;
      CameraObservation cam_obs;
      cam_obs.name = cam_obs_msg.name.data;
      cam_obs.image = cv_bridge::toCvCopy(cam_obs_msg.image);
      tf::transformMsgToEigen(cam_obs_msg.transform.transform, cam_obs.transform);
      observation.cam_observations.push_back(cam_obs);
    }

    observations_.push_back(observation);
  }
}

void NumericDiffMutualInformationCost::normalizeIntensity(pcl::PointCloud<pcl::PointXYZI>& cloud) const {
  float max = 0;
  for (auto point_it = cloud.begin(); point_it != cloud.end(); ++point_it) {
    const pcl::PointXYZI& point = *point_it;
    if (point.intensity > max) {
      max = point.intensity;
    }
  }

  float scaling = 255.0f / max;
  for (auto point_it = cloud.begin(); point_it != cloud.end(); ++point_it) {
    pcl::PointXYZI& point = *point_it;
    point.intensity *= scaling;
  }
}

Histogram NumericDiffMutualInformationCost::computeHistogram(const Eigen::Affine3d &cam_transform) const {
  Histogram histogram(bin_count_);
  // Iterate over each observation (scan - images pair)
  for (std::vector<Observation>::const_iterator obs_it = observations_.begin(); obs_it != observations_.end(); ++obs_it) {
    const Observation& observation = *obs_it;
    // Iterate over each point in scan
    for (pcl::PointCloud<pcl::PointXYZI>::const_iterator scan_it = observation.scan.begin(); scan_it != observation.scan.end(); ++scan_it) {
      const pcl::PointXYZI& point = *scan_it;
      // Transform to cam head frame
      Eigen::Vector3d point_in(point.x, point.y, point.z);
      Eigen::Vector3d point_transformed;
      pcl::transformPoint(point_in, point_transformed, cam_transform);
      // Iterate over each cam
      for (std::vector<CameraObservation>::const_iterator cam_it = observation.cam_observations.begin(); cam_it != observation.cam_observations.end(); ++cam_it) {
        const CameraObservation& cam_obs = *cam_it;
        // Transform to specific cam frame
        Eigen::Vector3d point_cam;
        pcl::transformPoint(point_transformed, point_cam, cam_obs.transform);
        // Project to image
        Eigen::Vector2d pixel;
        if (camera_model_.getCamera(cam_obs.name).worldToPixel(point_cam, pixel)) {
          // TODO check cam mask
          Eigen::Vector2i pixel_rounded(std::round(pixel(0)), std::round(pixel(1))); //TODO interpolate instead?
          uchar gray = cam_obs.image->image.at<uchar>(pixel_rounded(1), pixel_rounded(0)) / bin_fraction_;
          histogram.intensity_hist.at<float>(gray) += 1;
          histogram.intensity_sum += gray;
          uchar reflectance = static_cast<uchar>(point.intensity / bin_fraction_);
          histogram.reflectance_hist.at<float>(reflectance) += 1;
          histogram.reflectance_sum += reflectance;
          histogram.count++;
        }
      }
    }
  }

  return histogram;
}

Probability NumericDiffMutualInformationCost::computeProbability(const Histogram& histogram) const {
  float mu_intensity = histogram.intensity_sum / histogram.count;
  float mu_reflectance = histogram.reflectance_sum / histogram.count;
  //Covariances
  double sigma_intensity = 0;
  double sigma_reflectance = 0;

  Probability prob(bin_count_);

  for (int i = 0; i < bin_count_; i++) {
    //calculate sample covariance
    sigma_intensity += histogram.intensity_hist.at<float>(i)*(i - mu_intensity)*(i - mu_intensity);
    sigma_reflectance += histogram.reflectance_hist.at<float>(i)*(i - mu_reflectance)*(i - mu_reflectance);

    //Normalize the histogram so that the value is between (0,1)
    prob.intensity_prob.at<float>(i) = histogram.intensity_hist.at<float>(i)/histogram.count;
    prob.reflectance_prob.at<float>(i) = histogram.reflectance_hist.at<float>(i)/histogram.count;
    for (int j = 0; j < bin_count_; j++) {
      prob.joint_prob.at<float>(i, j) = histogram.joint_hist.at<float>(i, j)/(histogram.count);
    }
  }

  sigma_intensity = sigma_intensity / histogram.count;
  sigma_reflectance = sigma_reflectance / histogram.count;

  //Compute the optimal bandwidth (Silverman's rule of thumb)
  sigma_intensity = 1.06 * std::sqrt(sigma_intensity) / std::pow(histogram.count, 0.2);
  sigma_reflectance = 1.06 * std::sqrt(sigma_reflectance) / std::pow(histogram.count, 0.2);

  cv::GaussianBlur(prob.intensity_prob, prob.intensity_prob, cv::Size(0, 0), sigma_intensity);
  cv::GaussianBlur(prob.reflectance_prob, prob.reflectance_prob, cv::Size(0, 0), sigma_reflectance);
  cv::GaussianBlur(prob.joint_prob, prob.joint_prob, cv::Size(0, 0), sigma_intensity, sigma_reflectance);

  prob.count = histogram.count;
  return prob;
}

float NumericDiffMutualInformationCost::computeMutualInformationCost(const Probability& prob) const {
  //Calculate log of density estimate
  cv::Mat joint_log, intensity_log, reflectance_log;

  cv::log(prob.joint_prob, joint_log);
  cv::log(prob.intensity_prob, intensity_log);
  cv::log(prob.reflectance_prob, reflectance_log);

  cv::Mat joint_entropy, intensity_entropy, reflectance_entropy;
  cv::multiply(prob.joint_prob, joint_log, joint_entropy);
  cv::multiply(prob.intensity_prob, intensity_log, intensity_entropy);
  cv::multiply(prob.reflectance_prob, reflectance_log, reflectance_entropy);

  //Sum all the elements
  float Hx  = cv::norm(intensity_entropy, cv::NORM_L1);
  float Hy  = cv::norm(reflectance_entropy, cv::NORM_L1);
  float Hxy = cv::norm(joint_entropy, cv::NORM_L1);

  return -(Hx + Hy - Hxy);
}

}
}
