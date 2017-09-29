#include <hector_camera_lidar_calibration/mutual_information_cost.h>

bool hasNanInf(const cv::Mat& mat) {
  for (int row = 0; row < mat.rows; row++) {
    for (int col = 0; col < mat.cols; col++) {
      float v = mat.at<float>(row, col);
      if (std::isnan(v) || std::isinf(v)) {
        return true;
      }
    }
  }
  return false;
}

cv::Mat drawHistogram(const cv::Mat& hist) {
  int image_height = 256;
  int image_width = 256;
  int bin_width = cvRound( (double) image_width/hist.cols);
  cv::Mat hist_image(image_height, image_width, CV_8UC3, cv::Scalar(255,255,255));

  double min, max;
  cv::minMaxLoc(hist, &min, &max, NULL, NULL);
//  ROS_INFO_STREAM("Histogram max: " << max);
  double scaling = image_height / max;

  for( int i = 1; i < hist.cols; i++ ) {
    cv::line(hist_image, cv::Point(bin_width * (i-1), cvRound(image_height - scaling * hist.at<float>(i-1))),
                         cv::Point(bin_width * i, cvRound(image_height - scaling * hist.at<float>(i))),
                         cv::Scalar(0, 0, 0), 2, 8, 0);
  }
  return hist_image;
}


cv::Mat drawProbability(const cv::Mat& hist) {
  int image_height = 256;
  int image_width = 256;
  int bin_width = cvRound( (double) image_width/hist.cols);
  cv::Mat prob_image(image_height, image_width, CV_8UC3, cv::Scalar(255,255,255));
  double max = 1.0;
  double scaling = image_height / max;

  for( int i = 1; i < hist.cols; i++ ) {
    cv::line(prob_image, cv::Point(bin_width * (i-1), cvRound(image_height - scaling * hist.at<float>(i-1))),
                         cv::Point(bin_width * i, cvRound(image_height - scaling * hist.at<float>(i))),
                         cv::Scalar(0, 0, 0), 2, 8, 0);
  }
  return prob_image;
}

void normalizeIntensity(pcl::PointCloud<pcl::PointXYZI>& cloud) {
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
  if (gradient != NULL) {
    double **jacobian_ptr = &gradient;
    if (!cost_function_->Evaluate(parameters_ptr, cost, jacobian_ptr)) {
      return false;
    }
    std::cout << "Current cost: " << cost[0] << std::endl;
    std::cout << " --- gradient: ";
    for (int i = 0; i < NumParameters(); i++) {
      std::cout << gradient[i] << ", ";
    }
    std::cout << std::endl;
  } else {
    if (!cost_function_->Evaluate(parameters_ptr, cost, NULL)) {
      return false;
    }
    std::cout << "Current cost: " << cost[0] << std::endl;
  }

  return true;
}

NumericDiffMutualInformationCost::NumericDiffMutualInformationCost(const std::vector<hector_calibration_msgs::CameraLidarCalibrationData> &calibration_data,
                                               const camera_model::CameraModelLoader& camera_model, int bin_fraction)
  : camera_model_(camera_model), bin_fraction_(bin_fraction) {
  readData(calibration_data);
  bin_count_ = 256 / bin_fraction;
}


void NumericDiffMutualInformationCost::readData(const std::vector<hector_calibration_msgs::CameraLidarCalibrationData> &calibration_data) {
  for (std::vector<hector_calibration_msgs::CameraLidarCalibrationData>::const_iterator data_it = calibration_data.begin(); data_it != calibration_data.end(); ++data_it) {
    const hector_calibration_msgs::CameraLidarCalibrationData& data = *data_it;

    Observation observation;
    // Read scan
    pcl::fromROSMsg(data.scan, observation.scan);
    ROS_INFO_STREAM("Reading scan of size " << observation.scan.size());
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
      cam_obs.mask = cv_bridge::toCvCopy(cam_obs_msg.mask);
      tf::transformMsgToEigen(cam_obs_msg.transform.transform, cam_obs.transform);
      observation.cam_observations.push_back(cam_obs);
      ROS_INFO_STREAM("Reading image of cam " << cam_obs.name);
    }

    observations_.push_back(observation);
  }
  ROS_INFO_STREAM("Data reading finished.");
}

bool NumericDiffMutualInformationCost::operator()(const double* const parameters, double* cost) const {
  Eigen::Affine3d calibration(Eigen::AngleAxisd(parameters[5], Eigen::Vector3d::UnitZ())
      * Eigen::AngleAxisd(parameters[4], Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(parameters[3], Eigen::Vector3d::UnitX()));
  calibration.translation() = Eigen::Vector3d(parameters[0], parameters[1], parameters[2]);
  Histogram hist = computeHistogram(calibration);
//  ROS_INFO_STREAM("joint hist has nan/inf? " << hasNanInf(hist.joint_hist));
//  ROS_INFO_STREAM("reflectance hist has nan/inf? " << hasNanInf(hist.reflectance_hist));
  Probability prob = computeProbability(hist);
//  ROS_INFO_STREAM("reflectance prob has nan/inf? " << hasNanInf(prob.reflectance_prob));
//  ROS_INFO_STREAM("joint prob has nan/inf? " << hasNanInf(prob.joint_prob));

  cost[0] = computeMutualInformationCost(prob);
  return true;
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
          Eigen::Vector2i pixel_rounded(std::round(pixel(0)), std::round(pixel(1))); //TODO interpolate instead?
          // check image mask
          const cv::Mat& mask = cam_obs.mask->image;
          if (mask.empty() || mask.at<uchar>(pixel_rounded(1), pixel_rounded(0)) > 0) {
            uchar intensity = cam_obs.image->image.at<uchar>(pixel_rounded(1), pixel_rounded(0)) / bin_fraction_;
            uchar reflectance = static_cast<uchar>(point.intensity / bin_fraction_);

            histogram.intensity_hist.at<float>(intensity) += 1;
            histogram.reflectance_hist.at<float>(reflectance) += 1;
            histogram.joint_hist.at<float>(intensity, reflectance) += 1;

            histogram.intensity_sum += intensity;
            histogram.reflectance_sum += reflectance;

            histogram.count++;
          }
        }
      }
    }
  }

//  cv::Mat intensity_hist_image = drawHistogram(histogram.intensity_hist);
//  cv::namedWindow("Intensity Histogram", CV_WINDOW_AUTOSIZE);
//  cv::imshow("Intensity Histogram", intensity_hist_image);

//  cv::Mat reflectance_hist_image = drawHistogram(histogram.reflectance_hist);
//  cv::namedWindow("Reflectance Histogram", CV_WINDOW_AUTOSIZE);
//  cv::imshow("Reflectance Histogram", reflectance_hist_image);

//  cv::waitKey(0);
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
      prob.joint_prob.at<float>(i, j) = histogram.joint_hist.at<float>(i, j)/histogram.count;
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

//  cv::Mat intensity_prob_image = drawProbability(prob.intensity_prob);
//  cv::namedWindow("Intensity PD", CV_WINDOW_AUTOSIZE);
//  cv::imshow("Intensity PD", intensity_prob_image);

//  cv::Mat reflectance_prob_image = drawProbability(prob.reflectance_prob);
//  cv::namedWindow("Reflectance PD", CV_WINDOW_AUTOSIZE);
//  cv::imshow("Reflectance PD", reflectance_prob_image);

//  cv::waitKey(0);
  return prob;
}

float NumericDiffMutualInformationCost::computeMutualInformationCost(const Probability& prob) const {
  //Calculate log of density estimate
  cv::Mat joint_log, intensity_log, reflectance_log;

  cv::Mat joint_prob_no_zero;
  prob.joint_prob.copyTo(joint_prob_no_zero);
  joint_prob_no_zero.setTo(1e-7, prob.joint_prob == 0);

  cv::log(prob.intensity_prob, intensity_log);
  cv::log(prob.reflectance_prob, reflectance_log);
//  ROS_INFO_STREAM("reflectance log prob has nan/inf? " << hasNanInf(reflectance_log));
  cv::log(joint_prob_no_zero, joint_log);
//  ROS_INFO_STREAM("log joint prob has nan/inf? " << hasNanInf(joint_log));


  cv::Mat joint_entropy, intensity_entropy, reflectance_entropy;
  cv::multiply(prob.intensity_prob, intensity_log, intensity_entropy);
  cv::multiply(prob.reflectance_prob, reflectance_log, reflectance_entropy);
//  ROS_INFO_STREAM("reflectance entropy has nan/inf? " << hasNanInf(reflectance_entropy));
  cv::multiply(prob.joint_prob, joint_log, joint_entropy);
//  ROS_INFO_STREAM("log entropy has nan/inf? " << hasNanInf(joint_entropy));


  //Sum all the elements
  float Hx  = cv::norm(intensity_entropy, cv::NORM_L1);
//  ROS_INFO_STREAM("Hx: " << Hx);
  float Hy  = cv::norm(reflectance_entropy, cv::NORM_L1);
//  ROS_INFO_STREAM("Hy: " << Hy);
  float Hxy = cv::norm(joint_entropy, cv::NORM_L1);
//  ROS_INFO_STREAM("Hxy: " << Hxy);

  float mi = Hx + Hy - Hxy;
  ROS_INFO_STREAM("MI: " << mi);
  return -mi;
}

}
}
