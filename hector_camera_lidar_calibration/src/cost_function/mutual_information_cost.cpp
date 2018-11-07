#include <hector_camera_lidar_calibration/cost_function/mutual_information_cost.h>

namespace hector_calibration {
namespace hector_camera_lidar_calibration {

MutualInformationCost::MutualInformationCost(const std::vector<hector_calibration_msgs::CameraLidarCalibrationData> &calibration_data,
                                               const camera_model::CameraModelLoader& camera_model, int bin_fraction, int scan_sample_size)
  : camera_model_(camera_model), bin_fraction_(bin_fraction), scan_sample_size_(scan_sample_size) {
  ros::NodeHandle pnh("~");
  processed_cloud_pub_ = pnh.advertise<pcl::PointCloud<pcl::PointXYZI>>("processed_cloud", 10, true);

  readData(calibration_data);
  bin_count_ = 256 / bin_fraction;
}


void MutualInformationCost::readData(const std::vector<hector_calibration_msgs::CameraLidarCalibrationData> &calibration_data) {
  // Iterate over scan-images pairs
  for (std::vector<hector_calibration_msgs::CameraLidarCalibrationData>::const_iterator data_it = calibration_data.begin(); data_it != calibration_data.end(); ++data_it) {
    const hector_calibration_msgs::CameraLidarCalibrationData& data = *data_it;
    int obs_number = data_it - calibration_data.begin();

    std::vector<ros::Publisher> result_pubs;

    Observation observation;
    // Read scan
    pcl::fromROSMsg(data.scan, observation.scan);
    ROS_INFO_STREAM("Reading scan of size " << observation.scan.size());
    // Filter scan
    std::vector<int> mapping;
    pcl::removeNaNFromPointCloud(observation.scan, observation.scan, mapping);
    float size = 0.75f;
    Eigen::Vector4f min(-size, -size, -size, 1);
    Eigen::Vector4f max(size, size, size, 1);
    observation.scan = cropBox(observation.scan, min, max);
    observation.scan = sampleCloud(observation.scan, static_cast<unsigned int>(scan_sample_size_));
    observation.scan = cutReflectance(observation.scan, 0.0f, 100.0f);
    normalizeReflectance(observation.scan, false, 100.0f); // max normal reflectance of vlp16 is 100

    // Publish processed cloud
//    sensor_msgs::PointCloud2 cloud_msg;
//    pcl::toROSMsg(observation.scan, cloud_msg);
    processed_cloud_pub_.publish(observation.scan);

    // Read images
    ros::NodeHandle pnh("~");
    for (std::vector<hector_calibration_msgs::CameraObservation>::const_iterator cam_obs_it = data.camera_observations.begin();
         cam_obs_it != data.camera_observations.end(); ++cam_obs_it) {
      const hector_calibration_msgs::CameraObservation& cam_obs_msg = *cam_obs_it;
      CameraObservation cam_obs;
      cam_obs.name = cam_obs_msg.name.data;

      cam_obs.cv_image_ptr = cv_bridge::toCvCopy(cam_obs_msg.image);
      cv::cvtColor(cam_obs.cv_image_ptr->image, cam_obs.cv_image_ptr->image, cv::COLOR_RGB2GRAY);
      cam_obs.cv_image_color_ptr = cv_bridge::toCvCopy(cam_obs_msg.image);
      cam_obs.cv_image_ptr->encoding = sensor_msgs::image_encodings::TYPE_8UC1;
      cv::imwrite("gray_image.jpg", cam_obs.cv_image_ptr->image);

      cam_obs.cv_mask_ptr = cv_bridge::toCvCopy(cam_obs_msg.mask);
//      cv::imwrite("mask.jpg", cam_obs.cv_mask_ptr->image);

      tf::transformMsgToEigen(cam_obs_msg.transform.transform, cam_obs.transform);
      observation.cam_observations.push_back(cam_obs);
      ROS_INFO_STREAM("Reading image of cam " << cam_obs.name);
      ros::Publisher result_image_pub = pnh.advertise<sensor_msgs::Image>("result_image_obs_" + std::to_string(obs_number) + "_" + cam_obs.name, 10, true);
      result_pubs.push_back(result_image_pub);
    }
    result_image_pubs_.push_back(result_pubs);
    observations_.push_back(observation);
  }
  ROS_INFO_STREAM("Data reading finished.");
}

Histogram MutualInformationCost::computeHistogram(const Eigen::Affine3d &cam_transform) const {
  Histogram histogram(bin_count_);
  // Iterate over each observation (scan - images pair)
  for (unsigned int obs_number = 0; obs_number < observations_.size(); ++obs_number) {
    const Observation& observation = observations_[obs_number];
    // Create debug images
    std::vector<cv_bridge::CvImage> result_images;
    for (std::vector<CameraObservation>::const_iterator cam_it = observation.cam_observations.begin(); cam_it != observation.cam_observations.end(); ++cam_it) {
      const CameraObservation& cam_obs = *cam_it;
      cv_bridge::CvImage result_image;
      result_image.encoding = cam_obs.cv_image_color_ptr->encoding;
      cam_obs.cv_image_color_ptr->image.copyTo(result_image.image);
      result_images.push_back(result_image);
    }

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
        int cam_number = cam_it - observation.cam_observations.begin();

        // Transform to specific cam frame
        Eigen::Vector3d point_cam;
        pcl::transformPoint(point_transformed, point_cam, cam_obs.transform);
        // Project to image
        Eigen::Vector2d pixel;
        if (camera_model_.getCamera(cam_obs.name).worldToPixel(point_cam, pixel)) {
//          ROS_INFO_STREAM("pixel: " << pixel);
          uchar intensity = static_cast<uchar>(interpolate(cam_obs.cv_image_ptr->image, pixel) / bin_fraction_);
//          uchar intensity = cam_obs.cv_image_ptr->image.at<uchar>(static_cast<int>(pixel(0)), static_cast<int>(pixel(1)));
          uchar reflectance = static_cast<uchar>(point.intensity / bin_fraction_);
//          ROS_INFO_STREAM("intensity: " << static_cast<int>(intensity) << ", reflectance :" << static_cast<int>(reflectance));

          histogram.intensity_hist.at<float>(intensity) += 1;
          histogram.reflectance_hist.at<float>(reflectance) += 1;
          histogram.joint_hist.at<float>(intensity, reflectance) += 1;

          histogram.intensity_sum += intensity;
          histogram.reflectance_sum += reflectance;

          histogram.count++;

          // Debug/Result image
          if (point_cam.norm() < 3) {
            cv::Point p(cvRound(pixel(0)), cvRound(pixel(1)));
            result_images[cam_number].image.at<cv::Vec3b>(p) = cv::Vec3b(reflectance, reflectance, reflectance);
          }

        }
      }
    }

    // Write out for debug
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);

    std::ostringstream oss;
    oss << std::put_time(&tm, "%d-%m-%Y %H-%M-%S");
    std::string time_str = oss.str();

    for (unsigned int cam_number = 0; cam_number < result_images.size(); cam_number++) {
      result_image_pubs_[obs_number][cam_number].publish(result_images[cam_number].toImageMsg());
      std::string filepath = "result_image_obs_" + std::to_string(obs_number) + "_" + observation.cam_observations[cam_number].name + "_" + time_str + ".jpg";
//      ROS_INFO_STREAM("Writing debug image to " << filepath);
      cv::imwrite(filepath, result_images[cam_number].image);
    }

  }

//  cv::Mat intensity_hist_image = drawHistogram(histogram.intensity_hist, true);
//  cv::imwrite("Intensity_Histogram.jpg", intensity_hist_image);

//  cv::Mat reflectance_hist_image = drawHistogram(histogram.reflectance_hist, true);
//  cv::imwrite("Reflectance_Histogram.jpg", reflectance_hist_image);
  return histogram;
}

Probability MutualInformationCost::computeProbability(const Histogram& histogram) const {
  float mu_intensity = histogram.intensity_sum / histogram.count;
//  ROS_INFO_STREAM("mu intensity: " << mu_intensity);
  float mu_reflectance = histogram.reflectance_sum / histogram.count;
//  ROS_INFO_STREAM("mu reflectance: " << mu_reflectance);
  //Covariances
  double sigma_intensity = 0;
  double sigma_reflectance = 0;

  Probability prob(bin_count_);

  for (int i = 0; i < bin_count_; i++) {
    //calculate sample covariance
    sigma_intensity += histogram.intensity_hist.at<float>(i) * std::pow(i - mu_intensity, 2);
    sigma_reflectance += histogram.reflectance_hist.at<float>(i) * std::pow(i - mu_reflectance, 2);

    //Normalize the histogram so that the value is between (0,1)
    prob.intensity_prob.at<float>(i) = histogram.intensity_hist.at<float>(i)/histogram.count;
    prob.reflectance_prob.at<float>(i) = histogram.reflectance_hist.at<float>(i)/histogram.count;
    for (int j = 0; j < bin_count_; j++) {
      prob.joint_prob.at<float>(i, j) = histogram.joint_hist.at<float>(i, j)/histogram.count;
    }
  }

  sigma_intensity /= histogram.count;
  sigma_reflectance /= histogram.count;

  //Compute the optimal bandwidth (Silverman's rule of thumb)
  sigma_intensity = 1.06 * std::sqrt(sigma_intensity) / std::pow(histogram.count, 0.2);
  sigma_reflectance = 1.06 * std::sqrt(sigma_reflectance) / std::pow(histogram.count, 0.2);

  cv::GaussianBlur(prob.intensity_prob, prob.intensity_prob, cv::Size(0, 0), sigma_intensity);
  cv::GaussianBlur(prob.reflectance_prob, prob.reflectance_prob, cv::Size(0, 0), sigma_reflectance);
  cv::GaussianBlur(prob.joint_prob, prob.joint_prob, cv::Size(0, 0), sigma_intensity, sigma_reflectance);

  prob.count = histogram.count;

//  cv::Mat intensity_prob_image = drawHistogram(prob.intensity_prob);
//  cv::imwrite("Intensity_PD.jpg", intensity_prob_image);

//  cv::Mat reflectance_prob_image = drawHistogram(prob.reflectance_prob);
//  cv::imwrite("Reflectance_PD.jpg", reflectance_prob_image);

  return prob;
}

float MutualInformationCost::computeMutualInformationCost(const Eigen::Affine3d &cam_transform) const {
  Histogram hist = computeHistogram(cam_transform);
//  ROS_INFO_STREAM("joint hist has nan/inf? " << hasNanInf(hist.joint_hist));
//  ROS_INFO_STREAM("reflectance hist has nan/inf? " << hasNanInf(hist.reflectance_hist));
  Probability prob = computeProbability(hist);
//  ROS_INFO_STREAM("reflectance prob has nan/inf? " << hasNanInf(prob.reflectance_prob));
//  ROS_INFO_STREAM("joint prob has nan/inf? " << hasNanInf(prob.joint_prob));
  //Calculate log of density estimate
  cv::Mat joint_log, intensity_log, reflectance_log;

  // Create copy of probability estimates without 0
  cv::Mat intensity_prob_no_zero, reflectance_prob_no_zero, joint_prob_no_zero;
  prob.intensity_prob.copyTo(intensity_prob_no_zero);
  intensity_prob_no_zero.setTo(1e-7, prob.intensity_prob == 0);
  prob.reflectance_prob.copyTo(reflectance_prob_no_zero);
  reflectance_prob_no_zero.setTo(1e-7, prob.reflectance_prob == 0);
  prob.joint_prob.copyTo(joint_prob_no_zero);
  joint_prob_no_zero.setTo(1e-7, prob.joint_prob == 0);

  // Compute log probabilities
  cv::log(intensity_prob_no_zero, intensity_log);
  cv::log(reflectance_prob_no_zero, reflectance_log);
  cv::log(joint_prob_no_zero, joint_log);
  //  ROS_INFO_STREAM("reflectance log prob has nan/inf? " << hasNanInf(reflectance_log));
//  ROS_INFO_STREAM("log joint prob has nan/inf? " << hasNanInf(joint_log));

  // Compute entropy
  cv::Mat intensity_entropy, reflectance_entropy, joint_entropy;
  cv::multiply(prob.intensity_prob, intensity_log, intensity_entropy);
  cv::multiply(prob.reflectance_prob, reflectance_log, reflectance_entropy);
  cv::multiply(prob.joint_prob, joint_log, joint_entropy);
//  ROS_INFO_STREAM("reflectance entropy has nan/inf? " << hasNanInf(reflectance_entropy));
//  ROS_INFO_STREAM("log entropy has nan/inf? " << hasNanInf(joint_entropy));

  //Sum all the elements
  float Hx  = cv::norm(intensity_entropy, cv::NORM_L1);
  float Hy  = cv::norm(reflectance_entropy, cv::NORM_L1);
  float Hxy = cv::norm(joint_entropy, cv::NORM_L1);
  float mi = Hx + Hy - Hxy;
  //  ROS_INFO_STREAM("Hx: " << Hx);
  //  ROS_INFO_STREAM("Hy: " << Hy);
  //  ROS_INFO_STREAM("Hxy: " << Hxy);
  ROS_INFO_STREAM(std::setprecision(17) << "MI: " << mi);
  return -mi;
}

}
}
