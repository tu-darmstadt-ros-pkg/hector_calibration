#include <hector_camera_lidar_calibration/helper.h>

#include <kdl/frames.hpp>

#include <pcl/filters/random_sample.h>
#include <pcl/filters/crop_box.h>

namespace hector_calibration {
namespace hector_camera_lidar_calibration {

std::string parametersToString(const double * parameters) {
  std::stringstream ss;
//  ss << std:  :setprecision(17);
  ss << "[x=" << parameters[0] << ", y=" << parameters[1] << ", z=" << parameters[2]
     << "; roll=" << parameters[3] << ", pitch=" << parameters[4] << ", yaw=" << parameters[5] << "]";
  return ss.str();
}

bool containsNanOrInf(const cv::Mat& mat) {
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

uchar interpolate(const cv::Mat& img, cv::Point2f pt) {
  assert(!img.empty());
  assert(img.channels() == 1);

  // Round down
  int x = static_cast<int>(pt.x);
  int y = static_cast<int>(pt.y);

  // Border handling
  int x0 = cv::borderInterpolate(x,   img.cols, cv::BORDER_REFLECT_101);
  int x1 = cv::borderInterpolate(x+1, img.cols, cv::BORDER_REFLECT_101);
  int y0 = cv::borderInterpolate(y,   img.rows, cv::BORDER_REFLECT_101);
  int y1 = cv::borderInterpolate(y+1, img.rows, cv::BORDER_REFLECT_101);

  float a = pt.x - static_cast<float>(x);
  float c = pt.y - static_cast<float>(y);

  // Read values of 4 neighboring pixels
  float v1 = static_cast<float>(img.at<uchar>(y0, x0));
  float v2 = static_cast<float>(img.at<uchar>(y0, x1));
  float v3 = static_cast<float>(img.at<uchar>(y1, x0));
  float v4 = static_cast<float>(img.at<uchar>(y1, x1));

  uchar value = static_cast<uchar>(cvRound((v1 * (1.f - a) + v2 * a) * (1.f - c) + (v3 * (1.f - a) + v4 * a) * c));
  return value;
}

uchar interpolate(const cv::Mat& img, Eigen::Vector2d pt) {
    cv::Point2f cv_pt(pt(0), pt(1)); // precision loss, but should be ok
    return interpolate(img, cv_pt);
}

cv::Mat drawHistogram(const cv::Mat& hist, bool autoscale, float max) {
  int image_height = 256;
  int image_width = 256;
  int bin_width = cvRound( static_cast<double>(image_width)/hist.cols);
  cv::Mat hist_image(image_height, image_width, CV_8UC3, cv::Scalar(255,255,255));

  float scaling;
  if (autoscale) {
    double hist_min, hist_max;
    cv::minMaxLoc(hist, &hist_min, &hist_max, NULL, NULL);
    scaling = image_height / hist_max;
  } else {
   if (max != 0.0) {
     scaling = image_height / max;
   } else {
     scaling = image_height;
   }
  }


  for( int i = 1; i < hist.cols; i++ ) {
    cv::line(hist_image, cv::Point(bin_width * (i-1), cvRound(image_height - scaling * hist.at<float>(i-1))),
                         cv::Point(bin_width * i, cvRound(image_height - scaling * hist.at<float>(i))),
                         cv::Scalar(0, 0, 0), 2, 8, 0);
  }
  return hist_image;
}

pcl::PointCloud<pcl::PointXYZI> cutReflectance(const pcl::PointCloud<pcl::PointXYZI>& cloud, float min, float max) {
  pcl::PassThrough<pcl::PointXYZI> filter;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::copyPointCloud(cloud, *cloud_ptr);
  filter.setInputCloud(cloud_ptr);
  filter.setFilterFieldName("intensity");
  filter.setFilterLimits(min, max);

  pcl::PointCloud<pcl::PointXYZI> cloud_out;
  filter.filter(cloud_out);
  return cloud_out;
}


void normalizeReflectance(pcl::PointCloud<pcl::PointXYZI>& cloud, bool autoscale, float max) {
  float scaling;
  if (autoscale) {
    float reflectance_max = 0;
    for (auto point_it = cloud.begin(); point_it != cloud.end(); ++point_it) {
      const pcl::PointXYZI& point = *point_it;
      if (point.intensity > reflectance_max) {
        reflectance_max = point.intensity;
      }
    }
    scaling = 255.0f / reflectance_max;
    ROS_INFO_STREAM("Reflectance max is " << reflectance_max << ". Scaling with " << scaling);
  } else {
    scaling = 255.0f / max;
  }
  for (auto point_it = cloud.begin(); point_it != cloud.end(); ++point_it) {
    pcl::PointXYZI& point = *point_it;
    if (point.intensity > max) {
      point.intensity = max;
    } else {
      point.intensity *= scaling;
    }
  }
}

pcl::PointCloud<pcl::PointXYZI> sampleCloud(const pcl::PointCloud<pcl::PointXYZI>& cloud, unsigned int samples)
{
  pcl::RandomSample<pcl::PointXYZI> random_sample;
  random_sample.setSample(samples);

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::copyPointCloud(cloud, *cloud_ptr);
  random_sample.setInputCloud(cloud_ptr);

  pcl::PointCloud<pcl::PointXYZI> cloud_out;
  random_sample.filter(cloud_out);
  return cloud_out;
}

pcl::PointCloud<pcl::PointXYZI> cropBox(const pcl::PointCloud<pcl::PointXYZI>& cloud, const Eigen::Vector4f& min, const Eigen::Vector4f& max)
{
  pcl::CropBox<pcl::PointXYZI> crop_box;
  crop_box.setMin(min);
  crop_box.setMax(max);
  crop_box.setNegative(true);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::copyPointCloud(cloud, *cloud_ptr);
  crop_box.setInputCloud(cloud_ptr);

  pcl::PointCloud<pcl::PointXYZI> cloud_out;
  crop_box.filter(cloud_out);
  return cloud_out;
}

Eigen::Vector3d rotToRpy(const Eigen::Matrix3d& rot)
{
//  auto data = rot.data();
//  Eigen::Vector3d rpy;
//  // Taken from kdl::Rotation::GetRPY
//  double epsilon=1E-12;
//  rpy[1] = atan2(-data[6], sqrt(data[0]*data[0] + data[3]*data[3] ));
//  if ( fabs(rpy[1]) > (M_PI/2.0-epsilon) ) {
//    rpy[2] = atan2(	-data[1], data[4]);
//    rpy[0]  = 0.0 ;
//  } else {
//    rpy[0]  = atan2(data[7], data[8]);
//    rpy[2]  = atan2(data[3], data[0]);
//  }

  Eigen::Quaterniond rot_quad(rot);
  KDL::Rotation kdl_rot = KDL::Rotation::Quaternion(rot_quad.x(), rot_quad.y(), rot_quad.z(), rot_quad.w());
  Eigen::Vector3d rpy;
  kdl_rot.GetRPY(rpy[0], rpy[1], rpy[2]);
  return rpy;
}

Eigen::Matrix3d rpyToRot(const Eigen::Vector3d& rpy)
{
  Eigen::Matrix3d rot;
  rot = Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ())
      * Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX());
  return rot;
}

}
}
