#include <hector_camera_lidar_calibration/helper.h>

#include <pcl/filters/random_sample.h>
#include <pcl/filters/crop_box.h>

namespace hector_calibration {
namespace camera_lidar_calibration {

std::string parametersToString(const double * parameters) {
  std::stringstream ss;
  ss << std::setprecision(17);
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
    cv::Mat patch;
    cv::getRectSubPix(img, cv::Size(1,1), pt, patch);
    return static_cast<uchar>(patch.at<float>(0,0));
}

uchar interpolate(const cv::Mat& img, Eigen::Vector2d pt) {
    cv::Point2f cv_pt(pt(0), pt(1)); // precision loss, but should be ok
    return interpolate(img, cv_pt);
}

cv::Mat drawHistogram(const cv::Mat& hist, bool autoscale, float max) {
  int image_height = 256;
  int image_width = 256;
  int bin_width = cvRound( (double) image_width/hist.cols);
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

}
}
