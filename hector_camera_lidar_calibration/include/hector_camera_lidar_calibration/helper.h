#ifndef HELPER_H
#define HELPER_H

#include <ros/ros.h>

#include <opencv/cv.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>

#include <iomanip>

namespace hector_calibration {
namespace camera_lidar_calibration {

std::string parametersToString(const double * parameters);

cv::Mat drawHistogram(const cv::Mat& hist, bool autoscale = true, float max = 0.0);
bool containsNanOrInf(const cv::Mat& mat);
pcl::PointCloud<pcl::PointXYZI> sampleCloud(const pcl::PointCloud<pcl::PointXYZI>& cloud, unsigned int samples);
pcl::PointCloud<pcl::PointXYZI> cropBox(const pcl::PointCloud<pcl::PointXYZI>& cloud, const Eigen::Vector4f& min, const Eigen::Vector4f& max);
uchar interpolate(const cv::Mat& img, cv::Point2f pt);
uchar interpolate(const cv::Mat& img, Eigen::Vector2d pt);

pcl::PointCloud<pcl::PointXYZI> cutReflectance(const pcl::PointCloud<pcl::PointXYZI>& cloud, float min, float max);
void normalizeReflectance(pcl::PointCloud<pcl::PointXYZI>& cloud, bool autoscale = true, float max = 255.0f);

}
}

#endif
