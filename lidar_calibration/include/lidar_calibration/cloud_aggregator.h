//=================================================================================================
// Copyright (c) 2016, Stefan Kohlbrecher, Martin Oehler, TU Darmstadt
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
#ifndef CLOUD_AGGREGATOR_H
#define CLOUD_AGGREGATOR_H

// ros
#include <ros/ros.h>
#include <laser_geometry/laser_geometry.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_srvs/Empty.h>
#include <hector_calibration_msgs/RequestScans.h>

// tf
#include <tf/transform_listener.h>

// pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl_conversions/pcl_conversions.h>

#include <pcl_ros/transforms.h>

namespace hector_calibration {

namespace lidar_calibration {

typedef std::pair<boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>, double> pc_roll_tuple;

/**
 * Subscribes to rotating LIDAR clouds and publishes and asssembles
 * aggregated clouds 
 */
class CalibrationCloudAggregator
{
public:
  CalibrationCloudAggregator();
  void publishClouds();

  void setPeriodicPublishing(bool status, double period);
private:
  void timerCallback(const ros::TimerEvent&);
  void cloudCallback (const sensor_msgs::PointCloud2::ConstPtr& cloud_in);
  void resetCallback(const std_msgs::Empty::ConstPtr&);
  bool requestScansCallback(hector_calibration_msgs::RequestScans::Request& request,
                              hector_calibration_msgs::RequestScans::Response& response);
  bool resetSrvCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
  void publishCloud(const ros::Publisher& pub, sensor_msgs::PointCloud2 &cloud_msg);

protected:
  void resetClouds();
  void savePointCloud(const sensor_msgs::PointCloud2::ConstPtr& pc_msg, const tf::StampedTransform& transform);
  void transformCloud(const std::vector<pc_roll_tuple>& cloud_agg, sensor_msgs::PointCloud2& cloud);
  void scanToMsg(const std::vector<pc_roll_tuple>& cloud_agg, sensor_msgs::PointCloud2& scan, std_msgs::Float64MultiArray& angles);
  ros::NodeHandle nh_;
  ros::Subscriber scan_sub_;
  ros::Subscriber reset_sub_;
  ros::Publisher point_cloud1_pub_;
  ros::Publisher point_cloud2_pub_;

  ros::ServiceServer request_scans_srv_;
  ros::ServiceServer reset_clouds_srv_;

  boost::shared_ptr<tf::TransformListener> tfl_;
  ros::Duration wait_duration_;

  // bool p_use_high_fidelity_projection_;
  std::string p_target_frame_;

  double prior_roll_angle_;

  unsigned int captured_clouds_;
  int rotations_;

  std::vector<pc_roll_tuple> cloud_agg1_;
  std::vector<pc_roll_tuple> cloud_agg2_;

  sensor_msgs::PointCloud2 half_scan1_;
  std::vector<double> angles1_;
  sensor_msgs::PointCloud2 half_scan2_;
  std::vector<double> angles2_;

  std::string laser_frame_;

  sensor_msgs::PointCloud2 cloud1_;
  sensor_msgs::PointCloud2 cloud2_;

  ros::Timer timer_;
};

}
}
#endif

