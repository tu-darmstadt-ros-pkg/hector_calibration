#include <multi_lidar_calibration/multi_lidar_calibration.h>

// using namespace std::chrono_literals;

// class MultiLidarCalibrationNode
// {
//   public:
//     MultiLidarCalibrationNode()
//       : node_(std::make_shared<rclcpp::Node>("multi_lidar_calibration_node")), initialized_(false)
//     {
//       this->initialize();
//     }

//     std::shared_ptr<rclcpp::Node> get_node() const {
//       return node_;
//     }

//   private:
//     void initialize() {
//       mlc_ = std::make_shared<hector_calibration::lidar_calibration::MultiLidarCalibration>(node_->shared_from_this());
//       RCLCPP_INFO(node_->get_logger(), "Waiting for point clouds.");
//       cloud1_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
//         "cloud1", 1, std::bind(&MultiLidarCalibrationNode::cloud1_cb, this, std::placeholders::_1));
//       cloud2_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
//         "cloud2", 1, std::bind(&MultiLidarCalibrationNode::cloud2_cb, this, std::placeholders::_1));

//       node_->declare_parameter("min_scans", 1);
//       node_->get_parameter("min_scans", min_scans_);
      
//       rclcpp::Rate rate(10);

//       while (rclcpp::ok() && (scan1_counter_ < 2 || scan2_counter_ < 2)) {
//         rclcpp::spin_some(node_);
//         rate.sleep();
//       }
//     }

//     void cloud1_cb(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg_ptr) {
//       if (scan1_counter_ < min_scans_) {                
//         pcl::PointCloud<pcl::PointXYZ> tmp_cloud;
//         pcl::fromROSMsg(*cloud_msg_ptr, tmp_cloud);
//         cloud1_ += tmp_cloud;
//         scan1_counter_++;
//         RCLCPP_INFO(node_->get_logger(), "topic %s: received cloud %d/%d | total points: %ld", cloud1_sub_->get_topic_name(),
//          scan1_counter_, min_scans_, cloud1_.size());
//       }
//       if (scan1_counter_ >= min_scans_ && scan2_counter_ >= min_scans_) {
//         cloud1_sub_.reset();
//         mlc_->calibrate(cloud1_, cloud2_);
//         // rclcpp::shutdown();
//       }
//     }    

//     void cloud2_cb(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg_ptr) {
//       if (scan2_counter_ < min_scans_) {
//         pcl::PointCloud<pcl::PointXYZ> tmp_cloud;
//         pcl::fromROSMsg(*cloud_msg_ptr, tmp_cloud);
//         cloud2_ += tmp_cloud;
//         scan2_counter_++;
//         RCLCPP_INFO(node_->get_logger(), "topic %s: received cloud %d/%d | total points: %ld", cloud2_sub_->get_topic_name(),
//         scan2_counter_, min_scans_, cloud2_.size());
//       }
//       if (scan1_counter_ >= min_scans_ && scan2_counter_ >= min_scans_) {
//         cloud2_sub_.reset();
//       }
//     }    

//     std::shared_ptr<rclcpp::Node> node_;
//     rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud1_sub_;
//     rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud2_sub_;
//     std::shared_ptr<hector_calibration::lidar_calibration::MultiLidarCalibration> mlc_;
//     rclcpp::TimerBase::SharedPtr timer_;

//     pcl::PointCloud<pcl::PointXYZ> cloud1_;
//     pcl::PointCloud<pcl::PointXYZ> cloud2_;
//     unsigned int scan1_counter_ = 0;
//     unsigned int scan2_counter_ = 0;
//     unsigned int min_scans_ = 3;

//     bool initialized_;
// };

using namespace hector_calibration::lidar_calibration;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);  
  rclcpp::spin(std::make_shared<MultiLidarCalibration>());
  rclcpp::shutdown();
  return 0;
}
