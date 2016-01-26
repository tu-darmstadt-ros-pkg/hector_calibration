# lidar_calibration

## Installation

Install following dependencies for ceres:

```
sudo apt-get install libgoogle-glog-dev libatlas-base-dev libeigen3-dev libsuitesparse-dev
```

## Usage
### Creating a launch file

#### cloud_aggregator_node

**Parameters**

| Parameter | Type | Default | Description |
|:-----|:-----|:-----|:-----|
| target_frame | String | "base_link" |Fixed frame for point clouds (actuator frame). |
| rotations | Integer | 1 |Number of rotations to accumulate. |
| tf_wait_duration | Double | 0.5 | Time to wait for transforms. |


**Subscribtions**

| Topic Name | Type | Description |
|:-----|:-----|:-----|
| cloud | sensor_msgs::PointCloud2 | Input for laser scan data. |
| reset_clouds | std_msgs::Empty | Resets the saved point clouds. |

**Publications**

| Topic Name | Type | Description |
|:-----|:-----|:-----|
| half_scan_1 | sensor_msgs::PointCloud2 | First accumulated point cloud. |
| half_scan_2 | sensor_msgs::PointCloud2 | Second accumulated point cloud. |

**Service Servers**

| Service name | Type | Description |
|:-----|:-----|:-----|
| reset_clouds | std_srvs::Empty | Resets clouds |
| request_scans | hector_calibration_msgs::RequestScans | Requests accumulated point clouds in laser frame. |

#### lidar_calibration_node

**Parameters**

| Parameter | Type | Default | Description |
|:-----|:-----|:-----|:-----|
| actuator_frame | String | "lidar_actuator_frame" | Name of the actuator frame. |
| max_iterations | Integer | 20 | Maximum number of outer iterations. |
| max_sqrt_neighbor_dist | Double | 0.1 | Maximum squared distance of a neighbor. Increase this if not enough neighbors are found. |
| sqrt_convergence_diff_thres | Double | 1e-6 | If the squared change between the current and last calibration is smaller, iteration stops. |
| normals_radius | Double | 0.07 |Radius used to estimate surface normals. |
| detect_ground_plane | Boolean | false | If enabled, calibrates roll-angle by detecting and rectifying the ground plane. |
| save_calibration | Boolean | false | If enabled, saves the calibration as an urdf origin-block to the location specified by *save_path*. |
| save_path | String | "" | Full save path for calibration file. |
| rotation_offset_roll | Double | 0.0 | Specify an offset if your laser frame doesn't follow ros conventions (rotate around x-axis). |
| rotation_offset_pitch | Double | 0.0 | Specify an offset if your laser frame doesn't follow ros conventions (rotate around x-axis). |
| rotation_offset_yaw | Double | 0.0 | Specify an offset if your laser frame doesn't follow ros conventions (rotate around x-axis). |
| o_spin_frame | String | "" | Base frame where calibration has to be applied. Will only affect saved calibration. |
| o_laser_frame | String | "" | Target frame where calibration has to be applied. Will only affect saved calibration. |
| tf_wait_duration | Double | 5 | Duration to wait for transformation between *o_spin_frame* and *o_laser_frame*. |

**Publications**

| Topic Name | Type | Description |
|:-----|:-----|:-----|
| result_cloud_1 | sensor_msgs::PointCloud2 | Calibration results after each step of cloud 1. |
| result_cloud_2 | sensor_msgs::PointCloud2 | Calibration results after each step of cloud 2. |
| ground_plane | sensor_msgs::PointCloud2 | Publishes the detected ground plane, if detection is activated. |
| neighbor_mapping | visualization_msgs::MarkerArray | Visualizes the found neighbor mapping with arrows. |
| planarity | visualization_msgs::MarkerArray | Visualizes the planarity (weight) of normals. |

**Service Clients**

| Service name | Type | Description |
|:-----|:-----|:-----|
| reset_clouds | std_srvs::Empty | Resets clouds of aggregator |
| request_scans | hector_calibration_msgs::RequestScans | Requests accumulated point clouds from aggregator. |

#### Interpreting the results
