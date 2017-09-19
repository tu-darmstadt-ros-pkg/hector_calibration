# initial transform (calibration) between base_link and camera head
geometry_msgs/TransformStamped cam_transform

# accumulated XYZI cloud
sensor_msgs/PointCloud2 scan

# Corresponding camera images
# Same ordering as cameras in camera config
CameraObservation[] camera_observations