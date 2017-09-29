std_msgs/String name  # name of the camera
sensor_msgs/Image image # captured image 
sensor_msgs/Image mask # mask for image, 1 is a valid pixel
geometry_msgs/TransformStamped transform  # transform from camera head to this camera
