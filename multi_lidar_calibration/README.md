# Hector Calibration - Multi Lidar Calibration

This package provides a tool for mutually calibrating two lidar sensors. 

The nodes subscribes to two point cloud topics. The (accumulated) cloud from topic 2 is then registered with the (accumulated) cloud from topic 1.
As a result you will get the transform from the first to second frame.
An IMU topic can be used for gravitiy alignment - this makes the visualization in rviz more understandable, but is not required.

## How to Use

Run 

    ros2 launch multi_lidar_calibration dual_livox_calibration.launch.py

Set topic remappings and arguments either in the launch file or via command line.

Works best in structured environment with salient features like walls or plane ceiling.

### TODO 
write tests
