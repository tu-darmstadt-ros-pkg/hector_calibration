import launch
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir, PathJoinSubstitution

def generate_launch_description():

    # Declare launch arguments
    use_imu_arg = DeclareLaunchArgument(
        'use_imu', default_value='true', description='Use IMU data for gravity alignment of point clouds'
    )

    init_guess_rpy_arg = DeclareLaunchArgument(
        'init_guess_rpy', default_value='[60.0, 0.0, 0.0]', description='Initial guess for roll, pitch, yaw in degrees'
    )

    init_guess_xyz_arg = DeclareLaunchArgument(
        'init_guess_xyz', default_value='[0.0, -0.0, 0.0]', description='Initial guess for x, y, z in mm'
    )

    min_scans_arg = DeclareLaunchArgument(
        'min_scans', default_value='1', description='Minimum number of scans per topic before starting calibration'
    )
    max_sqr_dist_arg = DeclareLaunchArgument(
        'max_sqr_dist', default_value='0.1', description='Maximum square distance for matching points between two point clouds'
    )
    neighbor_mapping_vis_count_arg = DeclareLaunchArgument(
        'neighbor_mapping_vis_count', default_value='100'
    )
    normals_radius_arg = DeclareLaunchArgument(
        'normals_radius', default_value='0.15'
    )
    crop_distance_arg = DeclareLaunchArgument(
        'crop_dist', default_value='1.0'
    )
    voxel_leaf_size_arg = DeclareLaunchArgument(
        'voxel_leaf_size', default_value='0.10', description='Voxel grid leaf size [m] for downsampling point clouds. 0 means no downsampling'
    )
    max_iterations_arg = DeclareLaunchArgument(
        'max_iterations', default_value='20'
    )
    parameter_diff_threshold_arg = DeclareLaunchArgument(
        'parameter_diff_thresh', default_value='1e-4'
    )
    save_path_arg = DeclareLaunchArgument(
        'save_path', default_value=''
    )

    # Create launch configurations
    use_imu = LaunchConfiguration('use_imu')
    init_guess_rpy = LaunchConfiguration('init_guess_rpy')
    init_guess_xyz = LaunchConfiguration('init_guess_xyz')
    min_scans = LaunchConfiguration('min_scans')
    max_sqr_dist = LaunchConfiguration('max_sqr_dist')
    neighbor_mapping_vis_count = LaunchConfiguration('neighbor_mapping_vis_count')
    normals_radius = LaunchConfiguration('normals_radius')
    crop_distance = LaunchConfiguration('crop_dist')
    voxel_leaf_size = LaunchConfiguration('voxel_leaf_size')
    max_iterations = LaunchConfiguration('max_iterations')
    parameter_diff_threshold = LaunchConfiguration('parameter_diff_thresh')
    save_path = LaunchConfiguration('save_path')

    # Define the node with parameters
    multi_lidar_calibration_node = Node(
        package='multi_lidar_calibration',
        executable='multi_lidar_calibration_node',
        name='multi_lidar_calibration_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'use_imu': use_imu,
            'init_guess_rpy': init_guess_rpy,
            'init_guess_xyz': init_guess_xyz,
            'min_scans': min_scans,
            'max_sqr_dist': max_sqr_dist,
            'neighbor_mapping_vis_count': neighbor_mapping_vis_count,
            'normals_radius': normals_radius,
            'crop_dist': crop_distance,
            'voxel_leaf_size': voxel_leaf_size,
            'max_iterations': max_iterations,
            'parameter_diff_thresh': parameter_diff_threshold,
            'save_path': save_path
        }],
        remappings=[
            ('/cloud1', '/livox_lidar/front/points'),
            ('/cloud2', '/livox_lidar/back/points'),
            ('/imu1', '/livox_lidar/front/imu'),
        ]
    )

    # rviz node
    rviz_config = PathJoinSubstitution([ThisLaunchFileDir(), '../rviz', 'multi_lidar_calibration.rviz'])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )

    return launch.LaunchDescription([
        use_imu_arg,
        init_guess_rpy_arg,
        init_guess_xyz_arg,
        min_scans_arg,
        max_sqr_dist_arg,
        neighbor_mapping_vis_count_arg,
        normals_radius_arg,
        crop_distance_arg,
        voxel_leaf_size_arg,
        max_iterations_arg,
        parameter_diff_threshold_arg,
        save_path_arg,
        multi_lidar_calibration_node,
        rviz_node
    ])

