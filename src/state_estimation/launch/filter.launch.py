import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    package_dir = get_package_share_directory('state_estimation')

    # Path to our robot_localization configuration file
    ekf_config = os.path.join(
        package_dir,
        'config',
        'orientation_ekf.yaml'
    )

    bag_path = '/home/tommy/GeoDesiC_ws/src/bagged_data/rosbag2_2026_09_24-15_50_32_0.mcap'

    bag_player = ExecuteProcess(
        cmd=[
            'ros2',
            'bag',
            'play',
            bag_path,
            '--clock'
        ],
        output='screen'
    )
    
    imu_static_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_static_transform_publisher',
        parameters=[{
            'use_sim_time': True
        }],
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.0',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'imu'
        ],
        output='screen'
    )


    madgwick_filter = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_madgwick',
        output='screen',

        parameters=[{
            'use_mag': False,
            'use_sim_time': True,
            'publish_tf': False
        }],

        remappings=[
            ('imu/data_raw', '/imu_broadcaster/imu'),
            ('imu/data', '/imu/data')
        ]
    )


    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',

        parameters=[
            ekf_config,
            {'use_sim_time': True}
        ]
    )

    return LaunchDescription([
        imu_static_transform,
        madgwick_filter,
        ekf,
        bag_player
    ])