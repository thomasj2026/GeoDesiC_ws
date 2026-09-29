# filter.launch.py is the bringup file for the orientation state estimatuon of the IMU bagged data

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Created file directories 
    state_dir = get_package_share_directory('state_estimation')

    # Paths to EKF confiugration and URDF model
    ekf_config = os.path.join(state_dir,'config', 'orientation_ekf.yaml')
    covariance_config = os.path.join(state_dir, 'config', 'imu_covariance.yaml')
    
    # Path to bagged data
    bag_path = '/home/tommy/GeoDesiC_ws/src/bagged_data/rosbag2_2026_09_24-15_50_32_0.mcap'

    # Use madgwick filter to estimate orientation of IMU in quarternions
    madgwick_filter = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_madgwick',
        output='screen',

        parameters=[{
            'use_mag': False,       # No magnetometer
            'use_sim_time': True,   # Want to utilize the bagged data clock so nothing is "out of sync"
            'publish_tf': False     # Turn off tf publisher or else it conflicts with robot_state_publisher
        }],

        remappings=[
            ('imu/data_raw', '/imu_broadcaster/imu'),
            ('imu/data', '/imu/data')
        ]
    )

    # Initalize our Extended Kalman Filter Node
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',

        parameters=[
            ekf_config,
            {'use_sim_time': True}      # Use the bagged data time frame
        ]
    )
    
    imu_covariance_node = Node(
        package = 'state_estimation',
        executable = 'imu_covariance_node.py',
        name = 'imu_covariance_node',
        output = 'screen',
        parameters = [
            covariance_config,
            {'use_sim_time': True}
        ]
    )
    
    # The actual bag process
    bag_process = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', bag_path, '--clock', '100'],
        output='screen'
    )

    # Delay the bag so RViz and the filters are ready
    bag_player = TimerAction(
        period=3.0,
        actions=[bag_process]
    )

    # When the bag process finishes, shut down this whole launch
    shutdown_on_bag_end = RegisterEventHandler(
        OnProcessExit(
            target_action=bag_process,
            on_exit=[EmitEvent(event=Shutdown(reason='bag finished'))]
        )
    )

    return LaunchDescription([
        madgwick_filter,
        ekf,
        imu_covariance_node,
        bag_player,
        shutdown_on_bag_end,
    ])