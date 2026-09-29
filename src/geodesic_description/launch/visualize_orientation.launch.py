import os
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Create path directories to the directory and files inside such as urdf and rviz configs
    desc_dir = get_package_share_directory('geodesic_description')
    xacro_file = os.path.join(desc_dir, 'urdf', 'simple_quadcopter.urdf.xacro')
    rviz_config = os.path.join(desc_dir, 'rviz', 'quadcopter.rviz')
    
    # Convert the .urdf.xacro to a URDF XML file as the robot_description parameter
    robot_description = ParameterValue(
        Command(['xacro', ' ', xacro_file]), value_type=str)

    return LaunchDescription([
        # Initialize robot state publisher to publish the transformations to tf
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description,
                         'use_sim_time': True}]
        ),
        
        # Initialize joint state publisher to publish state of joints that are continuous
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        
        # Launch Rviz2 with our custom configuration file
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}]
        ),
    ])