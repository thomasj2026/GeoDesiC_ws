import os

from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    package_dir = get_package_share_directory(
        'geodesic_description'
    )
    
    rviz_config = os.path.join(
        package_dir,
        'rviz',
        'quadcopter.rviz')

    xacro_file = os.path.join(
        package_dir,
        'urdf',
        'simple_quadcopter.urdf.xacro'
    )

    robot_description = {
        'robot_description': Command([
            'xacro ',
            xacro_file
        ])
    }

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )
    

    return LaunchDescription([
        joint_state_publisher,
        robot_state_publisher,
        rviz
    ])