from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch.substitutions import LaunchConfiguration

import os

def generate_launch_description():
    # ... your existing nodes ...

    # SLAM Toolbox for mapping
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        parameters=[
            get_package_share_directory('start_creating_robots') + '/config/mapping.yaml'
        ],
        output='screen'
    )


    return LaunchDescription([
        slam_toolbox
    ])
