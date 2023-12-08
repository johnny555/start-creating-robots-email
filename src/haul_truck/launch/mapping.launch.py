from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnExecutionComplete

from os.path import join


def generate_launch_description():
    # This allows us to have the with_sensors as an argument on the command line
    rviz_config_arg =  DeclareLaunchArgument(
        'rviz_config', default_value="mapping.yaml"
    )
    # This allows us to use the with_sensors variable in substitutions in this launch description.
    rviz_config = LaunchConfiguration('rviz_config', default="vis.rviz")

    base_path = get_package_share_directory("haul_truck")
        
    # We will include everything from the gazebo launch file, making sure that sensors are now enabled however.

    gazebo = IncludeLaunchDescription(join(base_path, "launch","gazebo.launch.py"))

    # SLAM Toolbox for mapping
    slam_toolbox = Node( package='slam_toolbox', 
                         executable='async_slam_toolbox_node', 
                         parameters=[
                                get_package_share_directory('haul_truck') + '/config/mapping.yaml'
                        ], output='screen'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=[
            '-d',
            PathJoinSubstitution([base_path, 'config', rviz_config])
        ]
    )


    static_tf = Node(package='tf2_ros', 
                                 executable='static_transform_publisher', 
                                 name='base_fp_linkTF', 
                                 output='log', 
                                 arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'lidar_3d_v1',  'haul_truck/base_footprint/lidar_3d_v1', ])

    return LaunchDescription([gazebo,                              
                                slam_toolbox, 
                                rviz, 
                                rviz_config_arg, static_tf])
