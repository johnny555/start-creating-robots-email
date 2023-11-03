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

    base_path = get_package_share_directory("start_creating_robots")
        
    # We will include everything from the gazebo launch file, making sure that sensors are now enabled however.

    with_sensors_true = SetLaunchConfiguration("with_sensors","true")
    gazebo = IncludeLaunchDescription(join(base_path, "launch","gazebo.launch.py"),
                                      launch_arguments=[('with_sensors','true')])

    # Extended Gazebo Bridge: To do mapping requires alot more info from the simulation. We need sensor data and estimates of the robot joint positions.
    extended_bridge = Node( package='ros_gz_bridge', name="extended_gazebo_bridge", executable='parameter_bridge', 
    arguments=['/model/krytn/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                   '/model/krytn/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',

                   '/lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                   '/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',

                   '/realsense/image@sensor_msgs/msg/Image[gz.msgs.Image',
                   '/realsense/depth@sensor_msgs/msg/Image[gz.msgs.Image',
                   '/realsense/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',

                   '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',

                   '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model'], 
                   output='screen', remappings=[('/model/krytn/odometry','/odom'),
                    ('/model/krytn/tf','/tf')]
    )

    # Gazebo fortress has a bug that won't respect our frame_id tags. So we have to publish a transform 
    depth_cam_link_tf = Node(package='tf2_ros', 
                             executable='static_transform_publisher', 
                             name='depthCamLinkTF', 
                             output='log', 
                             arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 
                             'realsense_d435', 'krytn/base_footprint/realsense_d435'])

    krytn_base_fp_link_tf = Node(package='tf2_ros', 
                                 executable='static_transform_publisher', 
                                 name='base_fp_linkTF', 
                                 output='log', 
                                 arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0',  'krytn/base_footprint', 'base_footprint'])

    # SLAM Toolbox for mapping
    slam_toolbox = Node( package='slam_toolbox', 
                         executable='async_slam_toolbox_node', 
                         parameters=[
                                get_package_share_directory('start_creating_robots') + '/config/mapping.yaml'
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


    return LaunchDescription([with_sensors_true, 
                                gazebo,
                                extended_bridge, 
                                depth_cam_link_tf, 
                                krytn_base_fp_link_tf,
                                slam_toolbox, 
                                rviz, 
                                rviz_config_arg])