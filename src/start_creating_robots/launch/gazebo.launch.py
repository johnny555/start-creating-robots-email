from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import get_launch_description_from_python_launch_file
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from os.path import join

import xacro


def generate_launch_description():

    models_path = join(get_package_share_directory("start_creating_robots"), "worlds_and_models")
    cafe_world_uri = join(models_path,"cafe.sdf")
    path = join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
    robot_file = join(models_path, 'robot.sdf')

    #cafe_world_uri = 'empty.sdf'
    gazebo_sim = IncludeLaunchDescription(path,
                                          launch_arguments=[("gz_args", '-r ' + cafe_world_uri)])
    

    robot = ExecuteProcess(
        cmd=["ros2", "run", "ros_gz_sim", "create", "-file", robot_file, "-z", "0.5"],
        name="spawn robot",
        output="both"
    )

    robot_description_config = xacro.process_file(
        robot_file
    )
    robot_description = {'robot_description': robot_description_config.toxml()}

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )

    # Bridge
    lidar_gazebo_topic_preamble = '/world/cafe_world/model/robot/model/lidar_2d_v1/link/link/sensor/lidar_2d_v1'
    camera_gazebo_topic_preamble = '/world/cafe_world/model/robot/model/realsense_d435/link/link/sensor/realsense_d435'
    
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/model/magni/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                   '/model/magni/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                   lidar_gazebo_topic_preamble + '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                   lidar_gazebo_topic_preamble + '/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
                   camera_gazebo_topic_preamble + '/image@sensor_msgs/msg/Image[gz.msgs.Image',
                   camera_gazebo_topic_preamble + '/depth@sensor_msgs/msg/Image[gz.msgs.Image',
                   camera_gazebo_topic_preamble + '/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
                   '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                   '/model/robot/model/magni/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                   '/world/cafe_world/model/robot/joint_state@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                   '/world/cafe_world/model/robot/model/magni/joint_state@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                   '/world/cafe_world/model/robot/model/realsense_d435/joint_state@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                   '/world/cafe_world/model/robot/model/lidar_2d_v1/joint_state@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'],
        output='screen',
        remappings=[(lidar_gazebo_topic_preamble + '/scan','/scan'), 
                    (lidar_gazebo_topic_preamble + '/scan/points','/scan/points'), 
                    (camera_gazebo_topic_preamble + '/image' ,    '/realsense/image'),
                    (camera_gazebo_topic_preamble + '/depth' ,    '/realsense/depth'),
                    (camera_gazebo_topic_preamble + '/points',    '/realsense/points'),
                    ('/model/robot/model/magni/pose','/tf'),
                    ('/world/cafe_world/model/robot/joint_state','/tf'),
                    ('/world/cafe_world/model/robot/model/magni/joint_state','/tf'),
                    ('/world/cafe_world/model/robot/model/realsense_d435/joint_state','/tf'),
                    ('/world/cafe_world/model/robot/model/lidar_2d_v1/joint_state', '/tf')
                    ]
    )

    robot_steering = Node(
        package="rqt_robot_steering",
        executable="rqt_robot_steering",
        remappings=[('/cmd_vel','/model/magni/cmd_vel')]
    )

    return LaunchDescription([gazebo_sim, bridge, robot, robot_steering, robot_state_publisher])