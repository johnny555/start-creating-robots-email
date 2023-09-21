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

    robot_file = join(models_path, "krytn","krytn.urdf.xacro")
    robot_description_config = xacro.process_file(robot_file)
    robot_description = {'robot_description': robot_description_config.toxml()}
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )

    gazebo_sim = IncludeLaunchDescription(path,
                                          launch_arguments=[("gz_args", '-r ' + cafe_world_uri)])
    

    robot = ExecuteProcess(
        cmd=["ros2", "run", "ros_gz_sim", "create", "-topic", "robot_description", "-z", "0.5"],
        name="spawn robot",
        output="both"
    )


    # Gazebo Bridge: This allows communication from the Gazebo world to the ROS system.
    
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/model/krytn/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                   '/model/krytn/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                   '/model/krytn/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                   '/lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                   '/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
                   '/realsense/image@sensor_msgs/msg/Image[gz.msgs.Image',
                   '/realsense/depth@sensor_msgs/msg/Image[gz.msgs.Image',
                   '/realsense/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
                   '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                   '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model'],
        output='screen',
        remappings=[('/model/krytn/odometry','/odom'),
                    ('/model/krytn/tf','/tf')]
    )

    # Gazebo fortress has a bug that won't respect our frame_id tags. So we have to publish a transform 
    depth_cam_link_tf = Node(package='tf2_ros',
                     executable='static_transform_publisher',
                     name='depthCamLinkTF',
                     output='log',
                     arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'realsense_d435', 'krytn/base_footprint/realsense_d435'])

    krytn_base_fp_link_tf = Node(package='tf2_ros',
                     executable='static_transform_publisher',
                     name='base_fp_linkTF',
                     output='log',
                     arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0',  'krytn/base_footprint', 'base_footprint'])


    robot_steering = Node(
        package="rqt_robot_steering",
        executable="rqt_robot_steering",
        remappings=[('/cmd_vel','/model/krytn/cmd_vel')]
    )

    return LaunchDescription([gazebo_sim, bridge, robot, robot_steering, robot_state_publisher, depth_cam_link_tf, krytn_base_fp_link_tf])