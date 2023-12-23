from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess,DeclareLaunchArgument, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnExecutionComplete

from os.path import join


def generate_launch_description():

    base_path = get_package_share_directory("scorpion")

    gazebo = IncludeLaunchDescription(join(base_path, "launch","gazebo.launch.py"))

    moveit =  IncludeLaunchDescription(join(base_path, "launch","moveit.launch.py"))

    bridge_2 = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name="bridge_2",
        arguments=[  
           '/realsense/image@sensor_msgs/msg/Image[gz.msgs.Image',
           '/realsense/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
           '/realsense/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/realsense/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/tail_camera@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'
                   ],
        output='screen'
    )

    remappings=[
          ('rgb/image','/realsense/image'),
          ('rgb/camera_info','/realsense/camera_info'),
          ('scan_cloud', '/realsense/points'),
          ('depth/image', '/realsense/depth_image')
            ]
    
    parameters={
          'frame_id':'base_link',
          'publish_tf_odom': False,
          'use_sim_time':True,
          'subscribe_scan_cloud':True,
          'scan_cloud_topic': "/realsense/points",
          'qos_image':2,
          'qos_imu':2,
          'Optimizer/GravitySigma':'0' # Disable imu constraints (we are already in 2D)
    }

    rtabmap_slam = Node(package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters],
            remappings=remappings,
            arguments=['-d'])
    

    transform = ExecuteProcess(
        cmd=["ros2", "run", "tf2_ros", "static_transform_publisher","0", "0", "0", "3.141","3.141","3.141", "chassis_camera_link","chassis_camera_optical_link"],
        name="spawn robot",
        output="both"
    )

    return LaunchDescription([ gazebo, moveit, bridge_2,  rtabmap_slam, transform])