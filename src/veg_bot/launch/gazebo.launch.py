from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import get_launch_description_from_python_launch_file
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from os.path import join
from launch.substitutions import LaunchConfiguration, Command

import xacro


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    # This allows us to have the with_sensors as an argument on the command line
    with_sensors_arg =  DeclareLaunchArgument(
        'with_sensors', default_value="false"
    )
    # This allows us to use the with_sensors variable in substitutions in this launch description.
    with_sensors = LaunchConfiguration('with_sensors', default="false")

    models_path = join(get_package_share_directory("veg_bot"), "worlds_and_models")

    # Start a simulation with the cafe world
    cafe_world_uri = join(models_path,"packing_world", "packing.sdf")
    path = join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
    
    gazebo_sim = IncludeLaunchDescription(path,
                                          launch_arguments=[("gz_args", cafe_world_uri)])

    # Create a robot in the world.
    # Steps: 
    # 1. Process a file using the xacro tool to get an xml file containing the robot description.
    # 2. Publish this robot description using a ros topic so all nodes can know about the joints of the robot. 
    # 3. Spawn a simulated robot in the gazebo simulation using the published robot description topic. 

    # Step 1. Process robot file. 
    #robot_file = join(get_package_share_directory("veg_bot"), "worlds_and_models","kuka","kr6r900sixx.xacro")

    robot_file = join(get_package_share_directory("kr6_moveit"), "config","kuka_kr6.urdf.xacro")

    robot_xml = Command(["xacro ",robot_file])

    #Step 2. Publish robot file to ros topic /robot_description
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{'robot_description':robot_xml, 
                     'use_sim_time':True}],
    )

    # Step 3. Spawn a robot in gazebo by listening to the published topic.
    robot = ExecuteProcess(
        cmd=["ros2", "run", "ros_gz_sim", "create", "-topic", "robot_description"],
        name="spawn robot",
        output="both"
    )

    # Gazebo Bridge: This allows ROS to send messages to drive the robot in simulation. 
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
                  '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                   '/lemon/detach@std_msgs/msg/Empty@gz.msgs.Empty',
                   '/lemon/attach@std_msgs/msg/Empty@gz.msgs.Empty',
                   '/realsense/image@sensor_msgs/msg/Image[gz.msgs.Image',
                   '/realsense/depth@sensor_msgs/msg/Image[gz.msgs.Image',
                   '/realsense/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
                   ],
        output='screen',
    )

    depth_cam_link_tf = Node(package='tf2_ros', 
                             executable='static_transform_publisher', 
                             name='depthCamLinkTF', 
                             output='log', 
                             arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 
                             'realsense_d435', 'kr6_moveit/base_link/realsense_d435'])


    demo = IncludeLaunchDescription(join(get_package_share_directory("kr6_moveit"), "launch","demo.launch.py"), 
                                    launch_arguments=[("use_sim_time",use_sim_time)])

 
    sim_time = DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock')
    
    # Step 3. Spawn a robot in gazebo by listening to the published topic.
    set_simtime_movegroup = ExecuteProcess(
        cmd=["ros2", "param", "set", "/move_group","use_sim_time",  "True"],
        name="move group sim time",
        output="both"
    )

    # Step 3. Spawn a robot in gazebo by listening to the published topic.
    set_simtime_rviz = ExecuteProcess(
        cmd=["ros2", "param", "set", "/rviz","use_sim_time",  "True"],
        name="move group sim time",
        output="both"
    )

    notebook_dir = join(get_package_share_directory("veg_bot"), "veg_bot")
    start_notebook = ExecuteProcess(
        cmd=["cd {} && python3 -m notebook".format(notebook_dir)],
        shell=True,
        output="screen",
)

    return LaunchDescription([gazebo_sim, bridge, robot, start_notebook, depth_cam_link_tf, robot_state_publisher, with_sensors_arg, demo , sim_time, set_simtime_movegroup, set_simtime_rviz])