from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument, RegisterEventHandler
from launch.launch_description_sources import get_launch_description_from_python_launch_file
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from os.path import join
from launch.substitutions import LaunchConfiguration, Command
from launch.event_handlers import OnProcessStart


import xacro


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    base_path = get_package_share_directory("scorpion")

    # This allows us to use the with_sensors variable in substitutions in this launch description.

    models_path = join(get_package_share_directory("scorpion"), "worlds_and_models")

    # Start a simulation with the cafe world
    cafe_world_uri = join(models_path,"scorp_world.sdf")
    path = join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
    
    gazebo_sim = IncludeLaunchDescription(path,
                                          launch_arguments=[("gz_args",   cafe_world_uri )])

    # Create a robot in the world.
    # Steps: 
    # 1. Process a file using the xacro tool to get an xml file containing the robot description.
    # 2. Publish this robot description using a ros topic so all nodes can know about the joints of the robot. 
    # 3. Spawn a simulated robot in the gazebo simulation using the published robot description topic. 

    # Step 1. Process robot file. 
    robot_file = join(models_path, "scorpion","scorpion.urdf.xacro")
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
        cmd=["ros2", "run", "ros_gz_sim", "create", "-topic", "robot_description", " -x 1.9 -y -1.2 -z 0.95"],
        name="spawn robot",
        output="both"
    )

    # A gui tool for easy tele-operation.
    robot_steering = Node(
        package="rqt_robot_steering",
        executable="rqt_robot_steering",
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name="bridge_1",
        arguments=[  
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',

                   ],
        output='screen'
    )


    sim_time = DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock')
    
    

    controllers = IncludeLaunchDescription(join(base_path, "launch","controllers.launch.py"))


    return LaunchDescription([gazebo_sim, sim_time,bridge,  robot_steering, robot_state_publisher, controllers])