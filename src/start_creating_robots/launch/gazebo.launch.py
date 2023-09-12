from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import get_launch_description_from_python_launch_file
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from os.path import join

def generate_launch_description():

    models_path = join(get_package_share_directory("start_creating_robots"), "worlds_and_models")
    cafe_world_uri = join(models_path,"cafe.sdf")
    path = join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
    robot_file = join(models_path, 'robot.sdf')

    gazebo_sim = IncludeLaunchDescription(path,
                                          launch_arguments=[("gz_args", cafe_world_uri)])
    
    spawn_robot = Node(
            name='spawn_robot',
            package='ros_gz_sim',
            executable='create',
            output='screen',
           
            arguments =[ f"-world cafe_world -file '{robot_file}' -z 0.5"],
            
        )
    robot = ExecuteProcess(
        cmd=["ros2", "run", "ros_gz_sim", "create", "-file", robot_file, "-z", "0.5"],
        name="spawn robot",
        output="both"
    )

       # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/model/X1/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                   '/model/X1/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry'],
        output='screen'
    )



    return LaunchDescription([gazebo_sim, robot, bridge])