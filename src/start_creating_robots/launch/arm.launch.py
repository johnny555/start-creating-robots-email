""" This launch file explores use of robot arms

"""

from launch import LaunchDescription
from launch_ros.actions import Node 
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from os.path import join
from launch.substitutions import LaunchConfiguration, Command


def generate_launch_description():

    base_path = get_package_share_directory("start_creating_robots")

    # We will include everything from the mapping launch file, making sure that sensors are now enabled and setting up RVIZ for navigation.
    nav = IncludeLaunchDescription(join(base_path, "launch","navigation.launch.py"),
                                      launch_arguments=[("with_sensors","true"), ("rviz_config","nav.rviz")])
    
    # Now, lets spawn a robot arm. Start by making a new robot description publisher
    robot_file = join(base_path,"worlds_and_models", "ur5_rg2","model.sdf")
    robot_xml = Command(["xacro ",robot_file])

    
    #Step 2. Publish robot file to ros topic /robot_description
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_ur5',
        output='both',
        parameters=[{'robot_description':robot_xml, 
                     'use_sim_time':True}],
    )

    # Step 3. Spawn a robot in gazebo by listening to the published topic. Note the arm will spawn fixed to the world.
    # We are positioning it so it is on a nearby bench.
    robot = ExecuteProcess(
        cmd=["ros2", "run", "ros_gz_sim", "create", "-topic", "robot_description", "-x=-0.8","-y=4.0","-z=1.35"],
        name="spawn robot",
        output="both"
    )


    return LaunchDescription([
        nav, robot, robot_state_publisher
    ])