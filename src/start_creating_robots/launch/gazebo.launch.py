from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import get_launch_description_from_python_launch_file
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from os.path import join

def generate_launch_description():


    path = join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")

    gazebo_sim = IncludeLaunchDescription(path,
                                          launch_arguments=[("gz_args", "shapes.sdf")])
    
    return LaunchDescription([gazebo_sim])