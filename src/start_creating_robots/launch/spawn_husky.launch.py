from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import get_launch_description_from_python_launch_file
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from os.path import join

def generate_launch_description():

    models_path = join(get_package_share_directory("start_creating_robots"), "worlds_and_models")
    cafe_world_uri = join(models_path,"cafe.sdf")
    path = join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")

    spawn_husky = Node(
            name='spawn_husky',
            package='ros_gz_sim',
            executable='create',
            output='screen',
            parameters=[{
                '-world': 'cafe_world',
                '-file': join(models_path, 'husky.sdf')
                }],
        )

    

    return LaunchDescription([spawn_husky])
