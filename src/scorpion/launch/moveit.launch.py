from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument, RegisterEventHandler
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from os.path import join
from launch.event_handlers import OnProcessStart


def generate_launch_description():

    # This demo script may also be trying to start another ros2 controller!!!! Need to pull it apart!  
    move_group = IncludeLaunchDescription(join(get_package_share_directory("scorpion_moveit"), "launch","move_group.launch.py"), 
                                    launch_arguments=[("use_sim_time","true")])

    set_sim_time = IncludeLaunchDescription(join(get_package_share_directory("scorpion"), "launch","set_sim_time.launch.py"))

    rviz = IncludeLaunchDescription(join(get_package_share_directory("scorpion_moveit"), "launch","moveit_rviz.launch.py"), 
                                    launch_arguments=[("use_sim_time","true")])
    
    static_joint = IncludeLaunchDescription(join(get_package_share_directory("scorpion_moveit"), "launch","static_virtual_joint_tfs.launch.py"), 
                                    launch_arguments=[("use_sim_time","true")])
    

    return LaunchDescription([move_group, rviz, set_sim_time , static_joint ])