from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from os.path import join

def generate_launch_description():

        # Step 3. Spawn a robot in gazebo by listening to the published topic.
    


    setters = [ExecuteProcess(
        cmd=["ros2", "param", "set", f"/{v}","use_sim_time",  "true"],
        name=f"{v} sim time",
        output="both"
    ) for v in ["move_group","joint_state_broadcaster", "rviz", "gz_ros2_control","moveit_simple_controller_manager"]]



  
    return LaunchDescription(setters)