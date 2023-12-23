from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch import LaunchDescription


def generate_launch_description():

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_diff_drive_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'diff_drive_base_controller'],
        output='screen'
    )

    load_tail_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'tail_controller'],
        output='screen'
    )

    return LaunchDescription([load_diff_drive_controller,  load_tail_controller, load_joint_state_controller])