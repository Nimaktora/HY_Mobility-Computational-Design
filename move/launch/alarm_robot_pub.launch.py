import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import xacro

# this is the function launch  system will look for
def generate_launch_description():
    gcamp_gazebo_pkg_name = 'gcamp_gazebo'
    gcamp_gazebo_pkg_path = os.path.join(get_package_share_directory(gcamp_gazebo_pkg_name))
    #urdf_file = os.path.join(gcamp_gazebo_pkg_path, "urdf", robot_file)

    alarm_pkg_name = 'alarm'
    alarm_pkg_path = os.path.join(get_package_share_directory(alarm_pkg_name))

    gcamp_world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gcamp_gazebo_pkg_path, 'launch', 'gcamp_world.launch.py'))
    )
    alarm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(alarm_pkg_path, 'launch', 'alarm.launch.py'))
    )
    move_node = Node(
        package = 'move',
        executable = 'alarm_cmd_vel_node',
        remappings = [
            ('cmd_vel', '/skidbot/cmd_vel'),
        ]
    )


    return LaunchDescription({
        alarm_launch,
        gcamp_world_launch,
        move_node,
        })
