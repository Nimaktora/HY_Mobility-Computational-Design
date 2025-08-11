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
    emergency_node = Node(
        package = 'midterm',
        executable = 'emergency_node',
    )
    wpdriving_node = Node(
        package = 'midterm',
        executable = 'wpdriving_node',
    )



    return LaunchDescription({
        emergency_node,
        wpdriving_node,
        })

