from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():
    ld = LaunchDescription()
    alarm_pub = Node(
        package = 'alarm',
        executable = 'alarm_pub',
        name = 'sim'
    )

    alarm_sub = Node(
        package = 'alarm',
        executable = 'alarm_sub',
        name = 'sim'

    )

    ld.add_action(alarm_pub)
    ld.add_action(alarm_sub)

    return ld
    
    
    from setuptools import find_packages, setup