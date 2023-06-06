import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    path="~/ECN-1-2-Robot/ros2/src/global_params.yaml"

    gui = Node(
        package = 'workstation',
        executable = 'gui.py',
        output = 'screen',
        parameters=[path]
    )

    # Launch
    return LaunchDescription([
		gui
    ])

