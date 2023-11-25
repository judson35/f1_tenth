from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    ld = LaunchDescription()
    mpc_node = Node(
        package='mpc',
        executable='mpc_node.py',
    )

    # finalize
    ld.add_action(mpc_node)
    return ld