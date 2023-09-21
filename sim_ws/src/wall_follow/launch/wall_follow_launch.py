from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('wall_follow'),
        'config',
        'wall_follow.yaml'
        )
    config_dict = yaml.safe_load(open(config, 'r'))
    wall_follow_node = Node(
        package='wall_follow',
        executable='wall_follow_node',
        parameters=[{'kp': config_dict['PID']['kp']},
                    {'ki': config_dict['PID']['ki']},
                    {'kd': config_dict['PID']['kd']},
                    {'integral_cap': config_dict['PID']['integral_cap']},
                    {'lookahead': config_dict['lookahead']},
                    {'velocity_gain': config_dict['velocity_gain']},
                    {'scan_beams': config_dict['scan_beams']},
                    {'scan_range': config_dict['scan_range']}]
    )

    # finalize
    ld.add_action(wall_follow_node)
    return ld