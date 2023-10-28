from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('pure_pursuit'),
        'config',
        'params.yaml'
        )
    config_dict = yaml.safe_load(open(config, 'r'))
    pure_pursuit_node = Node(
        package='pure_pursuit',
        executable='pure_pursuit_node',
        parameters=[{'pose_topic': config_dict['topics']['pose_topic']},
                    {'target_waypoint_topic': config_dict['topics']['target_waypoint_topic']},
                    {'drive_topic': config_dict['topics']['drive_topic']},
                    {'waypoints_filename': config_dict['waypoints_filename']},
                    {'lookahead': config_dict['lookahead']},
                    {'speed_gain': config_dict['speed_gain']}]
    )

    # finalize
    ld.add_action(pure_pursuit_node)
    return ld