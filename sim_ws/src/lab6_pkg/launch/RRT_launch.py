from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('lab6_pkg'),
        'config',
        'params.yaml'
        )
    config_dict = yaml.safe_load(open(config, 'r'))
    pure_pursuit_node = Node(
        package='lab6_pkg',
        executable='rrt_node',
        parameters=[{'pose_topic': config_dict['topics']['pose_topic']},
                    {'scan_topic': config_dict['topics']['scan_topic']},
                    {'drive_topic': config_dict['topics']['drive_topic']},
                    {'occupancy_grid_topic': config_dict['topics']['occupancy_grid_topic']},
                    {'waypoints_filename': config_dict['waypoints_filename']},
                    {'occupancy_grid_density': config_dict['occupancy_grid']['density']},
                    {'occupancy_grid_obstacle_padding': config_dict['occupancy_grid']['obstacle_padding']},
                    {'occupancy_grid_max_range': config_dict['occupancy_grid']['max_range']},
                    {'occupancy_grid_threshold': config_dict['occupancy_grid']['threshold']}]
    )

    # finalize
    ld.add_action(pure_pursuit_node)
    return ld