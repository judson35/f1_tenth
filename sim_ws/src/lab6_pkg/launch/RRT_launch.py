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
                    {'occupancy_grid_visualization_topic': config_dict['topics']['occupancy_grid_visualization_topic']},
                    {'rrt_tree_visualization_topic': config_dict['topics']['rrt_tree_visualization_topic']},
                    {'waypoints_visualization_topic': config_dict['topics']['waypoints_visualization_topic']},
                    {'target_waypoint_visualization_topic': config_dict['topics']['target_waypoint_visualization_topic']},
                    {'waypoints_filename': config_dict['waypoints_filename']},
                    {'occupancy_grid_density': config_dict['occupancy_grid']['density']},
                    {'occupancy_grid_obstacle_padding': config_dict['occupancy_grid']['obstacle_padding']},
                    {'occupancy_grid_max_range': config_dict['occupancy_grid']['max_range']},
                    {'occupancy_grid_threshold': config_dict['occupancy_grid']['threshold']},
                    {'rrt_goal_dist_threshold': config_dict['rrt']['goal_dist_threshold']},
                    {'rrt_max_expansion_dist': config_dict['rrt']['max_expansion_dist']},
                    {'rrt_tree_visualization_point_scale': config_dict['rrt']['tree_visualization']['point_scale']},
                    {'rrt_tree_visualization_edge_scale': config_dict['rrt']['tree_visualization']['edge_scale']},
                    {'L': config_dict['L']}]
    )

    # finalize
    ld.add_action(pure_pursuit_node)
    return ld