from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('gap_follow'),
        'config',
        'gap_follow.yaml'
        )
    sim_config = os.path.join(
        get_package_share_directory('f1tenth_gym_ros'),
        'config',
        'sim.yaml'
        )
    config_dict = yaml.safe_load(open(config, 'r'))
    sim_config_dict = yaml.safe_load(open(sim_config, 'r'))
    gap_follow_node = Node(
        package='gap_follow',
        executable='reactive_node',
        parameters=[{'scan_cutoff_range': config_dict['scan_cutoff_range']},
                    {'reject_threshold': config_dict['reject_threshold']},
                    {'disparity_threshold': config_dict['disparity_threshold']},
                    {'gap_threshold': config_dict['gap_threshold']},
                    {'mean_window': config_dict['mean_window']},
                    {'disparity_bubble': config_dict['disparity_bubble']},
                    {'velocity_gain': config_dict['velocity_gain']},
                    {'safety_threshold': config_dict['safety_threshold']},
                    {'min_gap_length': config_dict['min_gap_length']},
                    {'scan_beams': sim_config_dict['bridge']['ros__parameters']['scan_beams']},
                    {'scan_fov': sim_config_dict['bridge']['ros__parameters']['scan_fov']}]
    )

    # finalize
    ld.add_action(gap_follow_node)
    return ld
    