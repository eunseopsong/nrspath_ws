from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='polishing_removal_ros2',
            executable='polishing_removal_node',
            name='polishing_removal_node',
            output='screen',
            parameters=[{
                'xyz_path': '',
                'vxyz_path': '',
                'fxyz_path': '',
                'rpy_path': '',
                'waypoint_path': '',
                'removal_npz_path': '',
                'out_dir': './polishing_analysis',
                'regenerated_waypoint_path': './real_flat_filtered_new.txt',
                'cell_mm': 1.0,
                'k_preston': 1.0,
                'tool_axis': '-Z',
                'speed_mode': 'xy',
                'contact_threshold_N': 0.5,
                'speed_threshold_mm_s': 0.1,
                'pad_radius_mm': 0.0,
                'n_out': 0,
                'gain': 1.0,
                'w_min': 0.3,
                'w_max': 3.0,
                'smooth_window': 0,
            }]
        )
    ])