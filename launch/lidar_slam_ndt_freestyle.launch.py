import os

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch.actions import SetEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription

def generate_launch_description():
    """Launch lidarslam Node."""
    ndt_param_file = launch.substitutions.LaunchConfiguration(
        'ndt_freestyle_prefix',
        default = os.path.join(get_package_share_directory('slam_tools'),
            'config', 'lidar_slam_ndt_freestyle.param.yaml'))

    lidarslam_node_param = DeclareLaunchArgument(            
        'ndt_param_file', 
        default_value=ndt_param_file,
        description='Path to config file of lidarslam node.'),

    # mapping
    mapping_node = launch_ros.actions.Node(
        package='scanmatcher',
        executable='scanmatcher_node',
        parameters=[ndt_param_file],
        remappings=[('/input_cloud','/points')],
        output='screen'
        )

    slam_node = launch_ros.actions.Node(
        package='graph_based_slam',
        executable='graph_based_slam_node',
        parameters=[ndt_param_file],
        output='screen'
        )


    return launch.LaunchDescription([
        mapping_node,
        slam_node])
