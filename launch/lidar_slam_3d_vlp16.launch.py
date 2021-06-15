import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from os.path import expanduser

def generate_launch_description():
    # Configure environment
    stdout_linebuf_envvar = SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')
    stdout_colorized_envvar = SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1')

    # Simulated time
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Unified Robot Description Format
    pkg_share = FindPackageShare('slam_tools').find('slam_tools')
    urdf_file = os.path.join(pkg_share, 'urdf', 'freestyle_basic.urdf.xml')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
    rsp_params = {'robot_description': robot_desc}

    # Package directory prefix
    lidar_driver_pkg_prefix = get_package_share_directory('velodyne')
    slam_tools_pkg_prefix = get_package_share_directory('slam_tools')

    # Nodes configurations
    rviz_config_file = os.path.join(slam_tools_pkg_prefix, 'rviz', 'lidar_slam_3d_vlp16.rviz')

    # Drivers
    lidar_driver = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([lidar_driver_pkg_prefix,
                '/launch/vlp16_nodes.launch.py'])
    )

    # Tf transformations
    robot_state = Node(
            package='robot_state_publisher', 
            executable='robot_state_publisher',
            output='screen',
            parameters=[
                rsp_params,
                {'use_sim_time': use_sim_time},
            ]
    )

    # SLAM
    slam_tool = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([slam_tools_pkg_prefix,
                '/launch/lidar_slam_ndt_freestyle.launch.py'])
    )
    # Visualization_tool
    visaul_tool = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
    )

    return LaunchDescription([
        lidar_driver,
        robot_state,
        slam_tool,
        visaul_tool
    ])
