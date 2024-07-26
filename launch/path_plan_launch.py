import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessStart, OnProcessExit


def generate_launch_description():

    rviz_config_file = os.path.join(get_package_share_directory('rviz_click_path_plan'), 'rviz', 'rcpp.rviz')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file]
        )
    
    path_plan = Node(
        package="rviz_click_path_plan",
        executable="click_planner",
        name="click_planner",
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription([
        path_plan,
        # include_pointcloud_to_grid,
        rviz,

    ])
