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
    include_pointcloud_to_grid = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare('pointcloud_to_grid'), '/launch/pcd_map_generator.launch.py']),
        launch_arguments={'pcd_file': LaunchConfiguration('pcd_file')}.items(),
    )

    rviz_config_file = os.path.join(get_package_share_directory('rviz_click_path_plan'), 'rviz', 'rcpp.rviz')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file]
        )
    
    path_plan = Node(
        package="rviz_click_path_plan",
        executable="record_path_generator",
        name="record_path_generator",
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription([
        include_pointcloud_to_grid,
        path_plan,
        # include_pointcloud_to_grid,
        rviz,

    ])
