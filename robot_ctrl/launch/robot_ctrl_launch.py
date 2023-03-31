import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    robot_ctrl = Node(
        package='robot_ctrl',
        executable='robot_ctrl',
        name='robot_ctrl',
        output='screen',
    )

    return LaunchDescription([
        robot_ctrl,
    ])
