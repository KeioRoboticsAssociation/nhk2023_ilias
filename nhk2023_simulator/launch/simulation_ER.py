
        #  ...                                     ..:.
        # :..:::::.......               ........:::::..:
        # .::..:::::::::::.            .:::::::::::..::
        #  :::..:::::::::::.         .:::::::::::..:::.
        #  .::::   .:::::::::.      .:::::::::.  .::::
        #  .::::.  :::::::::::.   .:::::::::::. .:::::
        #   ::::::::::::::::::::. :::::::::::::::::::.
        #   :::::::::::::::::::::. ::::::::::::::::::.
        #  .:::::::::::::::::::::::..:::::::::::::::::
        #    .::::::::::::::::::::::. ::::::::::::::.
        #      .::::::::::::::::::::::..::::::::::.
        #        .:::::::::::::::::::::. :::::::.
        #          .:::::::::::::::::::::..:::.
        #            .::::::::::::::::::::. .
        #            ...::::::::::::::::::::.
        #          .:::...:::::::::::::::::::.
        #         .::::::...:::::::::::::::::::.
        #       .::::::::::...::::::::::::::::::.
        #      ::::::::::::::...::::::::::::::::::.
        #    .:::::::::::::::::. .::::::::::::::::::
        #   ::::::::::::::::::.    .:::::::::::::::::.
        #  :::::::::::::::::.        .::::::::::::::::.
        #  :::::::::::::::.            .::::::::::::::.
        #  :::::::::::::.                .::::::::::::.
        #   ::::::::::.                    .:::::::::.
        #    ..::::..                        .:::::.


# Path: nhk2023_ilias/nhk2023_simulator/launch/simulation_ER.py

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node

def generate_launch_description():
    nhk2023_simulator = get_package_share_directory('nhk2023_simulator')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': '-r nhk2023.world'
        }.items()
    )

    rviz = Node(
            package='rviz2',
            executable='rviz2',
            arguments=['d', os.path.join(nhk2023_simulator, 'config/rviz/simulation.rviz')],
            condition=IfCondition(LaunchDescription('rviz'))
    )

    bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/nhk2023/robot_description', '/robot_description'],
            parameters=[os.path.join(nhk2023_simulator, 'config/parameter_bridge.yaml')],
            output='screen'
    )

    return LaunchDescription([
        gz_sim,
        rviz,
        bridge,
        DeclareLaunchArgument('rviz', default_value='true',description='Open RViz.')
    ])