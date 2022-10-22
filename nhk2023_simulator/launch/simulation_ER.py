
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

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    nhk2023_simulator = get_package_share_directory('nhk2023_simulator')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': '-r worlds/nhk2023.world --render-engine ogre'
            #WSLで動かせるようにogreを指定しているが、他の環境では不要
        }.items(),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['d', os.path.join(nhk2023_simulator, 'config/rviz/simulation.rviz')],
        #rvizの設定ファイルを指定、モード毎に読み込むファイルを変えるみたいなのがあっても良いかも
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/navsat@sensor_msgs/msg/NavSatFix@ignition.msgs.NavSat'],
        #ここに追加すると、rosとignitionの間でデータをやり取りできる、現状は仮置き
        output='screen'
    )

    return LaunchDescription([
        gz_sim,
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Open RViz.'),
        rviz,
        bridge
    ])

