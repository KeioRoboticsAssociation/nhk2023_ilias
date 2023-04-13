from launch import LaunchDescription
from launch_ros.actions import Node
# include
# import os
# from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    dummpy_system = Node(package='dummy_system',
                         executable='dummy_system',
                         name='dummy_system',
                         output='screen',
                         emulate_tty=True,
                         namespace='rr',
                         parameters=[{
                             'base_frame_id': 'rr/base_link',
                             'odom_frame_id': 'rr/odom',
                         }])
    return LaunchDescription([
        dummpy_system,
    ])
