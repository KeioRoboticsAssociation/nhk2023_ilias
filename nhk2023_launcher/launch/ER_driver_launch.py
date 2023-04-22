from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    wheelctrl_ros = Node(
        package='wheelctrl_ros2',
        executable='wheelctrl_ros2',
        name='wheelctrl_ros2',
        output='screen',
        emulate_tty=True,
        namespace='er',
        parameters=[
            os.path.join(get_package_share_directory('wheelctrl_ros2'),
                         'config', 'er.yaml')
        ],
    )

    joy_server = Node(
        package='joy_server',
        executable='er_joy_server',
        name='er_joy_server',
        output='screen',
        emulate_tty=True,
        namespace='er',
    )

    return LaunchDescription([
        wheelctrl_ros,
        joy_server,
    ])
