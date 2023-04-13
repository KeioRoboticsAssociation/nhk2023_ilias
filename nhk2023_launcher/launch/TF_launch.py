from launch import LaunchDescription
from launch_ros.actions import Node
# include


def generate_launch_description():

    static_tf_broadcaster_node = Node(
        package='nhk2023_launcher',
        executable='static_tf_broadcaster',
        name='static_tf_broadcaster',
        output='screen',
    )

    return LaunchDescription([static_tf_broadcaster_node])
