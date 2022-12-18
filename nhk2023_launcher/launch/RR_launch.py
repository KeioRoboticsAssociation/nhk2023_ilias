from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    urdf_file_name = 'simple_odom_robot.urdf'
    urdf = os.path.join(
        get_package_share_directory('nhk2023_launcher'),
        urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_description = infp.read()

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}, {'rate': 100}],
        arguments=[urdf]
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}, {'rate': 100}]
    )

    return LaunchDescription([
        joint_state_publisher_node,
        robot_state_publisher_node
    ])