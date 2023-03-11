from launch import LaunchDescription
from launch_ros.actions import Node
import launch_ros.actions
# include
from launch.actions import IncludeLaunchDescription
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = True
    autostart = True
    urdf_file_name = 'simple_odom_robot.urdf'
    urdf = os.path.join(
        get_package_share_directory('nhk2023_launcher'),
        urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_description = infp.read()

    dummy_system = Node(
        package='dummy_system',
        executable='dummy_system',
        name='dummy_system',
        output='screen',
    )

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

    static_tf_broadcaster_node = Node(
        package='nhk2023_launcher',
        executable='static_tf_broadcaster',
        name='static_tf_broadcaster',
        output='screen'
    )

    robot_ctrl = Node(
        package='robot_ctrl',
        executable='rr_robot_ctrl',
        name='rr_robot_ctrl',
        output='screen',
    )

    state_ctrl = Node(
        package='rr_state_ctrl',
        executable='rr_state_ctrl',
        name='rr_state_ctrl',
        output='screen',
    )


    joy = Node(
        package='joy_linux',
        executable='joy_linux_node',
        name='joy_linux_node',
        output='screen',
    )


    return LaunchDescription([
        dummy_system,
        joint_state_publisher_node,
        robot_state_publisher_node,
        static_tf_broadcaster_node,
        robot_ctrl,
        state_ctrl,
        joy,
    ])