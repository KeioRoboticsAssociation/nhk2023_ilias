from launch import LaunchDescription
from launch_ros.actions import Node
# include
from launch.actions import IncludeLaunchDescription
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

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        emulate_tty=True, # https://github.com/ros2/launch/issues/188
        arguments=['config/map/combined_params.yaml']
    )

    static_map_odom_tf_broadcaster_node = Node(
        package='nhk2023_launcher',
        executable='static_odom_map_broadcaster',
        name='static_odom_map_broadcaster',
        output='screen'
    )

    wheelctrl_ros_launch = IncludeLaunchDescription(
        launch_description_source=os.path.join(
        get_package_share_directory('wheelctrl_ros2'),
        'launch',
        'wheelctrl_ros2_launch.py')
    )
    
    rogi_link_2 = Node(
        package='rogilink2',
        executable = 'rogilink2',
        name='rogilink2',
        output='screen',
        emulate_tty = True,
        parameters=[{'config_path':os.path.join(
            get_package_share_directory('rogilink2'),
            'config','rrconfig.yaml')}]
    )

    robot_ctrl_launch_file = os.path.join(
        get_package_share_directory('robot_ctrl'),
        'launch',
        'robot_ctrl_launch.py')

    robot_ctrl_launch = IncludeLaunchDescription(
        launch_description_source=robot_ctrl_launch_file
    )


    return LaunchDescription([
        joint_state_publisher_node,
        robot_state_publisher_node,
        map_server_node,
        static_map_odom_tf_broadcaster_node,
        wheelctrl_ros_launch,
        robot_ctrl_launch
    ])