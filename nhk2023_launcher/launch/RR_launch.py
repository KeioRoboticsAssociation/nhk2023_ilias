from launch import LaunchDescription
from launch_ros.actions import Node
# import launch_ros.actions
# include
# from launch.actions import IncludeLaunchDescription
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # use_sim_time = True
    # autostart = True
    urdf_file_name = 'simple_odom_robot.urdf'
    urdf = os.path.join(get_package_share_directory('nhk2023_launcher'),
                        urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_description = infp.read()

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        namespace='rr',
        parameters=[{
            'robot_description': robot_description
        }, {
            'rate': 100
        }],
        arguments=[urdf],
    )
    robot_state_publisher_node = Node(package='robot_state_publisher',
                                      executable='robot_state_publisher',
                                      name='robot_state_publisher',
                                      output='screen',
                                      namespace='rr',
                                      parameters=[{
                                          'robot_description':
                                          robot_description
                                      }, {
                                          'rate': 100,
                                          'frame_prefix': 'rr/'
                                      }])

    # map_server_config_path = os.path.join(
    #     get_package_share_directory('nhk2023_launcher'),
    #     'config', 'map', 'combined_params.yaml')

    # map_server_node = Node(
    #     package='nav2_map_server',
    #     executable='map_server',
    #     name='map_server',
    #     output='screen',
    #     emulate_tty=True, # https://github.com/ros2/launch/issues/188
    #     arguments=[map_server_config_path]
    # )

    # lifecycle_nodes = ['map_server']
    # start_lifecycle_manager_cmd = launch_ros.actions.Node(
    #     package='nav2_lifecycle_manager',
    #     executable='lifecycle_manager',
    #     name='lifecycle_manager',
    #     output='screen',
    #     emulate_tty=True,  # https://github.com/ros2/launch/issues/188
    #     parameters=[{'use_sim_time': use_sim_time},
    #                 {'autostart': autostart},
    #                 {'node_names': lifecycle_nodes}])

    # wheelctrl_ros = Node(
    #     package='wheelctrl_ros2',
    #     executable = 'wheelctrl_ros2',
    #     name='wheelctrl_ros2',
    #     output='screen',
    #     emulate_tty = True,
    #     parameters=[os.path.join(
    #         get_package_share_directory('wheelctrl_ros2'),
    #         'config','rr.yaml')]

    # )

    robot_ctrl = Node(
        package='robot_ctrl',
        executable='rr_robot_ctrl',
        name='rr_robot_ctrl',
        namespace='rr',
        output='screen',
    )

    joy = Node(package='joy_linux',
               executable='joy_linux_node',
               name='joy_linux_node',
               namespace='rr',
               output='screen',
               parameters=[{
                   'deadzone': 0.02,
               }])

    simple_gui = Node(
        package='simple_gui',
        executable='simple_gui',
        name='simple_gui',
        output='screen',
        namespace='rr',
    )

    rr_state_ctrl = Node(
        package='rr_state_ctrl',
        executable='rr_state_ctrl',
        name='rr_state_ctrl',
        output='screen',
        namespace='rr',
    )

    joy_server = Node(
        package='joy_server',
        executable='rr_joy_server',
        name='rr_joy_server',
        output='screen',
        namespace='rr',
    )

    return LaunchDescription([
        joint_state_publisher_node,
        robot_state_publisher_node,
        # map_server_node,
        # start_lifecycle_manager_cmd,
        # wheelctrl_ros,
        robot_ctrl,
        # rogi_link_2,
        joy,
        simple_gui,
        rr_state_ctrl,
        joy_server
    ])
