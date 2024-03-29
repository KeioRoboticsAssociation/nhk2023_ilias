from launch import LaunchDescription
from launch_ros.actions import Node
# include
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

    joint_state_publisher_node = Node(package='joint_state_publisher',
                                      executable='joint_state_publisher',
                                      name='joint_state_publisher',
                                      output='screen',
                                      parameters=[{
                                          'robot_description':
                                          robot_description
                                      }, {
                                          'rate': 100
                                      }],
                                      arguments=[urdf],
                                      namespace='er')

    robot_state_publisher_node = Node(package='robot_state_publisher',
                                      executable='robot_state_publisher',
                                      name='robot_state_publisher',
                                      output='screen',
                                      parameters=[{
                                          'robot_description':
                                          robot_description
                                      }, {
                                          'rate': 100,
                                          'frame_prefix': 'er/'
                                      }],
                                      namespace='er')

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

    rogi_link_2 = Node(package='rogilink2',
                       executable='rogilink2',
                       name='rogilink2',
                       output='screen',
                       emulate_tty=True,
                       parameters=[{
                           'config_path':
                           os.path.join(
                               get_package_share_directory('rogilink2'),
                               'config', 'er.yaml')
                       }],
                       namespace='er')

    robot_ctrl = Node(
        package='robot_ctrl',
        executable='er_robot_ctrl',
        name='er_robot_ctrl',
        output='screen',
        namespace='er',
    )

    joy = Node(package='joy_linux',
               executable='joy_linux_node',
               name='joy_linux_node',
               output='screen',
               parameters=[{
                   'deadzone': 0.02
               }],
               namespace='er')

    simple_gui = Node(
        package='simple_gui',
        executable='simple_gui',
        name='simple_gui',
        output='screen',
        namespace='er',
    )

    er_state_ctrl = Node(
        package='er_state_ctrl',
        executable='er_state_ctrl',
        name='er_state_ctrl',
        output='screen',
        namespace='er',
    )

    return LaunchDescription([
        joint_state_publisher_node,
        robot_state_publisher_node,
        # wheelctrl_ros,
        robot_ctrl,
        rogi_link_2,
        joy,
        simple_gui,
        er_state_ctrl
    ])
