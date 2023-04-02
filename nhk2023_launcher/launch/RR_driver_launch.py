from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    wheelctrl_ros = Node(package='wheelctrl_ros2',
                         executable='wheelctrl_ros2',
                         name='wheelctrl_ros2',
                         output='screen',
                         emulate_tty=True,
                         parameters=[
                             os.path.join(
                                 get_package_share_directory('wheelctrl_ros2'),
                                 'config', 'rr.yaml')
                         ])

    rogi_link_2 = Node(package='rogilink2',
                    executable='rogilink2',
                    name='rogilink2',
                    output='screen',
                    emulate_tty=True,
                    parameters=[{
                        'config_path':
                        os.path.join(
                            get_package_share_directory('rogilink2'),
                            'config', 'rr.yaml')
                    }])


    bno055 = Node(
        package='rogibno055',
        executable='rogibno055',
        name='rogibno055',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'pose_frame_id': "imu_link",
            'publish_tf': False,
            'port': "/dev/BNO",
            'mode': "imu",
        }]
    )

    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        emulate_tty=True,
        # read parameters from file
        parameters=[os.path.join(
            get_package_share_directory('nhk2023_launcher'),
             'ekf.yaml')],
    )
    # print(os.path.join(
    #         get_package_share_directory('nhk2023_launcher'),
    #          'ekf.yaml'))


    return LaunchDescription([
        rogi_link_2,
        wheelctrl_ros,
        bno055,
        ekf,
    ])
