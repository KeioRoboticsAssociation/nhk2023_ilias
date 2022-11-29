import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': '-r worlds/nhk2023.world --render-engine ogre'
            #WSLで動かせるようにogreを指定しているが、他の環境では不要
        }.items(),
    )

    # nhk2023_simulator = get_package_share_directory('nhk2023_simulator')
    # rviz = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     arguments=['d', os.path.join(nhk2023_simulator, 'config/rviz/simulation.rviz')],
    #     #rvizの設定ファイルを指定、モード毎に読み込むファイルを変えるみたいなのがあっても良いかも
    # )
    # # rvizはメインのlaunchファイルで起動するので、ここではコメントアウト

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan', #Lidar
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU', #IMU
            # '/world/nhk2023_field/model/er/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist', #cmd_vel
            '/model/er/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V' #odm_robotの位置情報
            #ここに追加すると、rosとignitionの間でデータをやり取りできるようになる
            ],
        output='screen',
        remappings=[
                ('/world/nhk2023_field/model/er/link/base_link/sensor/lidar/scan', '/er/lidar/scan'),
                ('/model/er/pose', '/er/pose'),
                # ('/world/nhk2023_field/model/er/cmd_vel','/er/cmd_vel')
                #ここでremappingすると、rosのトピック名を変えられる
            ],
    )

    odom_publisher = Node(
        package= 'nhk2023_simulator',
        executable='er_odom_publisher',
        output='screen',
    )


    return LaunchDescription([
        gz_sim,
        # DeclareLaunchArgument('rviz', default_value='true',
        #                       description='Open RViz.'),
        # rviz,
        bridge,
        odom_publisher,
    ])

