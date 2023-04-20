from launch import LaunchDescription
from launch_ros.actions import Node
# include
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    lidar = Node(package='sick_scan',
                 executable='sick_generic_caller',
                 name='lidar',
                 output='screen',
                 arguments=[
                     get_package_share_directory('sick_scan') +
                     '/launch/sick_tim_5xx.launch', 'hostname:=192.168.0.80',
                     'frame_id:=er/base_link', 'sw_pll_only_publish:=False'
                 ],
                 parameters=[{
                     'hostname': '192.168.0.80',
                     'frame_id': 'er/base_link',
                     'sw_pll_only_publish': False,
                 }])

    circle_detection = Node(
        package='circle_detection',
        executable='circle_detection',
        name='circle_detection',
        output='screen',
        namespace='er',
    )
    return LaunchDescription([lidar, circle_detection])
