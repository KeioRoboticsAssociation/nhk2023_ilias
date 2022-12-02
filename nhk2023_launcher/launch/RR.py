# make launch file for RR robot

from rclpy.node import Node

from launch import LaunchDescription

# main
def generate_launch_description():
  # launch robot_ctrl node
  robot_ctrl = Node(
    package='robot_ctrl',
    executable='robot_ctrl',
    name='robot_ctrl',
    output='screen',
  )

  # launch
  return LaunchDescription([
    robot_ctrl,
  ])
