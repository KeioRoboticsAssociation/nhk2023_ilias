import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import SetEnvironmentVariable

from launch_ros.actions import Node
import xacro


def generate_launch_description():

    # use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    # world_file_name = 'model.sdf'
    pkg_dir = get_package_share_directory('nhk2023_simulator')

    model_path = os.environ["GAZEBO_MODEL_PATH"] = os.path.join(pkg_dir, 'models') + ":" + os.path.join(pkg_dir, 'worlds')
    # world = os.path.join(pkg_dir, 'worlds', 'field2023', world_file_name)

    xacro_file = os.path.join(pkg_dir, 'models','rr', 'rr.xacro')
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # include launch file from gazebo_ros package
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join
            (get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        # launch_arguments={'world': world, 'verbose': 'true', 'use_sim_time': use_sim_time}.items(),
    )




    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-entity', 'rr', '-topic', 'robot_description', '-x', '0', '-y', '0', '-z', '0.1'],
                        output='screen')



    return LaunchDescription([
        SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value = model_path),
        gazebo,
        spawn_entity,
        node_robot_state_publisher,
    ])