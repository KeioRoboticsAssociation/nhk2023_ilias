import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import SetEnvironmentVariable
from launch.actions import ExecuteProcess

from launch_ros.actions import Node
import xacro


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    world_file_name = 'model.world'
    pkg_dir = get_package_share_directory('nhk2023_simulator')

    world = os.path.join(pkg_dir, 'worlds', 'field2023' , world_file_name)

    # if 'GAZEBO_MODEL_PATH' in os.environ:
    #     model_path =  os.environ['GAZEBO_MODEL_PATH'] + ':' +  os.path.join(pkg_dir, 'models') + ":" + os.path.join(pkg_dir, 'worlds')
    # else:
    #     model_path =   os.path.join(pkg_dir, 'models') + ":" + os.path.join(pkg_dir, 'worlds')

    # os.environ['GAZEBO_MODEL_PATH'] = os.path.join(pkg_dir, 'models') + ":" + os.path.join(pkg_dir, 'worlds')



    xacro_file = os.path.join(pkg_dir, 'models','rr', 'rr.xacro')
    urdf_file = os.path.join(pkg_dir , 'models','rr', 'rr.urdf')
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

        # include launch file from gazebo_ros package
    gazebo = ExecuteProcess(
            cmd=['gazebo', '--verbose', world, '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so'],
            output='screen')
    # gazebo = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([get_package_share_directory('gazebo_ros'), '/launch', '/gazebo.launch.py']),
    #     launch_arguments={'verbose': 'true', 'debug': 'true', 'gui': 'true', 'paused': 'false', 'use_sim_time': use_sim_time}.items(),
    # )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-file', urdf_file,
                                   '-entity', 'rr',
                                   '-x', '5.0', '-y', '0.0', '-z', '3.0'],
                        output='screen')

    return LaunchDescription([
        # SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value = model_path),
        # ExecuteProcess(
        #     cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
        #     output='screen'),
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
    ])