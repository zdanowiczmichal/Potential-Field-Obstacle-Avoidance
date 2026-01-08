import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_share = get_package_share_directory('box_bot_sim')
    
    # Fix for Gazebo networking
    set_gz_ip = SetEnvironmentVariable('GZ_IP', '127.0.0.1')

    # Robot State Publisher
    robot_description = ParameterValue(
        Command(['xacro ', os.path.join(pkg_share, 'urdf', 'box_bot.urdf.xacro')]),
        value_type=str
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # Gazebo Sim (using the command that works for you)
    # Gazebo Sim
    world_path = os.path.join(pkg_share, 'worlds', 'myworld.sdf')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': f'-r {world_path}'
        }.items(),
    )


    # Spawn
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'box_bot', '-topic', 'robot_description', '-z', '0.2'],
        output='screen',
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={os.path.join(pkg_share, "config", "bridge.yaml")}'],
        output='screen'
    )

    return LaunchDescription([set_gz_ip, rsp, gz_sim, spawn, bridge])
