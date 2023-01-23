import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    model_pkg = get_package_share_directory('mini_task')
    env = {'IGN_GAZEBO_SYSTEM_PLUGIN_PATH':
           ':'.join([os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', default=''),
                     os.environ.get('LD_LIBRARY_PATH', default='')])}

    spawn = Node(package='ros_ign_bridge', executable='parameter_bridge',
                 arguments=[
                     '/diff_drive/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist'],

                 )
    commands = Node(package='mini_task', executable='CommandPublisher')
    return LaunchDescription([
        DeclareLaunchArgument('ign_args', default_value=model_pkg + '/model/diff_drive.sdf',
                              description='Arguments to be passed to Ignition Gazebo'),
        ExecuteProcess(
            cmd=['ign gazebo',
                 LaunchConfiguration('ign_args'),
                 ],
            output='screen',
            additional_env=env,
            shell=True
        ),
        spawn,
        commands
    ])
