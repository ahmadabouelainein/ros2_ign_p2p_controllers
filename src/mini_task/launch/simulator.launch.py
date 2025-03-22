import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    model_pkg = get_package_share_directory('mini_task')
    env = {'IGN_GAZEBO_SYSTEM_PLUGIN_PATH': os.pathsep.join([
        os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', ''),
        os.environ.get('LD_LIBRARY_PATH', '')
    ])}

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/diff_drive/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'],
        output='screen'
    )

    command_publisher = Node(
        package='mini_task',
        executable='CommandPublisher',
        output='screen'
    )

    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', LaunchConfiguration('ign_args')],
        output='screen',
        additional_env=env
    )

    declare_ign_args = DeclareLaunchArgument(
        'ign_args',
        default_value=os.path.join(model_pkg, 'model', 'diff_drive.sdf'),
        description='Arguments to be passed to Ignition Gazebo'
    )

    return LaunchDescription([
        declare_ign_args,
        gazebo,
        bridge,
        command_publisher
    ])
