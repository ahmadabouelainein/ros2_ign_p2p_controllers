from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():  
    commands = Node(package='mini_task', executable='CommandPublisher')
    return LaunchDescription([commands])
