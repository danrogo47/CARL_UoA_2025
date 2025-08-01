from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='carl_controller',
            executable='drive',
            name='drive'),
        Node(
            package='carl_controller',
            executable='joint',
            name='joint'),
  ])
