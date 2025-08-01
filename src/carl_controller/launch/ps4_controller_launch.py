from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """Launches the controller and controller state monitor nodes."""
    return LaunchDescription([
        Node(
            package='carl_controller',
            executable='controller',
            name='controller'),
        Node(
            package='carl_controller',
            executable='controller_state_monitor',
            name='controller_state_monitor'),
  ])
