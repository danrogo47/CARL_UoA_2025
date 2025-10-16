from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Your existing controller nodes
        Node(
            package='carl_controller',
            executable='controller',
            name='controller'),
        Node(
            package='carl_controller',
            executable='controller_state_monitor',
            name='controller_state_monitor'),
        # Add rosbridge for web dashboard
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            parameters=[{
                'port': 9090,
            }]
        ),
    ])