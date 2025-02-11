from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rio_bringup',
            executable='webrtc_node',
            name='webrtc_node',
            parameters=[
                {'port': 8080, 'host': '0.0.0.0'}
            ],
            output='screen'
        )
    ])