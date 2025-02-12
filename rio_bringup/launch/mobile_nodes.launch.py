from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import  ExecuteProcess
import os


def generate_launch_description():
    ollama_params_path = os.path.join(
        get_package_share_directory('rio_bringup'),
        'params',
        'ollama_params.yaml'
    )
    ollama_nlp_node = Node(
        package='rio_bringup',
        executable='ollama_nlp_node',
        name='ollama_nlp',
        output='screen',
        parameters=[
                ollama_params_path,
                {'navigation_locations_path': os.path.join(
                    get_package_share_directory('rio_bringup'),
                    'params',
                    'navigation_location.yaml'
                )}
        ]
    )
    webrtc_node = Node(
        package='rio_bringup',
        executable='webrtc_node',
        name='webrtc_node',
        parameters=[
            {'port': 8080, 'host': '0.0.0.0'}
        ],
        output='screen'
    )
    rosbridge_websocket = ExecuteProcess(
        cmd=['ros2', 'run', 'rosbridge_server', 'rosbridge_websocket'],
        output='screen'
    )
    return LaunchDescription([
        ollama_nlp_node,
        webrtc_node,
        rosbridge_websocket
    ])
