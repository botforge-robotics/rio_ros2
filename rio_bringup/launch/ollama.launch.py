from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get paths to parameter files
    ollama_params_path = os.path.join(
        get_package_share_directory('rio_bringup'),
        'params',
        'ollama_params.yaml'
    )
    

    return LaunchDescription([
        Node(
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
    ])