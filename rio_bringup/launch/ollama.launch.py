from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rio_bringup',
            executable='ollama_nlp_node',
            name='ollama_nlp',
            output='screen',
            parameters=[
                # Load default parameters
                get_package_share_directory('rio_bringup') + '/params/ollama_params.yaml',
                # You can add additional parameter files here
            ],
            arguments=['--ros-args', '--log-level', 'info']
        )
    ])