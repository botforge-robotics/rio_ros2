from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
import os


def generate_launch_description():
    # Declare launch argument for LLM backend
    llm_backend_arg = DeclareLaunchArgument(
        'llm_backend',
        default_value='ollama',
        description='LLM backend to use (ollama or groq)'
    )
    
    # Paths for params
    ollama_params_path = os.path.join(
        get_package_share_directory('rio_bringup'),
        'params',
        'ollama_params.yaml'
    )
    
    groq_params_path = os.path.join(
        get_package_share_directory('rio_bringup'),
        'params',
        'groq_params.yaml'
    )
    
    navigation_locations_path = os.path.join(
        get_package_share_directory('rio_bringup'),
        'params',
        'navigation_location.yaml'
    )
    
    # Create conditions for each node
    condition_ollama = IfCondition(PythonExpression(["'", LaunchConfiguration('llm_backend'), "' == 'ollama'"]))
    condition_groq = IfCondition(PythonExpression(["'", LaunchConfiguration('llm_backend'), "' == 'groq'"]))
    
    # Ollama LLM Node
    ollama_llm_node = Node(
        package='rio_bringup',
        executable='ollama_llm_node',
        name='ollama_llm',
        output='screen',
        parameters=[
            ollama_params_path,
            {'navigation_locations_path': navigation_locations_path}
        ],
        condition=condition_ollama
    )
    
    # groq LLM Node
    groq_llm_node = Node(
        package='rio_bringup',
        executable='groq_llm_node',
        name='groq_llm',
        output='screen',
        parameters=[
            groq_params_path,
            {'navigation_locations_path': navigation_locations_path}
        ],
        condition=condition_groq
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
        llm_backend_arg,
        ollama_llm_node,
        groq_llm_node,
        webrtc_node,
        rosbridge_websocket
    ])
