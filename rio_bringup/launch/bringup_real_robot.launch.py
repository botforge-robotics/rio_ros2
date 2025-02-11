from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, PythonLaunchDescriptionSource
import os
import xacro
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package share directory
    description_pkg_share = FindPackageShare('rio_description')

    use_sim_time_value = LaunchConfiguration('use_sim_time')
    port_value = LaunchConfiguration('port')

    xacro_file = PathJoinSubstitution(
        [description_pkg_share, 'urdf', 'rio_urdf.xacro'])
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_urdf}],
        output='screen'
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time_value}, {'rate': 100}],
        output='screen'
    )

    odom_tf_broadcaster_node = Node(
        package='rio_bringup',
        executable='odom_tf_broadcaster',
        name='odom_tf_broadcaster_node',
        output='screen'
    )

    # Add micro-ROS agent
    micro_ros_agent = ExecuteProcess(
        cmd=['ros2', 'run', 'micro_ros_agent',
             'micro_ros_agent', 'udp4', '--port', port_value],
        output='screen'
    )

    # Add LIDAR UDP node
    lidar_udp_node = Node(
        package='rio_bringup',
        executable='lidar_udp_node',
        name='lidar_udp_node',
        parameters=[{
            'udp_port': 9999,
            'frame_id': 'lidar',
            'angle_min': -3.14159,  # -180 degrees
            'angle_max': 3.14159,   # 180 degrees
            'range_min': 0.15,
            'range_max': 6.0
        }],
        output='screen'
    )

    # Add WebRTC and Ollama includes
    webrtc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('rio_bringup'),
            '/launch/webrtc.launch.py'
        ])
    )
    
    ollama_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('rio_bringup'),
            '/launch/ollama_nlp.launch.py'
        ])
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'port',
            default_value='8888',
            description='Port for micro-ROS agent'
        ),
        webrtc_launch,
        ollama_launch,
        micro_ros_agent,
        odom_tf_broadcaster_node,
        robot_state_publisher_node,
        joint_state_publisher_node,
        lidar_udp_node,
    ])
