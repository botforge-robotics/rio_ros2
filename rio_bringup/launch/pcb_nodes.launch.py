import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
import xacro


def generate_launch_description():
    agent_port_value = LaunchConfiguration('agent_port')

    odom_tf_broadcaster_node = Node(
        package='rio_bringup',
        executable='odom_tf_broadcaster',
        name='odom_tf_broadcaster_node',
        output='screen'
    )

    # Add micro-ROS agent
    micro_ros_agent = ExecuteProcess(
        cmd=['ros2', 'run', 'micro_ros_agent',
             'micro_ros_agent', 'udp4', '--port', agent_port_value],
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

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'agent_port',
            default_value='8888',
            description='Port for micro-ROS agent'
        ),
        micro_ros_agent,
        odom_tf_broadcaster_node,
        lidar_udp_node,
    ])
