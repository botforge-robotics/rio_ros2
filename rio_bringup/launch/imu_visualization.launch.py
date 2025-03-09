from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Launch arguments
    imu_topic_arg = DeclareLaunchArgument(
        'imu_topic',
        default_value='/imu/data',
        description='Topic name for IMU data'
    )

    # Nodes
    mobile_frame_node = Node(
        package='rio_bringup',
        executable='mobile_frame',
        name='mobile_frame',
        output='screen',
        on_exit=lambda exit_status: print(f"Mobile frame node exited with status {exit_status}")
    )
    
    imu_viz_node = Node(
        package='rio_bringup',
        executable='imu_viz_2d',
        name='imu_viz_2d',
        output='screen',
        arguments=[LaunchConfiguration('imu_topic')],
        on_exit=lambda exit_status: print(f"IMU viz node exited with status {exit_status}")
    )
    
    # Create and return launch description
    return LaunchDescription([
        imu_topic_arg,
        mobile_frame_node,
        imu_viz_node
    ]) 