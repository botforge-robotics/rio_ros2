import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package share directory
    description_pkg_share = get_package_share_directory('rio_description')

    use_sim_time_value = LaunchConfiguration('use_sim_time')
    agent_port_value = LaunchConfiguration('agent_port')

    # Correct way to handle xacro file path
    xacro_file = os.path.join(description_pkg_share, 'urdf', 'rio_urdf.xacro')
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

    # Include mobile nodes launch file
    mobile_nodes_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('rio_bringup'),
            'launch',
            'mobile_nodes.launch.py'
        )),
        launch_arguments={
            'use_sim_time': use_sim_time_value
        }.items()
    )

    # Include PCB nodes launch file
    pcb_nodes_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('rio_bringup'),
            'launch',
            'pcb_nodes.launch.py'
        )),
        launch_arguments={
            'agent_port': agent_port_value
        }.items()
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
        mobile_nodes_launch,  # Include mobile nodes
        pcb_nodes_launch,  
        robot_state_publisher_node,
        joint_state_publisher_node,
    ])
