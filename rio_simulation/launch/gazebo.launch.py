from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from ament_index_python import get_package_share_directory
import os
import xacro


def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('rio_simulation')
    pkg_share_rio_description = get_package_share_directory('rio_description')
    world_file_value = LaunchConfiguration('world')
    use_sim_time_value = LaunchConfiguration('use_sim_time')

    # Construct full world path
    world_path = PathJoinSubstitution([pkg_share, 'worlds', world_file_value])

    xacro_file = os.path.join(
        pkg_share_rio_description, 'urdf', 'rio_urdf.xacro')
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
        parameters=[{'use_sim_time': use_sim_time_value}, {'rate': 30}],
        output='screen'
    )

    # Gazebo classic launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={
            'world': world_path,
            'use_sim_time': use_sim_time_value
        }.items()
    )

    # Spawn robot
    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'rio',
            '-z', '0.1',
        ],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='empty.world',
            description='World file name (will be loaded from worlds directory)'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        robot_state_publisher_node,
        joint_state_publisher_node,
        gazebo,
        spawn,
    ])
