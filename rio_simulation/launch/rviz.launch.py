from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('rio_simulation')

    rviz_config_value = LaunchConfiguration('rviz_config')

    # Construct full config path
    config_path = PathJoinSubstitution([
        pkg_share,
        'rviz',
        rviz_config_value
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', config_path],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz_config',
            default_value='default.rviz',
            description='RViz config file name (will be loaded from config directory)'
        ),
        rviz_node,
    ])
