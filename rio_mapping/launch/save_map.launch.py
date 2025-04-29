from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.substitutions import ThisLaunchFileDir


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'map_name',
            default_value='my_map',
            description='Base name for map files'
        ),

        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                '-f', PathJoinSubstitution([
                    ThisLaunchFileDir(),
                    '../maps',
                    LaunchConfiguration('map_name')
                ])
            ],
            output='screen'
        )
    ])
