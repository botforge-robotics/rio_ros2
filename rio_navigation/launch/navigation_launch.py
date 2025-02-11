import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    # Get the package path using FindPackageShare
    pkg_share_navigation = FindPackageShare('rio_navigation').find(
        'rio_navigation')  # Get the package share directory
    pkg_share_mapping = FindPackageShare('rio_mapping').find(
        'rio_mapping')  # Get the package share directory
    params_file_path = PathJoinSubstitution([
        pkg_share_navigation, 'params', LaunchConfiguration('params_file')
    ])
    map_file_path = PathJoinSubstitution([
        pkg_share_mapping, 'maps', LaunchConfiguration('map_file')
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_file',
            default_value='house.yaml',
            description='Full path to map file to load'
        ),

        DeclareLaunchArgument(
            'params_file',
            default_value='nav2_sim_params.yaml',
            description='Full path to param file to load'
        ),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),


        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('nav2_bringup').find('nav2_bringup'),
                    'launch',
                    'bringup_launch.py'
                ])),
            launch_arguments={
                'params_file': params_file_path,
                'map': map_file_path,
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }.items(),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=[
                '-d', PathJoinSubstitution([pkg_share_navigation, 'rviz', 'navigation.rviz'])],
        )
    ])
