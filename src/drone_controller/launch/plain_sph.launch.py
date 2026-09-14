"""Spawn one plain_sph node per drone.

Usage:
  ros2 launch drone_controller plain_sph.launch.py total_robots:=2
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import OpaqueFunction


def _nodes(context):
    total_robots = int(LaunchConfiguration('total_robots').perform(context))
    robot_prefix = LaunchConfiguration('robot_prefix').perform(context)
    fixed_z = LaunchConfiguration('fixed_z').perform(context)

    return [
        Node(
            package='drone_controller',
            executable='plain_sph',
            name=f'plain_sph_{robot_id}',
            output='screen',
            parameters=[{
                'robot_id': robot_id,
                'robot_prefix': robot_prefix,
                'total_robots': total_robots,
                'fixed_z': float(fixed_z),
            }],
        )
        for robot_id in range(total_robots)
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('total_robots', default_value='2'),
        DeclareLaunchArgument('robot_prefix', default_value='cf_'),
        DeclareLaunchArgument('fixed_z', default_value='0.5'),
        OpaqueFunction(function=_nodes),
    ])