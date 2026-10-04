"""Spawn one plain_sph node per drone, with every tunable exposed as a launch arg.

Usage:
  ros2 launch drone_controller plain_sph.launch.py total_robots:=5 R:=0.8 f_k:=3.0

Defaults match the defaults declared in plain_sph.py, so launching with no
args behaves exactly like before.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# name -> default. All are floats except the ints/strings handled below.
FLOAT_ARGS = {
    'fixed_z': 0.5,
    'particle_mass': 7000.0,
    'smoothing_h': 3.0,
    'rho_0': 1000.0,
    'gamma': 7.0,
    'alpha': 1.0,
    'beta': 2.0,
    'zetta_extra': 30.0,
    'xc': 2.0,
    'yc': 2.0,
    'R': 0.5,
    'f_beta': 1.0,
    'f_k': 2.0,
}


def _nodes(context):
    total_robots = int(LaunchConfiguration('total_robots').perform(context))
    robot_prefix = LaunchConfiguration('robot_prefix').perform(context)
    shared = {k: float(LaunchConfiguration(k).perform(context)) for k in FLOAT_ARGS}

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
                **shared,
            }],
        )
        for robot_id in range(total_robots)
    ]


def generate_launch_description():
    args = [
        DeclareLaunchArgument('total_robots', default_value='2'),
        DeclareLaunchArgument('robot_prefix', default_value='cf_'),
    ]
    args += [DeclareLaunchArgument(k, default_value=str(v)) for k, v in FLOAT_ARGS.items()]
    return LaunchDescription(args + [OpaqueFunction(function=_nodes)])